
import time
import threading
from collections import deque

import numpy as np
import torch
import mujoco
import mujoco.viewer

from berkeley_humanoid_lite_lowlevel.policy.config import Cfg
from berkeley_humanoid_lite_lowlevel.policy.gamepad import Se2Gamepad


def quat_rotate_inverse(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate a vector by the inverse of a quaternion.

    Args:
        q (torch.Tensor): Quaternion [w, x, y, z]
        v (torch.Tensor): Vector to rotate

    Returns:
        torch.Tensor: Rotated vector
    """
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = torch.cross(q_vec, v, dim=-1) * q_w * 2.0
    c = q_vec * (torch.dot(q_vec, v)) * 2.0
    return a - b + c


def quat_mul(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    """Multiply two quaternions [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return torch.stack([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def uniform_noise(shape, scale: float) -> torch.Tensor:
    """Sample uniform noise in [-scale, scale]."""
    return (torch.rand(shape) * 2.0 - 1.0) * scale


def load_mujoco_model(cfg: Cfg) -> mujoco.MjModel:
    """Load the sim2sim scene of a deployment config.

    The scene is the config's mjcf_path. Every actuator
    becomes a torque motor (the course robot's MJCF has position servos), since the PD loop of MujocoSimulator
    computes the joint torques. An optional sim_payload entry welds the payload to the base, mass only, like the
    payload event of the training task: one solid box (mass in kg, box center in the base frame, edge lengths in m),
    or `parts`, a list of such boxes (the F3 parts), each a separate rigid body. An optional sim_body_masses entry
    (body name: kg) sets body masses, scaling each body's inertia with its mass (the real walking test policies).
    """
    mjcf_path = cfg.get("mjcf_path", None)
    if mjcf_path is None:
        raise ValueError("The deployment config has no mjcf_path. Use a config of the course robot, for example "
                         "policies/baseline/policy.yaml (scene: source/robot_models/data/robots/asmr2026/).")
    spec = mujoco.MjSpec.from_file(mjcf_path)

    for actuator in spec.actuators:
        if actuator.biastype != mujoco.mjtBias.mjBIAS_NONE:
            # keeps the force range; the control range inherited from the joint range would clip the torque
            actuator.set_to_motor()
            actuator.inheritrange = 0.0
            actuator.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE

    payload = cfg.get("sim_payload", None)
    if payload is not None:
        parts = payload.parts if "parts" in payload else [payload]
        for part in parts:
            a, b, c = part.size
            spec.body("base").add_body(
                name=part.get("name", "payload") if "parts" in payload else "payload",
                explicitinertial=True,
                mass=part.mass,
                ipos=list(part.position),
                inertia=[part.mass / 12.0 * (b**2 + c**2), part.mass / 12.0 * (a**2 + c**2),
                         part.mass / 12.0 * (a**2 + b**2)],
            )

    model = spec.compile()
    for name, mass in (cfg.get("sim_body_masses", None) or {}).items():
        body = model.body(name)
        model.body_inertia[body.id] *= mass / model.body_mass[body.id]
        model.body_mass[body.id] = mass
    return model


class MujocoEnv:
    def __init__(self, cfg: Cfg):
        self.cfg = cfg

        self.mj_model = load_mujoco_model(cfg)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = self.cfg.physics_dt
        self.mj_viewer = mujoco.viewer.launch_passive(self.mj_model, self.mj_data)


class MujocoVisualizer(MujocoEnv):
    """MuJoCo simulation environment for the Berkeley Humanoid Lite robot.

    This class handles the physics simulation, state observation, and control
    of the robot in the MuJoCo environment.

    Args:
        cfg (Cfg): Configuration object containing simulation parameters
    """
    def __init__(self, cfg: Cfg):
        super().__init__(cfg)

        self.num_dofs = self.mj_model.nu
        print(f"Number of DOFs: {self.num_dofs}")

    def reset(self) -> None:
        """Reset the simulation environment to initial state.

        Returns:
            torch.Tensor: Initial observations after reset
        """
        self.mj_data.qpos[0:3] = np.array([0.0, 0.0, 0.0])  # Reset base position to origin
        self.mj_data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])  # Default quaternion orientation
        self.mj_data.qpos[7:7 + self.num_dofs] = 0
        self.mj_data.qvel[:] = 0

    def step(self, robot_observations: np.array) -> None:
        """Execute one simulation step with the given actions.

        Args:
            actions (torch.Tensor): Joint position targets for controlled joints

        Returns:
            torch.Tensor: Updated observations after executing the action
        """
        robot_base_quat = robot_observations[0:4]
        robot_base_ang_vel = robot_observations[4:7]
        robot_joint_pos = robot_observations[7:7 + self.num_dofs]
        robot_joint_vel = robot_observations[7 + self.num_dofs:7 + self.num_dofs * 2]
        robot_mode = robot_observations[7 + self.num_dofs * 2]
        command_velocity = robot_observations[7 + self.num_dofs * 2 + 1:7 + self.num_dofs * 2 + 4]

        self.mj_data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
        self.mj_data.qpos[3:7] = robot_base_quat
        self.mj_data.qvel[0:3] = np.array([0.0, 0.0, 0.0])
        self.mj_data.qvel[3:6] = robot_base_ang_vel
        self.mj_data.qpos[7:] = robot_joint_pos
        self.mj_data.qvel[6:] = robot_joint_vel

        mujoco.mj_step(self.mj_model, self.mj_data)
        self.mj_viewer.sync()


class MujocoSimulator(MujocoEnv):
    """MuJoCo simulation environment for the Berkeley Humanoid Lite robot.

    This class handles the physics simulation, state observation, and control
    of the robot in the MuJoCo environment.

    Args:
        cfg (Cfg): Configuration object containing simulation parameters
        obs_noise (bool): Add sensor noise to the policy observations, matching the
            observation corruption used in training
    """
    # Uniform noise ranges of the policy observations in the training environment
    # (ObservationsCfg.PolicyCfg in tasks/locomotion/velocity/config/*/env_cfg.py).
    # Without it, trained policies can settle into a standing equilibrium and ignore velocity commands.
    OBS_NOISE_ANG_VEL = 0.3
    OBS_NOISE_GRAVITY = 0.05
    OBS_NOISE_JOINT_POS = 0.05
    OBS_NOISE_JOINT_VEL = 2.0

    def __init__(self, cfg: Cfg, obs_noise: bool = False):
        super().__init__(cfg)
        self.obs_noise = obs_noise
        # a task may train with less joint position noise (config entry obs_noise_joint_pos)
        self.obs_noise_joint_pos = cfg.get("obs_noise_joint_pos", self.OBS_NOISE_JOINT_POS)
        self.physics_substeps = int(np.round(self.cfg.policy_dt / self.cfg.physics_dt))

        # Initialize simulation parameters
        self.sensordata_dof_size = 3 * self.mj_model.nu
        self.gravity_vector = torch.tensor([0.0, 0.0, -1.0])

        # The policy observes the base frame, but the IMU can be mounted rotated on the base (the course robot's is
        # turned 90 deg about z). Its fixed orientation in the base frame converts the readings.
        imu_site = self.mj_model.sensor("imu_quat").objid[0]
        mujoco.mj_kinematics(self.mj_model, self.mj_data)
        base_rot = self.mj_data.body("base").xmat.reshape(3, 3)
        imu_rot = self.mj_data.site_xmat[imu_site].reshape(3, 3)
        self.imu_rot_in_base = torch.tensor(base_rot.T @ imu_rot, dtype=torch.float32)
        imu_quat_in_base = np.zeros(4)
        mujoco.mju_mat2Quat(imu_quat_in_base, (base_rot.T @ imu_rot).flatten())
        self.base_quat_in_imu = torch.tensor(imu_quat_in_base * [1.0, -1.0, -1.0, -1.0], dtype=torch.float32)

        # Initialize control parameters
        self.joint_kp = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.joint_kd = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)
        self.effort_limits = torch.zeros((self.cfg.num_joints,), dtype=torch.float32)

        self.joint_kp[:] = torch.tensor(self.cfg.joint_kp)
        self.joint_kd[:] = torch.tensor(self.cfg.joint_kd)
        self.effort_limits[:] = torch.tensor(self.cfg.effort_limits)

        # Optional joint friction model (config key joint_friction), replacing the model file's frictionloss and
        # damping: f = f_c (1 + (r - 1) exp(-(qd / v_st)^2)) + a |tau| + c tau^2 as MuJoCo's implicit frictionloss,
        # updated every physics step with the clipped PD torque tau, plus viscous damping. This is the measured
        # model of the course actuators (FrictionImplicitActuator in Isaac Lab); MuJoCo's frictionloss constraint
        # holds the joint at rest, so no smoothing velocity is needed.
        self.joint_friction = self.cfg.get("joint_friction", None)
        if self.joint_friction is not None:
            self.friction_dofs = np.array([self.mj_model.jnt_dofadr[self.mj_model.joint(name).id]
                                           for name in self.cfg.joints])
            self.friction_coulomb = np.array(self.joint_friction.coulomb, dtype=np.float64)
            self.friction_stribeck_ratio = np.array(self.joint_friction.stribeck_ratio, dtype=np.float64)
            self.mj_model.dof_damping[self.friction_dofs] = np.array(self.joint_friction.viscous, dtype=np.float64)
            self.mj_model.dof_frictionloss[self.friction_dofs] = self.friction_coulomb

        # Optional real-robot effects (config key sim_real_robot), as in the real walking test task (Isaac Lab
        # robots/asmr2026_real.py): encoder zero offset, gear backlash with the PD on the motor side, action, encoder
        # and IMU delays, IMU mounting tilt and gyro bias, and the armature, all drawn at every reset.
        self.real = self.cfg.get("sim_real_robot", None)
        if self.real is not None:
            # seeded from torch, so that torch.manual_seed in the evaluation scripts fixes the draws too
            self.rng = np.random.default_rng(int(torch.randint(0, 2**31 - 1, (1,)).item()))
            self.joint_dofs = np.array([self.mj_model.jnt_dofadr[self.mj_model.joint(name).id]
                                        for name in self.cfg.joints])
            self.is_ankle = np.array(["ankle" in name for name in self.cfg.joints])
            self.kp_np = self.joint_kp.numpy().astype(np.float64)
            self.kd_np = self.joint_kd.numpy().astype(np.float64)
            self.limit_np = self.effort_limits.numpy().astype(np.float64)
            self.max_impact_momentum = self.real.get("max_impact_momentum", 0.1)  # N m s (Isaac: 50 N m x 2 ms)

        # Optional Radxa torque clamp (config key torque_clamp_nm, tasks config/asmr_biped_real_clamp): once per
        # policy step the targets are clipped so that the PD torque at the readings the policy saw is within the
        # limit, with the nominal gains; the drives then run their PD with effort_limits (their current limit).
        self.torque_clamp = self.cfg.get("torque_clamp_nm", None)
        self.clamp_kp = self.joint_kp.clone()
        self.clamp_kd = self.joint_kd.clone()

        self.n_steps = 0

        print("Policy frequency: ", 1 / self.cfg.policy_dt)
        print("Physics frequency: ", 1 / self.cfg.physics_dt)
        print("Physics substeps: ", self.physics_substeps)
        print("Observation noise: ", self.obs_noise)

        # Initialize control mode and command variables
        self.is_killed = threading.Event()
        self.mode = 3.0  # Default to RL control mode
        self.command_velocity_x = 0.0
        self.command_velocity_y = 0.0
        self.command_velocity_yaw = 0.0

        # Start joystick thread
        self.command_controller = Se2Gamepad()
        self.command_controller.run()

    def reset(self) -> torch.Tensor:
        """Reset the simulation environment to initial state.

        Returns:
            torch.Tensor: Initial observations after reset
        """
        self.mj_data.qpos[0:3] = self.cfg.default_base_position
        self.mj_data.qpos[3:7] = torch.tensor([1.0, 0.0, 0.0, 0.0])  # Default quaternion orientation
        self.mj_data.qpos[7:] = self.cfg.default_joint_positions
        self.mj_data.qvel[:] = 0
        # update the sensors to the reset state
        mujoco.mj_forward(self.mj_model, self.mj_data)
        if self.real is not None:
            self._reset_real_robot()

        observations = self._get_observations()
        return observations

    def _steps_of(self, ms_range) -> int:
        low, high = (int(round(ms / 1000.0 / self.cfg.physics_dt)) for ms in ms_range)
        return int(self.rng.integers(low, high + 1))

    def _reset_real_robot(self):
        """Draw the real-robot effects and start the motor sides and the delay buffers at the reset state."""
        real, n = self.real, self.cfg.num_joints
        self.zero_offset = self.rng.uniform(-real.zero_offset, real.zero_offset, n)
        self.half_gap = 0.5 * self.rng.uniform(real.backlash[0], real.backlash[1], n)
        self.action_delay = self._steps_of(real.action_delay_ms)
        self.joint_obs_delay = self._steps_of(real.joint_obs_delay_ms)
        self.imu_obs_delay = self._steps_of(real.imu_obs_delay_ms)
        roll, pitch = self.rng.uniform(-real.imu_tilt, real.imu_tilt, 2)
        quat_roll = torch.tensor([np.cos(roll / 2), np.sin(roll / 2), 0.0, 0.0], dtype=torch.float32)
        quat_pitch = torch.tensor([np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0], dtype=torch.float32)
        self.imu_mount_quat = quat_mul(quat_roll, quat_pitch)
        self.gyro_bias = torch.tensor(self.rng.uniform(-real.gyro_bias, real.gyro_bias, 3), dtype=torch.float32)
        if "armature_6512" in real:
            armature = np.where(self.is_ankle, self.rng.uniform(*real.armature_5010, n),
                                self.rng.uniform(*real.armature_6512, n))
            self.mj_model.dof_armature[self.joint_dofs] = armature
        self.motor_inertia = self.mj_model.dof_armature[self.joint_dofs].copy()
        self.motor_pos = self._get_joint_pos().numpy().astype(np.float64)
        self.motor_vel = self._get_joint_vel().numpy().astype(np.float64)
        self.motor_torque = np.zeros(n)
        self.engaged = np.ones(n, dtype=bool)
        self.target = self.motor_pos.copy()
        self.mj_data.qfrc_applied[:] = 0.0
        self.command_history = deque([np.array(self.cfg.default_joint_positions, dtype=np.float64)],
                                     maxlen=self.action_delay + 1)
        self.reading_history = deque([self._real_robot_readings()],
                                     maxlen=max(self.joint_obs_delay, self.imu_obs_delay) + 1)

    def _real_robot_readings(self):
        """Sensor readings now: encoders (motor side, zero offset) and IMU (mounting tilt, gyro bias)."""
        encoder_pos = torch.tensor(self.motor_pos - self.zero_offset, dtype=torch.float32)
        encoder_vel = torch.tensor(self.motor_vel, dtype=torch.float32)
        imu_quat = quat_mul(self._get_base_quat(), self.imu_mount_quat)
        imu_ang_vel = quat_rotate_inverse(self.imu_mount_quat, self._get_base_ang_vel()) + self.gyro_bias
        return encoder_pos, encoder_vel, imu_quat, imu_ang_vel

    def _delayed_readings(self):
        history = self.reading_history
        joint = history[max(len(history) - 1 - self.joint_obs_delay, 0)]
        imu = history[max(len(history) - 1 - self.imu_obs_delay, 0)]
        return joint[0], joint[1], imu[2], imu[3]

    def step(self, actions: torch.Tensor) -> torch.Tensor:
        """Execute one simulation step with the given actions.

        Args:
            actions (torch.Tensor): Joint position targets for controlled joints

        Returns:
            torch.Tensor: Updated observations after executing the action
        """
        step_start_time = time.perf_counter()
        if self.torque_clamp is not None:
            actions = self._clamp_targets(actions)

        for _ in range(self.physics_substeps):
            self._apply_actions(actions)
            mujoco.mj_step(self.mj_model, self.mj_data)
            if self.real is not None:
                self._advance_real_robot()

        self.mj_viewer.sync()
        observations = self._get_observations()

        # Maintain real-time simulation
        time_until_next_step = self.cfg.policy_dt - (time.perf_counter() - step_start_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        self.n_steps += 1
        return observations

    def _clamp_targets(self, actions: torch.Tensor) -> torch.Tensor:
        """The Radxa torque clamp: q_sent = clip(q_target, q + (-L + Kd qd) / Kp, q + (L + Kd qd) / Kp)."""
        if self.real is not None:
            q, qd, _, _ = self._delayed_readings()
        else:
            q, qd = self._get_joint_pos(), self._get_joint_vel()
        limit = float(self.torque_clamp)
        low = q + (self.clamp_kd * qd - limit) / self.clamp_kp
        high = q + (self.clamp_kd * qd + limit) / self.clamp_kp
        return torch.minimum(torch.maximum(actions.to(torch.float32), low), high)

    def _apply_actions(self, actions: torch.Tensor):
        """Apply control actions to the robot.

        Implements PD control with torque limits and filtering.

        Args:
            actions (torch.Tensor): Target joint positions for controlled joints
        """
        target_positions = torch.zeros((self.cfg.num_joints,))
        target_positions[self.cfg.action_indices] = actions
        if self.real is not None:
            self._apply_real_robot_actions(target_positions.numpy().astype(np.float64))
            return

        # PD control
        output_torques = self.joint_kp * (target_positions - self._get_joint_pos()) + \
            self.joint_kd * (-self._get_joint_vel())

        # Apply EMA filtering and torque limits
        output_torques_clipped = torch.clip(output_torques, -self.effort_limits, self.effort_limits)

        self.mj_data.ctrl[:] = output_torques_clipped.numpy()

        if self.joint_friction is not None:
            self._update_friction(output_torques_clipped.numpy())

    def _update_friction(self, torque: np.ndarray):
        friction = self.joint_friction
        speed = self.mj_data.qvel[self.friction_dofs]
        torque = np.abs(torque)
        self.mj_model.dof_frictionloss[self.friction_dofs] = (
            self.friction_coulomb * (1.0 + (self.friction_stribeck_ratio - 1.0)
                                     * np.exp(-(speed / friction.stribeck_velocity) ** 2))
            + friction.load_linear * torque + friction.load_quadratic * torque**2)

    def _apply_real_robot_actions(self, command: np.ndarray):
        """PD on the motor side with the delayed, zero-offset target; the torque reaches the joint only when the
        motor pushes against a side of the backlash gap (as BacklashPDActuator in Isaac Lab)."""
        self.command_history.append(command)
        delayed = self.command_history[max(len(self.command_history) - 1 - self.action_delay, 0)]
        self.target = delayed + self.zero_offset
        torque = np.clip(self.kp_np * (self.target - self.motor_pos) - self.kd_np * self.motor_vel,
                         -self.limit_np, self.limit_np)
        gap = self.motor_pos - self._get_joint_pos().numpy()
        self.engaged = (((gap >= self.half_gap - 1e-6) & (torque > 0.0))
                        | ((gap <= -self.half_gap + 1e-6) & (torque < 0.0)))
        self.motor_torque = torque
        transmitted = np.where(self.engaged, torque, 0.0)
        self.mj_data.ctrl[:] = transmitted
        if self.joint_friction is not None:
            self._update_friction(transmitted)

    def _advance_real_robot(self):
        """Move the motor sides after a physics step and record the sensor readings."""
        dt = self.cfg.physics_dt
        q = self._get_joint_pos().numpy().astype(np.float64)
        qd = self._get_joint_vel().numpy().astype(np.float64)
        free = ~self.engaged
        torque = np.clip(self.kp_np * (self.target - self.motor_pos) - self.kd_np * self.motor_vel,
                         -self.limit_np, self.limit_np)
        vel = np.where(free, self.motor_vel + torque / self.motor_inertia * dt, qd)
        pos = np.where(free, self.motor_pos + vel * dt, q + np.sign(self.motor_torque) * self.half_gap)
        gap = pos - q
        hit = (gap > self.half_gap) | (gap < -self.half_gap)
        # plastic impact when a free motor reaches a side: the joint gets the momentum difference next step
        bound = self.max_impact_momentum / dt
        impulse = np.where(hit & free, np.clip(self.motor_inertia * (vel - qd) / dt, -bound, bound), 0.0)
        self.mj_data.qfrc_applied[:] = 0.0
        self.mj_data.qfrc_applied[self.joint_dofs] = impulse
        self.motor_pos = np.clip(pos, q - self.half_gap, q + self.half_gap)
        self.motor_vel = np.where(hit, qd, vel)
        self.reading_history.append(self._real_robot_readings())

    def _get_base_pos(self) -> torch.Tensor:
        """Get base position of the robot.

        Returns:
            torch.Tensor: Base position [x, y, z]
        """
        return torch.tensor(self.mj_data.qpos[:3], dtype=torch.float32)

    def _get_base_quat(self) -> torch.Tensor:
        """Get base orientation quaternion from sensors.

        Returns:
            torch.Tensor: Base orientation quaternion [w, x, y, z]
        """
        imu_quat = torch.tensor(self.mj_data.sensordata[self.sensordata_dof_size+0:self.sensordata_dof_size+4],
                                dtype=torch.float32)
        return quat_mul(imu_quat, self.base_quat_in_imu)

    def _get_base_ang_vel(self) -> torch.Tensor:
        """Get base angular velocity from sensors.

        Returns:
            torch.Tensor: Base angular velocity [wx, wy, wz]
        """
        imu_ang_vel = torch.tensor(self.mj_data.sensordata[self.sensordata_dof_size+4:self.sensordata_dof_size+7],
                                   dtype=torch.float32)
        return self.imu_rot_in_base @ imu_ang_vel

    def _get_projected_gravity(self) -> torch.Tensor:
        """Get gravity vector in the robot's base frame.

        Returns:
            torch.Tensor: Projected gravity vector
        """
        base_quat = self._get_base_quat()
        projected_gravity = quat_rotate_inverse(base_quat, self.gravity_vector)
        return projected_gravity

    def _get_joint_pos(self) -> torch.Tensor:
        """Get joint positions from sensors.

        Returns:
            torch.Tensor: Joint positions
        """
        return torch.tensor(self.mj_data.sensordata[0:self.cfg.num_joints], dtype=torch.float32)

    def _get_joint_vel(self) -> torch.Tensor:
        """Get joint velocities from sensors.

        Returns:
            torch.Tensor: Joint velocities
        """
        return torch.tensor(self.mj_data.sensordata[self.cfg.num_joints:2*self.cfg.num_joints],
                          dtype=torch.float32)

    def _get_observations(self) -> torch.Tensor:
        """Get complete observation vector for the policy.

        Returns:
            torch.Tensor: Concatenated observation vector containing base orientation,
                         angular velocity, joint positions, velocities, and command state
        """
        command_mode_switch = self.command_controller.commands["mode_switch"]
        command_velocity_x = self.command_controller.commands["velocity_x"]
        command_velocity_y = self.command_controller.commands["velocity_y"]
        command_velocity_yaw = self.command_controller.commands["velocity_yaw"]

        if command_mode_switch != 0:
            self.mode = command_mode_switch
        self.command_velocity_x = command_velocity_x
        self.command_velocity_y = command_velocity_y * 0.5
        self.command_velocity_yaw = command_velocity_yaw

        if self.real is not None:
            joint_pos, joint_vel, base_quat, base_ang_vel = self._delayed_readings()
            joint_pos, joint_vel = joint_pos[self.cfg.action_indices], joint_vel[self.cfg.action_indices]
        else:
            base_quat = self._get_base_quat()
            base_ang_vel = self._get_base_ang_vel()
            joint_pos = self._get_joint_pos()[self.cfg.action_indices]
            joint_vel = self._get_joint_vel()[self.cfg.action_indices]

        if self.obs_noise:
            # the policy sees gravity through the orientation, so tilt it by a small rotation about the
            # base x and y axes; this shifts the projected gravity x and y by up to OBS_NOISE_GRAVITY
            tilt = uniform_noise((2,), self.OBS_NOISE_GRAVITY)
            delta_quat = torch.tensor([1.0, tilt[0] / 2.0, tilt[1] / 2.0, 0.0])
            base_quat = quat_mul(base_quat, delta_quat / torch.linalg.norm(delta_quat))
            base_ang_vel = base_ang_vel + uniform_noise(base_ang_vel.shape, self.OBS_NOISE_ANG_VEL)
            joint_pos = joint_pos + uniform_noise(joint_pos.shape, self.obs_noise_joint_pos)
            joint_vel = joint_vel + uniform_noise(joint_vel.shape, self.OBS_NOISE_JOINT_VEL)

        return torch.cat([
            base_quat,
            base_ang_vel,
            joint_pos,
            joint_vel,
            torch.tensor([self.mode, self.command_velocity_x, self.command_velocity_y, self.command_velocity_yaw],
                        dtype=torch.float32),
        ], dim=-1)
