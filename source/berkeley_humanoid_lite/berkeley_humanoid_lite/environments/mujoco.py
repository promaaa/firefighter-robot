
import time
import threading

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

    The scene is the config's mjcf_path, or the Berkeley Humanoid Lite scene for its joint count. Every actuator
    becomes a torque motor (the course robot's MJCF has position servos), since the PD loop of MujocoSimulator
    computes the joint torques. An optional sim_payload entry (mass in kg, box center in the base frame, edge
    lengths in m) welds a solid box to the base, mass only, like the payload event of the training task.
    """
    mjcf_path = cfg.get("mjcf_path", None)
    if mjcf_path is None:
        scene = "bhl_scene.xml" if cfg.num_joints == 22 else "bhl_biped_scene.xml"
        mjcf_path = f"source/berkeley_humanoid_lite_assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite/mjcf/{scene}"
    spec = mujoco.MjSpec.from_file(mjcf_path)

    for actuator in spec.actuators:
        if actuator.biastype != mujoco.mjtBias.mjBIAS_NONE:
            # keeps the force range; the control range inherited from the joint range would clip the torque
            actuator.set_to_motor()
            actuator.inheritrange = 0.0
            actuator.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE

    payload = cfg.get("sim_payload", None)
    if payload is not None:
        a, b, c = payload.size
        spec.body("base").add_body(
            name="payload",
            explicitinertial=True,
            mass=payload.mass,
            ipos=list(payload.position),
            inertia=[payload.mass / 12.0 * (b**2 + c**2), payload.mass / 12.0 * (a**2 + c**2),
                     payload.mass / 12.0 * (a**2 + b**2)],
        )

    return spec.compile()


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

        observations = self._get_observations()
        return observations

    def step(self, actions: torch.Tensor) -> torch.Tensor:
        """Execute one simulation step with the given actions.

        Args:
            actions (torch.Tensor): Joint position targets for controlled joints

        Returns:
            torch.Tensor: Updated observations after executing the action
        """
        step_start_time = time.perf_counter()

        for _ in range(self.physics_substeps):
            self._apply_actions(actions)
            mujoco.mj_step(self.mj_model, self.mj_data)

        self.mj_viewer.sync()
        observations = self._get_observations()

        # Maintain real-time simulation
        time_until_next_step = self.cfg.policy_dt - (time.perf_counter() - step_start_time)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

        self.n_steps += 1
        return observations

    def _apply_actions(self, actions: torch.Tensor):
        """Apply control actions to the robot.

        Implements PD control with torque limits and filtering.

        Args:
            actions (torch.Tensor): Target joint positions for controlled joints
        """
        target_positions = torch.zeros((self.cfg.num_joints,))
        target_positions[self.cfg.action_indices] = actions

        # PD control
        output_torques = self.joint_kp * (target_positions - self._get_joint_pos()) + \
            self.joint_kd * (-self._get_joint_vel())

        # Apply EMA filtering and torque limits
        output_torques_clipped = torch.clip(output_torques, -self.effort_limits, self.effort_limits)

        self.mj_data.ctrl[:] = output_torques_clipped.numpy()

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
            joint_pos = joint_pos + uniform_noise(joint_pos.shape, self.OBS_NOISE_JOINT_POS)
            joint_vel = joint_vel + uniform_noise(joint_vel.shape, self.OBS_NOISE_JOINT_VEL)

        return torch.cat([
            base_quat,
            base_ang_vel,
            joint_pos,
            joint_vel,
            torch.tensor([self.mode, self.command_velocity_x, self.command_velocity_y, self.command_velocity_yaw],
                        dtype=torch.float32),
        ], dim=-1)
