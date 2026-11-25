"""The course humanoid as measured on the real robot, for the policy of the first real walking test.

Joint measurements of 2026-11-20 (Radxa ROCK 3C -> slcan CANable -> CAN 500 kbps -> XDrive MINI, left knee, hip
pitch and ankle pitch; the team's report "ASMR 관절 실측 모델"): the joints follow a torque PD with the course driver
gains (step time constants = Kd / Kp), a command reaches the driver in about 0.5 ms and the current about 2 ms
later, joint readings are 1.8 ms old (median; p95 3.4 ms) when all 12 are in, and the loop runs at up to 150 Hz.

On top of the FW6M model (course driver gains, measured friction; asmr2026_humanoid.ASMR2026_HUMANOID_FW6M_CFG)
this model adds what the implicit PhysX PD cannot represent, so the PD runs here at every physics step (explicit):
- a 6 N m torque limit on every joint, as on the drivers (motor.config.torque_lim = 0.4 N m on the motor side);
- gear backlash: the encoder is on the motor side, so the PD acts on the motor-side position, which can be up to
  half the backlash away from the joint; while the motor crosses the gap the joint gets no motor torque, and when
  it engages again the two sides exchange momentum (an impulse on the joint);
- a joint zero offset: the encoder zero is off by a constant per joint, which shifts the observed positions and
  the position targets together, since the driver uses the same encoder;
- the delay from a command to the joint torque (action delay), in physics steps.
The observation delays of the encoders and of the IMU, the IMU mounting tilt and gyro bias, and the measured link
masses are in RealRobotArticulation, which records the readings at every physics step.

Simplifications: the motor inertia (the armature) stays on the joint while the motor crosses the gap (the joint is
slightly too heavy then), the motor side has no friction of its own, and the gear friction acts on the joint with
the load term on the torque through the gear.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.utils.math as math_utils
from isaaclab.actuators import IdealPDActuator, IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions

from .asmr2026_humanoid import (
    ARMATURE_5010,
    ARMATURE_6512,
    ASMR2026_HUMANOID_CFG,
    FW_TORQUE_LIMIT,
    KD_5010_FW,
    KD_6512_FW,
    KP_5010_FW,
    KP_6512_FW,
    MEASURED_FRICTION,
    VELOCITY_LIMIT,
)

PHYSICS_DT = 0.002
"""Physics step (s) for this model: explicit PD and backlash need small steps (report: 2 ms or less)."""

##
# Standing pose
##

# The BHL-like standing pose (user choice 2026-11-20): knee 0.4 rad as in Berkeley Humanoid Lite's init pose, with
# the course pose's split between hip and ankle pitch (feet flat, torso upright: hip + ankle = knee). It is the
# training default pose and the rigid-stand pose on the robot before the policy starts. Holding it with the driver
# PD alone takes 2.8 N m at the knee in MuJoCo (the course pose, knee 54.6 deg, takes 5.5 N m of the 6).
_KNEE = 0.4
_HIP = _KNEE * 0.58519126 / 0.95241361
_ANKLE = _KNEE - _HIP
REAL_STANDING_JOINT_POS = {
    "leg_left_hip_roll_joint": 0.0,
    "leg_left_hip_yaw_joint": 0.0,
    "leg_left_hip_pitch_joint": _HIP,
    "leg_left_knee_pitch_joint": -_KNEE,
    "leg_left_ankle_pitch_joint": _ANKLE,
    "leg_left_ankle_roll_joint": 0.0,
    "leg_right_hip_roll_joint": 0.0,
    "leg_right_hip_yaw_joint": 0.0,
    "leg_right_hip_pitch_joint": -_HIP,
    "leg_right_knee_pitch_joint": _KNEE,
    "leg_right_ankle_pitch_joint": -_ANKLE,
    "leg_right_ankle_roll_joint": 0.0,
}
# base origin height with the feet flat on the floor (course MJCF forward kinematics; the course pose gives -0.03978)
REAL_STANDING_ROOT_HEIGHT = -0.01056

##
# Actuator
##


class BacklashPDActuator(IdealPDActuator):
    """Explicit joint PD on the motor-side position with backlash, encoder zero offset, action delay and the
    measured friction model (see the module docstring).

    State per environment and joint: the motor-side position and velocity (joint units, before the zero offset).
    compute() only reads the state, since Isaac Lab also calls it after a reset without stepping the physics;
    advance() moves the state after each physics step (RealRobotArticulation calls it from update()).
    """

    cfg: BacklashPDActuatorCfg

    def __init__(self, cfg: BacklashPDActuatorCfg, *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        shape = (self._num_envs, self.num_joints)
        device = self._device
        # friction model, per environment and joint so that the randomization events can change it
        self.coulomb_friction = torch.full(shape, cfg.coulomb_friction, device=device)
        self.viscous_friction = torch.full(shape, cfg.viscous_friction, device=device)
        self.friction_smoothing_velocity = torch.full(shape, cfg.friction_smoothing_velocity, device=device)
        self.stribeck_ratio = torch.full(shape, cfg.stribeck_ratio, device=device)
        self.load_friction_scale = torch.ones(shape, device=device)
        # drawn at every reset
        self.zero_offset = torch.zeros(shape, device=device)
        self.half_gap = torch.zeros(shape, device=device)
        self.action_delay = torch.zeros(self._num_envs, dtype=torch.long, device=device)
        # motor side
        self.motor_pos = torch.zeros(shape, device=device)
        self.motor_vel = torch.zeros(shape, device=device)
        self._needs_init = torch.ones(self._num_envs, dtype=torch.bool, device=device)
        # commands of the last physics steps, for the action delay (slot _cmd_ptr holds the newest)
        self._cmd_history = torch.zeros((max(cfg.action_delay_steps[1], 1), *shape), device=device)
        self._cmd_ptr = 0
        self._cmd_fresh = torch.ones(self._num_envs, dtype=torch.bool, device=device)
        self._latest_cmd = torch.zeros(shape, device=device)
        # set by compute(), used by advance()
        self._target = torch.zeros(shape, device=device)
        self._engaged = torch.ones(shape, dtype=torch.bool, device=device)
        self._impulse = torch.zeros(shape, device=device)
        self._env_index = torch.arange(self._num_envs, device=device)

    @property
    def encoder_pos(self) -> torch.Tensor:
        """Joint position the encoder reports: the motor side, off by the zero offset."""
        return self.motor_pos - self.zero_offset

    @property
    def encoder_vel(self) -> torch.Tensor:
        return self.motor_vel

    def reset(self, env_ids: Sequence[int] | None):
        if env_ids is None or isinstance(env_ids, slice):
            env_ids = self._env_index
        env_ids = torch.as_tensor(env_ids, device=self._device)
        shape = (len(env_ids), self.num_joints)
        half = self.cfg.zero_offset_range
        self.zero_offset[env_ids] = (torch.rand(shape, device=self._device) * 2.0 - 1.0) * half
        low, high = self.cfg.backlash_range
        self.half_gap[env_ids] = 0.5 * (low + (high - low) * torch.rand(shape, device=self._device))
        low, high = self.cfg.action_delay_steps
        self.action_delay[env_ids] = torch.randint(low, high + 1, (len(env_ids),), device=self._device)
        self._needs_init[env_ids] = True
        self._cmd_fresh[env_ids] = True
        self._impulse[env_ids] = 0.0
        self._engaged[env_ids] = True

    def _delayed_command(self, cmd: torch.Tensor) -> torch.Tensor:
        if self.cfg.action_delay_steps[1] == 0:
            return cmd
        length = self._cmd_history.shape[0]
        slot = (self._cmd_ptr - (self.action_delay - 1).clamp(min=0)) % length
        past = self._cmd_history[slot, self._env_index]
        use_now = (self.action_delay == 0) | self._cmd_fresh
        return torch.where(use_now[:, None], cmd, past)

    def _friction(self, joint_vel: torch.Tensor, torque: torch.Tensor) -> torch.Tensor:
        magnitude = self.coulomb_friction * (
            1.0 + (self.stribeck_ratio - 1.0) * torch.exp(-((joint_vel / self.cfg.stribeck_velocity) ** 2)))
        if self.cfg.load_friction_linear != 0.0 or self.cfg.load_friction_quadratic != 0.0:
            torque = torque.abs()
            magnitude = magnitude + self.load_friction_scale * (
                self.cfg.load_friction_linear * torque + self.cfg.load_friction_quadratic * torque**2)
        smoothing = self.friction_smoothing_velocity
        if self.cfg.adaptive_friction_smoothing:
            smoothing = smoothing * torch.clamp(magnitude / self.cfg.coulomb_friction, min=1.0)
        return magnitude * torch.tanh(joint_vel / smoothing) + self.viscous_friction * joint_vel

    def compute(self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor):
        self._latest_cmd[:] = control_action.joint_positions
        init = self._needs_init
        if torch.any(init):
            self.motor_pos[init] = joint_pos[init]
            self.motor_vel[init] = joint_vel[init]
        # the driver tracks the encoder target, i.e. the physical target shifted by the zero offset
        self._target = self._delayed_command(self._latest_cmd) + self.zero_offset
        self.computed_effort = self.stiffness * (self._target - self.motor_pos) - self.damping * self.motor_vel
        self.applied_effort = self._clip_effort(self.computed_effort)
        # the motor torque reaches the joint only at the side of the gap it pushes against
        gap = self.motor_pos - joint_pos
        tolerance = 1e-6
        push_plus = (gap >= self.half_gap - tolerance) & (self.applied_effort > 0.0)
        push_minus = (gap <= -self.half_gap + tolerance) & (self.applied_effort < 0.0)
        self._engaged = push_plus | push_minus
        transmitted = torch.where(self._engaged, self.applied_effort, torch.zeros_like(self.applied_effort))
        control_action.joint_efforts = transmitted - self._friction(joint_vel, transmitted) + self._impulse
        control_action.joint_positions = None
        control_action.joint_velocities = None
        return control_action

    def advance(self, joint_pos: torch.Tensor, joint_vel: torch.Tensor, dt: float):
        """Move the command history and the motor side after a physics step (joint state after the step)."""
        # command history
        if self.cfg.action_delay_steps[1] > 0:
            self._cmd_ptr = (self._cmd_ptr + 1) % self._cmd_history.shape[0]
            self._cmd_history[self._cmd_ptr] = self._latest_cmd
            fresh = self._cmd_fresh
            if torch.any(fresh):
                self._cmd_history[:, fresh] = self._latest_cmd[fresh]
        self._cmd_fresh[:] = False
        init = self._needs_init
        # a free motor crosses the gap under its own PD torque, with its inertia (the armature)
        free = ~self._engaged
        pos, vel = self.motor_pos, self.motor_vel
        if torch.any(free):
            inertia = self.armature.clamp(min=1e-4)
            h = dt / self.cfg.motor_substeps
            pos_f, vel_f = pos.clone(), vel.clone()
            for _ in range(self.cfg.motor_substeps):
                torque = self._clip_effort(self.stiffness * (self._target - pos_f) - self.damping * vel_f)
                vel_f = vel_f + torque / inertia * h
                pos_f = pos_f + vel_f * h
            pos = torch.where(free, pos_f, pos)
            vel = torch.where(free, vel_f, vel)
        # an engaged motor moves with the joint at the side it pushes
        side = torch.sign(self.applied_effort) * self.half_gap
        pos = torch.where(self._engaged, joint_pos + side, pos)
        vel = torch.where(self._engaged, joint_vel, vel)
        # a free motor that reaches a side of the gap (or that the joint runs into) engages: plastic impact, the
        # joint gets the momentum difference as an impulse at the next physics step
        gap = pos - joint_pos
        hit = (gap > self.half_gap) | (gap < -self.half_gap)
        impulse = self.armature * (vel - joint_vel) / dt
        limit = self.cfg.impact_torque_limit
        self._impulse = torch.where(hit & free, impulse.clamp(-limit, limit), torch.zeros_like(impulse))
        pos = torch.max(torch.min(pos, joint_pos + self.half_gap), joint_pos - self.half_gap)
        vel = torch.where(hit, joint_vel, vel)
        # envs reset since the last step start at the joint state
        self.motor_pos = torch.where(init[:, None], joint_pos, pos)
        self.motor_vel = torch.where(init[:, None], joint_vel, vel)
        self._impulse[init] = 0.0
        self._needs_init[:] = False


@configclass
class BacklashPDActuatorCfg(IdealPDActuatorCfg):
    """Explicit PD actuator with backlash, zero offset, action delay and joint friction (BacklashPDActuator)."""

    class_type: type = BacklashPDActuator

    backlash_range: tuple[float, float] = (0.0, 0.0)
    """Total backlash (rad, joint side), drawn per joint at every reset; the motor side moves within +- half."""
    zero_offset_range: float = 0.0
    """Half width (rad) of the encoder zero offset, drawn uniformly per joint at every reset."""
    action_delay_steps: tuple[int, int] = (0, 0)
    """Delay from a command to the PD (physics steps), drawn per environment at every reset."""
    motor_substeps: int = 4
    """Integration substeps of a free motor per physics step."""
    impact_torque_limit: float = 50.0
    """Bound (N m) of the impulse torque when the gear engages, for a stable simulation."""

    coulomb_friction: float = 0.0
    viscous_friction: float = 0.0
    friction_smoothing_velocity: float = 0.1
    stribeck_ratio: float = 1.0
    stribeck_velocity: float = 0.1
    load_friction_linear: float = 0.0
    load_friction_quadratic: float = 0.0
    adaptive_friction_smoothing: bool = False
    """Friction model as in asmr2026_humanoid.FrictionImplicitActuatorCfg."""


##
# Articulation with delayed sensor readings
##


class RealRobotArticulation(Articulation):
    """Articulation that records the encoder (motor side, zero offset) and IMU readings at every physics step and
    serves them with per-environment delays drawn at every reset.

    The IMU readings carry the errors of a real board, drawn at every reset: a mounting tilt (roll and pitch of the
    IMU frame against the base frame; the readings are the base vectors expressed in the tilted frame) and a gyro
    bias.
    """

    cfg: RealRobotArticulationCfg

    def _initialize_impl(self):
        super()._initialize_impl()
        if self.cfg.body_masses:
            self._set_body_masses(self.cfg.body_masses)
        num_envs, num_joints = self.num_instances, self.num_joints
        self._obs_length = max(self.cfg.joint_obs_delay_steps[1], self.cfg.imu_obs_delay_steps[1]) + 1
        shape = (self._obs_length, num_envs)
        self._enc_pos_history = torch.zeros((*shape, num_joints), device=self.device)
        self._enc_vel_history = torch.zeros((*shape, num_joints), device=self.device)
        self._ang_vel_history = torch.zeros((*shape, 3), device=self.device)
        self._gravity_history = torch.zeros((*shape, 3), device=self.device)
        self._obs_ptr = 0
        self._joint_obs_delay = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self._imu_obs_delay = torch.zeros(num_envs, dtype=torch.long, device=self.device)
        self._obs_fill = torch.ones(num_envs, dtype=torch.bool, device=self.device)
        self._env_index = torch.arange(num_envs, device=self.device)
        self._imu_mount_quat = torch.zeros((num_envs, 4), device=self.device)
        self._imu_mount_quat[:, 0] = 1.0
        self._gyro_bias = torch.zeros((num_envs, 3), device=self.device)

    def _set_body_masses(self, body_masses: dict[str, float]):
        """Set body masses, scaling each body's inertia with its mass (same mass distribution), as the nominal
        values that the mass randomization events start from."""
        masses = self.root_physx_view.get_masses()
        inertias = self.root_physx_view.get_inertias()
        for name, mass in body_masses.items():
            body_ids, _ = self.find_bodies(name)
            ratio = mass / masses[:, body_ids]
            inertias[:, body_ids] *= ratio[..., None]
            masses[:, body_ids] = mass
        indices = torch.arange(self.num_instances)
        self.root_physx_view.set_masses(masses, indices)
        self.root_physx_view.set_inertias(inertias, indices)
        self._data.default_mass = masses.clone()
        self._data.default_inertia = inertias.clone()

    def reset(self, env_ids: Sequence[int] | None = None):
        super().reset(env_ids)
        if env_ids is None or isinstance(env_ids, slice):
            env_ids = self._env_index
        env_ids = torch.as_tensor(env_ids, device=self.device)
        low, high = self.cfg.joint_obs_delay_steps
        self._joint_obs_delay[env_ids] = torch.randint(low, high + 1, (len(env_ids),), device=self.device)
        low, high = self.cfg.imu_obs_delay_steps
        self._imu_obs_delay[env_ids] = torch.randint(low, high + 1, (len(env_ids),), device=self.device)
        tilt = self.cfg.imu_tilt_range
        roll, pitch = ((torch.rand((2, len(env_ids)), device=self.device) * 2.0 - 1.0) * tilt).unbind(0)
        self._imu_mount_quat[env_ids] = math_utils.quat_from_euler_xyz(roll, pitch, torch.zeros_like(roll))
        bias = self.cfg.gyro_bias_range
        self._gyro_bias[env_ids] = (torch.rand((len(env_ids), 3), device=self.device) * 2.0 - 1.0) * bias
        self._obs_fill[env_ids] = True

    def _imu_readings(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Angular velocity and gravity direction in the (tilted) IMU frame, with the gyro bias."""
        ang_vel = math_utils.quat_rotate_inverse(self._imu_mount_quat, self._data.root_ang_vel_b) + self._gyro_bias
        gravity = math_utils.quat_rotate_inverse(self._imu_mount_quat, self._data.projected_gravity_b)
        return ang_vel, gravity

    def _encoder_readings(self) -> tuple[torch.Tensor, torch.Tensor]:
        pos, vel = self._data.joint_pos.clone(), self._data.joint_vel.clone()
        for actuator in self.actuators.values():
            if isinstance(actuator, BacklashPDActuator):
                pos[:, actuator.joint_indices] = actuator.encoder_pos
                vel[:, actuator.joint_indices] = actuator.encoder_vel
        return pos, vel

    def update(self, dt: float):
        super().update(dt)
        if not hasattr(self, "_obs_ptr"):
            return  # the parent's initialization updates once before the buffers exist
        joint_pos, joint_vel = self._data.joint_pos, self._data.joint_vel
        for actuator in self.actuators.values():
            if isinstance(actuator, BacklashPDActuator):
                actuator.advance(joint_pos[:, actuator.joint_indices], joint_vel[:, actuator.joint_indices], dt)
        pos, vel = self._encoder_readings()
        self._obs_ptr = (self._obs_ptr + 1) % self._obs_length
        self._enc_pos_history[self._obs_ptr] = pos
        self._enc_vel_history[self._obs_ptr] = vel
        self._ang_vel_history[self._obs_ptr], self._gravity_history[self._obs_ptr] = self._imu_readings()

    def _fill_obs_history(self):
        """Fill the history of environments reset since the last read with their current (reset) readings."""
        fill = self._obs_fill
        if not torch.any(fill):
            return
        pos, vel = self._data.joint_pos.clone(), self._data.joint_vel.clone()
        for actuator in self.actuators.values():
            if isinstance(actuator, BacklashPDActuator):
                # the motor side starts at the joint state after a reset
                pos[:, actuator.joint_indices] = pos[:, actuator.joint_indices] - actuator.zero_offset
        ang_vel, gravity = self._imu_readings()
        self._enc_pos_history[:, fill] = pos[fill]
        self._enc_vel_history[:, fill] = vel[fill]
        self._ang_vel_history[:, fill] = ang_vel[fill]
        self._gravity_history[:, fill] = gravity[fill]
        self._obs_fill[:] = False

    def delayed_joint_readings(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Encoder joint positions and velocities as the policy sees them (delayed), all joints."""
        self._fill_obs_history()
        slot = (self._obs_ptr - self._joint_obs_delay) % self._obs_length
        return self._enc_pos_history[slot, self._env_index], self._enc_vel_history[slot, self._env_index]

    def delayed_imu_readings(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Base angular velocity (body frame) and projected gravity as the policy sees them (delayed)."""
        self._fill_obs_history()
        slot = (self._obs_ptr - self._imu_obs_delay) % self._obs_length
        return self._ang_vel_history[slot, self._env_index], self._gravity_history[slot, self._env_index]


@configclass
class RealRobotArticulationCfg(ArticulationCfg):
    """Articulation with delayed encoder and IMU readings (RealRobotArticulation)."""

    class_type: type = RealRobotArticulation

    joint_obs_delay_steps: tuple[int, int] = (0, 0)
    """Delay of the encoder readings (physics steps), drawn per environment at every reset."""
    imu_obs_delay_steps: tuple[int, int] = (0, 0)
    """Delay of the IMU readings (physics steps), drawn per environment at every reset."""
    imu_tilt_range: float = 0.0
    """Half width (rad) of the IMU mounting roll and pitch, drawn uniformly at every reset."""
    gyro_bias_range: float = 0.0
    """Half width (rad/s) of the gyro bias per axis, drawn uniformly at every reset."""
    body_masses: dict[str, float] = {}
    """Body masses (kg) replacing the USD's, with the inertia scaled along; empty keeps the USD masses."""


##
# Robot
##

DEG = math.pi / 180.0

ZERO_OFFSET = 2.0 * DEG
"""Encoder zero offset half width (user request: +- 2 deg; zeroing by hand is not exact)."""
BACKLASH_RANGE = (0.0, 1.0 * DEG)
"""Total gear backlash (user choice: 0-1 deg, not measured yet)."""
ACTION_DELAY_STEPS = (1, 3)
"""2-6 ms at 2 ms steps: command to driver 0.5 ms, current loop 1-2 ms, policy inference (not measured yet)."""
JOINT_OBS_DELAY_STEPS = (0, 3)
"""0-6 ms on top of the step the motor side lags: readings are 1.8 ms old (median; p95 3.4, max 5.5 ms)."""
IMU_OBS_DELAY_STEPS = (0, 3)
"""0-6 ms: the course BOM's IMU is a Grove 6-axis board with a Bosch BMI088 (accelerometer + gyroscope over I2C, no
orientation output; devicemart no. 12169929). Its digital low-pass filters delay the samples by about 1-3 ms at the
usual 100-200 Hz bandwidths, and an I2C read takes under 1 ms. The orientation comes from a fusion filter on the
Radxa (gyro-propagated, so it adds little lag); its tilt error is in the projected gravity noise."""

IMU_TILT = 2.0 * DEG
"""IMU mounting tilt half width (user choice 11-23): the board on its mount is not exactly level with the base."""
GYRO_BIAS = 0.02
"""Gyro bias half width (rad/s, about 1 deg/s) left after a startup bias calibration (user choice 11-23)."""

# Link masses weighed on the real robot by the HW team ("Measured Mass Table", Google Doc from the 2026-10-27 sheet,
# filled by 2026-11-23), kg. The leg links are 0.04-0.17 kg heavier than the course model (motor mass missing from
# the model, as suspected), the foot 0.03 kg lighter: 3.80 kg per leg against 3.30. The torso with everything mounted
# weighs 5.411 kg including the IMU (the sheet counts 0.35 kg of it as an estimate for the power relay and switch);
# the IMU stays its own body here at the model's 46 g. The CoM and inertia shape of each link stay the model's.
MEASURED_BODY_MASSES = {
    "base": 5.41085 - 0.046,
    "leg_left_hip_roll": 0.77574,
    "leg_right_hip_roll": 0.77536,
    "leg_left_hip_yaw": 0.76690,
    "leg_right_hip_yaw": 0.76847,
    "leg_left_hip_pitch": 0.90315,
    "leg_right_hip_pitch": 0.89998,
    "leg_left_knee_pitch": 0.62051,
    "leg_right_knee_pitch": 0.62056,
    "leg_left_ankle_pitch": 0.16019,
    "leg_right_ankle_pitch": 0.16015,
    "leg_left_ankle_roll": 0.57774,
    "leg_right_ankle_roll": 0.57667,
}

_BACKLASH_COMMON = dict(
    backlash_range=BACKLASH_RANGE,
    zero_offset_range=ZERO_OFFSET,
    action_delay_steps=ACTION_DELAY_STEPS,
    effort_limit=FW_TORQUE_LIMIT,
    velocity_limit=VELOCITY_LIMIT,
    friction=0.0,
    **MEASURED_FRICTION,
)
ACTUATOR_6512_REAL_CFG = BacklashPDActuatorCfg(
    joint_names_expr=["leg_.*_hip_roll_joint", "leg_.*_hip_yaw_joint", "leg_.*_hip_pitch_joint",
                      "leg_.*_knee_pitch_joint"],
    stiffness=KP_6512_FW,
    damping=KD_6512_FW,
    armature=ARMATURE_6512,
    coulomb_friction=0.36,
    friction_smoothing_velocity=0.1,
    **_BACKLASH_COMMON,
)
ACTUATOR_5010_REAL_CFG = BacklashPDActuatorCfg(
    joint_names_expr=["leg_.*_ankle_pitch_joint", "leg_.*_ankle_roll_joint"],
    stiffness=KP_5010_FW,
    damping=KD_5010_FW,
    armature=ARMATURE_5010,
    coulomb_friction=0.317,
    friction_smoothing_velocity=0.3,
    **_BACKLASH_COMMON,
)

ASMR2026_HUMANOID_REAL_CFG = RealRobotArticulationCfg(
    spawn=ASMR2026_HUMANOID_CFG.spawn,
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, REAL_STANDING_ROOT_HEIGHT),
        joint_pos=REAL_STANDING_JOINT_POS,
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=ASMR2026_HUMANOID_CFG.soft_joint_pos_limit_factor,
    actuators={"legs": ACTUATOR_6512_REAL_CFG, "ankles": ACTUATOR_5010_REAL_CFG},
    joint_obs_delay_steps=JOINT_OBS_DELAY_STEPS,
    imu_obs_delay_steps=IMU_OBS_DELAY_STEPS,
    imu_tilt_range=IMU_TILT,
    gyro_bias_range=GYRO_BIAS,
    body_masses=MEASURED_BODY_MASSES,
)
"""The course humanoid with the real robot's link masses, actuators, sensor delays and the BHL-like standing pose."""
