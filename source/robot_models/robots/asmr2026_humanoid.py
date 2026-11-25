"""ASMR 2026 course humanoid (asmr2026-humanoid-v1): the 12-joint biped our hardware team builds.

Geometry and inertial properties are the course model's (data/robots/asmr2026/asmr2026_humanoid_v1, from the course
documentation), converted with scripts/convert_mjcf_to_urdf.py and scripts/convert_urdf_to_usd.py. Joint names are
the same as Berkeley Humanoid Lite's, but the joint signs differ: the left-leg pitch joints, both hip yaw and both
ankle roll axes are flipped, so the standing pose has mirrored signs.

Actuator parameters are the course's bench fit of one actuator of each type (mjlab/asmr2026_humanoid_constants.py):
PD gains from the armature at a 10 Hz natural frequency (damping ratio 2.0 for the 6512 joints, 0.7 for the 5010
ankles), the torque limit of the motor driver's 18 A current limit, and Coulomb + viscous joint friction.

The FW6 configs at the end use the course motor driver firmware gains (the real robot) and the friction measured
on our actuators; the real walking test model (asmr2026_real.py) builds on them.
"""

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuator, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.types import ArticulationActions

from .asmr2026_model import (
    LEG_JOINTS,
    MJCF_SCENE_PATH as _MJCF_SCENE_PATH,
    STANDING_JOINT_POSITIONS,
    STANDING_ROOT_HEIGHT,
    USD_PATH,
)

# Keep the former public names while the simulator-independent model spec is the source of truth.
ASMR2026_LEG_JOINTS = list(LEG_JOINTS)
STANDING_JOINT_POS = dict(zip(LEG_JOINTS, STANDING_JOINT_POSITIONS, strict=True))
MJCF_SCENE_PATH = str(_MJCF_SCENE_PATH)

##
# Actuators
##

GEAR_RATIO = 15.0
CURRENT_LIMIT_A = 18.0
VELOCITY_LIMIT = 50.0 * 2.0 * math.pi / GEAR_RATIO  # motor 50 turns/s, joint side (rad/s)
OMEGA_N = 2.0 * math.pi * 10.0


def pd_gains(armature: float, damping_ratio: float) -> tuple[float, float]:
    return armature * OMEGA_N**2, 2.0 * damping_ratio * armature * OMEGA_N


class FrictionImplicitActuator(ImplicitActuator):
    """Implicit PD actuator with Coulomb and viscous joint friction applied as a feed-forward torque.

    PhysX joint friction is a unitless coefficient on the force transmitted through the joint, not a friction torque,
    so the friction torque -f tanh(qd / v_s) - b qd is added to the joint each physics step instead. It acts on the
    joint besides the motor: the PD torque alone is clipped to the effort limit and reported as the applied torque.
    The smoothing velocity v_s keeps this explicit term stable: its slope f / v_s times the physics step stays
    below the joint inertia, which is at least the armature.

    The friction magnitude f is the Coulomb friction f_c, optionally raised at low speed (Stribeck) and by the
    torque through the gear (load-dependent friction of the course's 3D-printed gearboxes):
        f = f_c (1 + (r - 1) exp(-(qd / v_st)^2)) + s (a |tau| + c tau^2),
    with tau the clipped PD torque. With adaptive smoothing, v_s grows with f / f_c (never shrinks), which keeps the
    slope of the explicit term at most that of the nominal Coulomb friction.
    """

    cfg: "FrictionImplicitActuatorCfg"

    def __init__(self, cfg: "FrictionImplicitActuatorCfg", *args, **kwargs):
        super().__init__(cfg, *args, **kwargs)
        # per environment and joint, so that randomization events can change them
        shape = (self._num_envs, self.num_joints)
        self.coulomb_friction = torch.full(shape, cfg.coulomb_friction, device=self._device)
        self.viscous_friction = torch.full(shape, cfg.viscous_friction, device=self._device)
        self.friction_smoothing_velocity = torch.full(shape, cfg.friction_smoothing_velocity, device=self._device)
        self.stribeck_ratio = torch.full(shape, cfg.stribeck_ratio, device=self._device)
        self.load_friction_scale = torch.ones(shape, device=self._device)

    def compute(self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor):
        control_action = super().compute(control_action, joint_pos, joint_vel)
        magnitude = self.coulomb_friction * (
            1.0 + (self.stribeck_ratio - 1.0) * torch.exp(-((joint_vel / self.cfg.stribeck_velocity) ** 2)))
        if self.cfg.load_friction_linear != 0.0 or self.cfg.load_friction_quadratic != 0.0:
            torque = self.applied_effort.abs()
            magnitude = magnitude + self.load_friction_scale * (
                self.cfg.load_friction_linear * torque + self.cfg.load_friction_quadratic * torque**2)
        smoothing = self.friction_smoothing_velocity
        if self.cfg.adaptive_friction_smoothing:
            smoothing = smoothing * torch.clamp(magnitude / self.cfg.coulomb_friction, min=1.0)
        friction = magnitude * torch.tanh(joint_vel / smoothing) + self.viscous_friction * joint_vel
        # out of place: the efforts may be a view of the articulation's effort targets
        control_action.joint_efforts = control_action.joint_efforts - friction
        return control_action


@configclass
class FrictionImplicitActuatorCfg(ImplicitActuatorCfg):
    """Implicit PD actuator with Coulomb and viscous joint friction (see FrictionImplicitActuator)."""

    class_type: type = FrictionImplicitActuator

    coulomb_friction: float = 0.0
    """Coulomb friction torque (N m)."""
    viscous_friction: float = 0.0
    """Viscous friction coefficient (N m s / rad)."""
    friction_smoothing_velocity: float = 0.1
    """Joint speed (rad/s) over which the Coulomb friction rises to its full value (tanh)."""
    stribeck_ratio: float = 1.0
    """Friction at standstill relative to the Coulomb friction; the increase fades as exp(-(qd / v_st)^2)."""
    stribeck_velocity: float = 0.1
    """Joint speed v_st (rad/s) of the low-speed friction increase."""
    load_friction_linear: float = 0.0
    """Friction increase per N m of torque through the gear (the clipped PD torque)."""
    load_friction_quadratic: float = 0.0
    """Friction increase per (N m)^2 of torque through the gear."""
    adaptive_friction_smoothing: bool = False
    """Scale the smoothing velocity with the friction magnitude relative to the Coulomb friction (see the class)."""


# 6512 actuator (hip roll, hip yaw, hip pitch, knee): kt 0.067 N m/A
ARMATURE_6512 = 0.0228
KP_6512, KD_6512 = pd_gains(ARMATURE_6512, 2.0)
ACTUATOR_6512_CFG = FrictionImplicitActuatorCfg(
    joint_names_expr=["leg_.*_hip_roll_joint", "leg_.*_hip_yaw_joint", "leg_.*_hip_pitch_joint",
                      "leg_.*_knee_pitch_joint"],
    effort_limit_sim=CURRENT_LIMIT_A * 0.067 * GEAR_RATIO,
    velocity_limit_sim=VELOCITY_LIMIT,
    stiffness=KP_6512,
    damping=KD_6512,
    armature=ARMATURE_6512,
    friction=0.0,
    coulomb_friction=0.377,
    viscous_friction=0.090,
    # 0.377 N m x 0.005 s / 0.0228 kg m^2 = 0.083 rad/s
    friction_smoothing_velocity=0.1,
)

# 5010 actuator (ankle pitch, ankle roll): kt 0.0836 N m/A
ARMATURE_5010 = 0.00556
KP_5010, KD_5010 = pd_gains(ARMATURE_5010, 0.7)
ACTUATOR_5010_CFG = FrictionImplicitActuatorCfg(
    joint_names_expr=["leg_.*_ankle_pitch_joint", "leg_.*_ankle_roll_joint"],
    effort_limit_sim=CURRENT_LIMIT_A * 0.0836 * GEAR_RATIO,
    velocity_limit_sim=VELOCITY_LIMIT,
    stiffness=KP_5010,
    damping=KD_5010,
    armature=ARMATURE_5010,
    friction=0.0,
    coulomb_friction=0.317,
    viscous_friction=0.030,
    # 0.317 N m x 0.005 s / 0.00556 kg m^2 = 0.285 rad/s
    friction_smoothing_velocity=0.3,
)

ASMR2026_HUMANOID_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(USD_PATH),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=8, solver_velocity_iteration_count=4
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, STANDING_ROOT_HEIGHT),
        joint_pos=STANDING_JOINT_POS,
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={"legs": ACTUATOR_6512_CFG, "ankles": ACTUATOR_5010_CFG},
)
"""Configuration for the ASMR 2026 course humanoid."""

##
# Course motor driver firmware settings
##

# Gains of the course XDrive MINI configuration (xdrive_config.txt in omletkang/xdrive-mini-fw-instant-start).
# ODrive position mode without the velocity integrator is a joint PD with Kp = pos_gain * vel_gain * N^2 / 2 pi and
# Kd = vel_gain * N^2 / 2 pi; the configuration sets torque_constant to the bench kt, so the driver's N m are real.
FW_POS_GAIN_6512, FW_VEL_GAIN_6512 = 12.6, 0.223  # (turn/s) / turn, N m / (turn/s)
FW_POS_GAIN_5010, FW_VEL_GAIN_5010 = 20.0, 0.1
# joint-side torque limit of every joint (N m); on the driver: motor.config.torque_lim = FW_TORQUE_LIMIT / GEAR_RATIO
FW_TORQUE_LIMIT = 6.0


def odrive_pd_gains(pos_gain: float, vel_gain: float) -> tuple[float, float]:
    kd = vel_gain * GEAR_RATIO**2 / (2.0 * math.pi)
    return pos_gain * kd, kd


KP_6512_FW, KD_6512_FW = odrive_pd_gains(FW_POS_GAIN_6512, FW_VEL_GAIN_6512)  # 100.6, 7.99
KP_5010_FW, KD_5010_FW = odrive_pd_gains(FW_POS_GAIN_5010, FW_VEL_GAIN_5010)  # 71.6, 3.58

ASMR2026_HUMANOID_FW6_CFG = ASMR2026_HUMANOID_CFG.replace(
    actuators={
        "legs": ACTUATOR_6512_CFG.replace(stiffness=KP_6512_FW, damping=KD_6512_FW, effort_limit_sim=FW_TORQUE_LIMIT),
        "ankles": ACTUATOR_5010_CFG.replace(stiffness=KP_5010_FW, damping=KD_5010_FW, effort_limit_sim=FW_TORQUE_LIMIT),
    },
)
"""The course humanoid with the course driver firmware gains and a 6 N m torque limit on every joint."""

# Friction measured on two hip yaw 6512 actuators after about 80 min of break-in (team bench test, 2026-11-05/06;
# Notion "액추에이터 측정 결과"): Coulomb 0.35-0.36 N m from 0.37 to 5.8 rad/s, no viscous friction, about 20 % more
# at 0.10 rad/s (a Stribeck ratio of 1.54 at v_st 0.1 rad/s gives +20 % at 0.10 and +6 % at 0.15 rad/s), and an
# increase with load of 0.0125 |tau| + 0.016 tau^2 (+0.13 N m at 2.44 N m, +0.69 at 6.18 N m). The 5010 is not
# measured yet: the course fit's Coulomb friction with the same structure.
MEASURED_FRICTION = dict(
    viscous_friction=0.0,
    stribeck_ratio=1.54,
    stribeck_velocity=0.1,
    load_friction_linear=0.0125,
    load_friction_quadratic=0.016,
    adaptive_friction_smoothing=True,
)
ASMR2026_HUMANOID_FW6M_CFG = ASMR2026_HUMANOID_FW6_CFG.replace(
    actuators={
        "legs": ASMR2026_HUMANOID_FW6_CFG.actuators["legs"].replace(coulomb_friction=0.36, **MEASURED_FRICTION),
        "ankles": ASMR2026_HUMANOID_FW6_CFG.actuators["ankles"].replace(coulomb_friction=0.317, **MEASURED_FRICTION),
    },
)
"""FW6 with the measured friction model (Coulomb, low-speed increase and load-dependent friction, no viscous)."""
