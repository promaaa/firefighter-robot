"""ASMR 2026 course humanoid (asmr2026-humanoid-v1): the 12-joint biped our hardware team builds.

Geometry and inertial properties are the course model's (data/robots/asmr2026/asmr2026_humanoid_v1, from the course
documentation), converted with scripts/convert_mjcf_to_urdf.py and scripts/convert_urdf_to_usd.py. Joint names are
the same as Berkeley Humanoid Lite's, but the joint signs differ: the left-leg pitch joints, both hip yaw and both
ankle roll axes are flipped, so the standing pose has mirrored signs.

Actuator parameters are the course's bench fit of one actuator of each type (mjlab/asmr2026_humanoid_constants.py):
PD gains from the armature at a 10 Hz natural frequency (damping ratio 2.0 for the 6512 joints, 0.7 for the 5010
ankles), the torque limit of the motor driver's 18 A current limit, and Coulomb + viscous joint friction.
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
    so the friction torque -f_c tanh(qd / v_s) - b qd is added to the joint each physics step instead. It acts on the
    joint besides the motor: the PD torque alone is clipped to the effort limit and reported as the applied torque.
    The smoothing velocity v_s keeps this explicit term stable: its slope f_c / v_s times the physics step stays
    below the joint inertia, which is at least the armature.
    """

    cfg: "FrictionImplicitActuatorCfg"

    def compute(self, control_action: ArticulationActions, joint_pos: torch.Tensor, joint_vel: torch.Tensor):
        control_action = super().compute(control_action, joint_pos, joint_vel)
        friction = (self.cfg.coulomb_friction * torch.tanh(joint_vel / self.cfg.friction_smoothing_velocity)
                    + self.cfg.viscous_friction * joint_vel)
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
