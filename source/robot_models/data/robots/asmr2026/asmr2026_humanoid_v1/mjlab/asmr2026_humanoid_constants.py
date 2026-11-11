"""ASMR2026 Humanoid v1: actuator parameters and mjlab robot config.

Actuator values are bench-fitted on a single actuator of each type. They are
a starting point, not an exact model of your robot: characterize your own
actuators and update the values below.

Copyright (c) 2026 Seunghoon Kang, Soft Robotics and Bionics Laboratory (SRBL), Department of Mechanical Engineering, Seoul National University. All rights reserved.
"""

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco

from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

ASMR2026_HUMANOID_XML = Path(__file__).resolve().parent / "asmr2026_humanoid_mjlab.xml"

##
# Actuator parameters.
##

GEAR_RATIO = 15.0  # motor turns per joint turn


@dataclass(frozen=True)
class MotorFit:
  kt: float  # torque constant, motor side (N*m/A)
  armature: float  # reflected inertia, joint side (kg*m^2)
  friction_static: float  # Coulomb friction, joint side (N*m)
  friction_dynamic: float  # viscous friction, joint side (N*m*s/rad)


ACTUATOR_6512 = MotorFit(kt=0.067, armature=0.0228, friction_static=0.377, friction_dynamic=0.090)
ACTUATOR_5010 = MotorFit(kt=0.0836, armature=0.00556, friction_static=0.317, friction_dynamic=0.030)

# Motor driver limits (XDrive MINI configuration).
CURRENT_LIM_A = 18.0  # motor current limit (A)
VEL_LIMIT_TURNS_S = 50.0  # motor velocity limit (turns/s)
VELOCITY_LIMIT_RAD_S = VEL_LIMIT_TURNS_S * 2 * math.pi / GEAR_RATIO  # joint side

# PD gains: Kp = armature * wn^2, Kd = 2 * zeta * armature * wn.
OMEGA_N_RAD_S = 2 * math.pi * 10.0

# Joint group -> (actuator, damping ratio zeta, joint name patterns).
JOINT_GROUPS = {
  "hip_roll_yaw": (ACTUATOR_6512, 2.0, (".*hip_roll_joint", ".*hip_yaw_joint")),
  "hip_pitch": (ACTUATOR_6512, 2.0, (".*hip_pitch_joint",)),
  "knee_pitch": (ACTUATOR_6512, 2.0, (".*knee_pitch_joint",)),
  "ankle_pitch": (ACTUATOR_5010, 0.7, (".*ankle_pitch_joint",)),
  "ankle_roll": (ACTUATOR_5010, 0.7, (".*ankle_roll_joint",)),
}


def derive_kp_kd(armature: float, omega_n: float, zeta: float) -> tuple[float, float]:
  return armature * omega_n**2, 2.0 * zeta * armature * omega_n


def effort_limit(fit: MotorFit) -> float:
  """Joint-side torque limit (N*m)."""
  return CURRENT_LIM_A * fit.kt * GEAR_RATIO


def _actuator_cfg(group: str) -> BuiltinPositionActuatorCfg:
  fit, zeta, patterns = JOINT_GROUPS[group]
  kp, kd = derive_kp_kd(fit.armature, OMEGA_N_RAD_S, zeta)
  return BuiltinPositionActuatorCfg(
    target_names_expr=patterns,
    stiffness=kp,
    damping=kd,
    effort_limit=effort_limit(fit),
    armature=fit.armature,
    frictionloss=fit.friction_static,
    viscous_damping=fit.friction_dynamic,
  )


ASMR2026_HUMANOID_ACTUATORS = tuple(_actuator_cfg(group) for group in JOINT_GROUPS)

ASMR2026_HUMANOID_ARTICULATION = EntityArticulationInfoCfg(
  actuators=ASMR2026_HUMANOID_ACTUATORS,
  soft_joint_pos_limit_factor=0.9,
)

##
# Collisions: collision geoms touch the floor, not each other.
##

ASMR2026_HUMANOID_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(?!leg_(?:left|right)_ankle_pitch_collision$).*_collision$",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6, 0.005, 0.0001),
)

##
# Initial state: both feet flat on the ground.
##

STANDING_JOINT_POS = {
  "leg_left_hip_roll_joint": 0.0,
  "leg_left_hip_yaw_joint": 0.0,
  "leg_left_hip_pitch_joint": 0.58519126,
  "leg_left_knee_pitch_joint": -0.95241361,
  "leg_left_ankle_pitch_joint": 0.36722235,
  "leg_left_ankle_roll_joint": 0.0,
  "leg_right_hip_roll_joint": 0.0,
  "leg_right_hip_yaw_joint": 0.0,
  "leg_right_hip_pitch_joint": -0.58519126,
  "leg_right_knee_pitch_joint": 0.95241361,
  "leg_right_ankle_pitch_joint": -0.36722235,
  "leg_right_ankle_roll_joint": 0.0,
}
STANDING_ROOT_HEIGHT = -0.03977653  # base frame origin; the body sits above it

STANDING_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, STANDING_ROOT_HEIGHT),
  joint_pos=STANDING_JOINT_POS,
  joint_vel={".*": 0.0},
)

FOOT_GEOM_NAMES = ("left_foot_collision", "right_foot_collision")
FOOT_SITE_NAMES = ("left_foot", "right_foot")


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(ASMR2026_HUMANOID_XML))


def get_asmr2026_humanoid_cfg() -> EntityCfg:
  """Return a fresh robot config for an mjlab task."""
  return EntityCfg(
    init_state=STANDING_STATE,
    collisions=(ASMR2026_HUMANOID_COLLISION,),
    spec_fn=get_spec,
    articulation=ASMR2026_HUMANOID_ARTICULATION,
  )


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_asmr2026_humanoid_cfg())
  viewer.launch(robot.spec.compile())
