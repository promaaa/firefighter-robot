"""Simulator-independent deployment contract for the ASMR baseline policy.

This module is safe to import on the robot computer: it uses NumPy and the
standard library only. Joint limits and actuator values are read from the ASMR
MJCF rather than requiring MuJoCo at runtime.
"""

from xml.etree import ElementTree

import numpy as np

from .asmr2026_model import (
    LEG_JOINTS,
    MJCF_ROBOT_PATH,
    STANDING_JOINT_POSITIONS,
    BHL_TO_ASMR_JOINT_SIGNS,
)

POLICY_DT = 0.04
ACTION_SCALE = 0.25
TILT_LIMIT = 0.78
OBSERVATION_SIZE = 45
ACTION_SIZE = len(LEG_JOINTS)
DEFAULT = np.asarray(STANDING_JOINT_POSITIONS, dtype=np.float32)
BHL_TO_ASMR = np.asarray(BHL_TO_ASMR_JOINT_SIGNS, dtype=np.float32)


def _model_parameters():
    root = ElementTree.parse(MJCF_ROBOT_PATH).getroot()
    joint_ranges = {}
    for element in root.iter("joint"):
        name, values = element.get("name"), element.get("range")
        if name in LEG_JOINTS and values:
            joint_ranges[name] = tuple(float(x) for x in values.split())

    actuators = {}
    for element in root.iter("position"):
        name = element.get("joint")
        if name in LEG_JOINTS:
            force_range = tuple(float(x) for x in element.get("forcerange").split())
            actuators[name] = (
                float(element.get("kp")),
                float(element.get("kv")),
                max(abs(force_range[0]), abs(force_range[1])),
            )

    missing_joints = set(LEG_JOINTS) - joint_ranges.keys()
    missing_actuators = set(LEG_JOINTS) - actuators.keys()
    if missing_joints or missing_actuators:
        raise ValueError(
            f"ASMR MJCF is missing joint limits {sorted(missing_joints)} "
            f"or position actuators {sorted(missing_actuators)}"
        )

    return (
        np.asarray([joint_ranges[name] for name in LEG_JOINTS], dtype=np.float32),
        np.asarray([actuators[name][0] for name in LEG_JOINTS], dtype=np.float32),
        np.asarray([actuators[name][1] for name in LEG_JOINTS], dtype=np.float32),
        np.asarray([actuators[name][2] for name in LEG_JOINTS], dtype=np.float32),
    )


JOINT_LIMITS, KP, KD, EFFORT_LIMITS = _model_parameters()


def to_asmr(joint_values):
    """Convert drive-map joint positions or velocities to the ASMR model convention."""
    return np.asarray(joint_values) * BHL_TO_ASMR


def to_drive_coordinates(joint_values):
    """Convert ASMR joint positions or velocities to the drive-map convention."""
    # The coordinate transform only contains signs, so it is its own inverse.
    return np.asarray(joint_values) * BHL_TO_ASMR


def quat_rotate_inverse(quat, vector):
    """Rotate a vector into the frame of quat=[w,x,y,z]."""
    w, u = quat[0], quat[1:4]
    return vector * (2.0 * w * w - 1.0) - np.cross(u, vector) * w * 2.0 + u * np.dot(u, vector) * 2.0


def observation(command, quat, gyro, joint_pos, joint_vel, previous_action):
    """Build the baseline policy's 45-value observation in its trained order."""
    gravity = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0]))
    values = np.concatenate([
        command,
        gyro,
        gravity,
        joint_pos - DEFAULT,
        joint_vel,
        previous_action,
    ])
    if values.size != OBSERVATION_SIZE:
        raise ValueError(f"policy observation has {values.size} values, expected {OBSERVATION_SIZE}")
    return values.astype(np.float32, copy=False)[None, :]
