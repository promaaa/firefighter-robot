"""Backend-independent identity and asset paths for ASMR 2026 Humanoid v1.

Keep this module free of simulator imports so hardware and MuJoCo tools can use
the robot definition without importing Isaac Lab.
"""

from pathlib import Path
from typing import Literal

Simulator = Literal["isaaclab", "mujoco"]

ROBOT_ID = "asmr2026_humanoid_v1"
ASSET_DIR = Path(__file__).resolve().parents[1] / "data" / "robots" / "asmr2026" / ROBOT_ID

MJCF_ROBOT_PATH = ASSET_DIR / "mjcf" / "asmr2026_humanoid.xml"
MJCF_SCENE_PATH = ASSET_DIR / "mjcf" / "scene.xml"
MJLAB_SCENE_PATH = ASSET_DIR / "mjlab" / "scene.xml"
USD_PATH = ASSET_DIR / "usd" / "asmr2026_humanoid.usd"

LEG_JOINTS = (
    "leg_left_hip_roll_joint",
    "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint",
    "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint",
    "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint",
    "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint",
    "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint",
    "leg_right_ankle_roll_joint",
)

# Coordinate conversion from the legacy low-level/BHL joint convention to the
# ASMR MJCF convention. This describes model axes only; it is not a motor encoder sign map.
BHL_TO_ASMR_JOINT_SIGNS = (1.0, -1.0, -1.0, -1.0, -1.0, -1.0,
                           1.0, -1.0, 1.0, 1.0, 1.0, -1.0)

STANDING_JOINT_POSITIONS = (
    0.0,
    0.0,
    0.58519126,
    -0.95241361,
    0.36722235,
    0.0,
    0.0,
    0.0,
    -0.58519126,
    0.95241361,
    -0.36722235,
    0.0,
)
STANDING_ROOT_HEIGHT = -0.03977653

# Rotation whose columns are the MJCF IMU axes expressed in the robot base frame.
# The course asset mounts the IMU 90 degrees counterclockwise about base z.
IMU_TO_BODY = (
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
)


def asset_path(simulator: Simulator) -> Path:
    """Return the scene asset for a simulator backend."""
    if simulator == "mujoco":
        return MJCF_SCENE_PATH
    if simulator == "isaaclab":
        return USD_PATH
    raise ValueError(f"unsupported simulator {simulator!r}; choose 'isaaclab' or 'mujoco'")
