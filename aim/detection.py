"""Interface between the fire detector and the aiming code (aim/README.md, "Detector interface")."""
from dataclasses import dataclass


@dataclass
class FireDetection:
    timestamp: float  # s, time the image was taken (time.monotonic() on the robot computer)
    camera: str  # camera name in the robot config, e.g. "rgb" or "thermal"
    u: float  # px, fire centre column, 0 = left edge
    v: float  # px, fire centre row, 0 = top edge
    bbox: tuple[float, float, float, float]  # px, (u_min, v_min, u_max, v_max)
    confidence: float  # 0..1
    max_temp_c: float | None = None  # deg C, thermal camera only
    depth_m: float | None = None  # m, distance along the camera optical axis (like a depth image)
