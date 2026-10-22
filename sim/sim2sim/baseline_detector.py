"""BASELINE fire "detector" for simulated cameras. Not a real detector.

It marks bright, saturated orange-to-yellow pixels (an HSV colour threshold) and returns one detection per blob.
The simulated camera of robot/ (robot/config.toml [sim]) and sim/mission use it as a baseline.
The real detector is vision/detector.py (YOLO11n).

Interface (the same for every detector module): detect(rgb, t) -> list[FireDetection]
rgb: H x W x 3 uint8 image of the robot camera "rgb". t: time of the image, s.
"""
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "aim"))
from detection import FireDetection  # noqa: E402

MIN_AREA = 30  # px, smaller blobs are ignored


def detect(rgb, t):
    x = rgb.astype(np.float32) / 255.0
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    mx, mn = x.max(axis=2), x.min(axis=2)
    sat = (mx - mn) / (mx + 1e-6)
    hue = 60.0 * (g - b) / (mx - mn + 1e-6)  # degrees, valid where red is the largest channel
    mask = (r >= mx) & (r > 0.75) & (sat > 0.5) & (hue > 12.0) & (hue < 60.0)
    labels, n = ndimage.label(mask)
    dets = []
    for i, sl in enumerate(ndimage.find_objects(labels), start=1):
        area = int((labels[sl] == i).sum())
        if area < MIN_AREA:
            continue
        v0, v1, u0, u1 = sl[0].start, sl[0].stop, sl[1].start, sl[1].stop
        # the fire base is on the floor: use the bottom centre of the blob, so the floor-plane localisation is right
        dets.append(FireDetection(t, "rgb", (u0 + u1) / 2, v1 - 1, (u0, v0, u1, v1), 0.8))
    return dets
