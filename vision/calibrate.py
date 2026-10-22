"""Camera calibration with a printed checkerboard (OpenCV). Writes the intrinsics and the lens distortion to JSON.

Run from vision/:
  uv run python calibrate.py --camera /dev/video0                  # live: saves a view each second the board is found
  uv run python calibrate.py --images ~/calib_b0201                # or: a folder of .jpg/.png photos of the board
Output: camera_b0201.json (fx, fy, cx, cy, dist = k1 k2 p1 p2 k3, rms, hfov_deg). detector.py uses it when it exists.

Board: 9 x 6 inner corners (10 x 7 squares), 25 mm squares, flat on a rigid plate. Change with --cols, --rows, --square.
"""
import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "camera_b0201.json"


def board_points(cols, rows, square):
    p = np.zeros((rows * cols, 3), np.float32)
    p[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return p


def find(gray, cols, rows):
    ok, c = cv2.findChessboardCorners(gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        return None
    return cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))


def calibrate(grays, cols, rows, square):
    """grays: list of grey images. Returns the result dict (raises if fewer than 8 views show the board)."""
    obj, img = [], []
    for g in grays:
        c = find(g, cols, rows)
        if c is not None:
            obj.append(board_points(cols, rows, square))
            img.append(c)
    if len(img) < 8:
        raise SystemExit(f"calibrate: the board was found in {len(img)} images, need 8 or more")
    h, w = grays[0].shape
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, (w, h), None, None)
    d = dist.ravel()[:5]
    return dict(width=w, height=h, fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2], dist=d.tolist(), rms=rms, views=len(img),
                hfov_deg=math.degrees(2 * math.atan(w / 2 / K[0, 0])), date=time.strftime("%Y-%m-%d"))


def from_camera(dev, n, cols, rows, width, height):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    grays, last = [], 0.0
    print(f"Move the board: near, far, tilted, in all corners of the image. Need {n} views.")
    while len(grays) < n:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit(f"calibrate: no image from {dev}")
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if time.monotonic() - last > 1.0 and find(g, cols, rows) is not None:
            grays.append(g)
            last = time.monotonic()
            print(f"view {len(grays)}/{n}")
    return grays


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", help="V4L2 device, e.g. /dev/video0")
    ap.add_argument("--images", help="folder of board photos")
    ap.add_argument("--views", type=int, default=25)
    ap.add_argument("--cols", type=int, default=9, help="inner corners along the long side")
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025, help="square size, m")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    if a.camera:
        grays = from_camera(a.camera, a.views, a.cols, a.rows, a.width, a.height)
    else:
        files = sorted(p for p in Path(a.images).expanduser().iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        grays = [cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) for f in files]
    r = calibrate(grays, a.cols, a.rows, a.square)
    Path(a.out).write_text(json.dumps(r, indent=1) + "\n")
    print(json.dumps(r, indent=1))
    print("RMS reprojection error above 0.5 px: take more views, closer to the image corners." if r["rms"] > 0.5 else "RMS is fine.")


if __name__ == "__main__":
    main()
