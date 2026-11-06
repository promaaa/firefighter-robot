"""Camera calibration with a printed checkerboard (OpenCV). Writes the intrinsics and the lens distortion to JSON.

Run from vision/:
  uv run python calibrate.py --camera /dev/video0                  # live: saves a new, still board position each second
  uv run python calibrate.py --images ~/calib_b0201                # or: folders of .jpg/.png photos of the board (repeat --images)
  uv run python calibrate.py --board-pdf board.pdf                 # the board to print (A4, true size, same --cols --rows --square)
Output: camera_b0201.json (fx, fy, cx, cy, dist = k1 k2 p1 p2 k3, rms, hfov_deg). detector.py uses it when it exists.
Live mode shows the camera image: the found board in colour, the corners of the saved views as red dots (cover the whole
image with them). It saves each view in ~/runs/firefighter/calib/<date>_<W>x<H>/, so --images can calibrate again later.
results/calib_<W>x<H>.jpg shows the last view and the same view without the lens distortion.

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


def board_pdf(out, cols, rows, square):
    """A4 PDF of the board at true size: (cols + 1) x (rows + 1) squares, the long side along the page, a 100 mm check line."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    W, H, s = 210.0, 297.0, square * 1000  # mm
    nx, ny = rows + 1, cols + 1
    x0, y0 = (W - nx * s) / 2, (H - ny * s) / 2
    if min(x0, y0) < 15:
        raise SystemExit(f"board-pdf: {nx} x {ny} squares of {s:g} mm do not fit on A4 with a 15 mm white border")
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes((0, 0, 1, 1), xlim=(0, W), ylim=(0, H))
    ax.axis("off")
    for i in range(nx):
        for j in range(ny):
            if (i + j) % 2 == 0:
                ax.add_patch(plt.Rectangle((x0 + i * s, y0 + j * s), s, s, color="black", lw=0))
    ax.plot([x0, x0 + 100], [y0 / 2, y0 / 2], "k-", lw=0.8)
    ax.text(x0 + 103, y0 / 2, "this line is 100 mm: measure it", va="center", fontsize=8)
    ax.text(W / 2, H - y0 / 2, f"Calibration board: {cols} x {rows} inner corners, {s:g} mm squares. Print at 100 % (no scaling).",
            ha="center", va="center", fontsize=8)
    fig.savefig(out)


def board_points(cols, rows, square):
    p = np.zeros((rows * cols, 3), np.float32)
    p[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return p


def find(gray, cols, rows, fast=False):
    """Inner corners (full-size px) or None. The search runs on a copy at most 640 px wide (1920 x 1080: about 9 times
    faster), the refinement on the full image. fast: drop images without a board quickly (live view)."""
    k = min(1.0, 640 / gray.shape[1])
    small = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_AREA) if k < 1 else gray
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | (cv2.CALIB_CB_FAST_CHECK if fast else 0)
    ok, c = cv2.findChessboardCorners(small, (cols, rows), flags)
    if not ok:
        return None
    c = ((c + 0.5) / k - 0.5).astype(np.float32)  # pixel centres of the small copy -> full image
    w = round(5 / k)  # the same search window on the board as at 640 x 480
    return cv2.cornerSubPix(gray, c, (w, w), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))


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


def from_camera(dev, n, cols, rows, width, height, save):
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    try:  # live view, when there is a display
        import tkinter as tk

        from PIL import Image, ImageTk
        root = tk.Tk(className="calibrate")
        label = tk.Label(root)
        label.pack()
    except Exception:  # noqa: BLE001  no display: the terminal lines only
        root = None
    save.mkdir(parents=True, exist_ok=True)
    grays, seen, last, prev = [], [], 0.0, None
    print(f"Move the board: near, far, tilted, in all corners of the image. Need {n} views. Views go to {save}")
    while len(grays) < n:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit(f"calibrate: no image from {dev}")
        g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        c = find(g, cols, rows, fast=True)
        new = c is not None and all(np.linalg.norm(c - s, axis=-1).mean() > 0.05 * g.shape[1] for s in seen)  # not a copy of a saved view
        still = new and prev is not None and np.linalg.norm(c - prev, axis=-1).mean() < g.shape[1] / 640  # 1 px at 640 x 480: no motion blur
        prev = c
        if still and time.monotonic() - last > 1.0:
            grays.append(g)
            seen.append(c)
            last = time.monotonic()
            cv2.imwrite(str(save / f"view_{len(grays):02d}.png"), g)
            print(f"view {len(grays)}/{n}")
        if root:
            for p in np.concatenate(seen).reshape(-1, 2) if seen else []:
                cv2.circle(frame, (int(p[0]), int(p[1])), 2, (0, 0, 255), -1)
            if c is not None:
                cv2.drawChessboardCorners(frame, (cols, rows), c, True)
            k = min(1.0, 1280 / frame.shape[1])  # 1920 x 1080: the window fits on the screen
            frame = cv2.resize(frame, None, fx=k, fy=k, interpolation=cv2.INTER_AREA) if k < 1 else frame
            img = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)))
            label.configure(image=img)
            root.title(f"calibrate: {len(grays)} of {n} views")
            label.image = img
            root.update()
    if root:
        root.destroy()
    return grays


def figure(gray, r, out):
    """The view and the same view without the lens distortion, side by side, with the result."""
    K = np.array([[r["fx"], 0, r["cx"]], [0, r["fy"], r["cy"]], [0, 0, 1]])
    pair = cv2.cvtColor(np.concatenate([gray, cv2.undistort(gray, K, np.array(r["dist"]))], 1), cv2.COLOR_GRAY2BGR)
    text = (f"{r['width']} x {r['height']}: fx {r['fx']:.1f}, fy {r['fy']:.1f} px, HFOV {r['hfov_deg']:.1f} deg, "
            f"RMS {r['rms']:.2f} px, {r['views']} views. Left: camera image. Right: without lens distortion")
    cv2.putText(pair, text, (8, round(24 * r["width"] / 640)), cv2.FONT_HERSHEY_SIMPLEX, r["width"] / 1300, (0, 0, 255), max(1, r["width"] // 640))
    cv2.imwrite(str(out), pair)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", help="V4L2 device, e.g. /dev/video0")
    ap.add_argument("--images", action="append", help="folder of board photos. Repeat it to use several folders")
    ap.add_argument("--views", type=int, default=25)
    ap.add_argument("--cols", type=int, default=9, help="inner corners along the long side")
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025, help="square size, m")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--out", help="result file (default: camera_b0201.json at 640 x 480, else camera_b0201_<W>x<H>.json)")
    ap.add_argument("--board-pdf", help="write the board to print into this PDF file, then stop")
    a = ap.parse_args()
    if a.board_pdf:
        return board_pdf(a.board_pdf, a.cols, a.rows, a.square)
    if a.camera:
        save = Path.home() / "runs/firefighter/calib" / f"{time.strftime('%Y-%m-%d_%H%M')}_{a.width}x{a.height}"
        grays = from_camera(a.camera, a.views, a.cols, a.rows, a.width, a.height, save)
    else:
        files = sorted(p for d in a.images for p in Path(d).expanduser().iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
        grays = [cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) for f in files]
    r = calibrate(grays, a.cols, a.rows, a.square)
    r["images"] = [str(save)] if a.camera else a.images
    size = f"{r['width']}x{r['height']}"
    out = Path(a.out) if a.out else OUT if size == "640x480" else HERE / f"camera_b0201_{size}.json"
    out.write_text(json.dumps(r, indent=1) + "\n")
    area = [cv2.contourArea(cv2.convexHull(c)) if (c := find(g, a.cols, a.rows)) is not None else 0 for g in grays]
    figure(grays[int(np.argmax(area))], r, HERE / "results" / f"calib_{size}.jpg")  # the view where the board fills the most
    print(json.dumps(r, indent=1))
    print(f"wrote {out} and results/calib_{size}.jpg")
    print(f"fy / fx = {r['fy'] / r['fx']:.3f}: " + ("square pixels." if abs(r["fy"] / r["fx"] - 1) < 0.03 else
          "the camera squeezes or stretches the image in this mode. Check the mode (v4l2-ctl --list-formats-ext)."))
    print("RMS reprojection error above 0.5 px: take more views, closer to the image corners." if r["rms"] > 0.5 else "RMS is fine.")


if __name__ == "__main__":
    main()
