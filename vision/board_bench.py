"""Speed test of the fire detector on the robot board (Radxa ROCK 3C, RK3566): is it fast enough for vision?

    uv run --with rknn-toolkit-lite2==2.5.0 python vision/board_bench.py   # on the board: camera /dev/video0, NPU and CPU
    uv run python vision/board_bench.py                       # without rknnlite: the NPU row is skipped
    uv run python vision/board_bench.py --camera /dev/video2  # another camera device
    uv run python vision/board_bench.py --video FILE          # no camera: a video file stands in for it

It measures (1) the camera frame rate at 640 x 480, (2) the detector time per frame for each model and runtime
(NPU .rknn through rknn-toolkit-lite2, CPU .onnx through onnxruntime with 1 and 4 threads), and (3) the full loop
camera -> detector for 20 s. It prints a table with PASS or FAIL against the 10 fps target of aim/ and writes
vision/results/board_bench_<host>.md. A model whose runtime is missing is skipped, with the reason.
Needs only numpy, pillow, ffmpeg and the runtimes (onnxruntime and/or rknnlite).
"""
import argparse
import os
import platform
import socket
import statistics
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "aim")]
from detector import FireDetector  # noqa: E402

W, H, TARGET_FPS = 640, 480, 10.0
TEST_IMAGE = HERE.parent / "sim/firescenes/out/contact_sheet.jpg"  # in git: a few fire scenes
MODELS = [("NPU int8 416x320", "fire_yolo11n_416x320_int8.rknn", 0), ("CPU 416x320, 4 threads", "fire_yolo11n_416x320.onnx", 4),
          ("CPU 416x320, 1 thread", "fire_yolo11n_416x320.onnx", 1)]


def frames(args):
    """Raw 640 x 480 RGB frames from ffmpeg (camera or video file)."""
    src = (["-re", "-stream_loop", "-1", "-i", args.video] if args.video else
           ["-f", "v4l2", "-input_format", "mjpeg", "-video_size", f"{W}x{H}", "-framerate", "30", "-i", args.camera])
    p = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-nostdin", *src, "-vf", f"scale={W}:{H}", "-f", "rawvideo",
                          "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)
    try:
        while (b := p.stdout.read(W * H * 3)) and len(b) == W * H * 3:
            yield np.frombuffer(b, np.uint8).reshape(H, W, 3)
    finally:
        p.kill()


def camera_fps(args, seconds=5.0):
    n, t0 = 0, None
    for _ in frames(args):
        t0 = t0 or time.monotonic()
        n += 1
        if time.monotonic() - t0 >= seconds:
            return (n - 1) / (time.monotonic() - t0)
    return 0.0


def time_model(det, img, runs=50):
    for _ in range(5):  # warm-up
        det.boxes(img)
    ts = []
    for _ in range(runs):
        t = time.perf_counter()
        det.boxes(img)
        ts.append((time.perf_counter() - t) * 1000)
    ts.sort()
    return statistics.median(ts), ts[int(0.9 * len(ts)) - 1], len(det.boxes(img)[0])


def loop_fps(args, det, seconds=20.0):
    n, t0 = 0, None
    for f in frames(args):
        t0 = t0 or time.monotonic()
        det.boxes(f)
        n += 1
        if time.monotonic() - t0 >= seconds:
            return n / (time.monotonic() - t0)
    return 0.0


def board_info():
    def read(p):
        try:
            return Path(p).read_text().strip()
        except OSError:
            return None
    temp = read("/sys/class/thermal/thermal_zone0/temp")
    return {"host": socket.gethostname(), "machine": platform.machine(), "model": read("/proc/device-tree/model") or platform.processor(),
            "cpus": os.cpu_count(), "temp_c": round(int(temp) / 1000, 1) if temp else None, "load": os.getloadavg()}


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--video", help="video file instead of the camera")
    ap.add_argument("--seconds", type=float, default=20.0, help="length of each full-loop run")
    args = ap.parse_args()
    info = board_info()
    L = [f"# Board speed test: {info['model']} ({info['machine']}, {info['cpus']} CPUs), host {info['host']}",
         f"", f"Date {time.strftime('%Y-%m-%d %H:%M')}. CPU temperature at start: {info['temp_c']} °C. Load: {info['load']}.", ""]
    has_cam = bool(args.video) or Path(args.camera).exists()
    cam = camera_fps(args) if has_cam else None
    L.append(f"Camera ({args.video or args.camera}, {W} x {H}): " + (f"{cam:.1f} fps" if cam is not None else "not found, skipped"))
    L += ["", "| Model and runtime | Median (ms) | 90th percentile (ms) | Detector alone (fps) | Camera + detector (fps) | 10 fps target |",
          "|---|---|---|---|---|---|"]
    img = np.asarray(Image.open(TEST_IMAGE).convert("RGB").resize((W, H)))
    for label, name, threads in MODELS:
        path = HERE / "models" / name
        try:
            det = FireDetector(model=path, threads=threads or 4)
        except Exception as e:  # missing runtime (rknnlite, onnxruntime) or NPU driver
            L.append(f"| {label} | skipped: {type(e).__name__}: {str(e)[:80]} | | | | |")
            print(L[-1], flush=True)
            continue
        med, p90, nbox = time_model(det, img)
        full = loop_fps(args, det, args.seconds) if has_cam else None
        ok = (full >= min(TARGET_FPS, 0.95 * cam)) if full is not None else 1000 / p90 >= TARGET_FPS  # a 10 fps source caps the loop
        L.append(f"| {label} | {med:.1f} | {p90:.1f} | {1000 / med:.0f} | {f'{full:.1f}' if full is not None else '-'} | "
                 f"{'PASS' if ok else 'FAIL'} |")
        print(L[-1], flush=True)
    end = board_info()
    L += ["", f"CPU temperature at the end: {end['temp_c']} °C. A model passes when the camera + detector loop reaches 10 fps "
          "(or keeps up with a slower camera), or, without a camera, when the 90th percentile detector time allows 10 fps."]
    out = HERE / "results" / f"board_bench_{info['host']}.md"
    out.write_text("\n".join(L) + "\n")
    print("\n".join(L[:5]))
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
