"""Camera node: camera frames -> fire detector -> interface v2 frame messages to aim/run.py.

    uv run python robot/camera_node.py                     the robot camera (config.toml [camera], through ffmpeg)
    uv run python robot/camera_node.py --source sim        the synthetic camera of robot/fake_world.py (launch.py --sim)
    uv run python robot/camera_node.py --source folder:PATH     replay the .jpg or .png images of a folder
    uv run python robot/camera_node.py --source video:PATH      replay a video file (through ffmpeg)

The detector is a Python file with detect(rgb, t) -> list of FireDetection (or dicts with the same fields).
Each frame gives one message {"timestamp", "camera", "frame": true, "detections": [...]} on the detections port,
also when the list is empty (interface v2, aim/README.md). A copy goes to robot/launch.py (supervisor port).
Folder and video frames are resized to the configured width and height, so aim/run.py uses the right camera model.
"""
import argparse
import importlib.util
import json
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import asdict, is_dataclass
from pathlib import Path

import numpy as np

from robot_config import CONF, HOST, ROOT, path

CAM = CONF["camera"]
W, H, FPS = CAM["width"], CAM["height"], CAM["fps"]


def detector_file(name=CAM["detector"], source="real"):
    """The detector for this source. The synthetic camera draws colour blobs: config.toml [sim] detector (the baseline).
    Other sources: config.toml [camera] detector, else vision/detector.py when it exists, else the colour-threshold baseline."""
    if source == "sim" and not name:
        return path(CONF["sim"]["detector"])
    if name:
        return path(name)
    own = ROOT / "vision" / "detector.py"
    return own if own.exists() else ROOT / "sim" / "sim2sim" / "baseline_detector.py"


def load_detector(file):
    spec = importlib.util.spec_from_file_location("fire_detector", file)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as e:  # noqa: BLE001  a missing model file: say which detector, then stop the node
        sys.exit(f"camera_node: cannot load the detector {file}: {e!r}. Set config.toml [camera] detector.")
    return module.detect


class FfmpegFrames:
    """RGB frames from ffmpeg (a V4L2 camera or a video file). A thread keeps only the newest frame, so a slow
    detector never works on an old image."""

    def __init__(self, args):
        cmd = ["ffmpeg", "-loglevel", "error", "-nostdin", *args,
               "-vf", f"scale={W}:{H}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL)
        self.frame, self.t, self.ended, self.cond = None, 0.0, False, threading.Condition()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        n = W * H * 3
        while True:
            data = self.proc.stdout.read(n)
            if len(data) < n:
                with self.cond:
                    self.ended = True  # not "frame is None": that also means "no frame yet" at start-up
                    self.cond.notify_all()
                return
            with self.cond:
                self.frame, self.t = np.frombuffer(data, np.uint8).reshape(H, W, 3), time.monotonic()
                self.cond.notify_all()

    def __iter__(self):
        last = 0.0
        while True:
            with self.cond:
                self.cond.wait_for(lambda: self.ended or self.t > last, timeout=5.0)
                if self.t <= last:
                    raise RuntimeError(f"camera: {'stream ended' if self.ended else 'no frame for 5 s'} "
                                       f"(ffmpeg exit code {self.proc.poll()})")
                last, frame = self.t, self.frame
            yield last - CAM["latency_s"], frame

    def close(self):
        self.proc.terminate()


def paced(make_frame, fps=FPS):
    """Call make_frame() at a fixed rate. Yields (time, frame)."""
    tick = time.monotonic()
    while True:
        t = time.monotonic()
        yield t, make_frame()
        tick += 1 / fps
        time.sleep(max(0.0, tick - time.monotonic()))


def folder_frames(folder):
    from PIL import Image
    files = sorted(p for p in Path(folder).iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not files:
        sys.exit(f"camera: no .jpg or .png images in {folder}")
    i = iter(range(10 ** 12))
    return paced(lambda: np.asarray(Image.open(files[next(i) % len(files)]).convert("RGB").resize((W, H))))


def open_source(source, pump_file=None, fires=None):
    """(frames iterator, object to close or None, fake world or None)."""
    kind, _, arg = source.partition(":")
    if kind == "sim":
        from fake_world import FakeWorld
        world = FakeWorld(pump_file, fires)
        return paced(lambda: (world.update(), world.render())[1]), None, world
    if kind == "folder":
        return folder_frames(path(arg)), None, None
    if kind == "video":
        f = FfmpegFrames(["-re", "-stream_loop", "-1", "-i", str(path(arg))])  # no "-fflags nobuffer": ffmpeg 9 then writes nothing for a file
        return iter(f), f, None
    if kind == "real":
        f = FfmpegFrames(["-fflags", "nobuffer", "-f", "v4l2", "-input_format", CAM["input_format"], "-video_size", f"{W}x{H}",
                          "-framerate", str(FPS), "-i", arg or CAM["device"]])
        return iter(f), f, None
    sys.exit(f"camera: unknown source {source!r}: use real, sim, folder:PATH or video:PATH")


def message(t, dets, camera="rgb"):
    rows = [asdict(d) if is_dataclass(d) else dict(d) for d in dets]
    for r in rows:  # the frame gives the time and the camera (interface v2)
        r.pop("timestamp", None)
        r.pop("camera", None)
    return dict(timestamp=t, camera=camera, frame=True, detections=rows)


def _stop(*_):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--source", default="real", help="real[:DEVICE], sim, folder:PATH or video:PATH")
    ap.add_argument("--detector", default="", help="file with detect(rgb, t); default: config.toml [camera] detector ([sim] detector for --source sim)")
    ap.add_argument("--pump-file", help="with --source sim: the mock pump pin (water on the fake fires)")
    ap.add_argument("--fires", help='with --source sim: fire positions "x,y;x,y" in m (default: config.toml [sim] fires)')
    ap.add_argument("--seconds", type=float, help="stop after this time (for tests)")
    a = ap.parse_args(argv)
    signal.signal(signal.SIGTERM, _stop)
    det_file = detector_file(a.detector or CAM["detector"], a.source.partition(":")[0])
    detect = load_detector(det_file)
    fires = [[float(v) for v in f.split(",")] for f in a.fires.split(";")] if a.fires else None
    frames, closer, world = open_source(a.source, a.pump_file, fires)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ports = (CONF["ports"]["detections"], CONF["ports"]["supervisor"])
    print(f"camera_node: source {a.source}, detector {det_file.relative_to(ROOT) if det_file.is_relative_to(ROOT) else det_file}, "
          f"{W}x{H} at {FPS} fps, frames to udp {ports}", file=sys.stderr, flush=True)
    n, t0 = 0, time.monotonic()
    try:
        for t, rgb in frames:
            msg = message(t, detect(rgb, t))
            if world is not None:  # sim only: the fire health, for the log and the plot
                msg["sim_fire_health"] = [round(float(h), 3) for h in world.health]
            data = json.dumps(msg, default=float).encode()
            for port in ports:
                try:
                    tx.sendto(data, (HOST, port))
                except OSError:
                    pass
            n += 1
            if a.seconds and time.monotonic() - t0 > a.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if closer:
            closer.close()
        dt = time.monotonic() - t0
        print(f"camera_node: {n} frames in {dt:.1f} s ({n / max(dt, 1e-9):.1f} fps)", file=sys.stderr)


if __name__ == "__main__":
    main()
