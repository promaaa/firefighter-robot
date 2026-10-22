"""Export a trained run to ONNX, make an INT8 copy, and measure the CPU speed with onnxruntime.

Run from vision/:
  uv run python export.py all416                     # ONNX models of ~/runs/firefighter/vision/all416/weights/best.pt
  uv run python export.py all416 --speed --install   # also the speed table (results/speed.md), and copy to models/
All shapes go to ~/runs/firefighter/vision/<run>/onnx/. --install copies the 416x320 model to models/.

Each ONNX model has two outputs, "boxes" (1 x 4 x A: cx, cy, w, h in input pixels) and "scores" (1 x 1 x A).
Ultralytics joins them into one tensor. Two outputs let the RKNN INT8 conversion give each its own scale
(boxes go to about 400, scores to 1: one shared INT8 scale would round most scores to 0).
Input: 1 x 3 x H x W, RGB, 0..1. Shapes (W x H): 416 x 416, 416 x 320 (4:3 camera image), 320 x 320, 320 x 256, 640 x 480 (the
camera image without resize).
"""
import argparse
import platform
import shutil
import statistics
import time
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
from onnxruntime.quantization import CalibrationDataReader, QuantFormat, QuantType, quantize_static
from PIL import Image

from detector import FireDetector, letterbox

HERE = Path(__file__).resolve().parent
RUNS = Path.home() / "runs/firefighter/vision"
DATA = Path.home() / "datasets/firefighter/yolo"
MODELS = HERE / "models"
SHAPES = {"416": (416, 416), "416x320": (320, 416), "320": (320, 320), "320x256": (256, 320), "640x480": (480, 640)}  # (H, W)


def split_outputs(path):
    """Replace the last Concat (boxes, scores) -> output0 by two graph outputs, in place."""
    m = onnx.load(path)
    g = m.graph
    out = g.output[0].name
    cat = next(n for n in g.node if out in n.output and n.op_type == "Concat")
    a = g.output[0].type.tensor_type.shape.dim[2].dim_value
    g.node.remove(cat)
    del g.output[:]
    for name, new, ch in ((cat.input[0], "boxes", 4), (cat.input[1], "scores", 1)):
        g.node.append(onnx.helper.make_node("Identity", [name], [new]))
        g.output.append(onnx.helper.make_tensor_value_info(new, onnx.TensorProto.FLOAT, [1, ch, a]))
    onnx.checker.check_model(m)
    onnx.save(m, path)


def export(run, imgsz_name):
    from ultralytics import YOLO
    h, w = SHAPES[imgsz_name]
    best = RUNS / run / "weights/best.pt"
    f = Path(YOLO(best).export(format="onnx", imgsz=(h, w), opset=12, simplify=True, dynamic=False, batch=1))
    dst = RUNS / run / "onnx" / f"fire_yolo11n_{imgsz_name}.onnx"
    dst.parent.mkdir(exist_ok=True)
    shutil.move(f, dst)
    split_outputs(dst)
    print("wrote", dst)
    return dst


class Calib(CalibrationDataReader):
    def __init__(self, model, n=200):
        sess = ort.InferenceSession(str(model), providers=["CPUExecutionProvider"])
        self.name = sess.get_inputs()[0].name
        self.h, self.w = sess.get_inputs()[0].shape[2:]
        files = sorted((DATA / "val").glob("*/images/*.jpg"))
        self.files = iter(np.random.default_rng(0).choice(files, n, replace=False))

    def get_next(self):
        f = next(self.files, None)
        if f is None:
            return None
        return {self.name: letterbox(np.asarray(Image.open(f).convert("RGB")), self.w, self.h)[0]}


def quantize(model):
    """Static INT8 (QDQ) copy for onnxruntime, calibrated on 200 validation images."""
    dst = model.with_name(model.stem + "_int8.onnx")
    op13 = model.with_name(model.stem + "_opset13.onnx")  # per-channel INT8 needs opset 13
    onnx.save(onnx.version_converter.convert_version(onnx.load(model), 13), op13)
    quantize_static(str(op13), str(dst), Calib(model), quant_format=QuantFormat.QDQ, per_channel=True,
                    activation_type=QuantType.QUInt8, weight_type=QuantType.QInt8)
    op13.unlink()
    print("wrote", dst)
    return dst


def latency(model, threads, n=200):
    """Network alone: median and 10th percentile time (ms). detect() on a 640 x 480 frame (letterbox + network + decode):
    median (ms). The 10th percentile is the time of the fastest 10 % of runs: close to the time on a CPU with no other jobs."""
    det = FireDetector(model, conf=0.35, threads=threads)
    x = np.random.default_rng(0).random((1, 3, det.h, det.w), dtype=np.float32)
    frame = np.asarray(Image.open(sorted((DATA / "test/barman/images").glob("*.jpg"))[0]).convert("RGB").resize((640, 480)))
    for _ in range(10):
        det.sess.run(None, {det.input: x})
    t_net, t_all = [], []
    for _ in range(n):
        t0 = time.perf_counter()
        det.sess.run(None, {det.input: x})
        t_net.append(time.perf_counter() - t0)
        t0 = time.perf_counter()
        det(frame, 0.0)
        t_all.append(time.perf_counter() - t0)
    return 1000 * statistics.median(t_net), 1000 * np.percentile(t_net, 10), 1000 * statistics.median(t_all)


def speed(run):
    cpu = next((l.split(":", 1)[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")), platform.processor())
    load = open("/proc/loadavg").read().split()[:3]
    rows = []
    for f in sorted((RUNS / run / "onnx").glob("fire_yolo11n_*.onnx")):
        for threads in (1, 4):
            net, fast, full = latency(f, threads)
            rows.append(f"| `{f.name}` | {threads} | {net:.1f} | {fast:.1f} | {full:.1f} | {1000 / full:.0f} |")
            print(rows[-1], flush=True)
    text = (f"Measured on laptop: {cpu} (8 cores), onnxruntime {ort.__version__}, 200 runs each. "
            f"Load average before the test: {' '.join(load)}. Other jobs shared the CPU, so the median times are pessimistic. "
            "Fastest 10 %: the 10th percentile, closer to the time on a free CPU.\n\n"
            "| Model | Threads | Network, median (ms) | Network, fastest 10 % (ms) | detect() on a 640 x 480 frame, median (ms) | "
            "Frames per second, detect() median |\n|---|---|---|---|---|---|\n" + "\n".join(rows) + "\n")
    (HERE / "results/speed.md").write_text(text)
    print(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--speed", action="store_true")
    ap.add_argument("--install", action="store_true")
    a = ap.parse_args()
    for name in SHAPES:
        f = export(a.run, name)
        if name in ("416x320", "320x256"):  # the shapes for the 4:3 camera image
            quantize(f)
    if a.install:
        MODELS.mkdir(exist_ok=True)
        for name in ("416x320",):
            shutil.copy(RUNS / a.run / "onnx" / f"fire_yolo11n_{name}.onnx", MODELS)
    if a.speed:
        speed(a.run)


if __name__ == "__main__":
    main()
