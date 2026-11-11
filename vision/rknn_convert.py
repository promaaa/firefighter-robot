# /// script
# requires-python = "==3.12.*"
# dependencies = ["rknn-toolkit2==2.5.0", "pillow", "setuptools<80"]  # rknn imports pkg_resources
# ///
"""Convert the ONNX fire detector to RKNN for the RK3566 NPU (Radxa ROCK 3C) and run it in the RKNN simulator.

Run from vision/. uv makes a separate environment from the header above (rknn-toolkit2 needs older numpy and onnx):
  uv run --script rknn_convert.py models/fire_yolo11n_416x320.onnx
Writes ~/runs/firefighter/vision/rknn/<model>_int8.rknn and <model>_fp16.rknn, and the simulator predictions on a
fixed subset of the test images (preds_<model>_<type>.npz). A model in a training run (RUN/onnx/FILE) gets the run name in
front, as in screen_bench.py: p2bench_fire_yolo11n_416x320. Compare them with the ONNX model: evaluate.py --rknn FILE.
The simulator runs on the PC. It shows the accuracy of the converted model, not the speed on the board.

INT8 on the frames of a screen bench run (about 0.1 s per frame at 416 x 320, 0.2 s at 640 x 480):
  uv run --script rknn_convert.py RUN/onnx/fire_yolo11n_416x320.onnx --bench ~/runs/firefighter/screen_bench/RUN_DIR
It writes det_<model>_int8_sim.json into RUN_DIR, next to the det files of the ONNX models: bench_compare.py RUN_DIR compares
them. "_sim", because the board writes det_<model>_int8.json with the real NPU (screen_bench.py report, docs/plan/vision.md
V6), and a det file that exists is not computed again. --videos Jahobr,GilesLaurent,flameless_candle_1 keeps only the
videos that a p2bench_a model did not see (train.py FOLDS).
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image
from rknn.api import RKNN

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detector import NMS_IOU, decode, letterbox  # noqa: E402

DATA = Path.home() / "datasets/firefighter/yolo"
OUT = Path.home() / "runs/firefighter/vision/rknn"
SUBSET = {"barman": 101, "synthetic": 68, "dfire": 300, "fasdd": 300, "coco": 150}  # test images per source


def uint8_input(f, w, h):
    """Letterboxed RGB image, H x W x 3 uint8 (the RKNN model divides by 255 itself), scale, (pad_x, pad_y)."""
    x, s, pad = letterbox(np.asarray(Image.open(f).convert("RGB")), w, h)
    return np.round(x[0].transpose(1, 2, 0) * 255).astype(np.uint8), s, pad


def convert(onnx_path, name, quant, w, h):
    rknn = RKNN(verbose=False)
    # disable_rules: rknn-toolkit2 2.5.0 fails in this fuse rule on the YOLO11 head (IndexError). Its log suggests this fix.
    rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform="rk3566",
                disable_rules=["seperate_conv_rs_split"])
    if rknn.load_onnx(model=str(onnx_path)) != 0:
        raise SystemExit("load_onnx failed")
    dataset = None
    if quant:
        calib = OUT / "calib"
        calib.mkdir(parents=True, exist_ok=True)
        files = sorted((DATA / "val").glob("*/images/*.jpg"))
        lines = []
        for f in random.Random(0).sample(files, 200):
            p = calib / f"{f.stem}_{w}x{h}.png"  # letterboxed, exactly the input size: the toolkit does not resize it
            if not p.exists():
                Image.fromarray(uint8_input(f, w, h)[0]).save(p)
            lines.append(str(p))
        dataset = OUT / f"calib_{w}x{h}.txt"
        dataset.write_text("\n".join(lines) + "\n")
    if rknn.build(do_quantization=quant, dataset=str(dataset) if dataset else None) != 0:
        raise SystemExit("build failed")
    out = OUT / f"{name}_{'int8' if quant else 'fp16'}.rknn"
    rknn.export_rknn(str(out))
    print("wrote", out, f"{out.stat().st_size / 1e6:.1f} MB")
    return rknn


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("onnx")
    ap.add_argument("--bench", type=Path, help="screen bench run folder: INT8 only, on its frames instead of the test images")
    ap.add_argument("--videos", type=lambda s: s.split(","), help="with --bench: only these source videos (comma-separated parts of names)")
    a = ap.parse_args()
    onnx_path = Path(a.onnx).resolve()
    name = f"{onnx_path.parent.parent.name}_{onnx_path.stem}" if onnx_path.parent.name == "onnx" else onnx_path.stem
    import onnx
    dims = onnx.load(str(onnx_path)).graph.input[0].type.tensor_type.shape.dim
    h, w = dims[2].dim_value, dims[3].dim_value
    OUT.mkdir(parents=True, exist_ok=True)
    if a.bench:  # source = the frame name in trials.jsonl, the key of the det files
        from bench_compare import trials
        files = [(f, a.bench / f) for t in trials(a.bench, a.videos) for f in t["frames"]]
    else:
        rng = random.Random(0)
        files = []
        for source, n in SUBSET.items():
            imgs = sorted((DATA / "test" / source / "images").glob("*.jpg"))
            files += [(source, f) for f in sorted(rng.sample(imgs, min(n, len(imgs))))]
    conf = 0.1 if a.bench else 0.001  # the det files of screen_bench.py keep the boxes from 0.1
    for quant in (True,) if a.bench else (True, False):
        rknn = convert(onnx_path, name, quant, w, h)
        if rknn.init_runtime() != 0:  # no target: the simulator
            raise SystemExit("init_runtime (simulator) failed")
        recs, t0 = [], time.time()
        for k, (source, f) in enumerate(files):
            x, s, (px, py) = uint8_input(f, w, h)
            b, sc = decode(rknn.inference(inputs=[x[None]], data_format="nhwc"), conf, NMS_IOU)
            im = Image.open(f)
            b = np.clip((b - [px, py, px, py]) / s, 0, [im.width, im.height, im.width, im.height])
            recs.append(dict(source=source, file=str(f), boxes=b[:100], scores=sc[:100]))
            if k % 100 == 0:
                print(k, len(files), f"{time.time() - t0:.0f} s", flush=True)
        kind = "int8" if quant else "fp16"
        if a.bench:  # the format of screen_bench.py detections()
            dst = a.bench / f"det_{name}_{kind}_sim.json"
            dst.write_text(json.dumps({r["source"]: [[round(float(v), 1) for v in x] + [round(float(y), 3)]
                                                     for x, y in zip(r["boxes"], r["scores"])] for r in recs}))
        else:
            dst = OUT / f"preds_{name}_{kind}.npz"
            np.savez_compressed(dst, recs=np.array(recs, dtype=object))
        print("wrote", dst)
        rknn.release()


if __name__ == "__main__":
    main()
