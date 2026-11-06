"""Train a YOLO11n fire detector on the dataset from data.py.

Run from vision/:
  uv run python train.py all416             # one variant (see VARIANTS)
  uv run python train.py all416 --epochs 3  # a short test
  uv run python train.py cam416 --epochs 15 # planned model changes: cam416, p2_416, y26_416 (docs/plan/vision.md)
  CUDA_VISIBLE_DEVICES=0 python train.py cam416 --epochs 15 --workers 8 --cache none   # on a lab server (cauchy, 5 Nov)
  python -c "import train; train.bench_list(())"   # once before parallel p2bench trainings: links and labels of the bench frames
Runs go to ~/runs/firefighter/vision/<variant>/ (weights, results.csv, curves). Nothing big goes into git.
"""
import argparse
import json
import random
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionTrainer

DATA = Path.home() / "datasets/firefighter/yolo"
RUNS = Path.home() / "runs/firefighter/vision"
WEIGHTS = RUNS / "weights/yolo11n.pt"  # COCO-pretrained YOLO11n (Ultralytics, AGPL-3.0)
FASDD_NEGATIVES = 6000  # FASDD training images without fire that we keep (of about 50 000): shorter epochs
SYN_REPEAT = 5  # the 203 synthetic training images appear 5 times in each epoch
VAL_PER_SOURCE = {"dfire": 800, "fasdd": 1200, "barman": 444, "coco": 250}  # validation images used during training
# Screen bench runs of the B0201 with the eight flames of set v2 (screen_bench.py): automatic exposure and 1 ms
BENCH = [Path.home() / "runs/firefighter/screen_bench" / r for r in ("2026-11-05_1834", "2026-11-05_2130")]
# Two folds of the v2 videos (part of the file name). The three Jahobr candles look alike, so they stay in one fold.
FOLDS = {"a": ("Jahobr", "GilesLaurent", "flameless_candle_1"),
         "b": ("LoKiLeCh", "candle_blown_out", "Rhetos", "candle_suffocated", "flameless_candle_2")}

VARIANTS = {  # name: (input size, use synthetic images, start weights, extra Ultralytics training arguments)
    "all416": (416, True, WEIGHTS, {}),
    "real416": (416, False, WEIGHTS, {}),
    # Small flames (candles): start from all416 and scale each training image by 0.1 to 1.9 (default 0.5 to 1.5), so that
    # many flames are only a few px wide. Run it with --epochs 8. Result on 2026-11-05: no gain, so all416 stays
    # (results/small416/metrics.md, docs/plan/vision.md "Results").
    "small416": (416, True, RUNS / "all416/weights/best.pt", dict(scale=0.9)),
    # Model changes after the screen bench (V2) and the candle test (V4), docs/plan/vision.md "Model changes". Planned.
    # cam416: start from all416 and change the training images like the B0201 does (CameraLike). Run it with --epochs 15.
    "cam416": (416, True, RUNS / "all416/weights/best.pt", dict(camera_like=True)),
    # p2_416: YOLOv8n with a fourth output at stride 4 (P2) for very small objects, from the COCO weights of YOLOv8n.
    "p2_416": (416, True, "yolov8n-p2.yaml", dict(pretrained="yolov8n.pt")),
    # y26_416: YOLO26n (Ultralytics 8.4), from its COCO weights. Check the RKNN conversion before a long training.
    "y26_416": (416, True, "yolo26n.pt", {}),
    # p2cam416: start from p2_416 and add CameraLike. On the B0201 screen frames (5 Nov, automatic exposure) p2_416 and
    # cam416 were the two best models, so this joins both changes. Run it with --epochs 15.
    "p2cam416": (416, True, RUNS / "p2_416/weights/best.pt", dict(camera_like=True)),
    # p2bench_a, p2bench_b: start from p2cam416 and add the B0201 screen bench frames (bench_list), without the videos of fold
    # A or B. The left-out fold then tests the model on flames and LED candles that it did not see:
    # bench_compare.py RUN --videos <fold>. p2bench: all videos, the model for the robot. Run them with --epochs 5.
    "p2bench_a": (416, True, RUNS / "p2cam416/weights/best.pt", dict(camera_like=True, bench_skip=FOLDS["a"])),
    "p2bench_b": (416, True, RUNS / "p2cam416/weights/best.pt", dict(camera_like=True, bench_skip=FOLDS["b"])),
    "p2bench": (416, True, RUNS / "p2cam416/weights/best.pt", dict(camera_like=True, bench_skip=())),
}
CAMERA_LIKE = False  # main() sets it for the variants with camera_like=True


class CameraLike:
    """Photometric changes like the B0201 camera makes them: blur (lens, focus), exposure and gamma, white balance error,
    sensor noise, MJPEG compression. Each one with probability p. The boxes stay as they are. Python and NumPy random
    numbers, so that each dataloader worker gets its own sequence (Ultralytics seeds both in each worker).
    Exposure: the screen bench (5 Nov) showed both ends. The automatic exposure brightens a dark scene, so the flame turns into
    a white blob with a halo. A short fixed exposure (1 ms) gives a dark scene with a sharp flame."""

    def __init__(self, p=0.5):
        self.p = p

    def __call__(self, labels):
        img = labels["img"]
        if random.random() < self.p:
            img = cv2.GaussianBlur(img, (0, 0), random.uniform(0.3, 1.5))
        if random.random() < self.p:  # exposure for a mean level of 0.15 to 0.6: a dark scene gets up to x3
            x = img.astype(np.float32) / 255
            g = float(np.clip(random.uniform(0.15, 0.6) / max(float(x.mean()), 1e-3), 0.3, 3.0))
            x *= g
            if g > 1:  # the light above the sensor maximum spreads into a halo
                x += random.uniform(0.3, 1.0) * cv2.GaussianBlur(np.maximum(x - 1, 0), (0, 0), random.uniform(2, 8))
            img = (np.clip(x, 0, 1) ** random.uniform(0.7, 1.4) * 255).astype(np.uint8)
        if random.random() < self.p:  # white balance error (the bench showed blue and pink casts)
            img = np.clip(img * np.random.uniform(0.8, 1.2, 3), 0, 255).astype(np.uint8)
        if random.random() < self.p:
            img = np.clip(img + np.random.normal(0, random.uniform(1, 6), img.shape), 0, 255).astype(np.uint8)
        if random.random() < self.p:
            img = cv2.imdecode(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, random.randint(30, 90)])[1], cv2.IMREAD_COLOR)
        labels["img"] = img
        return labels


def add_camera_like(dataset, hyp, p=0.5):
    """Put CameraLike before the last transform (Format) of a YOLO dataset. It stays when close_mosaic rebuilds the transforms."""
    build = dataset.build_transforms

    def build_with(hyp=None):
        t = build(hyp)
        t.transforms.insert(-1, CameraLike(p))
        return t
    dataset.build_transforms = build_with
    dataset.transforms = build_with(hyp)
    return dataset


class Trainer(DetectionTrainer):
    """Ultralytics doubles the dataloader workers for validation. This class keeps the total at --workers.
    It also adds CameraLike to the training images when CAMERA_LIKE is set."""

    def build_dataset(self, img_path, mode="train", batch=None):
        ds = super().build_dataset(img_path, mode, batch)
        return add_camera_like(ds, self.args) if mode == "train" and CAMERA_LIKE else ds

    def get_dataloader(self, dataset_path, batch_size=16, rank=0, mode="train"):
        if mode == "train":
            return super().get_dataloader(dataset_path, batch_size, rank, mode)
        w = self.args.workers
        self.args.workers = max(1, w // 2)
        try:
            return super().get_dataloader(dataset_path, batch_size, rank, mode)
        finally:
            self.args.workers = w


def train_list(syn):
    """Image list for one variant: all D-Fire, Barman and COCO training images, FASDD fire images plus some negatives."""
    files = []
    for source in ("dfire", "barman", "coco"):
        files += sorted((DATA / "train" / source / "images").glob("*.jpg"))
    fasdd = sorted((DATA / "train/fasdd/images").glob("*.jpg"))
    fire = [f for f in fasdd if (DATA / "train/fasdd/labels" / (f.stem + ".txt")).stat().st_size > 0]
    neg = sorted(set(fasdd) - set(fire))
    files += fire + random.Random(0).sample(neg, FASDD_NEGATIVES)
    if syn:
        files += sorted((DATA / "train/synthetic/images").glob("*.jpg")) * SYN_REPEAT
    return files


def bench_list(skip, out=DATA / "bench"):
    """B0201 screen bench frames as training images: the true box is the label, an LED candle frame has no label. Leaves out
    the videos whose file name holds a string of skip, and flames under 4 px. Writes links and labels into out."""
    (out / "images").mkdir(parents=True, exist_ok=True)
    (out / "labels").mkdir(exist_ok=True)
    files = []
    for run in BENCH:
        videos = [s["video"] for s in json.loads((run / "setup.json").read_text())["stimuli"]]
        for t in map(json.loads, open(run / "trials.jsonl")):
            x0, y0, x1, y1 = t["truth"] or (0, 0, 0, 0)
            if not t["ok"] or any(s in videos[t["stim"]] for s in skip) or (t["fire"] and x1 - x0 < 4):
                continue
            label = f"0 {(x0 + x1) / 1280:.5f} {(y0 + y1) / 960:.5f} {(x1 - x0) / 640:.5f} {(y1 - y0) / 480:.5f}\n" if t["fire"] else ""
            for f in t["frames"]:  # 640 x 480 camera frames. Existing files stay, so that parallel trainings can share them.
                img, lab = out / "images" / f"{run.name}_{Path(f).stem}.jpg", out / "labels" / f"{run.name}_{Path(f).stem}.txt"
                if not img.exists():
                    img.symlink_to(run / f)
                if not lab.exists():
                    lab.write_text(label)
                files.append(img)
    return files


def val_list():
    """A fixed part of the validation split, so that each epoch's validation is short. evaluate.py uses all of it."""
    files = []
    for source, n in VAL_PER_SOURCE.items():
        imgs = sorted((DATA / "val" / source / "images").glob("*.jpg"))
        files += random.Random(0).sample(imgs, min(n, len(imgs)))
    return files


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("variant", choices=list(VARIANTS))
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--cache", default="disk", choices=["disk", "none"], help="none on a lab server: no 50 GB of .npy files, no clash between runs")
    args = ap.parse_args()
    imgsz, syn, init, extra = VARIANTS[args.variant]
    extra = dict(extra)
    global CAMERA_LIKE
    CAMERA_LIKE = extra.pop("camera_like", False)
    skip = extra.pop("bench_skip", None)
    lst = DATA / f"train_{args.variant}.txt" if skip is not None else DATA / f"train_{'all' if syn else 'real'}.txt"
    files = train_list(syn) + (bench_list(skip) if skip is not None else [])
    lst.write_text("\n".join(str(f) for f in files) + "\n")
    val = DATA / "val_train.txt"
    val.write_text("\n".join(str(f) for f in val_list()) + "\n")
    yaml = DATA / f"data_{args.variant}.yaml"
    yaml.write_text(f"path: {DATA}\ntrain: {lst}\nval: {val}\nnames:\n  0: fire\n")
    print(args.variant, len(files), "training images per epoch")
    model = YOLO(init)
    # Optimizer: AdamW, lr0 0.001, 1 warm-up epoch. With lr 0.01 (SGD, or the Ultralytics "auto" choice), the training
    # loss rose with the learning rate and the validation mAP fell after epoch 1.
    model.train(trainer=Trainer, data=str(yaml), imgsz=imgsz, epochs=args.epochs, batch=args.batch, workers=args.workers,
                optimizer="AdamW", lr0=0.001, warmup_epochs=1,
                close_mosaic=max(2, args.epochs // 5), cache=args.cache if args.cache == "disk" else False, seed=0, project=str(RUNS), name=args.variant,
                exist_ok=True, plots=True, **extra)


if __name__ == "__main__":
    main()
