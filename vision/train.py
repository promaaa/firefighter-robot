"""Train a YOLO11n fire detector on the dataset from data.py.

Run from vision/:
  uv run python train.py all416             # one variant (see VARIANTS)
  uv run python train.py all416 --epochs 3  # a short test
Runs go to ~/runs/firefighter/vision/<variant>/ (weights, results.csv, curves). Nothing big goes into git.
"""
import argparse
import random
from pathlib import Path

from ultralytics import YOLO
from ultralytics.models.yolo.detect import DetectionTrainer

DATA = Path.home() / "datasets/firefighter/yolo"
RUNS = Path.home() / "runs/firefighter/vision"
WEIGHTS = RUNS / "weights/yolo11n.pt"  # COCO-pretrained YOLO11n (Ultralytics, AGPL-3.0)
FASDD_NEGATIVES = 6000  # FASDD training images without fire that we keep (of about 50 000): shorter epochs
SYN_REPEAT = 5  # the 203 synthetic training images appear 5 times in each epoch
VAL_PER_SOURCE = {"dfire": 800, "fasdd": 1200, "barman": 444, "coco": 250}  # validation images used during training

VARIANTS = {  # name: (input size, use synthetic images)
    "all416": (416, True),
    "real416": (416, False),
}


class Trainer(DetectionTrainer):
    """Ultralytics doubles the dataloader workers for validation. This class keeps the total at --workers."""

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
    args = ap.parse_args()
    imgsz, syn = VARIANTS[args.variant]
    lst = DATA / f"train_{'all' if syn else 'real'}.txt"
    files = train_list(syn)
    lst.write_text("\n".join(str(f) for f in files) + "\n")
    val = DATA / "val_train.txt"
    val.write_text("\n".join(str(f) for f in val_list()) + "\n")
    yaml = DATA / f"data_{args.variant}.yaml"
    yaml.write_text(f"path: {DATA}\ntrain: {lst}\nval: {val}\nnames:\n  0: fire\n")
    print(args.variant, len(files), "training images per epoch")
    model = YOLO(WEIGHTS)
    # Optimizer: AdamW, lr0 0.001, 1 warm-up epoch. With lr 0.01 (SGD, or the Ultralytics "auto" choice), the training
    # loss rose with the learning rate and the validation mAP fell after epoch 1.
    model.train(trainer=Trainer, data=str(yaml), imgsz=imgsz, epochs=args.epochs, batch=args.batch, workers=args.workers,
                optimizer="AdamW", lr0=0.001, warmup_epochs=1,
                close_mosaic=max(2, args.epochs // 5), cache="disk", seed=0, project=str(RUNS), name=args.variant,
                exist_ok=True, plots=True)


if __name__ == "__main__":
    main()
