"""Build one YOLO dataset (one class: fire) from public fire datasets, COCO fire-like negatives and our synthetic scenes.

Run from vision/:  uv run python data.py
Steps: download -> extract -> convert -> mark test images that are near-duplicates of training images -> statistics.
- Convert: resize each image to 640 px on its long side (never up), keep only the fire boxes, class 0 = fire.
- Big files go to ~/datasets/firefighter: raw/ (archives), extracted/, yolo/ (the dataset). Nothing big goes into git.
- Output in the repo: results/data_stats.md and results/data_stats.json (counts for DATA.md).
"""
import io
import json
import random
import re
import tarfile
import urllib.request
import zipfile
from collections import Counter, defaultdict
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
ROOT = Path.home() / "datasets/firefighter"
RAW, EXT, OUT = ROOT / "raw", ROOT / "extracted", ROOT / "yolo"
REPO_FIRESCENES = HERE.parent / "sim/firescenes/out/yolo/rgb"
LONG_SIDE = 640  # px, the camera image is 640 x 480
# Near-duplicate test images (the same photo, or a video frame of the same scene as a training image). Two steps:
# candidates within DUP_BITS of 64 (dHash), then the mean absolute difference of 32 x 32 grey thumbnails (0..255).
DUP_BITS = 10
DUP_MAD = 10.0  # chosen by eye: pairs at 6 to 10 are still the same scene
SEED = 0

HF = "https://huggingface.co/datasets"
DOWNLOADS = {  # file in raw/ -> URL, pinned to the revision used on 2026-10-15
    "dfire_train.tar": f"{HF}/hanvithSai/gavin-dfire-raw/resolve/fee27ef6934d5bb569ae6271c123673f10a877c8/train.tar",
    "dfire_test.tar": f"{HF}/hanvithSai/gavin-dfire-raw/resolve/fee27ef6934d5bb569ae6271c123673f10a877c8/test.tar",
    "fasdd_cv.zip": f"{HF}/seawsurf/fire_smoke_dataset_fasdd_cv/resolve/896dec15c93396ed4990ad502a187be9271e38e7/FASDD_CV.zip",
    "barman.zip": f"{HF}/baizhanquan/FireDetectionDataset-flame-indoor-characteristic-flame-barman/resolve/"
                  "96490dfdf78df2a897f38a5ea84c363b0d160bed/archive.zip",
    "coco_annotations_trainval2017.zip": f"{HF}/pcuenq/coco-2017-mirror/resolve/a4cd8b69bd45a35e9a0a9c8692f3a7f4f23321fe/"
                                         "annotations_trainval2017.zip",
    "coco_val2017.zip": f"{HF}/pcuenq/coco-2017-mirror/resolve/a4cd8b69bd45a35e9a0a9c8692f3a7f4f23321fe/val2017.zip",
    "coco_train2017.zip": f"{HF}/pcuenq/coco-2017-mirror/resolve/a4cd8b69bd45a35e9a0a9c8692f3a7f4f23321fe/train2017.zip",
    "firescenes_v1.zip": "https://github.com/promaaa/vision-sobang/releases/download/firescenes-v1/firescenes_v1.zip",
}

# COCO captions: an image is "fire-like" when a caption names a fire-coloured object or a light,
# and no caption names a fire, a flame, a candle or another heat source (it could show a real fire).
COCO_SAFE = re.compile(r"fire ?(hydrant|truck|engine|station|department)s?|firetrucks?")
COCO_BAD = re.compile(r"\b(fire|fires|flame|flames|flaming|burn\w*|smok\w*|candle\w*|fireplace\w*|campfire|bonfire|torch\w*|"
                      r"firework\w*|lit|lighter|lighters|matches|grill\w*|barbe\w*|bbq|blaz\w*|ember\w*|stove\w*|oven\w*|"
                      r"furnace|cigar\w*|incense|lava|explosion|sparkler\w*)\b")
COCO_GOOD = re.compile(r"\b(sunsets?|sunrise|dusk|twilight|orange|neon|lamps?|lanterns?|lights|lighting|glow\w*|illuminat\w*|"
                       r"traffic light|street ?lights?|headlights?|tail ?lights?|christmas lights|hydrants?|fire ?trucks?|"
                       r"firetrucks?)\b")
COCO_VAL_FROM_TRAIN = 500  # fire-like COCO train images that go to val instead of train


def download():
    RAW.mkdir(parents=True, exist_ok=True)
    for name, url in DOWNLOADS.items():
        dst = RAW / name
        if dst.exists():
            continue
        print("download", url)
        part = dst.with_suffix(dst.suffix + ".part")
        urllib.request.urlretrieve(url, part)
        part.rename(dst)


def extract():
    jobs = [("dfire", "dfire_train.tar"), ("dfire", "dfire_test.tar"), ("fasdd", "fasdd_cv.zip"), ("barman", "barman.zip"),
            ("firescenes", "firescenes_v1.zip")]
    for folder, name in jobs:
        dst = EXT / folder
        marker = dst / f".done_{name}"
        if marker.exists():
            continue
        print("extract", name)
        dst.mkdir(parents=True, exist_ok=True)
        if name.endswith(".tar"):
            with tarfile.open(RAW / name) as t:
                t.extractall(dst, members=[m for m in t if "/._" not in m.name and not m.name.startswith("._")
                                           and not m.name.endswith(".DS_Store")], filter="data")
        else:
            with zipfile.ZipFile(RAW / name) as z:
                z.extractall(dst)
        marker.touch()
    coco = EXT / "coco/annotations"
    if not (coco / "captions_val2017.json").exists():
        with zipfile.ZipFile(RAW / "coco_annotations_trainval2017.zip") as z:
            z.extractall(EXT / "coco", members=["annotations/captions_train2017.json", "annotations/captions_val2017.json"])


def read_yolo(label_file, fire_class):
    """Fire boxes (cx, cy, w, h), normalised, from a YOLO label file. None when the file is missing."""
    if not label_file.exists():
        return None
    boxes = []
    for line in label_file.read_text().splitlines():
        p = line.split()
        if len(p) >= 5 and int(float(p[0])) == fire_class:
            cx, cy, w, h = (float(x) for x in p[1:5])
            x0, y0, x1, y1 = max(cx - w / 2, 0), max(cy - h / 2, 0), min(cx + w / 2, 1), min(cy + h / 2, 1)
            if x1 > x0 and y1 > y0:
                boxes.append(((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0))
    return boxes


def yolo_folder(folder, fire_class, skip_unlabelled=False):
    for img in sorted((folder / "images").iterdir()):
        boxes = read_yolo(folder / "labels" / (img.stem + ".txt"), fire_class)
        if boxes is None and skip_unlabelled:
            continue
        yield img, boxes or []


def coco_fire_like(split):
    """Image file names of fire-like COCO images (see COCO_GOOD / COCO_BAD), and their licence ids."""
    ann = json.loads((EXT / f"coco/annotations/captions_{split}2017.json").read_text())
    caps = defaultdict(list)
    for a in ann["annotations"]:
        caps[a["image_id"]].append(a["caption"].lower())
    out = []
    for im in ann["images"]:
        text = COCO_SAFE.sub("", " ".join(caps[im["id"]]))
        if COCO_GOOD.search(text) and not COCO_BAD.search(text):
            out.append((im["file_name"], im["license"]))
    return sorted(out), {lic["id"]: lic["name"] for lic in ann["licenses"]}


def jobs():
    """(source, split, name, image source, fire boxes) for every image of the dataset."""
    dfire = EXT / "dfire"
    for img, boxes in yolo_folder(dfire / "train", fire_class=1):  # D-Fire labels: 0 = smoke, 1 = fire
        i = int(re.sub(r"\D", "", img.stem))
        split = "val" if (i // 100) % 10 == 9 else "train"  # blocks of 100 neighbouring frames, so video frames stay together
        yield "dfire", split, img.stem, img, boxes
    for img, boxes in yolo_folder(dfire / "test", fire_class=1):
        yield "dfire", "test", img.stem, img, boxes

    fasdd = EXT / "fasdd"
    rng = random.Random(SEED)
    for split, folder in (("train", "train"), ("val", "valid"), ("test", "test")):
        items = list(yolo_folder(fasdd / folder, fire_class=0))  # FASDD (Roboflow export): 0 = fire, 1 = smoke
        if split == "val":
            items = rng.sample(items, 3000)  # 3000 of the 9513 val images, so that each validation pass is short
        for img, boxes in items:
            yield "fasdd", split, img.stem.split("_jpg.rf.")[0] + "_" + img.stem[-8:], img, boxes

    barman = EXT / "barman/Flame"
    for split, folder in (("train", "train"), ("val", "valid"), ("test", "test")):
        for img, boxes in yolo_folder(barman / folder, fire_class=0, skip_unlabelled=True):  # unlabelled: status unknown
            yield "barman", split, img.stem.replace("_jpg.rf.", "_"), img, boxes

    names, _ = coco_fire_like("train")
    rng = random.Random(SEED)
    val_names = set(rng.sample([n for n, _ in names], COCO_VAL_FROM_TRAIN))
    for name, _ in names:
        yield "coco", "val" if name in val_names else "train", "train2017_" + Path(name).stem, (RAW / "coco_train2017.zip",
                                                                                               "train2017/" + name), []
    for name, _ in coco_fire_like("val")[0]:
        yield "coco", "test", "val2017_" + Path(name).stem, (RAW / "coco_val2017.zip", "val2017/" + name), []

    syn = REPO_FIRESCENES if REPO_FIRESCENES.exists() else EXT / "firescenes/out/yolo/rgb"
    for k, (img, boxes) in enumerate(yolo_folder(syn, fire_class=0)):
        yield "synthetic", "test" if k % 4 == 3 else "train", img.stem, img, boxes


@lru_cache(maxsize=4)
def open_zip(path):
    return zipfile.ZipFile(path)


def dhash(im):
    g = np.asarray(im.convert("L").resize((9, 8), Image.BILINEAR), dtype=np.int16)
    return int(np.packbits((g[:, 1:] > g[:, :-1]).ravel()).view(">u8")[0])


def convert(job):
    source, split, name, src, boxes = job
    if isinstance(src, tuple):
        im = Image.open(io.BytesIO(open_zip(src[0]).read(src[1])))
    else:
        im = Image.open(src)
    w0, h0 = im.size
    d = OUT / split / source
    out, lab = d / "images" / f"{name}.jpg", d / "labels" / f"{name}.txt"
    if out.exists() and lab.exists():  # done in an earlier run (a training run may read it now: do not write it again)
        im = Image.open(out).convert("RGB")
    else:
        im = im.convert("RGB")
        if max(w0, h0) > LONG_SIDE:
            s = LONG_SIDE / max(w0, h0)
            im = im.resize((round(w0 * s), round(h0 * s)), Image.LANCZOS)
        im.save(out, quality=90)
        lab.write_text("".join(f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n" for cx, cy, w, h in boxes))
    w, h = im.size
    return dict(source=source, split=split, name=name, orig=(w0, h0), size=(w, h), hash=dhash(im),
                thumb=np.asarray(im.convert("L").resize((32, 32), Image.BILINEAR), np.uint8),
                box_w=[round(b[2] * w, 1) for b in boxes], box_h=[round(b[3] * h, 1) for b in boxes])


def mark_test_duplicates(recs):
    """Mark each test image that is a near-duplicate of a training image. They stay in the test set. evaluate.py
    reports all test images and the test images without near-duplicates ("new scenes")."""
    train = [r for r in recs if r["split"] == "train"]
    th = np.array([r["hash"] for r in train], dtype=np.uint64)
    tt = np.stack([r["thumb"] for r in train]).astype(np.int16)
    dup = Counter()
    for r in recs:
        if r["split"] != "test":
            continue
        bits = np.bitwise_count(th ^ np.uint64(r["hash"]))
        cand = np.flatnonzero(bits <= DUP_BITS)
        mad = np.abs(tt[cand] - r["thumb"]).mean((1, 2)) if len(cand) else np.array([255.0])
        r["min_bits"], r["min_mad"] = int(bits.min()), float(mad.min())
        r["nearest"] = train[cand[int(mad.argmin())]]["name"] if len(cand) else ""
        r["near_dup"] = r["min_mad"] <= DUP_MAD
        dup[r["source"]] += r["near_dup"]
    tests = [r for r in recs if r["split"] == "test"]
    (OUT / "test_near_duplicates.txt").write_text("".join(f"{OUT}/test/{r['source']}/images/{r['name']}.jpg\n" for r in tests
                                                           if r["near_dup"]))
    np.savez_compressed(OUT / "test_duplicate_check.npz", name=[r["name"] for r in tests], source=[r["source"] for r in tests],
                        min_bits=[r["min_bits"] for r in tests], min_mad=[r["min_mad"] for r in tests],
                        nearest=[r["nearest"] for r in tests])
    return dup


def stats(recs, dup):
    rows, js = [], {}
    for split in ("train", "val", "test"):
        for source in ("dfire", "fasdd", "barman", "coco", "synthetic"):
            rs = [r for r in recs if r["split"] == split and r["source"] == source]
            if not rs:
                continue
            bw = np.array([w for r in rs for w in r["box_w"]])
            fire = sum(bool(r["box_w"]) for r in rs)
            s = dict(images=len(rs), fire_images=fire, no_fire_images=len(rs) - fire, boxes=len(bw),
                     small_boxes=int((bw < 32).sum()), median_box_w=float(np.median(bw)) if len(bw) else None,
                     sizes=Counter(f"{r['orig'][0]}x{r['orig'][1]}" for r in rs).most_common(3))
            js[f"{split}/{source}"] = s
            med = f"{s['median_box_w']:.0f}" if len(bw) else "-"
            sizes = ", ".join(f"{k} ({v})" for k, v in s["sizes"])
            rows.append(f"| {split} | {source} | {s['images']} | {fire} | {s['no_fire_images']} | {s['boxes']} | "
                        f"{s['small_boxes']} | {med} | {sizes} |")
    tests = [r for r in recs if "min_mad" in r]
    hist = Counter(min(int(r["min_mad"] // 2) * 2, 20) for r in tests)
    js["test_near_duplicates"] = dict(dup)
    js["test_min_mad_hist"] = {str(k): hist[k] for k in sorted(hist)}
    text = ("Written by `data.py`. Box width in pixels after the resize to 640 px on the long side. "
            "Small box: width under 32 px.\n\n"
            "| Split | Source | Images | With fire | Without fire | Fire boxes | Small boxes | Median box width (px) | "
            "Most common original sizes |\n|---|---|---|---|---|---|---|---|---|\n" + "\n".join(rows) +
            f"\n\nTest images that are near-duplicates of a training image (dHash within {DUP_BITS} bits of 64 and "
            f"mean grey difference of 32 x 32 thumbnails <= {DUP_MAD}): "
            + ", ".join(f"{k} {v}" for k, v in sorted(dup.items())) +
            "\n\nMean grey difference from each test image to the most similar training image (2-unit bins, 20 = 20 or more): "
            + ", ".join(f"{k}: {hist[k]}" for k in sorted(hist)) + "\n")
    (HERE / "results").mkdir(exist_ok=True)
    (HERE / "results/data_stats.md").write_text(text)
    (HERE / "results/data_stats.json").write_text(json.dumps(js, indent=1))
    print(text)


def write_yaml():
    dirs = {s: sorted(str(p.relative_to(OUT)) for p in (OUT / s).glob("*/images")) for s in ("train", "val", "test")}
    lines = [f"path: {OUT}"] + [f"{s}: [{', '.join(d)}]" for s, d in dirs.items()] + ["names:", "  0: fire"]
    (OUT / "data.yaml").write_text("\n".join(lines) + "\n")


def main():
    download()
    extract()
    all_jobs = list(jobs())
    for source, split in {(j[0], j[1]) for j in all_jobs}:
        for sub in ("images", "labels"):
            (OUT / split / source / sub).mkdir(parents=True, exist_ok=True)
    print(len(all_jobs), "images to convert")
    with Pool(6) as pool:
        recs = pool.map(convert, all_jobs, chunksize=64)
    stats(recs, mark_test_duplicates(recs))
    write_yaml()
    lic = {}
    for split in ("train", "val"):
        names, lic_names = coco_fire_like(split)
        lic[split] = Counter(lic_names[i] for _, i in names)
    (HERE / "results/coco_licences.json").write_text(json.dumps(lic, indent=1))  # COCO images keep their Flickr licences


if __name__ == "__main__":
    main()
