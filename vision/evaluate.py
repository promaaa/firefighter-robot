"""Evaluate a fire detector (ONNX, run through detector.py, so the same code as on the robot) on the held-out test sets.

Run from vision/:
  uv run python evaluate.py ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_416.onnx --name all416   # metrics, figures
  uv run python evaluate.py --compare all416 real416                       # table and training curves of the variants
  uv run python evaluate.py --conf 0.28 --deploy models/fire_yolo11n_416x320.onnx --rknn ~/runs/firefighter/vision/rknn/preds_*.npz
Writes results/<name>/ (metrics.md, metrics.json, figures). Predictions are cached in ~/runs/firefighter/vision/<name>/.

Terms: IoU = overlap of two boxes (intersection / union). A prediction is a true positive (TP) when its IoU with a
not-yet-matched true box is 0.5 or more (COCO-style matching, best score first). AP = area under the precision-recall
curve (101 points). mAP50 = AP at IoU 0.5. mAP50-95 = mean AP over IoU 0.5, 0.55, ..., 0.95.
Box width is in pixels after the resize to 640 px on the long side (the size in the 640 x 480 camera image).
"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from ultralytics.utils.metrics import compute_ap  # noqa: E402

from detector import FireDetector  # noqa: E402

HERE = Path(__file__).resolve().parent
DATA = Path.home() / "datasets/firefighter/yolo"
RUNS = Path.home() / "runs/firefighter/vision"
REAL = ("dfire", "fasdd", "barman")
SOURCES = REAL + ("coco", "synthetic")
IOUS = np.linspace(0.5, 0.95, 10)
WIDTH_BINS = [0, 8, 12, 16, 24, 32, 48, 64, 96, 128, 10_000]  # px
# Camera: Arducam B0201 at 640 x 480. ASSUMED: 120 deg diagonal FOV on the 16:9 sensor (datasheet), 640 x 480 = centre
# 4:3 crop, pinhole model. Then HFOV = 2 atan(tan(60 deg) * 16 / sqrt(16^2 + 9^2) * 3/4) = 97.1 deg and f = 283 px.
HFOV_DEG = math.degrees(2 * math.atan(math.tan(math.radians(60)) * 16 / math.hypot(16, 9) * 0.75))
F_PX = 320 / math.tan(math.radians(HFOV_DEG) / 2)


def load_gt(img, w, h):
    """True fire boxes (N x 4, x0 y0 x1 y1, pixels) of one dataset image of size w x h."""
    rows = [list(map(float, l.split()[1:5])) for l in Path(str(img).replace("/images/", "/labels/")).with_suffix(".txt").read_text().splitlines()
            if l.strip()]
    lab = np.array(rows, dtype=float).reshape(-1, 4)
    return np.stack([(lab[:, 0] - lab[:, 2] / 2) * w, (lab[:, 1] - lab[:, 3] / 2) * h,
                     (lab[:, 0] + lab[:, 2] / 2) * w, (lab[:, 1] + lab[:, 3] / 2) * h], 1)


def predict(model, name, split):
    """Predictions at a low threshold (0.001) for every image of one split. Cached in RUNS/<name>/preds_<split>.npz."""
    cache = RUNS / name / f"preds_{split}.npz"
    if cache.exists():
        return np.load(cache, allow_pickle=True)["recs"].tolist()
    files = [(source, img) for source in SOURCES for img in sorted((DATA / split / source / "images").glob("*.jpg"))]
    recs = predict_files(model, files)
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache, recs=np.array(recs, dtype=object))
    return recs


def predict_files(model, files):
    """Predictions at threshold 0.001 for a list of (source, image file)."""
    det = FireDetector(model, conf=0.001, threads=6)
    recs = []
    for k, (source, img) in enumerate(files):
        rgb = np.asarray(Image.open(img).convert("RGB"))
        h, w = rgb.shape[:2]
        boxes, scores = det.boxes(rgb)
        recs.append(dict(source=source, file=str(img), size=(w, h), gt=load_gt(img, w, h), boxes=boxes[:100], scores=scores[:100]))
        if k % 1000 == 0:
            print(Path(model).name, k, len(files), flush=True)
    return recs


def iou(a, b):
    lt, rb = np.maximum(a[:, None, :2], b[None, :, :2]), np.minimum(a[:, None, 2:], b[None, :, 2:])
    inter = np.prod(np.clip(rb - lt, 0, None), axis=2)
    area = lambda x: np.prod(x[:, 2:] - x[:, :2], axis=1)  # noqa: E731
    return inter / (area(a)[:, None] + area(b)[None, :] - inter + 1e-9)


def match(boxes, scores, gt, gt_ignore, thr):
    """COCO-style greedy matching at one IoU threshold. Boxes must be sorted by score (best first).
    Returns per prediction: tp, ignore. Per true box: the score of the prediction that matched it (-1 = none)."""
    tp, ign, gt_score = np.zeros(len(boxes), bool), np.zeros(len(boxes), bool), -np.ones(len(gt))
    if len(gt) == 0 or len(boxes) == 0:
        return tp, ign, gt_score
    m = iou(boxes, gt)
    for i in np.flatnonzero(m.max(1) >= thr):  # score order
        ok = (gt_score < 0) & (m[i] >= thr)
        if not ok.any():
            continue
        real = ok & ~gt_ignore  # a true box that counts is preferred to an ignored one
        j = int(np.argmax(np.where(real if real.any() else ok, m[i], -1.0)))
        gt_score[j], tp[i], ign[i] = scores[i], not gt_ignore[j], gt_ignore[j]
    return tp, ign, gt_score


_cache = {}


def image_result(r, wmin, wmax):
    """Per image: TP flags (N x 10 IoU thresholds) and kept flags of the predictions, (width, matched score) per true box."""
    key = (r["file"], wmin, wmax)
    if key not in _cache:
        gw = r["gt"][:, 2] - r["gt"][:, 0]
        gi = (gw < wmin) | (gw >= wmax)
        pw = r["boxes"][:, 2] - r["boxes"][:, 0]
        t_all = []
        for k, thr in enumerate(IOUS):
            tp, ign, gs = match(r["boxes"], r["scores"], r["gt"], gi, thr)
            t_all.append(tp)
            if k == 0:  # simplification: the ignore mask at IoU 0.5 is used for all thresholds (tiny effect on AP50-95)
                keep = ~(ign | (~tp & ((pw < wmin) | (pw >= wmax))))
                rows = [(w, s) for w, s, g in zip(gw, gs, gi) if not g]
        _cache[key] = (np.stack(t_all, 1)[keep], r["scores"][keep], rows, int((~gi).sum()))
    return _cache[key]


def evaluate(recs, wmin=0.0, wmax=1e9):
    """AP50, AP50-95 and per-box data for true boxes with width in [wmin, wmax). Other true boxes, and predictions of
    other widths that match nothing, are ignored (COCO area-range rule, here on the box width)."""
    res = [image_result(r, wmin, wmax) for r in recs]
    tp, conf = np.concatenate([x[0] for x in res]), np.concatenate([x[1] for x in res])
    gt_rows, n_gt = [row for x in res for row in x[2]], sum(x[3] for x in res)
    order = np.argsort(-conf, kind="stable")
    tp, conf = tp[order], conf[order]
    ctp = np.cumsum(tp, 0)
    prec, rec = ctp / np.arange(1, len(tp) + 1)[:, None], ctp / max(n_gt, 1)
    ap = np.array([compute_ap(rec[:, k], prec[:, k])[0] for k in range(len(IOUS))]) if n_gt else np.zeros(len(IOUS))
    return dict(n_gt=n_gt, ap50=float(ap[0]), ap5095=float(ap.mean()), tp50=tp[:, 0], conf=conf, prec50=prec[:, 0],
                rec50=rec[:, 0], gt=np.array(gt_rows).reshape(-1, 2))


def at_conf(e, c):
    """Precision and recall at confidence threshold c (IoU 0.5)."""
    k = int((e["conf"] >= c).sum())
    tp = int(e["tp50"][:k].sum())
    return dict(precision=tp / k if k else 1.0, recall=tp / max(e["n_gt"], 1), tp=tp, fp=k - tp, fn=e["n_gt"] - tp)


def fp_per_image(recs, c):
    neg = [r for r in recs if len(r["gt"]) == 0]
    fp = [int((r["scores"] >= c).sum()) for r in neg]
    return dict(images=len(neg), fp_per_image=float(np.mean(fp)) if neg else 0.0,
                images_with_fp=float(np.mean([f > 0 for f in fp])) if neg else 0.0)


def best_f1_conf(recs):
    e = evaluate(recs)
    f1 = 2 * e["prec50"] * e["rec50"] / (e["prec50"] + e["rec50"] + 1e-9)
    return float(e["conf"][int(np.argmax(f1))])


def recall_by_width(gt_rows, c):
    """Recall at threshold c for each width bin: (lo, hi, n, recall)."""
    out = []
    for lo, hi in zip(WIDTH_BINS[:-1], WIDTH_BINS[1:]):
        sel = (gt_rows[:, 0] >= lo) & (gt_rows[:, 0] < hi)
        out.append((lo, hi, int(sel.sum()), float((gt_rows[sel, 1] >= c).mean()) if sel.any() else float("nan")))
    return out


def min_width(bins, target):
    """Smallest bin lower edge from which every larger bin (with 20 boxes or more) has recall >= target. None if none."""
    ok = None
    for lo, hi, n, r in reversed(bins):
        if n < 20:
            continue
        if r < target:
            break
        ok = lo
    return ok


def grid(recs, c, out, seed=0):
    """3 rows of 4 test images: true positives (green), false positives (red), misses (true box in yellow)."""
    rng = np.random.default_rng(seed)
    real = [r for r in recs if r["source"] in ("dfire", "barman")]  # only D-Fire (CC0) and Barman images in this figure

    def kind(r):
        keep = r["scores"] >= c
        tp, _, gs = match(r["boxes"][keep], r["scores"][keep], r["gt"], np.zeros(len(r["gt"]), bool), 0.5)
        return tp, keep, gs

    rows = {"TP": [], "FP": [], "miss": []}
    for i in rng.permutation(len(real)):
        r = real[i]
        tp, keep, gs = kind(r)
        name = ("FP" if (~tp).any() and len(r["gt"]) == 0 else "miss" if (gs < 0).any() else
                "TP" if tp.any() and (tp.all()) else None)
        if name and len(rows[name]) < 4:
            rows[name].append((r, tp, keep, gs))
        if all(len(v) == 4 for v in rows.values()):
            break
    W, H, T = 400, 300, 30
    sheet = Image.new("RGB", (4 * W, 3 * H + 3 * T), "white")
    d0 = ImageDraw.Draw(sheet)
    for row, (name, items) in enumerate(rows.items()):
        y = row * (H + T)
        d0.text((6, y + 4), {"TP": "True positives (green)", "FP": "False alarms on images without fire (red)",
                             "miss": "Misses (true box yellow, not found)"}[name], fill="black", font=ImageFont.load_default(size=20))
        for col, (r, tp, keep, gs) in enumerate(items):
            im = Image.open(r["file"]).convert("RGB")
            d = ImageDraw.Draw(im)
            for g, s in zip(r["gt"], gs):
                if s < 0:
                    d.rectangle(list(g), outline=(255, 220, 0), width=3)
            for b, t, s in zip(r["boxes"][keep], tp, r["scores"][keep]):
                d.rectangle(list(b), outline=(0, 200, 0) if t else (255, 0, 0), width=3)
                d.text((b[0] + 3, max(b[1] - 12, 0)), f"{s:.2f}", fill=(0, 200, 0) if t else (255, 0, 0))
            im.thumbnail((W, H))
            sheet.paste(im, (col * W, y + T))
    sheet.save(out, quality=88)


def size_recall(recs, conf):
    """Recall by box width at threshold conf, with IoU >= 0.5 and with IoU >= 0.25."""
    loose = []
    for r in recs:
        keep = r["scores"] >= conf
        _, _, gs = match(r["boxes"][keep], r["scores"][keep], r["gt"], np.zeros(len(r["gt"]), bool), 0.25)
        loose += list(zip(r["gt"][:, 2] - r["gt"][:, 0], gs))
    return recall_by_width(evaluate(recs)["gt"], conf), recall_by_width(np.array(loose).reshape(-1, 2), conf)


def run(model, name, conf=None):
    out = HERE / "results" / name
    out.mkdir(parents=True, exist_ok=True)
    test = predict(model, name, "test")
    dups = set((DATA / "test_near_duplicates.txt").read_text().split())
    if conf is None:
        conf = best_f1_conf([r for r in predict(model, name, "val") if r["source"] in REAL])
    by = lambda recs, s, new=False: [r for r in recs if r["source"] in s and not (new and r["file"] in dups)]  # noqa: E731
    real, real_new = by(test, REAL), by(test, REAL, True)
    rows, js = [], dict(model=str(model), conf=conf, hfov_deg=HFOV_DEG, f_px=F_PX)
    for label, recs, wr in [("Real test (D-Fire + FASDD + Barman), all images", real, (0, 1e9)),
                            ("Real test, new scenes only", real_new, (0, 1e9)),
                            ("Real test, small fires (width < 32 px)", real, (0, 32)),
                            ("Real test, new scenes, small fires", real_new, (0, 32)),
                            ("Real test, large fires (width >= 32 px)", real, (32, 1e9)),
                            ("Real test, new scenes, large fires", real_new, (32, 1e9)),
                            ("D-Fire test, all", by(test, ("dfire",)), (0, 1e9)), ("D-Fire test, new scenes", by(test, ("dfire",), True), (0, 1e9)),
                            ("FASDD test, all", by(test, ("fasdd",)), (0, 1e9)), ("FASDD test, new scenes", by(test, ("fasdd",), True), (0, 1e9)),
                            ("Barman test (candles, lighters, matches), all", by(test, ("barman",)), (0, 1e9)),
                            ("Synthetic firescenes test", by(test, ("synthetic",)), (0, 1e9))]:
        e = evaluate(recs, *wr)
        p = at_conf(e, conf)
        js[label] = dict(images=len(recs), boxes=e["n_gt"], ap50=e["ap50"], ap5095=e["ap5095"], **p)
        rows.append(f"| {label} | {len(recs)} | {e['n_gt']} | {p['precision']:.3f} | {p['recall']:.3f} | {e['ap50']:.3f} | "
                    f"{e['ap5095']:.3f} |" if e["n_gt"] else f"| {label} | {len(recs)} | 0 | - | - | - | - |")
    neg_rows = []
    for label, recs in [("COCO fire-like (hard negatives)", by(test, ("coco",))), ("D-Fire test, no fire", by(test, ("dfire",))),
                        ("FASDD test, no fire", by(test, ("fasdd",))), ("Real test, new scenes, no fire", real_new),
                        ("Synthetic test, no fire", by(test, ("synthetic",)))]:
        f = fp_per_image(recs, conf)
        js[label] = f
        if f["images"]:
            neg_rows.append(f"| {label} | {f['images']} | {f['fp_per_image']:.3f} | {100 * f['images_with_fp']:.1f} % |")
    bins, bins_loose = size_recall(real, conf)
    nbins, nbins_loose = size_recall(real_new, conf)
    w90 = {f"{k}, recall {int(100 * t)} %": min_width(b, t) for t in (0.9, 0.5)
           for k, b in (("all, IoU 0.5", bins), ("all, IoU 0.25", bins_loose), ("new scenes, IoU 0.5", nbins),
                        ("new scenes, IoU 0.25", nbins_loose))}
    js.update(recall_by_width=bins, recall_by_width_iou025=bins_loose, recall_by_width_new=nbins,
              recall_by_width_new_iou025=nbins_loose, min_width_90=w90)
    dist = lambda w, flame: F_PX * flame / w  # noqa: E731
    text = [f"Model: `{Path(model).name}`. Confidence threshold: {conf:.3f} (best F1 on the real validation images). "
            "IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a "
            "training image (data.py).\n",
            "| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |", "|---|---|---|---|---|---|---|", *rows, "",
            "False positives on images without fire, at the same threshold:\n",
            "| Negative set | Images | False positives per image | Images with a false positive |", "|---|---|---|---|", *neg_rows, "",
            "Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with "
            "IoU >= 0.5 (or >= 0.25):\n",
            "| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | "
            "Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |", "|---|---|---|---|---|---|---|",
            *[f"| {lo}-{hi if hi < 10_000 else 'max'} | {n} | {r:.3f} | {rl:.3f} | {nn} | {nr:.3f} | {nrl:.3f} |"
              for (lo, hi, n, r), (_, _, _, rl), (_, _, nn, nr), (_, _, _, nrl) in zip(bins, bins_loose, nbins, nbins_loose)], "",
            "Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes "
            f"or more), and the largest distance for a flame of that width. Assumed camera: f = {F_PX:.0f} px at 640 x 480 "
            f"(HFOV {HFOV_DEG:.1f} deg, pinhole). Distance d = f x flame width / box width.\n",
            "| Test images, criterion | Box width | 10 cm flame | 30 cm flame |", "|---|---|---|---|",
            *[f"| {k} | {w} px | {dist(w, 0.1):.1f} m | {dist(w, 0.3):.1f} m |" if w else f"| {k} | not reached | - | - |"
              for k, w in w90.items()]]
    (out / "metrics.md").write_text("\n".join(text) + "\n")
    (out / "metrics.json").write_text(json.dumps(js, indent=1, default=float))
    print("\n".join(text))

    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    for label, recs in [("Real test, all", real), ("Real test, new scenes", real_new), ("Barman (candles)", by(test, ("barman",))),
                        ("Synthetic", by(test, ("synthetic",)))]:
        e = evaluate(recs)
        ax[0].plot(e["rec50"], e["prec50"], label=f"{label} (AP50 {e['ap50']:.2f})")
        p = at_conf(e, conf)
        ax[0].plot(p["recall"], p["precision"], "ko", ms=4)
    ax[0].set(xlabel="Recall", ylabel="Precision", xlim=(0, 1), ylim=(0, 1.02), title=f"Precision-recall, IoU 0.5 (dot: threshold {conf:.2f})")
    ax[0].grid(alpha=0.3)
    ax[0].legend(loc="lower left", fontsize=8)
    mids = [f"{lo}-{hi if hi < 10_000 else ''}" for lo, hi, _, _ in bins]
    for b, style, label in ((bins, "o-", "all, IoU 0.5"), (bins_loose, "s--", "all, IoU 0.25"), (nbins, "^-", "new scenes, IoU 0.5"),
                            (nbins_loose, "v--", "new scenes, IoU 0.25")):
        ax[1].plot(mids, [x[3] for x in b], style, label=label)
    ax[1].axhline(0.9, color="grey", lw=0.8)
    ax[1].set(xlabel="True box width (px, 640 px image)", ylabel="Recall", ylim=(0, 1.02), title="Recall by fire size, real test")
    ax[1].tick_params(axis="x", rotation=45)
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "pr_and_size.png", dpi=130)
    grid([r for r in test if r["file"] not in dups], conf, out / "examples.jpg")
    return js


def compare(names):
    """Variant table (from each results/<name>/metrics.json) and training curves (from each run's results.csv)."""
    rows = []
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    for n in names:
        js = json.loads((HERE / "results" / n / "metrics.json").read_text())
        r, nw, s = js["Real test (D-Fire + FASDD + Barman), all images"], js["Real test, new scenes only"], js["Synthetic firescenes test"]
        sm, c = js["Real test, small fires (width < 32 px)"], js["COCO fire-like (hard negatives)"]
        rows.append(f"| {n} | {js['conf']:.2f} | {r['precision']:.3f} | {r['recall']:.3f} | {r['ap50']:.3f} | {r['ap5095']:.3f} | "
                    f"{nw['ap50']:.3f} | {sm['recall']:.3f} | {c['fp_per_image']:.3f} | {s['ap50']:.3f} | "
                    f"{js['min_width_90']['all, IoU 0.25, recall 50 %']} |")
        with open(RUNS / n / "results.csv") as f:
            res = list(csv.DictReader(f))
        ep = [int(x["epoch"]) for x in res]
        ax[0].plot(ep, [float(x["train/box_loss"]) for x in res], label=f"{n} train")
        ax[0].plot(ep, [float(x["val/box_loss"]) for x in res], "--", label=f"{n} val")
        ax[1].plot(ep, [float(x["train/cls_loss"]) for x in res], label=f"{n} train")
        ax[1].plot(ep, [float(x["val/cls_loss"]) for x in res], "--", label=f"{n} val")
        ax[2].plot(ep, [float(x["metrics/mAP50(B)"]) for x in res], label=f"{n} mAP50")
        ax[2].plot(ep, [float(x["metrics/mAP50-95(B)"]) for x in res], "--", label=f"{n} mAP50-95")
    for a, t in zip(ax, ("Box loss", "Class loss", "Validation mAP")):
        a.set(xlabel="Epoch", title=t)
        a.grid(alpha=0.3)
        a.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(HERE / "results/training_curves.png", dpi=130)
    text = ("| Variant | Threshold | Precision | Recall | mAP50 | mAP50-95 | mAP50, new scenes | Recall, small fires | "
            "FP per image, COCO fire-like | Synthetic mAP50 | Smallest width at 50 % recall, IoU 0.25 (px) |\n"
            "|---|---|---|---|---|---|---|---|---|---|---|\n" + "\n".join(rows) + "\n")
    (HERE / "results/variants.md").write_text("Real test = D-Fire + FASDD + Barman test images. IoU 0.5.\n\n" + text)
    print(text)


def deploy_check(models, rknn_preds, conf):
    """The same fixed test subset (from rknn_convert.py) for ONNX models (run here) and RKNN simulator predictions."""
    base = np.load(rknn_preds[0], allow_pickle=True)["recs"]
    files = [(r["source"], r["file"]) for r in base]
    runs = []
    for m in models:
        cache = RUNS / "deploy" / f"{Path(m).resolve().parent.parent.name}_{Path(m).stem}.npz"  # e.g. all416_fire_yolo11n_640x480
        if not cache.exists():
            cache.parent.mkdir(exist_ok=True)
            np.savez_compressed(cache, recs=np.array(predict_files(m, files), dtype=object))
        runs.append((Path(m).name, np.load(cache, allow_pickle=True)["recs"].tolist()))
    for p in rknn_preds:
        recs = []
        for r in np.load(p, allow_pickle=True)["recs"]:
            w, h = Image.open(r["file"]).size
            recs.append(dict(r, gt=load_gt(r["file"], w, h)))
        runs.append((Path(p).stem.replace("preds_", "") + ".rknn (simulator)", recs))
    rows = []
    for label, recs in runs:
        _cache.clear()  # the cache key is the image file: one model at a time
        real = [r for r in recs if r["source"] in REAL]
        e = evaluate(real)
        p = at_conf(e, conf)
        rows.append(f"| {label} | {e['ap50']:.3f} | {e['ap5095']:.3f} | {p['precision']:.3f} | {p['recall']:.3f} | "
                    f"{at_conf(evaluate(real, 0, 32), conf)['recall']:.3f} | "
                    f"{fp_per_image([r for r in recs if r['source'] == 'coco'], conf)['fp_per_image']:.3f} | "
                    f"{evaluate([r for r in recs if r['source'] == 'synthetic'])['ap50']:.3f} |")
    n = {s: sum(1 for x, _ in files if x == s) for s in SOURCES}
    text = (f"Fixed test subset: {', '.join(f'{k} {v}' for k, v in n.items())} images (all test images, near-duplicates included). "
            f"Threshold {conf}. Real = D-Fire + FASDD + Barman.\n\n"
            "| Model | Real mAP50 | Real mAP50-95 | Real precision | Real recall | Real recall, small fires (< 32 px) | "
            "FP per image, COCO fire-like | Synthetic mAP50 |\n|---|---|---|---|---|---|---|---|\n" + "\n".join(rows) + "\n")
    (HERE / "results/deploy_check.md").write_text(text)
    print(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model", nargs="?")
    ap.add_argument("--name")
    ap.add_argument("--conf", type=float, help="confidence threshold (default: best F1 on the validation set)")
    ap.add_argument("--compare", nargs="+")
    ap.add_argument("--deploy", nargs="+", help="ONNX models to compare on the RKNN test subset (with --rknn)")
    ap.add_argument("--rknn", nargs="+", help="preds_*.npz files from rknn_convert.py")
    a = ap.parse_args()
    if a.compare:
        compare(a.compare)
    elif a.rknn:
        deploy_check(a.deploy or [], a.rknn, a.conf)
    else:
        run(a.model, a.name, a.conf)


if __name__ == "__main__":
    main()
