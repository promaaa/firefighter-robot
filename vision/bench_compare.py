"""Compare models on screen bench runs: detection rate of candle and tea-light flames in four width ranges, and LED-candle
frames with a box. Reads the det_<model>.json files that `screen_bench.py report --model` wrote into each run folder.
Each model uses its own threshold (best F1 on the validation images, results/<run>/metrics.json), or with --led the
lowest threshold at which at most that share of the LED-candle frames has a box (all models then have the same false alarms).

Run from vision/:
  uv run python bench_compare.py RUN_DIR [RUN_DIR ...]      # prints one markdown table per run
  uv run python bench_compare.py RUN_DIR --videos Jahobr,GilesLaurent,flameless_candle_1   # only these videos (train.py FOLDS)
  uv run python bench_compare.py RUN_DIR --led 0.02         # each model at the threshold that gives 2 % LED-candle frames with a box
"""
import argparse
import json
from pathlib import Path

import numpy as np

from screen_bench import CONF, HERE, near

RANGES = [(4, 8), (8, 16), (16, 32), (32, 128)]  # true flame width in camera px, from <= w < to


def threshold(model):
    """The model's own threshold. det_fire_yolo11n_* (the file in models/) is all416."""
    run = model.split("_fire_yolo11n")[0] if "_fire_yolo11n" in model else "all416"
    m = HERE / "results" / run / "metrics.json"
    return json.loads(m.read_text())["conf"] if m.exists() else CONF


def trials(run, videos=None):
    """The valid trials of a run. videos: only the source videos whose file name holds one of these strings."""
    src = [s["video"] for s in json.loads((Path(run) / "setup.json").read_text())["stimuli"]]
    return [t for t in map(json.loads, open(Path(run) / "trials.jsonl"))
            if t["ok"] and (not videos or any(v in src[t["stim"]] for v in videos))]


def scores(run, model, videos=None):
    """One (fire, true width, score) per frame. Score: the best box at the flame (fire image) or anywhere (LED candle), else 0."""
    d = json.loads((Path(run) / f"det_{model}.json").read_text())
    return [(t["fire"], t["truth"][2] - t["truth"][0], max((b[4] for b in d[f] if not t["fire"] or near(b, t["truth"])), default=0))
            for t in trials(run, videos) for f in t["frames"]]


def led_threshold(run, model, led, videos=None):
    """Lowest threshold at which at most the share led of the LED-candle frames has a box. Not under 0.1: the
    det files keep only boxes from 0.1."""
    s = sorted((x[2] for x in scores(run, model, videos) if not x[0]), reverse=True)
    k = int(led * len(s))
    return max(0.1, s[k] + 1e-6) if k < len(s) else 0.1


def rates(run, model, videos=None, conf=None):
    """{(from, to): share of fire frames found, "led": share of LED-candle frames with a box} (None: no frame).
    conf: the threshold (default: the model's own)."""
    conf = threshold(model) if conf is None else conf
    s = scores(run, model, videos)
    sel = {r: [x[2] >= conf for x in s if x[0] and r[0] <= x[1] < r[1]] for r in RANGES}
    sel["led"] = [x[2] >= conf for x in s if not x[0]]
    return {k: float(np.mean(v)) if v else None for k, v in sel.items()}


def table(run, videos=None, led=None):
    run = Path(run)
    only = f", only the videos with {' or '.join(videos)} in the file name" if videos else ""
    at = f"the threshold that gives at most {100 * led:.0f} % LED-candle frames with a box" if led is not None else "its own threshold"
    lines = [f"Run {run.name}{only}: {len(trials(run, videos))} images. Fire frames found (candle and tea light), each model at {at}.", "",
             "| Model | " + " | ".join(f"flame {a} to {b - 1} px" for a, b in RANGES) + " | LED candle frames with a box |",
             "|---|" + "---|" * (len(RANGES) + 1)]
    for p in sorted(run.glob("det_*.json")):
        m = p.stem[4:]
        conf = threshold(m) if led is None else led_threshold(run, m, led, videos)
        r = rates(run, m, videos, conf)
        cells = [f"{100 * r[k]:.0f} %" if r[k] is not None else "-" for k in [*RANGES, "led"]]
        lines.append(f"| {m} ({conf:.2f}) | " + " | ".join(cells) + " |")
    return "\n".join(lines)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--videos", type=lambda s: s.split(","), help="comma-separated parts of video file names")
    ap.add_argument("--led", type=float, help="share of LED-candle frames with a box that sets each model's threshold, e.g. 0.02")
    a = ap.parse_args()
    for r in a.runs:
        print(table(r, a.videos, a.led), "\n")
