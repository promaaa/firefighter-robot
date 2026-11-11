"""Range test: the robot camera films real flames (candles) at measured distances. Record once, then compare the
detectors on the same clips as often as needed.

Run from the repository root (its environment has scipy for the colour detector), one step at a time:
  uv run python vision/range_test.py record                   # asks you to place each fire, then records the clips
  uv run python vision/range_test.py record --fires none --seconds 60   # the room without fire: false alarms
  uv run python vision/range_test.py label SESSION            # a box around the flames, once per position and mode
  uv run python vision/range_test.py report SESSION [--model FILE] [--tiled FILE] [--detector FILE]
  uv run python vision/range_test.py record --video FILE --yes   # no camera: a video file stands in (to test the script)

record: for each fire and distance (--fires, --distances), it says what to place and waits for Enter. Then it records
--seconds s, after --settle s for the auto exposure, in each camera mode (--modes) and exposure (--exposures). ffmpeg keeps
the camera's own MJPEG frames. Writes ~/runs/firefighter/range_test/<date>/session.json and clips/.
The distance is on the floor, from the point under the camera lens to the candle.
label: shows one frame of each position and mode. Drag a box around the flame or the group of flames (a click is enough
for one small flame), then press Enter. n: no flame visible. q: stop. The boxes go into session.json.
report: 10 frames per second of each clip. The true flame box of a frame is the box of the bright pixels inside the label
box. --model runs a YOLO file on the whole frame, --tiled on 640 x 480 tiles of a larger frame (a digital zoom),
--detector any file with detect(rgb, t), for example sim/sim2sim/baseline_detector.py. Default: --model of detector.py.
Writes report.md, range.png and truth.jpg (each clip: label box yellow, true box green, detections of the first detector
red). det_<name>.json keeps the detections, so the next report is fast. On the robot board, --model with the .rknn file
adds the NPU rows on the same clips.
The clips without a flame also give the confirmed false alarms: the robot walks to a fire only after 3 frames with a box
within 0.6 s (aim/controller.py). The report gives them by threshold, and the lowest threshold without one. --conf sets
the threshold of all other tables: run the report again with that threshold to see the range that it keeps.
"""
import argparse
import importlib.util
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from detector import CONF, MODEL, FireDetector, nms  # noqa: E402
from screen_bench import bright_box, grey, model_name, near  # noqa: E402

RUNS = Path.home() / "runs/firefighter/range_test"
FPS = 10  # frames per second that the report uses: the controller rate (aim/run.py RATE), one detection per step
CONFIRM = (3, 0.6)  # aim/controller.py confirm_frames and confirm_window: frames with a fire within this time, then APPROACH
THRESHOLDS = [0.3, 0.4, 0.5, 0.6, 0.7]  # columns of the confirmed false alarm table


def alarms(has, n=CONFIRM[0], window=CONFIRM[1]):
    """Confirmed alarms in one clip: each time n frames with a box fall within window s, counted once until fewer are left.
    has: one bool per frame, FPS frames per second."""
    k, on, count = round(window * FPS) + 1, False, 0  # the frames within window s of a frame, itself included
    for i in range(len(has)):
        now = sum(has[max(0, i - k + 1):i + 1]) >= n
        count += now and not on
        on = now
    return count


# ---------------------------------------------------------------- record
def set_exposure(a, exp):
    """auto, or a manual exposure_time_absolute value (UVC unit: 100 us)."""
    if not a.video:
        ctl = "auto_exposure=3" if exp == "auto" else f"auto_exposure=1,exposure_time_absolute={exp}"
        subprocess.run(["v4l2-ctl", "-d", a.camera, "-c", ctl], check=False)


def record_clip(a, w, h, out):
    if a.video:  # test without a camera: the video file, scaled to the mode, compressed to MJPEG like the camera
        src, enc = ["-stream_loop", "-1", "-i", a.video], ["-vf", f"scale={w}:{h}", "-c:v", "mjpeg", "-q:v", "3"]
    else:
        src = ["-f", "v4l2", "-input_format", "mjpeg", "-video_size", f"{w}x{h}", "-framerate", str(a.fps), "-i", a.camera]
        enc = ["-c:v", "copy"]
    r = subprocess.run(["ffmpeg", "-loglevel", "error", "-nostdin", "-y", *src, "-t", str(a.settle + a.seconds), *enc, str(out)],
                       capture_output=True, text=True, check=False)
    if r.returncode or not out.exists():
        sys.exit(f"record: ffmpeg failed for {w}x{h}: {r.stderr.strip()[-300:]}\n"
                 f"Check the camera modes: v4l2-ctl -d {a.camera} --list-formats-ext")


def record(a):
    run = RUNS / time.strftime("%Y-%m-%d_%H%M%S")
    (run / "clips").mkdir(parents=True)
    fires = a.fires.split(",")
    steps = [("none", 0.0)] * ("none" in fires) + [(f, float(d)) for f in fires if f != "none" for d in a.distances.split(",")]
    s = dict(date=time.strftime("%Y-%m-%d %H:%M"), camera=a.video or a.camera, settle_s=a.settle, seconds=a.seconds,
             note=a.note, clips=[], labels={})
    if not a.video:
        s["v4l2"] = subprocess.run(["v4l2-ctl", "-d", a.camera, "--all", "--list-formats-ext"], capture_output=True, text=True,
                                   check=False).stdout
    for k, (fire, d) in enumerate(steps):
        what = "the view without any flame" if fire == "none" else f"{fire} at {d:g} m"
        if not a.yes:
            key = input(f"Step {k + 1}/{len(steps)}: {what}. Step out of the image, then press Enter (s: skip, q: stop): ")
            if key.strip().lower() == "q":
                break
            if key.strip().lower() == "s":
                continue
        for mode in a.modes.split(","):
            w, h = (int(v) for v in mode.split("x"))
            for e in a.exposures.split(","):
                set_exposure(a, e)
                name = f"{k:02d}_{fire}_{d:g}m_{mode}_{e}.mkv"
                record_clip(a, w, h, run / "clips" / name)
                s["clips"].append(dict(file=name, step=k, fire=fire, distance_m=d, width=w, height=h, exposure=e))
                print(f"  {name}", flush=True)
        (run / "session.json").write_text(json.dumps(s, indent=1))
    set_exposure(a, "auto")
    print(f"wrote {run}\nNext: uv run python vision/range_test.py label {run}")


# ---------------------------------------------------------------- label
def frames(run, c, settle, fps=FPS):
    """RGB frames of a clip (N x H x W x 3), fps per second, after the first settle s."""
    p = subprocess.run(["ffmpeg", "-loglevel", "error", "-ss", str(settle), "-i", str(run / "clips" / c["file"]), "-vf", f"fps={fps}",
                        "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True, check=True)
    return np.frombuffer(p.stdout, np.uint8).reshape(-1, c["height"], c["width"], 3)


def key(c):
    return f"{c['step']}_{c['width']}x{c['height']}"


def label(a):
    import tkinter as tk

    from PIL import ImageTk
    run = Path(a.session).expanduser()
    s = json.loads((run / "session.json").read_text())
    todo = {}
    for c in s["clips"]:  # the first clip of each position and mode: the camera and the flames do not move
        if c["fire"] != "none" and (a.redo or key(c) not in s["labels"]):
            todo.setdefault(key(c), c)
    root = tk.Tk(className="range-test")
    canvas = tk.Canvas(root, highlightthickness=0, cursor="crosshair")
    canvas.pack()
    for n, (k, c) in enumerate(todo.items()):
        f = frames(run, c, s["settle_s"], fps=1)
        z = min(1280 / c["width"], 900 / c["height"])  # display zoom: 2 for 640 x 480
        img = ImageTk.PhotoImage(Image.fromarray(f[len(f) // 2]).resize((round(c["width"] * z), round(c["height"] * z))))
        canvas.delete("all")
        canvas.configure(width=img.width(), height=img.height())
        canvas.create_image(0, 0, anchor="nw", image=img)
        rect = canvas.create_rectangle(0, 0, 0, 0, outline="yellow", width=2)
        root.title(f"{n + 1}/{len(todo)}: {c['fire']} at {c['distance_m']:g} m, {c['width']}x{c['height']}. Box around the flames, "
                   "then Enter. n: no flame visible. q: stop")
        box, done = [], []
        canvas.bind("<ButtonPress-1>", lambda e, box=box: box.__setitem__(slice(None), [e.x, e.y, e.x, e.y]))
        canvas.bind("<B1-Motion>", lambda e, box=box, rect=rect: (box.__setitem__(slice(2, None), [e.x, e.y]), canvas.coords(rect, *box)))
        for k_, v in (("<Return>", "ok"), ("n", "none"), ("q", "stop")):
            root.bind(k_, lambda e, v=v, done=done: done.append(v))
        while not done:
            root.update()
            time.sleep(0.02)
        if done[0] == "stop":
            break
        if done[0] == "ok" and box:
            x0, x1 = sorted((box[0] / z, box[2] / z))
            y0, y1 = sorted((box[1] / z, box[3] / z))
            g = 6 * c["width"] / 640 - min(x1 - x0, y1 - y0) / 2  # a click: a box of 12 px (at 640 x 480) around it
            if g > 0:
                x0, y0, x1, y1 = x0 - g, y0 - g, x1 + g, y1 + g
            s["labels"][k] = [round(v, 1) for v in (x0, y0, x1, y1)]
        else:
            s["labels"][k] = None
        (run / "session.json").write_text(json.dumps(s, indent=1))
    root.destroy()
    print(f"{sum(v is not None for v in s['labels'].values())} positions with a label box. "
          f"Next: uv run python vision/range_test.py report {run}")


# ---------------------------------------------------------------- report
def truth(f, roi):
    """True flame box: the bright pixels inside the label box (grown by 4 px at 640 x 480). The label box itself when the
    flame is too faint there. None without a label."""
    if roi is None:
        return None
    h, w = f.shape[:2]
    p = 4 * w / 640
    region = (max(0, int(roi[0] - p)), max(0, int(roi[1] - p)), min(w, math.ceil(roi[2] + p)), min(h, math.ceil(roi[3] + p)))
    return bright_box(grey(f), region, contrast=30) or [round(v) for v in roi]


def tiled(det, rgb, conf, tw=640, th=480):
    """Boxes of det on tw x th tiles of a larger image (each tile at full resolution), merged with NMS."""
    h, w = rgb.shape[:2]
    if w <= tw and h <= th:
        return det.boxes(rgb, conf)
    bs, ss = [], []
    for y in np.linspace(0, h - th, math.ceil(h / th)).astype(int):
        for x in np.linspace(0, w - tw, math.ceil(w / tw)).astype(int):
            b, sc = det.boxes(rgb[y:y + th, x:x + tw], conf)
            bs.append(b + [x, y, x, y])
            ss.append(sc)
    b, sc = np.concatenate(bs), np.concatenate(ss)
    k = nms(b, sc, 0.5) if len(sc) else np.zeros(0, int)
    return b[k], sc[k]


def detectors(a):
    """name -> function(rgb) -> list of [x0, y0, x1, y1, score]. YOLO files keep the boxes from 0.1, the rest uses CONF."""
    out = {}
    for m in a.model or ([] if a.tiled or a.detector else [str(MODEL)]):
        det = FireDetector(model=m, calib=None)
        out[model_name(m)] = lambda rgb, det=det: np.c_[det.boxes(rgb, 0.1)]
    for m in a.tiled or []:
        det = FireDetector(model=m, calib=None)
        out[model_name(m) + "+tiles"] = lambda rgb, det=det: np.c_[tiled(det, rgb, 0.1)]
    for f in a.detector or []:
        spec = importlib.util.spec_from_file_location(Path(f).stem, f)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        out[Path(f).stem] = lambda rgb, mod=mod: [[*d.bbox, max(d.confidence, CONF)] for d in mod.detect(rgb, 0.0)]
    return out


def report(a):
    global CONF
    CONF = a.conf  # the threshold of the tables, the tiles and the --detector boxes
    run = Path(a.session).expanduser()
    s = json.loads((run / "session.json").read_text())
    dets = detectors(a)
    cache = {n: json.loads(p.read_text()) if (p := run / f"det_{n}.json").exists() else {} for n in dets}
    rows, sheet = [], []
    for c in s["clips"]:
        f = frames(run, c, s["settle_s"])
        if not len(f):
            print(f"skipped {c['file']}: no frame after {s['settle_s']} s")
            continue
        roi = s["labels"].get(key(c))
        tr = [truth(x, roi) for x in f]
        for n, fn in dets.items():
            if c["file"] not in cache[n]:
                cache[n][c["file"]] = [[[round(float(v), 1) for v in b] for b in fn(x)] for x in f]
            per = [[b for b in fr if b[4] >= CONF] for fr in cache[n][c["file"]]]
            pad = 2 * c["width"] / 640
            hit = [t is not None and any(near(b, t, pad) for b in bs) for bs, t in zip(per, tr)]
            away = [any(t is None or not near(b, t, pad) for b in bs) for bs, t in zip(per, tr)]
            rows.append(dict(c, det=n, mode=f"{c['width']}x{c['height']}", frames=len(f), labelled=roi is not None,
                             hit=float(np.mean(hit)), away=float(np.mean(away)), any=float(np.mean([bool(b) for b in per])),
                             best=[max((b[4] for b in fr), default=0.0) for fr in cache[n][c["file"]]],  # best score per frame
                             tw=float(np.median([t[2] - t[0] for t in tr])) if roi else None,
                             th=float(np.median([t[3] - t[1] for t in tr])) if roi else None))
        sheet.append(tile(f[len(f) // 2], roi, tr[len(f) // 2], cache[next(iter(dets))][c["file"]][len(f) // 2], c))
    for n in dets:
        (run / f"det_{n}.json").write_text(json.dumps(cache[n]))
    write(run, s, rows, sheet)


def tile(f, roi, t, boxes, c, size=(320, 240)):
    """One frame of a clip: zoomed on the flame (3 x the 640 x 480 scale) when there is a label box, else the whole frame."""
    im = Image.fromarray(f)
    d = ImageDraw.Draw(im)
    lw = max(1, c["width"] // 640)
    for b in boxes:
        if b[4] >= CONF:
            d.rectangle(b[:4], outline=(255, 0, 0), width=lw)
    if roi:
        d.rectangle(roi, outline=(255, 255, 0), width=lw)
        d.rectangle(t, outline=(0, 255, 0), width=lw)
        cx, cy, hw, hh = (t[0] + t[2]) / 2, (t[1] + t[3]) / 2, c["width"] / 6, c["height"] / 6
        im = im.crop(tuple(round(v) for v in (cx - hw, cy - hh, cx + hw, cy + hh)))
    im = im.resize(size, Image.NEAREST)
    ImageDraw.Draw(im).text((4, 4), f"{c['fire']} {c['distance_m']:g} m {c['width']}x{c['height']} {c['exposure']}", fill=(255, 255, 0))
    return im


def write(run, s, rows, sheet):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fires = [f for f in dict.fromkeys(r["fire"] for r in rows) if f != "none"]
    combos = list(dict.fromkeys((r["det"], r["mode"], r["exposure"]) for r in rows))
    pct = lambda v: "-" if v is None else f"{100 * v:.0f} %"  # noqa: E731
    L = [f"# Range test {s['date']}", "", (f"Camera {s['camera']}. {len(s['clips'])} clips of {s['seconds']} s, {FPS} frames per "
                                           f"second after {s['settle_s']} s for the exposure. Detector threshold {CONF}. {s.get('note', '')}"), "",
         "Detection rate: frames with a box on the true flame box. Width and height: the median true flame box, camera px.", ""]
    for fire in fires:
        sel = [r for r in rows if r["fire"] == fire]
        dists = sorted(dict.fromkeys(r["distance_m"] for r in sel))
        modes = list(dict.fromkeys(r["mode"] for r in sel))
        L += [f"## {fire}", "", "| Distance (m) | " + " | ".join(f"Width x height, {m}" for m in modes) + " | "
              + " | ".join(f"{d}, {m}, {e}" for d, m, e in combos) + " |", "|---|" + "---|" * (len(modes) + len(combos))]
        def one(dist, **k):
            return next((r for r in sel if r["distance_m"] == dist and all(r[x] == v for x, v in k.items())), None)  # noqa: B023
        for dist in dists:
            size = [one(dist, mode=m) for m in modes]
            L.append(f"| {dist:g} | " + " | ".join(f"{r['tw']:.0f} x {r['th']:.0f}" if r and r["tw"] is not None else "no label" for r in size)
                     + " | " + " | ".join(pct(r["hit"]) if (r := one(dist, det=d, mode=m, exposure=e)) and r["labelled"] else "-"
                                         for d, m, e in combos) + " |")
        far = []
        for d, m, e in combos:
            ok = {r["distance_m"]: r["hit"] >= 0.5 for r in sel if (r["det"], r["mode"], r["exposure"]) == (d, m, e) and r["labelled"]}
            far.append(max((x for x in ok if all(ok[y] for y in ok if y <= x)), default=None))
        L += ["| 50 % or more up to | " + " | " * len(modes) + " | ".join(f"{x:g} m" if x else "-" for x in far) + " |", ""]
    L += ["## False alarms", "", "No flame: frames with any box. With a flame: frames with a box away from the true flame.", "",
          "| Detector, mode, exposure | No flame | With a flame |", "|---|---|---|"]
    for d, m, e in combos:
        sel = [r for r in rows if (r["det"], r["mode"], r["exposure"]) == (d, m, e)]
        no = [r["any"] * r["frames"] for r in sel if r["fire"] == "none"]
        yes = [r["away"] * r["frames"] for r in sel if r["fire"] != "none" and r["labelled"]]
        nf = sum(r["frames"] for r in sel if r["fire"] == "none")
        yf = sum(r["frames"] for r in sel if r["fire"] != "none" and r["labelled"])
        L.append(f"| {d}, {m}, {e} | " + (f"{100 * sum(no) / nf:.0f} % of {nf}" if nf else "-") + " | "
                 + (f"{100 * sum(yes) / yf:.0f} % of {yf}" if yf else "-") + " |")
    none = [r for r in rows if r["fire"] == "none"]
    if none:
        L += ["", "## Confirmed false alarms", "", (f"No flame: each time {CONFIRM[0]} frames with a box fall within {CONFIRM[1]} s. "
              "Then the robot walks to the fire (aim/controller.py). Per minute of video, by threshold. It is an upper bound: the "
              "controller also needs the boxes at one place on the floor. Choose the threshold here, not from single frames."), "",
              "| Detector, mode, exposure | Minutes | " + " | ".join(f"Threshold {t:g}" for t in THRESHOLDS)
              + " | Lowest threshold without one |", "|---|---|" + "---|" * (len(THRESHOLDS) + 1)]
        for d, m, e in combos:
            sel = [r for r in none if (r["det"], r["mode"], r["exposure"]) == (d, m, e)]
            if not sel:
                continue
            mins = sum(r["frames"] for r in sel) / FPS / 60
            count = lambda t: sum(alarms([b >= t for b in r["best"]]) for r in sel)  # noqa: E731, B023
            low = next((t for t in np.round(np.arange(0.1, 0.96, 0.05), 2) if count(t) == 0), None)
            L.append(f"| {d}, {m}, {e} | {mins:.1f} | " + " | ".join(f"{count(t) / mins:.1f}" for t in THRESHOLDS)
                     + f" | {f'{low:.2f}' if low is not None else 'over 0.95'} |")
    (run / "report.md").write_text("\n".join(L) + "\n")
    if fires:
        fig, axs = plt.subplots(1, len(fires), figsize=(5 * len(fires), 3.6), squeeze=False)
        for ax, fire in zip(axs[0], fires):
            for d, m, e in combos:
                pts = sorted((r["distance_m"], r["hit"]) for r in rows if r["fire"] == fire and r["labelled"]
                             and (r["det"], r["mode"], r["exposure"]) == (d, m, e))
                if pts:
                    ax.plot(*zip(*pts), "o-" if e == "auto" else "s--", label=f"{d}, {m}, {e}")
            ax.set(title=fire, xlabel="distance (m)", ylabel="frames with the flame detected", ylim=(-0.03, 1.03))
            ax.grid(alpha=0.3)
        axs[0][-1].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(run / "range.png", dpi=150)
    cols = 6
    im = Image.new("RGB", (320 * cols, 240 * -(-len(sheet) // cols)))
    for i, t in enumerate(sheet):
        im.paste(t, (320 * (i % cols), 240 * (i // cols)))
    im.save(run / "truth.jpg", quality=90)
    print(f"wrote {run / 'report.md'}, range.png, truth.jpg")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record", help="place each fire, record the clips")
    r.add_argument("--camera", default="/dev/video0")
    r.add_argument("--video", help="a video file instead of the camera (to test the script)")
    r.add_argument("--fires", default="none,candle,group", help="comma list. none: the view without a flame")
    r.add_argument("--distances", default="0.3,0.5,1,1.5,2,3", help="m, on the floor from the point under the lens")
    r.add_argument("--modes", default="640x480,1920x1080", help="camera modes, WxH (v4l2-ctl --list-formats-ext)")
    # 1 ms (10) and 0.3 ms (3): on the screen bench (6 Nov) 1 ms found about twice as many flames as auto. A real flame is
    # much brighter than the screen, so the shorter one checks whether the flame core still saturates at 1 ms.
    r.add_argument("--exposures", default="auto,10,3", help="auto or exposure_time_absolute values (100 us units)")
    r.add_argument("--seconds", type=float, default=4.0)
    r.add_argument("--settle", type=float, default=2.0, help="s before each clip for the auto exposure (the report skips them)")
    r.add_argument("--fps", type=int, default=30)
    r.add_argument("--note", default="", help="room, light, candle type: goes into the report")
    r.add_argument("--yes", action="store_true", help="do not wait for Enter at each step")
    lb = sub.add_parser("label", help="a box around the flames, once per position and mode")
    lb.add_argument("session")
    lb.add_argument("--redo", action="store_true", help="label again the positions that have a box")
    p = sub.add_parser("report", help="run the detectors on the clips and write report.md, range.png, truth.jpg")
    p.add_argument("session")
    p.add_argument("--model", action="append", help="YOLO file (.onnx, or .rknn on the board) on the whole frame. Repeat for more")
    p.add_argument("--tiled", action="append", help="YOLO file on 640 x 480 tiles of larger frames")
    p.add_argument("--detector", action="append", help="file with detect(rgb, t)")
    p.add_argument("--conf", type=float, default=CONF, help=f"detector threshold of the tables (default {CONF}, from 0.1)")
    a = ap.parse_args()
    {"record": record, "label": label, "report": report}[a.cmd](a)


if __name__ == "__main__":
    main()
