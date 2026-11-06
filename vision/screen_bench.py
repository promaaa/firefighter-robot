"""Screen bench: the robot camera films fire images on a screen, to test the detector with the real camera without a fire.

Run from vision/:
  uv run python screen_bench.py run                        # camera /dev/video0 films the focused screen
  uv run python screen_bench.py run --monitor DP-2         # show the images on this screen (names: hyprctl monitors)
  uv run python screen_bench.py run --monitor DP-2 --repeat 0   # setup and delay only (1 min): check setup.jpg
  uv run python screen_bench.py run --monitor DP-2 --stimuli v2 --repeat 3   # 8 different flames instead of 2
  uv run python screen_bench.py run --fake --repeat 1      # no camera, no window: an ideal camera sees the screen image
  uv run python screen_bench.py report RUN_DIR [--model FILE ...] [--baseline OTHER_RUN_DIR]

run, in this order:
1. Setup. A white screen gives the lit screen pixels (screen_mask.png) and their box in the camera image, a white square
   gives the scale (camera px per screen px). The bright boxes look only at the lit screen pixels, not at the desk or wall.
   setup.jpg shows the white screen and one candle: check the framing, the focus and the exposure there.
2. Delay. Black to white 20 times: the time from the screen change to the first bright camera frame. It is an upper bound
   of config.toml [camera] latency_s, because it also counts the screen delay (about 1 to 3 frames at 60 Hz).
3. Sweep. Candle, tea light and LED-candle (no fire) images at 12 flame widths and 9 screen positions, in a random order.
   Before each image, a white box on black shows where the flame goes. Its camera image minus the camera image of a black
   screen gives the true box, with the lens distortion and the blur. Then the camera saves --frames frames of the image.
   --exposure fixes the exposure time during the run (default: automatic exposure). It is set back to automatic at the end.
The flame box of each source image comes from the detector itself on the full-size image (score >= 0.5). On the LED
candles it is the best box at score >= 0.05: the place where the detector is most tempted.
Writes ~/runs/firefighter/screen_bench/<date>/: setup.jpg, setup.json, stimuli.jpg, trials.jsonl, frames/, report.md.

report: runs each model on the saved frames (det_<model>.json, kept for the next report) and writes report.md: detection
rate against the true flame width, by position, and the false alarms. report.png plots the detection rate, curves.json
keeps its points. --baseline OTHER_RUN adds the curves of another run, for example the ideal camera of a --fake run.
It needs only the detector packages, so on the robot board `report RUN_DIR --model models/fire_yolo11n_416x320_int8.rknn`
adds the NPU rows on the same frames.
"""
import argparse
import io
import json
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "robot")]
from detector import CALIB, CONF, MODEL, FireDetector  # noqa: E402

W, H = 640, 480  # camera image
VIDEOS = Path.home() / "datasets/firefighter/videos"
RUNS = Path.home() / "runs/firefighter/screen_bench"
SIZES = [2, 3, 4, 6, 8, 11, 16, 23, 32, 45, 64, 90]  # flame width in camera px (aimed: the true width is measured)
GRID = [0.1, 0.5, 0.9]  # flame centre on the screen, share of the width and of the height
STIMULI_SETS = {  # class, video in VIDEOS (results/video_sources.md), fire?, images to keep, from and to (s, None: the end)
    "v1": [  # 3 Nov: two flames, six images each
        ("candle", "candle_and_match_LoKiLeCh_CCBYSA3.ogv", True, 6, 0, None),
        ("tea_light", "candle_blow_GilesLaurent_CCBYSA4.webm", True, 6, 7.5, 15),  # before: a lighter flame, after: smoke
        ("led_candle", "flameless_candle_1_CC0.webm", False, 3, 0, None),
        ("led_candle", "flameless_candle_2_CC0.webm", False, 3, 0, None),
    ],
    "v2": [  # 5 Nov: eight flames (six candles, two tea lights), two images each
        ("candle", "candle_and_match_LoKiLeCh_CCBYSA3.ogv", True, 2, 0, None),
        ("candle", "blue_candle_Jahobr_CC0.webm", True, 2, 0, None),
        ("candle", "pink_candle_Jahobr_CC0.webm", True, 2, 0, None),
        ("candle", "white_candle_Jahobr_CC0.webm", True, 2, 0, None),
        ("candle", "candle_blown_out_CCBYSA3.ogv", True, 2, 0, None),
        ("candle", "candle_shadow_Rhetos_CCBYSA4.webm", True, 2, 0, None),
        ("tea_light", "candle_blow_GilesLaurent_CCBYSA4.webm", True, 2, 7.5, 15),
        ("tea_light", "candle_suffocated_CCBYSA3.ogv", True, 2, 0, None),
        ("led_candle", "flameless_candle_1_CC0.webm", False, 2, 0, None),
        ("led_candle", "flameless_candle_2_CC0.webm", False, 2, 0, None),
    ],
}
K_FAKE = 0.3  # --fake: camera px per screen px (a 1920 x 1080 screen fills 576 x 324 px of the camera image)
F_PX = json.loads(CALIB.read_text())["fx"] if CALIB.exists() else 283  # B0201 focal length at 640 x 480 (calibrate.py, else the old model value)
BOX_M = {"candle": 0.010, "tea_light": 0.007}  # flame box width, m. Tea light: 65 px box on its 38 mm cup (350 px). Candle: assumed


def video_frames(path, t0, t1, n=24):
    """n frames spread from t0 to t1 s (None: the end) of a video, at most 1280 px wide, without the black borders."""
    t1 = t1 or float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)]))
    out = []
    for k in range(n):
        png = subprocess.check_output(["ffmpeg", "-v", "error", "-ss", f"{t0 + (t1 - t0) * (k + 0.5) / n:.2f}", "-i", str(path), "-frames:v", "1",
                                       "-vf", "scale='min(1280,iw)':-2", "-f", "image2pipe", "-c:v", "png", "-"])
        img = np.asarray(Image.open(io.BytesIO(png)).convert("RGB"))
        rows, cols = np.flatnonzero(img.max((1, 2)) > 16), np.flatnonzero(img.max((0, 2)) > 16)
        if len(rows) and len(cols):
            out.append(img[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1])
    return out


def stimuli(det, out, entries):
    """Source images with their flame box (x0, y0, x1, y1, source px). Writes stimuli.jpg with the boxes in green."""
    found = []
    for cls, video, fire, keep, t0, t1 in entries:
        hits = []
        for img in video_frames(VIDEOS / video, t0, t1):
            b, _ = det.boxes(img, conf=0.5 if fire else 0.05)
            # a fire image shows one flame: no other box, even a weak one (candle_and_match: the match flame at the top)
            if len(b) and b[0][2] - b[0][0] >= 4 and (not fire or all(near(x, b[0], 4) for x in det.boxes(img, conf=0.1)[0])):
                hits.append(dict(cls=cls, video=video, fire=fire, img=Image.fromarray(img), box=[round(float(v), 1) for v in b[0]]))
        found += hits[::max(1, len(hits) // keep)][:keep]
    sheet = Image.new("RGB", (6 * 320, 240 * -(-len(found) // 6)))
    for i, s in enumerate(found):
        t = s["img"].copy()
        ImageDraw.Draw(t).rectangle(s["box"], outline=(0, 255, 0), width=max(2, t.width // 200))
        t.thumbnail((320, 240))
        sheet.paste(t, (320 * (i % 6), 240 * (i // 6)))
    sheet.save(out / "stimuli.jpg", quality=90)
    return found


def target_box(box, width, gx, gy, k, screen):
    """Screen box of a flame `width` camera px wide with the shape of box, centred at (gx, gy) of the screen and kept on it.
    None when it does not fit on the screen."""
    w = width / k
    h = w * (box[3] - box[1]) / (box[2] - box[0])
    if w > screen[0] or h > screen[1]:
        return None
    cx = min(max(gx * screen[0], w / 2), screen[0] - w / 2)
    cy = min(max(gy * screen[1], h / 2), screen[1] - h / 2)
    return [cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2]


def compose(img, box, target, screen):
    """Screen image: img scaled and moved so that box (source px) lands on target (screen px). The rest of the screen has
    the median colour of the image border, so a small image does not show as a box on the screen."""
    s = (target[2] - target[0]) / (box[2] - box[0])
    ox, oy = target[0] - box[0] * s, target[1] - box[1] * s  # screen position of the source origin
    src = (max(0, -ox / s), max(0, -oy / s), min(img.width, (screen[0] - ox) / s), min(img.height, (screen[1] - oy) / s))
    dst = [round(ox + src[0] * s), round(oy + src[1] * s), round(ox + src[2] * s), round(oy + src[3] * s)]
    a = np.asarray(img)
    out = Image.new("RGB", screen, tuple(int(v) for v in np.median(np.concatenate([a[0], a[-1], a[:, 0], a[:, -1]]), 0)))
    out.paste(img.resize((dst[2] - dst[0], dst[3] - dst[1]), Image.LANCZOS, box=src), dst[:2])
    return out


def mask(target, screen):
    """Black screen with the target box in white."""
    m = Image.new("RGB", screen)
    x0, y0, x1, y1 = (round(v) for v in target)
    ImageDraw.Draw(m).rectangle([x0, y0, x1 - 1, y1 - 1], fill="white")
    return m


def bright_box(g, region=(0, 0, W, H), contrast=40):
    """Box (x0, y0, x1, y1, camera px) of the pixels of g (H x W) in region that are above half-way from the median to the
    maximum. None when the maximum is less than `contrast` above the median."""
    x0, y0, x1, y1 = region
    g = g[y0:y1, x0:x1]
    lo, hi = np.median(g), g.max()
    if hi - lo < contrast:
        return None
    ys, xs = np.nonzero(g > (lo + hi) / 2)
    return [int(xs.min()) + x0, int(ys.min()) + y0, int(xs.max()) + x0 + 1, int(ys.max()) + y0 + 1]


def grey(frame):
    return frame.astype(np.float32).mean(-1)


def true_box(rig, target, settle):
    """Camera box of the white target box: the bright pixels of (mask frame - black frame). The difference removes what both
    frames show, for example the mouse pointer, or the bright wall that a camera, after it turned, sees inside the screen
    mask of the setup (5 Nov run: the camera turned 18 px after 20 min, and 85 % of the true boxes were wrong)."""
    black = grey(rig.look(Image.new("RGB", rig.size), settle)[0])
    return bright_box(grey(rig.look(mask(target, rig.size), settle)[0]) - black)


class FakeRig:
    """No camera and no window: an ideal camera sees the screen image, scaled by K_FAKE and centred."""
    size = (1920, 1080)

    def look(self, img, settle=0.0, n=1):
        w, h = round(self.size[0] * K_FAKE), round(self.size[1] * K_FAKE)
        f = Image.new("RGB", (W, H), (20, 20, 20))
        f.paste(img.resize((w, h), Image.BOX), ((W - w) // 2, (H - h) // 2))
        return [np.asarray(f)] * n

    def close(self):
        pass


class Rig:
    """A full-screen Tk window and the camera (robot/camera_node.py, through ffmpeg)."""

    def __init__(self, camera, monitor=None):
        import tkinter as tk

        from camera_node import open_source
        if monitor:  # Hyprland opens a new window on the focused monitor (Lua dispatcher, Hyprland 0.56)
            r = subprocess.run(["hyprctl", "dispatch", f'hl.dsp.focus({{ monitor = "{monitor}" }})'], capture_output=True, text=True)
            if r.stdout.strip() != "ok":
                sys.exit(f"cannot focus the monitor {monitor}: {r.stdout.strip()} (names: hyprctl monitors)")
        # Hyprland rule until the next config reload: full screen, opaque, no border, no screen lock while the window is open
        subprocess.run(["hyprctl", "eval", 'hl.window_rule({ match = { class = "^[Ss]creen-bench$" }, tag = "-default-opacity", '
                        'fullscreen = true, opacity = "1 1", border_size = 0, idle_inhibit = "always" })'], capture_output=True)
        self.root = tk.Tk(className="screen-bench")
        self.root.configure(bg="black", cursor="none")
        self.root.attributes("-fullscreen", True)
        self.label = tk.Label(self.root, bd=0, bg="black")
        self.label.pack(fill="both", expand=True)
        for _ in range(20):  # time for the window manager to make the window full screen
            self.root.update()
            time.sleep(0.1)
        self.size = self.root.winfo_width(), self.root.winfo_height()
        mons = json.loads(subprocess.run(["hyprctl", "monitors", "-j"], capture_output=True, text=True).stdout or "[]")
        if monitor and (other := [m["name"] for m in mons if m["name"] != monitor]):  # the mouse pointer was in the camera image
            subprocess.run(["hyprctl", "dispatch", f'hl.dsp.focus({{ monitor = "{other[0]}" }})'], capture_output=True)
        _, self.cam, _ = open_source(f"real:{camera}")

    def show(self, img):
        from PIL import ImageTk
        self.photo = ImageTk.PhotoImage(img)
        self.label.configure(image=self.photo)
        self.root.update()
        return time.monotonic()

    def grab(self, after):
        """The first camera frame that arrives after `after` (monotonic s): (arrival time, H x W x 3 uint8)."""
        c, end = self.cam, time.monotonic() + 5.0
        while time.monotonic() < end:
            self.root.update()  # the window keeps answering the window manager
            with c.cond:
                c.cond.wait_for(lambda: c.ended or c.t > after, timeout=0.05)
                if c.t > after:
                    return c.t, c.frame
                if c.ended:
                    break
        raise RuntimeError(f"camera: no frame for 5 s (ffmpeg exit code {c.proc.poll()})")

    def look(self, img, settle, n=1):
        """Show img, then return the first n camera frames that arrive `settle` s later (auto exposure)."""
        t = self.show(img) + settle
        out = []
        for _ in range(n):
            t, f = self.grab(t)
            out.append(f)
        return out

    def close(self):
        self.cam.close()
        self.root.destroy()


def setup(rig, out, sample, settle):
    """Screen box in the camera image, scale (camera px per screen px) and the lit screen pixels (H x W bool). Writes setup.jpg."""
    dark = grey(rig.look(Image.new("RGB", rig.size), 2 * settle)[0])
    white = rig.look(Image.new("RGB", rig.size, "white"), 2 * settle)[0]
    d = grey(white) - dark
    on = d > d.max() / 2  # the lit screen. With the lens distortion, the box around it also holds the bezel, desk and wall
    ys, xs = np.nonzero(on)
    if d.max() < 40 or len(xs) < 0.02 * W * H:
        Image.fromarray(white).save(out / "setup.jpg")
        sys.exit(f"setup: the camera does not see the screen. Look at {out / 'setup.jpg'}")
    screen = [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]
    side = rig.size[1] // 4
    x0, y0 = (rig.size[0] - side) // 2, (rig.size[1] - side) // 2
    b = bright_box(grey(rig.look(mask([x0, y0, x0 + side, y0 + side], rig.size), settle)[0]) * on, screen)
    if b is None:
        sys.exit("setup: the camera does not see the white square in the screen centre")
    k = (b[2] - b[0]) / side
    tb = target_box(sample["box"], 45, 0.5, 0.5, k, rig.size)
    candle = rig.look(compose(sample["img"], sample["box"], tb, rig.size), settle)[0]
    pic = Image.fromarray(np.concatenate([white, candle], 1))
    ImageDraw.Draw(pic).rectangle([screen[0], screen[1], screen[2] - 1, screen[3] - 1], outline=(255, 0, 0), width=2)
    pic.save(out / "setup.jpg", quality=95)
    return screen, k, on


def delay(rig, screen, n=20):
    """s from the screen change (black to white) to the first bright camera frame, n times."""
    x0, y0, x1, y1 = screen
    black, white = Image.new("RGB", rig.size), Image.new("RGB", rig.size, "white")
    lo = rig.look(black, 1.0)[0][y0:y1, x0:x1].mean()
    hi = rig.look(white, 1.0)[0][y0:y1, x0:x1].mean()
    ages = []
    for _ in range(n):
        rig.look(black, 1.0)
        t = t0 = rig.show(white)
        while t - t0 < 2.0:
            t, f = rig.grab(t)
            if f[y0:y1, x0:x1].mean() > (lo + hi) / 2:
                ages.append(round(t - t0, 4))
                break
    return ages


def run(args):
    out = RUNS / (time.strftime("%Y-%m-%d_%H%M") + ("_fake" if args.fake else ""))
    (out / "frames").mkdir(parents=True)
    stim = stimuli(FireDetector(calib=None), out, STIMULI_SETS[args.stimuli])
    print(f"{len(stim)} source images: {out / 'stimuli.jpg'}", flush=True)
    rig = None
    try:
        rig, settle = (FakeRig(), 0.0) if args.fake else (Rig(args.camera, args.monitor), args.settle)
        info = dict(date=time.strftime("%Y-%m-%d %H:%M"), fake=args.fake, camera=args.camera, monitor=args.monitor,
                    screen_px=list(rig.size), settle_s=settle, frames=args.frames, model=MODEL.name,
                    stimuli_set=args.stimuli, stimuli=[dict(cls=s["cls"], video=s["video"], fire=s["fire"], box=s["box"]) for s in stim])
        if not args.fake:
            if args.exposure:  # 5 Nov, 16 flames at 45 px: 7 found with the automatic exposure (white blob), 13 at 1 ms
                subprocess.run(["v4l2-ctl", "-d", args.camera, "-c", f"auto_exposure=1,exposure_time_absolute={args.exposure}"], check=True)
                info["exposure_ms"] = args.exposure / 10
            (out / "v4l2.txt").write_text(subprocess.run(["v4l2-ctl", "-d", args.camera, "--all"], capture_output=True, text=True).stdout)
        info["screen_box"], info["k"], on = setup(rig, out, next(s for s in stim if s["fire"]), settle)
        Image.fromarray(on.astype(np.uint8) * 255).save(out / "screen_mask.png")
        print(f"setup: screen {info['screen_box']} in the camera image, {info['k']:.3f} camera px per screen px", flush=True)
        if not args.fake:
            info["delay_s"] = delay(rig, info["screen_box"])
            print(f"delay screen -> camera frame: median {np.median(info['delay_s']) * 1000:.0f} ms", flush=True)
        (out / "setup.json").write_text(json.dumps(info, indent=1))
        trials = [(i, w, gx, gy) for i in range(len(stim)) for w in SIZES for gx in GRID for gy in GRID] * args.repeat
        random.Random(0).shuffle(trials)
        t_start = time.monotonic()
        with open(out / "trials.jsonl", "w") as log:
            for n, (i, w, gx, gy) in enumerate(trials):
                s = stim[i]
                tb = target_box(s["box"], w, gx, gy, info["k"], rig.size)
                if tb is None:
                    continue
                truth = true_box(rig, tb, min(settle, 0.4))
                names = []
                for j, f in enumerate(rig.look(compose(s["img"], s["box"], tb, rig.size), settle, args.frames)):
                    names.append(f"frames/{n:05d}_{j}.jpg")
                    Image.fromarray(f).save(out / names[-1], quality=95)
                ok = truth is not None and abs(truth[2] - truth[0] - w) <= max(3, w / 2)
                log.write(json.dumps(dict(n=n, stim=i, cls=s["cls"], fire=s["fire"], width=w, gx=gx, gy=gy,
                                          target=[round(v, 1) for v in tb], truth=truth, ok=ok, frames=names)) + "\n")
                log.flush()
                if n % 100 == 99:
                    dt = time.monotonic() - t_start
                    print(f"{n + 1}/{len(trials)} images, {dt / 60:.0f} min, about {dt / (n + 1) * (len(trials) - n - 1) / 60:.0f} min left",
                          flush=True)
    finally:
        if rig:
            rig.close()
        if not args.fake and args.exposure:
            subprocess.run(["v4l2-ctl", "-d", args.camera, "-c", "auto_exposure=3"])
    if args.repeat:  # --repeat 0: setup and delay only, to check the framing
        report(out, args.model or [MODEL], args.baseline)


def model_name(m):
    """Short name of a model file. A file in a training run (RUN/onnx/FILE) gets the run name, so that two runs differ."""
    p = Path(m)
    return f"{p.parent.parent.name}_{p.stem}" if p.parent.name == "onnx" else p.stem


def detections(run_dir, model, trials):
    """{frame: [[x0, y0, x1, y1, score], ...]} of a model (boxes with score >= 0.1), kept in det_<model>.json."""
    p = run_dir / f"det_{model_name(model)}.json"
    if p.exists():
        return json.loads(p.read_text())
    det, d = FireDetector(model=model, calib=None), {}
    for t in trials:
        for f in t["frames"]:
            b, s = det.boxes(np.asarray(Image.open(run_dir / f).convert("RGB")), conf=0.1)
            d[f] = [[round(float(v), 1) for v in x] + [round(float(y), 3)] for x, y in zip(b, s)]
    p.write_text(json.dumps(d))
    return d


def near(b, t, pad=2):
    """Box b overlaps the true box t grown by pad px."""
    return b[0] < t[2] + pad and b[2] > t[0] - pad and b[1] < t[3] + pad and b[3] > t[1] - pad


def width50(sel):
    """Smallest width bin from which the detection rate stays at 50 % or more for all larger bins (None: never)."""
    ok = {s: np.mean(v) >= 0.5 for s in SIZES if (v := [x["hit"] for x in sel if x["bin"] == s])}
    return next((s for s in sorted(ok) if all(ok[b] for b in ok if b >= s)), None)


def report(run_dir, models, baseline=None):
    run_dir = Path(run_dir)
    info = json.loads((run_dir / "setup.json").read_text())
    trials = [json.loads(line) for line in open(run_dir / "trials.jsonl")]
    good = [t for t in trials if t["ok"]]
    rows = []  # one per frame and model
    for m in models:
        d = detections(run_dir, m, good)
        for t in good:
            tr = t["truth"]
            tw = tr[2] - tr[0]
            r = np.hypot((tr[0] + tr[2]) / 2 - W / 2, (tr[1] + tr[3]) / 2 - H / 2) / np.hypot(W / 2, H / 2)
            for f in t["frames"]:
                boxes = [b for b in d[f] if b[4] >= CONF]
                rows.append(dict(model=model_name(m), cls=t["cls"], src=info["stimuli"][t["stim"]].get("video", t["cls"]), fire=t["fire"], tw=tw,
                                 bin=min(SIZES, key=lambda s: abs(np.log(s / max(tw, 1)))),
                                 pos="centre" if r < 0.3 else "middle" if r < 0.6 else "edge",
                                 hit=any(near(b, tr) for b in boxes), away=any(not near(b, tr) for b in boxes), any=bool(boxes)))

    def rate(sel, key):
        return f"{100 * np.mean([x[key] for x in sel]):.0f} % ({len(sel)})" if sel else "-"

    cols = [(m, c) for m in dict.fromkeys(x["model"] for x in rows) for c in sorted({x["cls"] for x in rows if x["fire"]})]
    L = [f"# Screen bench {info['date']}" + (" (fake camera)" if info["fake"] else ""), "",
         f"Screen {info['screen_px'][0]} x {info['screen_px'][1]} px, seen at {info['screen_box']} in the camera image, "
         f"{info['k']:.3f} camera px per screen px. Detector threshold {CONF}. {len(good)} of {len(trials)} images kept "
         "(the others: the true box was not found or not at the aimed width).", ""]
    if info.get("delay_s"):
        ds = np.array(info["delay_s"]) * 1000
        L += [f"Delay from the screen change to the camera frame: median {np.median(ds):.0f} ms, {ds.min():.0f} to {ds.max():.0f} ms "
              f"({len(ds)} runs). This includes the screen delay, so latency_s in robot/config.toml is at most this value.", ""]
    if not info["fake"]:
        exp = f"fixed at {info['exposure_ms']} ms" if "exposure_ms" in info else "automatic"
        L += [f"Camera exposure: {exp}.", ""]
    L += ["## Fire images: detection rate against the true flame width", "",
          "A frame counts when a box overlaps the true flame box (grown by 2 px). In brackets: number of frames.", "",
          "| Flame width (camera px) | " + " | ".join(f"{c}, {m}" for m, c in cols) + " |", "|---|" + "---|" * len(cols)]
    w50 = {}
    for s in SIZES:
        L.append(f"| {s} | " + " | ".join(rate([x for x in rows if x["model"] == m and x["cls"] == c and x["bin"] == s], "hit")
                                         for m, c in cols) + " |")
    for m, c in cols:
        w50[m, c] = width50([x for x in rows if x["model"] == m and x["cls"] == c])
    L.append("| 50 % width from | " + " | ".join(f"{w50[k]} px" if w50[k] else "-" for k in cols) + " |")
    L.append("| Distance of the real flame | " + " | ".join(f"{F_PX * BOX_M[c] / w50[m, c]:.2f} m" if w50[m, c] else "-"
                                                          for m, c in cols) + " |")
    L += ["", f"The distance uses f = {F_PX:.0f} px ({'lens calibration' if CALIB.exists() else 'model value, not calibrated'}) and these flame box widths: "
          + ", ".join(f"{c} {v * 1000:.0f} mm" for c, v in BOX_M.items()) + " (tea light: from its 38 mm cup, candle: assumed).", "",
          "## Fire images by position in the camera image, flames 16 px wide or less", "",
          "Centre: less than 0.3 of the half diagonal from the image centre. Edge: more than 0.6.", "",
          "| Position | " + " | ".join(f"{c}, {m}" for m, c in cols) + " |", "|---|" + "---|" * len(cols)]
    for p in ("centre", "middle", "edge"):
        L.append(f"| {p} | " + " | ".join(rate([x for x in rows if x["model"] == m and x["cls"] == c and x["pos"] == p
                                                 and x["tw"] <= 16], "hit") for m, c in cols) + " |")
    names = list(dict.fromkeys(x["model"] for x in rows))
    srcs = list(dict.fromkeys(x["src"] for x in rows if x["fire"]))
    L += ["", "## 50 % width for each source video", "", "The same flame width threshold, for each flame of the test on its own.", "",
          "| Source video | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for v in srcs:
        L.append(f"| {v} | " + " | ".join(f"{w} px" if (w := width50([x for x in rows if x['model'] == m and x['src'] == v])) else "-"
                                         for m in names) + " |")
    L += ["", "## False alarms", "", "LED candles: frames with any box. Fire images: frames with a box away from the flame.", "",
          "| Flame width (camera px) | " + " | ".join(f"LED candle, {m}" for m in names) + " |", "|---|" + "---|" * len(names)]
    for s in SIZES:
        L.append(f"| {s} | " + " | ".join(rate([x for x in rows if x["model"] == m and not x["fire"] and x["bin"] == s], "any")
                                         for m in names) + " |")
    L.append("| Fire images, box away from the flame | " + " | ".join(rate([x for x in rows if x["model"] == m and x["fire"]], "away")
                                                                     for m in names) + " |")
    (run_dir / "report.md").write_text("\n".join(L) + "\n")
    curves = {f"{c}, {m}": [[s, float(np.mean(v))] for s in SIZES
                            if (v := [x["hit"] for x in rows if x["model"] == m and x["cls"] == c and x["bin"] == s])] for m, c in cols}
    (run_dir / "curves.json").write_text(json.dumps(curves))
    plot(run_dir, curves, info, baseline)
    print(f"wrote {run_dir / 'report.md'}, report.png")


def plot(run_dir, curves, info, baseline):
    """report.png: detection rate against the flame width. --baseline RUN_DIR adds that run's curves.json as dashed lines."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    colour = {}  # by curve name: the class order depends on the first trial of each run
    for name, pts in curves.items():
        ax.plot(*zip(*pts), "o-", color=colour.setdefault(name, f"C{len(colour)}"), label=name)
    if baseline:
        for name, pts in json.loads((Path(baseline) / "curves.json").read_text()).items():
            ax.plot(*zip(*pts), "--", color=colour.setdefault(name, f"C{len(colour)}"), alpha=0.6, label=f"{name} ({Path(baseline).name})")
    ax.set_xscale("log")
    ax.set_xticks(SIZES, [str(s) for s in SIZES])
    ax.set(xlabel="flame width in the camera image (px)", ylabel="frames with the flame detected", ylim=(-0.03, 1.03),
           title=f"Screen bench {info['date']}" + (" (fake camera)" if info["fake"] else ""))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(run_dir / "report.png", dpi=150)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="show the images and save the camera frames, then report with the default model")
    r.add_argument("--camera", default="/dev/video0")
    r.add_argument("--monitor", help="Hyprland monitor that shows the images (hyprctl monitors). Default: the focused one")
    r.add_argument("--fake", action="store_true", help="no camera and no window: an ideal camera sees the screen image")
    r.add_argument("--stimuli", default="v1", choices=sorted(STIMULI_SETS), help="source images: v1 (2 flames, 3 Nov) or v2 (8 flames)")
    r.add_argument("--repeat", type=int, default=4, help="passes over all images, widths and positions (0: setup and delay only)")
    r.add_argument("--settle", type=float, default=1.0, help="s from an image change to the saved frames (auto exposure)")
    r.add_argument("--exposure", type=int, default=0, help="fixed exposure time in 100 us steps (10: 1 ms), 0: automatic exposure")
    r.add_argument("--frames", type=int, default=2, help="camera frames saved per image")
    r.add_argument("--model", action="append", help="models for the report at the end (default: the detector.py model)")
    r.add_argument("--baseline", help="for the report at the end: another run folder, its curves dashed in report.png")
    p = sub.add_parser("report", help="run the models on the saved frames and write report.md")
    p.add_argument("run_dir")
    p.add_argument("--model", action="append", help="model file (.onnx, or .rknn on the board). Repeat for more models")
    p.add_argument("--baseline", help="another run folder: its curves as dashed lines in report.png (for example the --fake run)")
    a = ap.parse_args()
    run(a) if a.cmd == "run" else report(a.run_dir, a.model or [MODEL], a.baseline)


if __name__ == "__main__":
    main()
