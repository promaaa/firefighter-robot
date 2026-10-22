"""Run the detector on real fire videos with open licences (Wikimedia Commons): share of frames with a detection,
a confidence timeline and a short demo video with the boxes.

Run from vision/. The clips are in ~/datasets/firefighter/videos/. results/video_sources.md gives the download commands.
  uv run python video_demo.py
Writes results/video_check.md, results/video_timeline.png and results/demo.mp4 (CC BY-SA 4.0, see results/video_sources.md).
Each frame is scaled to fit 640 x 480 (black border), like the robot camera image, at 10 frames per second.
"""
import subprocess
from pathlib import Path

import matplotlib
import numpy as np
from PIL import Image, ImageDraw

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from detector import CONF, FireDetector  # noqa: E402

HERE = Path(__file__).resolve().parent
VIDEOS = Path.home() / "datasets/firefighter/videos"
W, H, FPS, SECONDS = 640, 480, 10, 15
CLIPS = [  # file, what the frames show, start (s)
    ("candle_and_match_LoKiLeCh_CCBYSA3.ogv", "fire: candle, close", 0),
    ("candle_blow_GilesLaurent_CCBYSA4.webm", "mixed: tea light lit, then blown out", 3),
    ("burning_pet_bottle_CC0.ogv", "fire: burning plastic bottle", 0),
    ("campfire_Vijayanrajapuram_CCBYSA4.webm", "fire: small campfire, far away", 0),
    ("flameless_candle_1_CC0.webm", "no fire: LED candle", 0),
    ("flameless_candle_2_CC0.webm", "no fire: LED candle", 0),
]


def frames(path, start):
    vf = f"fps={FPS},scale={W}:{H}:force_original_aspect_ratio=decrease,pad={W}:{H}:(ow-iw)/2:(oh-ih)/2"
    p = subprocess.run(["ffmpeg", "-loglevel", "error", "-ss", str(start), "-t", str(SECONDS), "-i", str(path), "-vf", vf,
                        "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True, check=True)
    return np.frombuffer(p.stdout, np.uint8).reshape(-1, H, W, 3)


def main():
    det = FireDetector(threads=4)
    enc = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                            "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-crf", "30", "-preset", "slow", "-pix_fmt", "yuv420p",
                            str(HERE / "results/demo.mp4")], stdin=subprocess.PIPE)
    rows, timeline = [], []
    for name, what, start in CLIPS:
        conf_max = []
        for i, rgb in enumerate(frames(VIDEOS / name, start)):
            b, sc = det.boxes(rgb, conf=0.05)
            conf_max.append(float(sc.max()) if len(sc) else 0.0)
            im = Image.fromarray(rgb)
            d = ImageDraw.Draw(im)
            for box, s in zip(b, sc):
                if s >= CONF:
                    d.rectangle(list(box), outline=(0, 255, 0), width=3)
                    d.text((box[0] + 3, max(box[1] - 12, 0)), f"fire {s:.2f}", fill=(0, 255, 0))
            d.rectangle([0, 0, W, 16], fill=(0, 0, 0))
            d.text((4, 3), f"{what} | {name.split('_CC')[0]} | t = {i / FPS:4.1f} s", fill=(255, 255, 255))
            enc.stdin.write(np.asarray(im).tobytes())
        c = np.array(conf_max)
        timeline.append((what, c))
        rows.append(f"| {name} | {what} | {len(c)} | {100 * (c >= CONF).mean():.0f} % | {100 * (c >= 0.5).mean():.0f} % | {c.max():.2f} |")
    enc.stdin.close()
    enc.wait()
    text = (f"Detector threshold {CONF}. {FPS} frames per second, up to "
            f"{SECONDS} s per clip.\n\n| Clip | Content | Frames | Frames with a detection >= {CONF} | Frames with a detection "
            ">= 0.5 | Highest confidence |\n|---|---|---|---|---|---|\n"
            + "\n".join(rows) + "\n")
    (HERE / "results/video_check.md").write_text(text)
    print(text)
    fig, ax = plt.subplots(len(timeline), 1, figsize=(8, 1.4 * len(timeline)), sharex=True)
    for a, (what, c) in zip(ax, timeline):
        a.plot(np.arange(len(c)) / FPS, c, lw=1)
        a.axhline(CONF, color="r", lw=0.8)
        a.set_ylim(0, 1)
        a.set_title(what, fontsize=8, loc="left")
    ax[-1].set_xlabel("Time in clip (s). Red line: threshold")
    fig.supylabel("Highest confidence in the frame", fontsize=8)
    fig.tight_layout()
    fig.savefig(HERE / "results/video_timeline.png", dpi=120)


if __name__ == "__main__":
    main()
