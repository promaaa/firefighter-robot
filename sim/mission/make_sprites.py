"""Build the real-fire flame sprites in sprites/ from public fire videos (sprites/SOURCES.md has the sources and licences).

    uv run python sim/mission/make_sprites.py              download the videos if needed, write sprites/*.webp
    uv run python sim/mission/make_sprites.py --preview    also write a contact sheet to ~/runs/firefighter/mission/sprites/

Each clip: 2 s of the video at 30 frames per second, cut to a box around the flames, scaled to 128 x 256 pixels (W x H).
The box is squeezed to the billboard shape of the fire model (width about half the height, world.flame_size).
Alpha: from the brightness (between a black level and a white level, per clip) and the colour (grey smoke and blue
pixels lose alpha, the white flame core keeps it). Soft edges at the box sides and top. The last 0.4 s fades into the
first frames, so the clip loops without a jump. Output: one RGBA WebP image per clip, 60 frames in a 10 x 6 grid, row 0 = top.
"""
import argparse
import hashlib
import subprocess
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = HERE / "sprites"
VIDEOS = Path.home() / "datasets" / "firefighter" / "flame_videos"
FW, FH, N, COLS, FPS = 128, 256, 60, 10, 30  # frame size, frames per loop, grid columns, playback rate (world.SPRITE_FPS)
FADE = 12  # frames of the loop cross-fade (0.4 s)
UA = {"User-Agent": "firefighter-robot-student-project"}
C = "https://upload.wikimedia.org/wikipedia/commons/"

# name: Commons file, download URL, SHA-1, start (s), crop box in source pixels (x0, y0, x1, y1, may leave the frame),
# black level and white level of the alpha (0..1, max of R, G, B), playback speed of the source (4: 120 fps slow motion)
CLIPS = {
    "free_flame": ("Fire.ogv", C + "8/8f/Fire.ogv", "6ec195e5d68b22ee2bd6bab42cf56690702dc260", 1.0, (60, 10, 390, 510), 0.12, 0.55, 1),
    "basket": ("Feuer.webm", C + "0/02/Feuer.webm", "69dcbb0c747b7c1c37d5fad4940af01a26813b70", 2.0, (760, -60, 1200, 680), 0.15, 0.6, 1),
    "campfire_a": ("Campfire Video 1.webm", C + "a/ab/Campfire_Video_1.webm", "e98a17611cbb3149fe153f8e01b59c53b180b363", 2.0,
                   (175, 300, 435, 620), 0.5, 0.85, 1),
    "campfire_b": ("Campfire video 2.webm", C + "0/0a/Campfire_video_2.webm", "d07b3e23797e5d43b6d38765a6b76f1091678ba5", 2.0,
                   (170, 50, 470, 530), 0.5, 0.85, 1),
    "wood_fire": ("Fire burning.ogv", C + "e/e8/Fire_burning.ogv", "da05fa17c8d42fa75681334bad34b9d3bc666bd9", 1.0, (180, 0, 460, 470),
                  0.2, 0.6, 1),
}


def video(name):
    """Local copy of a Commons video (downloaded once, SHA-1 checked when known)."""
    title, url, sha1 = CLIPS[name][:3]
    p = VIDEOS / title.replace(" ", "_")
    if not p.exists():
        VIDEOS.mkdir(parents=True, exist_ok=True)
        data = urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=300).read()
        p.write_bytes(data)
    if sha1:
        assert hashlib.sha1(p.read_bytes()).hexdigest() == sha1, f"{p}: not the Commons file"
    return p


def frames(path, t0, box, speed, n):
    """n frames at 30 fps (source time t0 + k / 30 / speed), cropped to box (black outside the frame), uint8."""
    w, h = map(int, subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                                             "-of", "csv=p=0", str(path)]).decode().split(",")[:2])
    x0, y0, x1, y1 = box
    cx0, cy0, cx1, cy1 = max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)
    raw = subprocess.check_output(["ffmpeg", "-v", "error", "-ss", str(t0), "-i", str(path), "-vf",
                                   f"setpts=PTS/{speed},fps={FPS},crop={cx1 - cx0}:{cy1 - cy0}:{cx0}:{cy0}",
                                   "-frames:v", str(n), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"])
    f = np.frombuffer(raw, np.uint8).reshape(-1, cy1 - cy0, cx1 - cx0, 3)
    assert len(f) == n, (path, len(f), n)
    out = np.zeros((n, y1 - y0, x1 - x0, 3), np.uint8)
    out[:, cy0 - y0:cy1 - y0, cx0 - x0:cx1 - x0] = f
    return out


def rgba(img, lo, hi):
    """Source frame (FH x FW x 3 uint8, fire on a dark background) -> straight RGBA float32.
    Where alpha is 1, the colour is the source colour. Where alpha is lower, the colour keeps the source hue."""
    x = img.astype(np.float32) / 255
    v = x.max(-1)
    a = np.clip((v - lo) / (hi - lo), 0, 1)
    a = a * a * (3 - 2 * a)  # smoothstep
    warm = np.clip((x[..., 0] - x[..., 2]) / np.maximum(x[..., 0], 1e-3) / 0.35, 0, 1)  # red above blue: flame, not smoke
    core = np.clip((v - 0.75) / 0.15, 0, 1)  # the white core is saturated: keep it
    a *= np.maximum(warm, core) * (x[..., 0] >= 0.9 * v)  # red must be (nearly) the largest channel
    yy, xx = np.mgrid[0:FH, 0:FW]
    edge = np.clip(np.minimum(xx, FW - 1 - xx) / (0.12 * FW), 0, 1) * np.clip(yy / (0.08 * FH), 0, 1) * np.clip((FH - 1 - yy) / (0.02 * FH), 0, 1)
    # straight colour with the source hue: the composite adds src * min(1, alpha / max(R, G, B)), never more than the source
    rgb = x / np.maximum(v, 1e-3)[..., None] * np.minimum(1, v / np.maximum(a, 1e-3))[..., None] * (a[..., None] > 0)
    a *= edge  # after the colour: the soft edge fades the flame, it does not brighten it
    return np.concatenate([rgb, a[..., None]], -1)


def clip(name):
    title, _, _, t0, box, lo, hi, speed = CLIPS[name]
    src = frames(video(name), t0, box, speed, N + FADE)
    f = np.stack([rgba(np.asarray(Image.fromarray(s).resize((FW, FH), Image.LANCZOS)), lo, hi) for s in src])
    pm = np.concatenate([f[..., :3] * f[..., 3:], f[..., 3:]], -1)  # premultiplied, for the cross-fade
    w = ((np.arange(FADE) + 0.5) / FADE)[:, None, None, None]
    pm[:FADE] = w * pm[:FADE] + (1 - w) * pm[N:N + FADE]  # frame N-1 runs on into frame 0: a seamless loop
    pm = pm[:N]
    a = pm[..., 3:]
    return np.concatenate([np.clip(pm[..., :3] / np.maximum(a, 1e-3), 0, 1) * (a > 0), a], -1)


def save(name, f):
    rows = N // COLS
    sheet = f.reshape(rows, COLS, FH, FW, 4).transpose(0, 2, 1, 3, 4).reshape(rows * FH, COLS * FW, 4)
    OUT.mkdir(exist_ok=True)
    # lossy WebP (the videos are lossy already): about 1 MB per clip instead of 2.5 MB as PNG. The alpha stays lossless
    Image.fromarray(np.round(sheet * 255).astype(np.uint8), "RGBA").save(OUT / f"{name}.webp", quality=92, alpha_quality=100, method=6)


def preview(clips, path):
    """Frames 0, 15, 30, 45 of each clip over a grey and a dark background, one row per clip."""
    tiles = []
    for name, f in clips.items():
        row = []
        for k in (0, 15, 30, 45):
            for bg in (0.45, 0.08):
                a = f[k, ..., 3:]
                row.append(f[k, ..., :3] * a + bg * (1 - a))
        tiles.append(np.concatenate(row, 1))
    Image.fromarray(np.round(np.concatenate(tiles, 0) * 255).astype(np.uint8)).save(path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preview", action="store_true")
    ap.add_argument("clips", nargs="*", default=list(CLIPS))
    a = ap.parse_args()
    done = {}
    for name in a.clips:
        done[name] = clip(name)
        save(name, done[name])
        print(f"{name}: {(OUT / f'{name}.webp').stat().st_size / 1e6:.2f} MB, mean alpha {done[name][..., 3].mean():.3f}")
    if a.preview:
        p = Path.home() / "runs" / "firefighter" / "mission" / "sprites" / "sprites_preview.png"
        p.parent.mkdir(parents=True, exist_ok=True)
        preview(done, p)
        print(p)
