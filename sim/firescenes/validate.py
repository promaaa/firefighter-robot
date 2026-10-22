"""Check the labels: project each fire centre with aim/geometry.py and compare it with the fire's box.
The error is the distance from the projected centre to the box centre. Boxes cut by the image edge are skipped.
Also checks the 1280x720 thermal PNG: no clipped pixel, and the hottest pixel in each fire box is a flame temperature.

uv run --with numpy python sim/firescenes/validate.py [OUT_DIR]
"""
import glob
import json
import os
import struct
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "aim"))
import geometry as G  # noqa: E402


def read_png16(path):
    """16-bit grey PNG written by firescenes.png() (one IDAT stream, no row filters) -> uint16 array."""
    data, pos, idat = open(path, "rb").read(), 8, b""
    while pos < len(data):
        n, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + n]
        if kind == b"IHDR":
            w, h, depth, ctype = struct.unpack(">IIBB", body[:10])
        elif kind == b"IDAT":
            idat += body
        pos += 12 + n
    raw = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, 1 + 2 * w)
    assert depth == 16 and ctype == 0 and not raw[:, 0].any(), path
    return raw[:, 1:].copy().view(">u2").astype(np.uint16)


def main(out):
    cfg = dict(G.CONCEPTS["A"])
    cam = cfg["cameras"]["rgb"]
    err, inside, cut, hot = {}, {}, {}, []
    for path in sorted(glob.glob(f"{out}/labels/*.json")):
        with open(path) as fh:
            d = json.load(fh)
        c = d["camera"]
        assert d["images"]["thermal"]["unit"] == "decikelvin", path
        th = read_png16(os.path.join(out, d["images"]["thermal"]["file"]))
        assert th.max() < 65535, f"clipped thermal pixel in {path}"
        for f in d["fires"]:
            b = f["boxes"]["thermal"]
            if b and b[2] - b[0] >= 6 and b[3] - b[1] >= 6:  # big enough to have pixels fully covered by flame
                t = th[b[1]:b[3], b[0]:b[2]].max() / 10 - 273.15
                hot.append(f["temp_min_c"] - 1 <= t <= f["temp_max_c"] + 1)
        for name, im in d["images"].items():
            K = G.Intrinsics(im["fx"], im["fy"], im["cx"], im["cy"], im["width"], im["height"])
            cfg["cameras"] = {name: dict(cam, K=K)}
            R, t = G.camera_pose(cfg, name, c["pan_rad"], c["tilt_rad"])
            assert np.allclose(R, c["R_robot_from_optical"]) and np.allclose(t, c["t_robot"]), path  # same pose as the renderer
            for f in d["fires"]:
                b = f["boxes"][name]
                p = G.project(cfg, name, c["pan_rad"], c["tilt_rad"], f["centre_robot"])
                if b is None or p is None:
                    continue
                if b[0] <= 0 or b[1] <= 0 or b[2] >= im["width"] or b[3] >= im["height"]:  # fire cut by the image edge
                    cut[name] = cut.get(name, 0) + 1
                    continue
                err.setdefault(name, []).append(np.hypot(p[0] - (b[0] + b[2]) / 2, p[1] - (b[1] + b[3]) / 2))
                inside.setdefault(name, []).append(b[0] <= p[0] <= b[2] and b[1] <= p[1] <= b[3])
    print("image        fires  inside box  mean error (px)  median (px)  max (px)  skipped: cut by image edge")
    for name, e in err.items():
        print(f"{name:12s} {len(e):5d}  {np.mean(inside[name]):9.0%}  {np.mean(e):15.1f}  {np.median(e):11.1f}  {np.max(e):8.1f}  {cut.get(name, 0):5d}")
    print(f"thermal PNG: no clipped pixel; hottest pixel in the fire box is in the flame range for {np.mean(hot):.0%} of {len(hot)} fires")
    assert err and np.mean(inside["rgb"]) > 0.8, "projected fire centres miss their boxes"
    assert np.mean(hot) > 0.9, "fire boxes do not hold flame temperatures"


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "out"))
