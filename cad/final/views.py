"""Labelled section drawing of the layout: side view (x-z), front view (y-z) and the middle shelf from the top.

    uv run --project cad --with matplotlib python cad/final/views.py     # F2 -> out_f2/layout_views.png
    DESIGN=F1 uv run --project cad --with matplotlib python cad/final/views.py     # F1 -> out/layout_views.png

Reads <out>/layout.json (cad/final/layout.py) and the tank STLs (cad/final/tank.py).
"""
import json
import os
from pathlib import Path

import matplotlib
import trimesh

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = Path(__file__).parent
F2 = os.environ.get("DESIGN", "F2").upper() == "F2"
OUT = HERE / ("out_f2" if F2 else "out")
TANKS = ["tank_left.stl", "tank_right.stl"] if F2 else ["r1f_tank.stl"]
LAY = json.loads((OUT / "layout.json").read_text())
B = LAY["boxes_mm"]
INK, GRID = "#222222", "#bbbbbb"
FILL = {"battery": "#4a4a4a", "power": "#c9c9c9", "compute": "#9fc5a8", "printed": "#eeeeee", "water": "#9cc3e6"}
LABEL = {"lipo_6s_a": "6S motor pack", "lipo_6s_b": "6S motor pack", "battery_holder": "double holder",
         "shelf_acrylic": "acrylic shelf", "relay_aep18024": "relay K1", "estop_contacts": "e-stop blocks",
         "boost_ham5925": "boost 12→24 V", "imu_im10a": "IMU", "lipo_3s": "3S pack", "rock3c": "ROCK 3C",
         "canable_1": "CAN", "buck_5v": "5 V", "diode_mbrt400100": "diode", "shunt_100a": "shunt",
         "meter_hav6001": "meter", "estop_head": "e-stop", "pump_dwp385": "pump", "brake_resistor_1": "brake R",
         "brake_resistor_2": "brake R", "relay_k2": "K2", "mosfet_dfr0457": "FET", "filter_inline": "filter"}
TUBES = LAY.get("tube_boxes_mm", {})
GROUP = {n: next((i["group"] for i in LAY["items"] if i["name"] == n), "printed") for n in B}
SKIP = {"wiring_small", "frame_clips", "rock3c_plugs", "canable_2", "meter_hav1808", "meter_hav1625a", "pod_cover",
        "pod_basket", "pod_spacer_left", "pod_spacer_right", "fill_extension", "imu_mount", "pump_holder",
        "resistor_mount_left", "resistor_mount_right", "switch_pushlock_24v", "switch_pushlock_12v"}


def frame(ax, h, v):
    """2020 posts and rails of the torso frame in the plane (h, v) = ('x'|'y', 'z') or ('x', 'y')."""
    k = {"x": 60.0, "y": 65.0}
    if v == "z":
        for s in (-1, 1):
            ax.add_patch(Rectangle((s * k[h] - 10, 551), 20, 240, fc="#dddddd", ec=INK, lw=0.6))
        for z in (551, 771):
            ax.add_patch(Rectangle((-k[h] - 10, z), 2 * k[h] + 20, 20, fc="#dddddd", ec=INK, lw=0.6))
    else:
        for sx in (-1, 1):
            for sy in (-1, 1):
                ax.add_patch(Rectangle((sx * 60 - 10, sy * 65 - 10), 20, 20, fc="#dddddd", ec=INK, lw=0.6))


def boxes(ax, h, v, select=lambda n, b: True, labels=True):
    idx = {"x": (0, 1), "y": (2, 3), "z": (4, 5)}
    for n, b in B.items():
        if n in SKIP or not select(n, b):
            continue
        (a0, a1), (b0, b1) = (b[i] for i in idx[h]), (b[i] for i in idx[v])
        g = "battery" if n.startswith("lipo") else GROUP[n]
        ax.add_patch(Rectangle((a0, b0), a1 - a0, b1 - b0, fc=FILL.get(g, "#dddddd"), ec=INK, lw=0.7, alpha=0.9))
        if labels and n in LABEL:
            ax.text((a0 + a1) / 2, (b0 + b1) / 2, LABEL[n], ha="center", va="center", fontsize=6.5,
                    color="white" if g == "battery" else INK)


def tank(ax, normal, origin, h, v):
    idx = {"x": 0, "y": 1, "z": 2}
    for f in TANKS:
        s = trimesh.load(OUT / f).section(normal, origin)
        for e in (s.discrete if s is not None else []):
            ax.fill(e[:, idx[h]], e[:, idx[v]], fc=FILL["water"], ec="#2a5d8f", lw=0.6, alpha=0.6)


def tubes(ax, h, v):
    idx = {"x": (0, 1), "y": (2, 3), "z": (4, 5)}
    for b in TUBES.values():
        (a0, a1), (b0, b1) = (b[i] for i in idx[h]), (b[i] for i in idx[v])
        ax.add_patch(Rectangle((a0, b0), a1 - a0, b1 - b0, fc="#2a5d8f", ec="none", alpha=0.8))


def main():
    fig, axs = plt.subplots(1, 3, figsize=(15, 6.2), gridspec_kw={"width_ratios": [1.35, 1.1, 1.1]})
    a, f, t = axs
    # side view: section at y = 0 for the tank, all boxes projected
    frame(a, "x", "z")
    tank(a, [0, 1, 0], [0, 101 if F2 else 20, 0], "x", "z")
    boxes(a, "x", "z", select=lambda n, b: n != "estop_head" and not (F2 and n.startswith("lipo_6s")))
    tubes(a, "x", "z")
    if F2:
        a.text(-4, 822, f"blue: left R2 tank (section) and tubes, {LAY['volume_l']:.2f} L in 2 tanks", ha="center", fontsize=8, color="#1d4a73")
    else:
        a.text(0, 752, "R1F tank\n0.59 L", ha="center", fontsize=8, color="#1d4a73")
    a.annotate("front →", (75, 545), fontsize=8, ha="right")
    a.add_patch(Rectangle((-136, 583), 61, 205.5, fc="none", ec=INK, lw=0.8, ls="--"))
    a.text(-105, 795, "pod cover", ha="center", fontsize=7)
    a.set(xlim=(-145, 100), ylim=(540, 850), title=f"Side view (tank section at y = {101 if F2 else 20} mm, parts projected)",
          xlabel="x (mm), front to the right", ylabel="z above the floor (mm)")
    a.add_patch(Rectangle((-83, 791), 158, 6, fc="#e08a5a", ec=INK, lw=0.6))
    a.text(-4, 800, "deck 96 mm wide, pan-tilt above", ha="center", fontsize=7)
    # front view
    frame(f, "y", "z")
    tank(f, [1, 0, 0], [-20, 0, 0], "y", "z")
    tubes(f, "y", "z")
    boxes(f, "y", "z", select=lambda n, b: b[1] > -75 and n != "imu_im10a")  # the rear pod and the IMU hide the inside
    for s in (1, -1):
        f.add_patch(Rectangle((s * 65 - 15, 791), 30, 45, fc="#333333", ec=INK, lw=0.6))
    f.text(65, 840, "handle", ha="center", fontsize=7)
    f.set(xlim=(-135, 135), ylim=(540, 850), title="Front view (rear pod left out)", xlabel="y (mm), left of the robot to the right")
    # top view of the middle shelf level
    frame(t, "x", "y")
    boxes(t, "x", "y", select=lambda n, b: b[4] < (730 if F2 else 700) and b[5] > 645 and b[0] > -76
          and n not in ("battery_holder", "estop_head"))
    t.add_patch(Rectangle((-136, -77.5), 61, 155, fc=FILL["compute"], ec=INK, lw=0.7, alpha=0.6))
    t.text(-105.5, 0, "rear pod:\nROCK 3C, CAN ×2,\n5 V buck, FET,\ndiode, shunt, K2,\n3S pack,\nmeters, switches",
           ha="center", va="center", fontsize=6.2)
    if F2:
        tank(t, [0, 0, 1], [0, 0, 640], "x", "y")
        tubes(t, "x", "y")
    else:
        for x, y, _ in json.loads((OUT / "r1f.json").read_text())["legs_mm"]:
            t.add_patch(plt.Circle((x, y), 6, fc="white", ec=INK, lw=0.7))
        t.text(44, 56, "tank legs", fontsize=6.5, ha="center")
    t.set(xlim=(-145, 100), ylim=(-135, 135), title=f"Middle level from the top (z 646 to {730 if F2 else 700})",
          xlabel="x (mm), front to the right", ylabel="y (mm), left up")
    for ax in axs:
        ax.set_aspect("equal")
        ax.grid(color=GRID, lw=0.3)
    fig.suptitle(f"{LAY.get('design', 'F1')} layout in the course robot torso (BHL frame, lab changes from the photos). Boxes = bought parts at "
                 "datasheet size.", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "layout_views.png", dpi=160)


if __name__ == "__main__":
    main()
