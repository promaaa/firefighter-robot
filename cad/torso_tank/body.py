"""Water tanks in and on the BHL torso for the TCS M400K-V micropump: R1 (inner tank) and R2 (side tanks).

    uv run --project cad python cad/torso_tank/body.py

R1: one tank inside the torso frame, above the mini PC and under the pan-tilt deck. The pump hangs under the tank
    floor in a printed drip tray, inside the torso. The tank hangs from the deck (deck_r1 = the cad/parts deck with
    4 more M4 holes).
R2: two tanks on the side faces of the torso posts, above the hip actuators. The pump sits in a holder in front of
    the right hip, below the water.

Writes STEP + STL to out/, prints volumes and masses, runs the checks (bed fit, watertight STL, wall >= 2 mm,
torso clearance, leg clearance), and writes out/assembly.json (masses and centres for sim/payload, water at
full / half / empty, inner sizes for the slosh model). Robot frame, mm: x forward, y left, z up, feet on the floor,
torso frame top z = 791. The STLs stack on the robot without any transform.
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
from build123d import CenterOf, Cylinder, Plane, Pos, Rot, export_step, export_stl, mirror
from scipy.spatial import cKDTree

HERE = Path(__file__).parent
OUT = HERE / "out"
sys.path[:0] = [str(HERE.parent / "backpack"), str(HERE.parent / "parts")]
import body as bp  # noqa: E402  cad/backpack/body.py: box(), leg sweep, BHL path
import top_module as tm  # noqa: E402  cad/parts/top_module.py: deck()

box = bp.box
T = 791.0                     # top face of the torso frame
WALL = 2.5                    # tank wall; print 100 % solid (6 perimeters) so the walls hold water
BAFFLE_T = 2.0
RHO = 1.27                    # PETG g/cm3, printed solid
WATER = 1.0                   # g/cm3
TORSO_CLEAR_MIN = 1.5         # mm, tank to torso mesh (pads that touch the frame on purpose are left out)
BED = bp.BED                  # 220 x 220 x 250

# Pump: TCS Micropumps M400K-V, brushless centrifugal, 3.2 mm bore tubing, 40 x 26 x 25 mm, 32-34 g,
# 12 V 5.8 W, 380 mbar, 1150 mL/min free flow (TCS M400 range page, read 2026-10-13). Not self-priming:
# it must sit below the water. The -SUB variant is the self-priming, submersible one.
PUMP_L, PUMP_W, PUMP_H, PUMP_G = 40.0, 26.0, 25.0, 33.0
TAP_D, BOSS_D, BOSS_L = 4.2, 10.0, 7.0   # M5 thread for a brass M5 x 3.2 mm hose barb
INSERT_D, INSERT_L = 5.6, 8.5            # M4 heat-set insert

# ---------------- R1: inner tank ----------------
# Free space measured on base_visual.stl: above the mini PC (top z 679) and the USB board (x 14..40, z 685..711),
# between the shoulder brackets (|y| >= 66.7) and under the deck (z 791). The pan base hangs 29 mm under the deck
# (x -78..2, |y| <= 40) and the deck cable slot is at x -5..35, y -56..-44 (cad/parts/out_sts3215).
R1_A = (-48.0, 48.0, -63.0, 63.0, 714.0, 786.0)   # between the top rails (x = +-50) and the shoulder brackets
R1_B = (-62.0, 62.0, -52.0, 52.0, 714.0, 769.0)   # front and rear bays between the posts, under the top rails
R1_NOTCH = (-90.0, 4.0, -42.0, 42.0, 759.0, 800.0)  # pan base and pan servo
R1_CHAN = (-8.0, 40.0, -70.0, -42.0, 700.0, 800.0)  # cables and the jet hose, under the deck slot
R1_BAFFLES_X = (-30.0, 0.0, 30.0)
R1_BAFFLES_Y = (-30.0, 30.0)
R1_PUMP = (-52.0, -12.0, -13.0, 13.0, 686.0, 711.0)  # under the floor, 7 mm over the mini PC
R1_OUTLET = (-5.0, 0.0)                             # floor outlet x, y; a short loop to the pump inlet
R1_NECK = (735.0, 26.0, 19.0, -78.0)                # fill neck to the rear: z, OD, ID, end x (rear face x = -70)
R1_PILLARS = [(30.0, 50.0), (30.0, -30.0), (-42.0, 52.0), (-42.0, -52.0)]  # M4 from the deck into the tank top
R1_EXTRA_NOTCHES = []   # more (x0, x1, y0, y1, z0, z1) cuts with walls; cad/final adds one over the relay
R1_LEGS = []            # (x, y, z_bottom): solid legs under the floor down to a shelf; cad/final uses them

# ---------------- R2: side tanks (left drawn, right = mirror) ----------------
# Free space: outside the posts (y > 75) under the shoulder brackets (z < 681), above the hip actuators (z 581).
# The battery end sticks out of the left side at x -27..30, y 76..89, z 601..615 (both tanks get the pocket).
R2_BOX = (-66.0, 66.0, 78.0, 124.0, 592.0, 678.0)
R2_POCKET = (-33.0, 36.0, 76.0, 94.0, 596.0, 621.0)
R2_BAFFLES_X = (-33.0, 0.0, 33.0)
R2_SCREWS = [(x, z) for x in (-60.0, 60.0) for z in (603.0, 667.0)]  # M5 through dry tubes into the post side slots
R2_NECK = (57.0, 101.0, 22.0, 16.0, 692.0)          # x, y, OD, ID, top z
R2_OUTLET = (101.0, 601.0)                           # y, z on the front face (x = 66)
R2_PUMP = (50.0, 76.0, -120.0, -80.0, 562.0, 587.0)  # pump in front of the right hip, below the tank floor
R2_HOLDER_T = 2.5
R2_TUBE_D = 12.0                                     # dry screw tube; cad/final F2 uses 14 (2 mm around the counterbore)


def shrink(b, d):
    return (b[0] + d, b[1] - d, b[2] + d, b[3] - d, b[4] + d, b[5] - d)


def cyl_along(axis, c, d, a0, a1):
    """Cylinder of diameter d along axis 'x', 'y' or 'z' from a0 to a1, through the point c (3 values)."""
    p = list(c)
    i = "xyz".index(axis)
    p[i] = (a0 + a1) / 2
    r = {"x": Rot(0, 90, 0), "y": Rot(90, 0, 0), "z": Rot(0, 0, 0)}[axis]
    return Pos(*p) * r * Cylinder(d / 2, abs(a1 - a0))


def baffle(axis, c, cav, window):
    """Thin wall across the tank, normal to axis 'x' or 'y' at c, with a window (two ranges: the other horizontal
    axis, then z) and two drain slots at the floor."""
    t, (a0, a1, z0, z1) = BAFFLE_T / 2, window
    if axis == "x":
        b = box(c - t, c + t, cav[2] - 1, cav[3] + 1, cav[4] - 1, cav[5] + 1) - box(c - 2, c + 2, a0, a1, z0, z1)
        for y in (cav[2] + 6, cav[3] - 6):
            b -= box(c - 2, c + 2, y - 4, y + 4, cav[4] - 2, cav[4] + 4)
    else:
        b = box(cav[0] - 1, cav[1] + 1, c - t, c + t, cav[4] - 1, cav[5] + 1) - box(a0, a1, c - 2, c + 2, z0, z1)
        for x in (cav[0] + 6, cav[1] - 6):
            b -= box(x - 4, x + 4, c - 2, c + 2, cav[4] - 2, cav[4] + 4)
    return b


def r1_tank():
    outer = box(*R1_A) + box(*R1_B) - box(*R1_NOTCH) - box(*R1_CHAN)
    cav = box(*shrink(R1_A, WALL)) + box(*shrink(R1_B, WALL)) - box(*shrink(R1_NOTCH, -WALL)) - box(*shrink(R1_CHAN, -WALL))
    for n in R1_EXTRA_NOTCHES:
        outer -= box(*n)
        cav -= box(*shrink(n, -WALL))
    span = shrink(R1_B, WALL)[:2] + shrink(R1_A, WALL)[2:4] + shrink(R1_A, WALL)[4:]
    for xb in R1_BAFFLES_X:
        cav -= baffle("x", xb, span, (-40, 40, 724, 752))
    for yb in R1_BAFFLES_Y:
        cav -= baffle("y", yb, span, (-50, 50, 724, 752))
    for x, y in R1_PILLARS:  # solid pillars under the top for the deck screws (closed, no water path)
        cav -= cyl_along("z", (x, y, 0), 11.0, R1_A[5] - 12, R1_A[5] + 1)
    z, od, idia, x_end = R1_NECK
    neck = cyl_along("x", (0, 0, z), od, x_end, R1_B[0] + 1)
    outer += neck
    shell = outer - cav - cyl_along("x", (0, 0, z), idia, x_end - 1, R1_B[0] + WALL + 1)
    ox, oy = R1_OUTLET
    shell += cyl_along("z", (ox, oy, 0), BOSS_D, R1_A[4] - BOSS_L, R1_A[4] + 1)
    shell -= cyl_along("z", (ox, oy, 0), TAP_D, R1_A[4] - BOSS_L - 1, R1_A[4] + WALL + 1)
    for x, y in R1_PILLARS:
        shell += cyl_along("z", (x, y, 0), 11.0, R1_A[5] - 1, T)          # 5 mm spacer up to the deck underside
        shell -= cyl_along("z", (x, y, 0), INSERT_D, T - INSERT_L - 5, T + 1)
    # drip tray under the pump: floor 2 mm under the pump, side walls up to the tank floor, 6 mm end lips
    x0, x1, y0, y1, z0, _ = R1_PUMP
    tray = box(x0 - 3, x1 + 3, y0 - 3 - WALL, y1 + 3 + WALL, z0 - 2 - WALL, z0 - 2)
    for s in (-1, 1):
        y_in = y1 + 3 if s > 0 else y0 - 3 - WALL
        tray += box(x0 - 3, x1 + 3, y_in, y_in + WALL, z0 - 2 - WALL, R1_A[4] + 0.5)
    for x in (x0 - 3 - WALL, x1 + 3):
        tray += box(x, x + WALL, y0 - 3 - WALL, y1 + 3 + WALL, z0 - 2 - WALL, z0 + 4)
    tray -= cyl_along("z", ((x0 + x1) / 2, 0, 0), 6.0, z0 - 10, z0)  # drain hole: route a tube out of the torso
    for x, y, zb in R1_LEGS:  # 12 mm legs, M4 heat-set insert at the foot, screw up through the shelf
        tray += cyl_along("z", (x, y, 0), 12.0, zb, R1_A[4] + 0.5)
        tray -= cyl_along("z", (x, y, 0), INSERT_D, zb - 1, zb + INSERT_L)
    return shell + tray, cav


def r2_tank():
    outer, cav = box(*R2_BOX), box(*shrink(R2_BOX, WALL))
    if R2_POCKET:                       # cad/final F2 sets None: its 6S packs end inside the posts
        outer -= box(*R2_POCKET)
        cav -= box(*shrink(R2_POCKET, -WALL))
    c = shrink(R2_BOX, WALL)
    for xb in R2_BAFFLES_X:
        cav -= baffle("x", xb, c, (c[2] + 6, c[3] - 6, 612, 660))
    for x, z in R2_SCREWS:
        cav -= cyl_along("y", (x, 0, z), R2_TUBE_D, R2_BOX[2] - 1, R2_BOX[3] + 1)
    x, y, od, idia, ztop = R2_NECK
    outer += cyl_along("z", (x, y, 0), od, R2_BOX[5] - 1, ztop)
    shell = outer - cav - cyl_along("z", (x, y, 0), idia, R2_BOX[5] - WALL - 1, ztop + 1)
    for x, z in R2_SCREWS:
        shell += cyl_along("y", (x, 0, z), 16.0, 75.0, R2_BOX[2] + 1)      # pad against the post side face
        shell -= cyl_along("y", (x, 0, z), 5.5, 74.0, R2_BOX[3] + 1)       # M5 x 55 into a drop-in T-nut
        shell -= cyl_along("y", (x, 0, z), 10.0, R2_BOX[3] - 6, R2_BOX[3] + 1)  # counterbore on the outer face
    oy, oz = R2_OUTLET
    shell += cyl_along("x", (0, oy, oz), BOSS_D, R2_BOX[1] - 1, R2_BOX[1] + BOSS_L)
    shell -= cyl_along("x", (0, oy, oz), TAP_D, R2_BOX[1] - WALL - 1, R2_BOX[1] + BOSS_L + 1)
    return shell, cav


def r2_holder():
    """Sleeve for the pump in front of the right hip, open to the front (pump slides in, ports forward).
    Its top plate is glued under the right tank floor."""
    x0, x1, y0, y1, z0, z1 = R2_PUMP
    t = R2_HOLDER_T
    return box(x0 - t, x1, y0 - t, y1 + t, z0 - t, R2_BOX[4]) - box(x0, x1 + 1, y0, y1, z0, z1)


def r1_deck():
    d = Pos(0, 0, T) * tm.deck()
    for x, y in R1_PILLARS:
        d -= cyl_along("z", (x, y, 0), 4.5, T - 1, T + tm.DECK_T + 1)
    return d


def pump_ref(b):
    x0, x1, y0, y1, z0, z1 = b
    return box(x0, x1, y0, y1, z0, z1)


def water(cav, frac):
    """Water body at a fill fraction (level found by bisection), tank level, no slosh."""
    full = cav.volume
    if frac >= 1:
        return cav
    bb = cav.bounding_box()
    lo, hi = bb.min.Z, bb.max.Z
    for _ in range(30):
        z = (lo + hi) / 2
        w = cav & box(bb.min.X - 1, bb.max.X + 1, bb.min.Y - 1, bb.max.Y + 1, bb.min.Z - 1, z)
        lo, hi = (z, hi) if w.volume < frac * full else (lo, z)
    return w


def props(s, g_per_cm3):
    c = s.center(CenterOf.MASS)
    bb = s.bounding_box()
    return {"mass_kg": round(s.volume / 1000 * g_per_cm3 / 1000, 4), "com_m": [round(v / 1000, 4) for v in (c.X, c.Y, c.Z)],
            "size_m": [round(v / 1000, 4) for v in (bb.size.X, bb.size.Y, bb.size.Z)]}


def slosh_dims(cav, nx, ny):
    """Inner length and width of the tank and the number of baffle cells along x and y (for the slosh model)."""
    bb = cav.bounding_box()
    return {"lx_m": round(bb.size.X / 1000, 4), "ly_m": round(bb.size.Y / 1000, 4), "h_full_m": round(bb.size.Z / 1000, 4),
            "cells_x": nx + 1, "cells_y": ny + 1}


def torso_points():
    m = trimesh.load(bp.BHL / "meshes/base_visual.stl")
    m.apply_translation([-0.0030965, 0.000264765, 0.675059 - 0.039])  # full URDF base visual origin, then the shift
    m.apply_scale(1000)
    return np.vstack([m.vertices, m.sample(300000)])


def main():
    OUT.mkdir(exist_ok=True)
    r1, r1_cav = r1_tank()
    r2l, r2_cav = r2_tank()
    r2r = mirror(r2l, Plane.XZ)
    holder = r2_holder()
    printed = {"r1_tank": r1, "r1_deck": r1_deck(), "r2_tank_left": r2l, "r2_tank_right": r2r, "r2_pump_holder": holder}
    # print splits: R1 in two halves at y = 0 (each half printed on its cut face, glued with epoxy);
    # R2 tub printed on its outer face, inner wall plate printed flat and glued on.
    big = 400.0
    cut_y = R2_BOX[2] + WALL
    prints = {"r1_half_left": r1 & box(-big, big, 0, big, 0, 1000), "r1_half_right": r1 & box(-big, big, -big, 0, 0, 1000),
              "r2_tub": r2l & box(-big, big, cut_y, big, 0, 1000), "r2_inner_plate": r2l & box(-big, big, 0, cut_y, 0, 1000),
              "r1_deck": printed["r1_deck"], "r2_pump_holder": holder}
    refs = {"ref_pump_r1": pump_ref(R1_PUMP), "ref_pump_r2": pump_ref(R2_PUMP)}
    meshes = {}
    for name, s in {**printed, **prints, **refs}.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
        meshes[name] = trimesh.load(OUT / f"{name}.stl")
        if name in printed or name in prints:
            export_step(s, str(OUT / f"{name}.step"))
    for name, s in {**printed, **prints}.items():
        bb = s.bounding_box().size
        print(f"{name:16s} {s.volume / 1000:7.1f} cm3 {s.volume / 1000 * RHO:6.0f} g  {bb.X:.0f} x {bb.Y:.0f} x {bb.Z:.0f} mm")

    # ---------- checks ----------
    for name in prints:
        dims = sorted(meshes[name].extents)
        assert all(a <= b for a, b in zip(dims, sorted(BED))), f"{name} does not fit the bed"
        assert meshes[name].is_watertight, f"{name} STL is not watertight"
    assert min(WALL, BAFFLE_T, R2_HOLDER_T, (BOSS_D - TAP_D) / 2, (11.0 - INSERT_D) / 2) >= bp.WALL_MIN
    print("checks passed: bed fit, watertight STL, wall parameters >= 2 mm")
    tp = cKDTree(torso_points())
    touch = {"r2_tank_left": 75.0, "r2_tank_right": -75.0}  # pads touch the post side faces on purpose
    clear = {}
    for name in ("r1_tank", "r2_tank_left", "r2_tank_right", "r2_pump_holder", "ref_pump_r1", "ref_pump_r2"):
        p = np.vstack([meshes[name].vertices, meshes[name].sample(40000)])
        if name in touch:
            p = p[np.abs(p[:, 1]) > abs(touch[name]) + 2.5]
        d = tp.query(p)[0]
        clear[name] = float(d.min())
        print(f"torso clearance {name:16s} {clear[name]:5.1f} mm at {np.round(p[d.argmin()]).tolist()}")
        assert clear[name] >= TORSO_CLEAR_MIN, f"{name} is {clear[name]:.1f} mm from the torso"
    pts = {n: np.vstack([meshes[n].vertices, meshes[n].sample(30000)]) for n in
           ("r2_tank_left", "r2_tank_right", "r2_pump_holder", "ref_pump_r2")}
    legs = {}
    for label, kmax in (("full URDF range", None), (f"knee <= {bp.KNEE_MAX} rad", bp.KNEE_MAX)):
        worst = bp.sweep(pts, knee_max=kmax)
        legs[label] = {n: [round(float(d), 1) if np.isfinite(d) else None, side, vals] for n, (d, side, vals) in worst.items()}
        print(f"leg clearance, {label}:")
        for n, (d, side, vals) in sorted(worst.items(), key=lambda kv: kv[1][0]):
            print(f"  {n:16s} " + (f"min {d:5.1f} mm, {side} leg, roll/yaw/pitch/knee = {vals} rad" if side else f"more than {bp.CAP:.0f} mm"))
    for n, (d, side, vals) in worst.items():
        assert d > bp.LEG_CLEAR_MIN, f"{n} is {d:.1f} mm from the {side} leg at {vals}"
    print(f"leg clearance > {bp.LEG_CLEAR_MIN:.0f} mm with knee <= {bp.KNEE_MAX} rad. R1 sits inside the frame (no leg check).")

    # ---------- mass model for sim/payload ----------
    hw = {"R1": 0.045, "R2": 0.140}  # GUESS: barbs, stopper, 0.5-1 m of tubing with water, screws and nuts
    asm = {"pump": {"model": "TCS M400K-V", "mass_kg": PUMP_G / 1000, "size_mm": [PUMP_L, PUMP_W, PUMP_H]}, "options": {}}
    for key, shells, cav, pump, extra, nb in (
            ("R1", [r1], [r1_cav], R1_PUMP, [], (len(R1_BAFFLES_X), len(R1_BAFFLES_Y))),
            ("R2", [r2l, r2r], [r2_cav, mirror(r2_cav, Plane.XZ)], R2_PUMP, [holder], (len(R2_BAFFLES_X), 0))):
        parts = [dict(name=f"shell_{i}", **props(s, RHO)) for i, s in enumerate(shells + extra)]
        x0, x1, y0, y1, z0, z1 = pump
        parts.append({"name": "pump", "mass_kg": PUMP_G / 1000, "com_m": [(x0 + x1) / 2e3, (y0 + y1) / 2e3, (z0 + z1) / 2e3],
                      "size_m": [(x1 - x0) / 1e3, (y1 - y0) / 1e3, (z1 - z0) / 1e3]})
        c = parts[0]["com_m"]
        parts.append({"name": "hardware_tubing", "mass_kg": hw[key], "com_m": [c[0], 0.0, c[2]], "size_m": [0.1, 0.1, 0.05]})
        wat = {}
        for frac, tag in ((1.0, "full"), (0.5, "half")):
            wat[tag] = [dict(name=f"water_{i}", **props(water(cv, frac), WATER)) for i, cv in enumerate(cav)]
        asm["options"][key] = {"parts": parts, "water": wat, "slosh": slosh_dims(cav[0], *nb),
                               "volume_l": round(sum(cv.volume for cv in cav) / 1e6, 3),
                               "torso_clearance_mm": {k: round(v, 1) for k, v in clear.items()
                                                      if k.startswith(key.lower()) or k.endswith(key.lower())}}
        dry = sum(p["mass_kg"] for p in parts)
        wet = dry + sum(w["mass_kg"] for w in wat["full"])
        print(f"{key}: water {asm['options'][key]['volume_l']:.3f} L, empty {dry:.3f} kg, full {wet:.3f} kg")
    asm["leg_clearance_r2"] = legs
    # for render.py: pump boxes, ports, fill necks (mm), and water bodies at 70 % fill (render only)
    asm["geometry"] = {"R1": {"pump": R1_PUMP, "outlet": [*R1_OUTLET, R1_A[4] - BOSS_L], "neck": [R1_NECK[3], 0.0, R1_NECK[0]]},
                       "R2": {"pump": R2_PUMP, "outlets": [[R2_BOX[1] + BOSS_L, s * R2_OUTLET[0], R2_OUTLET[1]] for s in (1, -1)],
                              "necks": [[R2_NECK[0], s * R2_NECK[1], R2_NECK[4]] for s in (1, -1)]}}
    for name, cv in (("water_r1", r1_cav), ("water_r2_left", r2_cav), ("water_r2_right", mirror(r2_cav, Plane.XZ))):
        export_stl(water(cv, 0.7), str(OUT / f"{name}.stl"), tolerance=0.1, angular_tolerance=0.3)
    (OUT / "assembly.json").write_text(json.dumps(asm, indent=1))


if __name__ == "__main__":
    main()
