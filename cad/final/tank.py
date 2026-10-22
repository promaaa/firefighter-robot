"""Tanks and deck of the final design F2: two R2 tanks on the side faces of the torso posts, and the 96 mm deck.
DESIGN=F1 builds the old design F1 (tank R1F inside the torso).

    uv run --project cad python cad/final/tank.py              # F2: out_f2/tank_*.step/.stl, out_f2/deck_f1.*, out_f2/r2.json
    DESIGN=F1 uv run --project cad python cad/final/tank.py    # F1: out/r1f_*.step/.stl, out/deck_f1.*, out/r1f.json

The R2 tank shape comes from cad/torso_tank/body.py, with the fixes below and without the inner pocket for the BHL
battery end (the Zeee packs end at |y| = 69, inside the posts). The handles sit on the side top rails, so the deck is
96 mm wide (y +-48) and bolts to the front and rear rails. Same robot frame and units as cad/torso_tank (mm, torso
frame top z = 791).
"""
import importlib.util
import json
import os
import sys
from pathlib import Path

import numpy as np
import trimesh
from build123d import Pos, Rot, export_step, export_stl

HERE = Path(__file__).parent
DESIGN = os.environ.get("DESIGN", "F2").upper()
assert DESIGN in ("F1", "F2"), DESIGN
F2 = DESIGN == "F2"
OUT = HERE / ("out_f2" if F2 else "out")      # F1 outputs stay in out/ and kit_f1/, F2 in out_f2/ and kit/
sys.path[:0] = [str(HERE.parent / "backpack"), str(HERE.parent / "parts")]
_spec = importlib.util.spec_from_file_location("torso_tank_body", HERE.parent / "torso_tank/body.py")
tt = importlib.util.module_from_spec(_spec)  # cad/torso_tank/body.py, loaded by path: its name clashes with
_spec.loader.exec_module(tt)                 # cad/backpack/body.py, which it imports itself
import top_module as tm  # noqa: E402
import top_module_sts3215 as st  # noqa: E402  ST3215 pan-tilt: its deck has a larger servo window

SHELF_TOP = 646.0                                        # cad/final/layout.py: SHELF_Z + 5 (shelf on the bracket bars, z 641)
RELAY_NOTCH = (5.0, 51.0, -40.0, 40.0, 700.0, 727.0)     # relay x 8..48, y +-37.8, top z 724, plus 3 mm
DECK_Y, DECK_SCREW_Y, DECK_SLOT = 96.0, 40.0, (25.0, -38.0)
HANDLE_Y_IN = 50.0                                       # inner face of the handles, ESTIMATED from photos

tt.R1_PILLARS = []
tt.R1_EXTRA_NOTCHES = [RELAY_NOTCH]
tt.R1_CHAN = (3.0, 47.0, -48.0, -28.0, 700.0, 800.0)
tt.R1_LEGS = [(x, y, SHELF_TOP) for x in (44.0, -44.0) for y in (47.0, -47.0)]
tm.DECK_Y, tm.RAIL_SCREW_Y, tm.DECK_SLOT = DECK_Y, DECK_SCREW_Y, DECK_SLOT
# F2 fixes of the cad/torso_tank R2 tank. Its outlet hole (z 601) ends in the lower dry screw tube (z 597..609) and opens
# into the screw bore, not into the water. The counterbores left 1 mm walls in the 12 mm tubes. The neck (x 57, OD 22)
# came 2 mm past the front face. So: lower screws z 611, upper z 663, tubes 14 mm, outlet z 599, neck x 54.
tt.R2_POCKET = None
tt.R2_SCREWS = [(x, z) for x in (-60.0, 60.0) for z in (611.0, 663.0)]
tt.R2_TUBE_D = 14.0
tt.R2_OUTLET = (101.0, 599.0)
tt.R2_NECK = (54.0, 101.0, 22.0, 16.0, 692.0)


def pan_sweep_clearance():
    """Closest approach of the turning pan-tilt parts to the handle boxes over a full pan turn (mm)."""
    src = HERE.parent / "parts/out_sts3215"
    pan = json.loads((src / "assembly.json").read_text())["pan_axis_mm"]
    pts = np.vstack([trimesh.load(src / f"{n}.stl").sample(4000) for n in ("pan_yoke", "tilt_u", "payload_plate", "cam_usb32")])
    pts = pts + [0, 0, tt.T]                 # module frame: deck underside at z = 0
    pts = pts[pts[:, 2] < 836.0 + 5]         # only parts lower than the handle tops can touch them
    if not len(pts):
        return float("inf")
    best = np.inf
    for a in np.radians(np.arange(0, 360, 5)):
        c, s = np.cos(a), np.sin(a)
        x, y = pts[:, 0] - pan[0], pts[:, 1] - pan[1]
        best = min(best, float(HANDLE_Y_IN - np.abs(s * x + c * y + pan[1]).max()))
    return best


def deck_shape():
    return Pos(0, 0, tt.T) * st.deck()   # tm.deck() plus the ST3215 window (tm.deck() alone hits the pan servo)


def check_deck(deck):
    db = deck.bounding_box()
    assert db.max.Y <= HANDLE_Y_IN - 1.5 and db.min.Y >= -HANDLE_Y_IN + 1.5, "deck reaches the handles"
    sweep = pan_sweep_clearance()
    print(f"pan-tilt to handles over a full turn: {sweep:.1f} mm")
    return db, sweep


def main_f2():
    """R2 tanks: left drawn by cad/torso_tank/body.py, right = mirror. Each tank prints as a tub (outer face down)
    and an inner plate, glued with epoxy (cut at y = 78 + wall, as body.py)."""
    from build123d import Plane, mirror
    from scipy.spatial import cKDTree
    import layout
    OUT.mkdir(exist_ok=True)
    left, cav_l = tt.r2_tank()
    tanks = {"left": left, "right": mirror(left, Plane.XZ)}
    cavs = {"left": cav_l, "right": mirror(cav_l, Plane.XZ)}
    deck = deck_shape()
    big, cut = 400.0, tt.R2_BOX[2] + tt.WALL
    prints = {"deck_f1": deck}
    for side, s in tanks.items():
        sg = 1 if side == "left" else -1
        outer = tt.box(-big, big, *sorted((sg * cut, sg * big)), 0, 1000)
        inner = tt.box(-big, big, *sorted((0, sg * cut)), 0, 1000)
        prints[f"tank_{side}_tub"], prints[f"tank_{side}_plate"] = s & outer, s & inner
    for name, s in {"tank_left": tanks["left"], "tank_right": tanks["right"], **prints}.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
        export_step(s, str(OUT / f"{name}.step"))
    for side, cv in cavs.items():
        export_stl(tt.water(cv, 0.7), str(OUT / f"water_{side}.stl"), tolerance=0.1, angular_tolerance=0.3)
    for name in prints:
        m = trimesh.load(OUT / f"{name}.stl")
        assert all(a <= b for a, b in zip(sorted(m.extents), sorted(tt.BED))), f"{name} does not fit the bed"
        assert m.is_watertight, f"{name} STL is not watertight"
        assert len(m.split(only_watertight=False)) == 1, f"{name} is more than one body"
    # torso mesh (course robot: no shoulders, BHL battery and PC removed); the pads touch the posts on purpose
    tp = cKDTree(layout.torso_points())
    torso = {}
    for side in tanks:
        m = trimesh.load(OUT / f"tank_{side}.stl")
        p = np.vstack([m.vertices, m.sample(40000)])
        p = p[np.abs(p[:, 1]) > 75.0 + 2.5]
        torso[side] = round(float(tp.query(p)[0].min()), 2)
    print("tank to torso mesh (mm, pads left out):", torso)
    assert min(torso.values()) >= tt.TORSO_CLEAR_MIN
    db, sweep = check_deck(deck)
    shells = {side: tt.props(s, tt.RHO) for side, s in tanks.items()}
    full = {side: tt.props(tt.water(cv, 1.0), tt.WATER) for side, cv in cavs.items()}
    half = {side: tt.props(tt.water(cv, 0.5), tt.WATER) for side, cv in cavs.items()}
    bb = cav_l.bounding_box()
    out = {"design": "F2", "volume_l": round(sum(cv.volume for cv in cavs.values()) / 1e6, 3),
           "volume_each_l": round(cav_l.volume / 1e6, 3), "shells": shells, "water_full": full, "water_half": half,
           "water_z_mm": {"floor": round(bb.min.Z, 1), "full_surface": round(bb.max.Z, 1)},
           "slosh": tt.slosh_dims(cav_l, len(tt.R2_BAFFLES_X), 0), "box_mm": tt.R2_BOX,
           "screws_xz_mm": tt.R2_SCREWS, "neck_mm": tt.R2_NECK, "outlet_yz_mm": tt.R2_OUTLET,
           "outlet_barb_x_mm": tt.R2_BOX[1] + tt.BOSS_L, "torso_clearance_mm": torso, "deck": tt.props(deck, tt.RHO),
           "deck_mm": {"y": DECK_Y, "screw_y": DECK_SCREW_Y, "slot": DECK_SLOT,
                       "bbox": [round(v, 1) for v in (db.min.X, db.max.X, db.min.Y, db.max.Y)]},
           "pan_sweep_to_handles_mm": round(sweep, 1)}
    (OUT / "r2.json").write_text(json.dumps(out, indent=1))
    print(f"R2: {out['volume_l']} L ({out['volume_each_l']} L each), shells "
          f"{sum(v['mass_kg'] for v in shells.values()) * 1000:.0f} g, prints fit the bed and are closed")


def main():
    if F2:
        return main_f2()
    OUT.mkdir(exist_ok=True)
    tank, cav = tt.r1_tank()
    deck = deck_shape()
    big = 400.0
    prints = {"r1f_half_left": tank & tt.box(-big, big, 0, big, 0, 1000),
              "r1f_half_right": tank & tt.box(-big, big, -big, 0, 0, 1000), "deck_f1": deck}
    for name, s in {"r1f_tank": tank, **prints}.items():
        export_stl(s, str(OUT / f"{name}.stl"), tolerance=0.05, angular_tolerance=0.2)
        export_step(s, str(OUT / f"{name}.step"))
    export_stl(tt.water(cav, 0.7), str(OUT / "water_r1f.stl"), tolerance=0.1, angular_tolerance=0.3)
    for name in prints:
        m = trimesh.load(OUT / f"{name}.stl")
        assert all(a <= b for a, b in zip(sorted(m.extents), sorted(tt.BED))), f"{name} does not fit the bed"
        assert m.is_watertight, f"{name} STL is not watertight"
    db, sweep = check_deck(deck)
    shell = tt.props(tank, tt.RHO)
    full, half = tt.props(tt.water(cav, 1.0), tt.WATER), tt.props(tt.water(cav, 0.5), tt.WATER)
    deck_p = tt.props(deck, tt.RHO)
    out = {"volume_l": round(cav.volume / 1e6, 3), "shell": shell, "water_full": full, "water_half": half,
           "pump": {"mass_kg": tt.PUMP_G / 1000, "box_mm": tt.R1_PUMP}, "deck": deck_p,
           "slosh": tt.slosh_dims(cav, len(tt.R1_BAFFLES_X), len(tt.R1_BAFFLES_Y)),
           "legs_mm": tt.R1_LEGS, "relay_notch_mm": RELAY_NOTCH, "channel_mm": tt.R1_CHAN,
           "deck_mm": {"y": DECK_Y, "screw_y": DECK_SCREW_Y, "slot": DECK_SLOT,
                       "bbox": [round(v, 1) for v in (db.min.X, db.max.X, db.min.Y, db.max.Y)]},
           "pan_sweep_to_handles_mm": round(sweep, 1)}
    (OUT / "r1f.json").write_text(json.dumps(out, indent=1))
    print(f"R1F: {out['volume_l']} L, shell {shell['mass_kg'] * 1000:.0f} g, deck {deck_p['mass_kg'] * 1000:.0f} g solid, "
          f"halves fit the bed and are closed")


if __name__ == "__main__":
    main()
