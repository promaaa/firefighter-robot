"""Final design F2 (default; DESIGN=F1 for the old design F1): the course robot torso with every part of the course
BOM placed. Two R2 side tanks (tank.py), the pump on the middle shelf, the e-stop 50 mm higher than on the lab robot.

    uv run --project cad python cad/final/layout.py          # checks, out_f2/layout.json (F1: out/layout.json)

The torso is the BHL torso (base_visual.stl), checked against 13 photos of the lab's prototype. The photos show the
same 2020 frame, the same battery holder and the same hip actuators. The lab changed: no shoulder actuators, two
carry handles on the side top rails, an e-stop on the left side face, the relay on the middle shelf (the BHL PC
shelf) and the computer at the top level. The layout keeps the lab's bottom and middle levels and moves the
top-level electronics to a rear pod outside the frame.

Robot frame, mm: x forward, y left, z up, feet on the floor, torso frame top z = 791 (same as cad/torso_tank).
Every part is an axis-aligned box: (name, mass g, x0, x1, y0, y1, z0, z1, group, centre). kit.py places the printed
parts and the bought parts around them and writes their boxes, masses and centres to <out>/kit.json. Run kit.py first.
"""
import json
import os
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

HERE = Path(__file__).parent
DESIGN = os.environ.get("DESIGN", "F2").upper()   # the same switch as tank.py and kit.py
F2 = DESIGN == "F2"
OUT = HERE / ("out_f2" if F2 else "out")
ROOT = HERE.parent.parent
sys.path[:0] = [str(HERE.parent / "backpack")]
import body as bp  # noqa: E402  cad/backpack/body.py: BHL path, leg sweep

P = {p["id"]: p for p in json.loads((HERE / "parts.json").read_text())}

# ---------- lab changes seen in the photos (ESTIMATED from photos, check on the robot) ----------
HANDLE = (-62.0, 62.0, 50.0, 80.0, 791.0, 836.0)  # left handle; grip about 30 mm, 45 mm high, on the side top rail
SHELF_Z = 641.0       # middle acrylic shelf, 5 mm, on the bars of the BHL PC shelf brackets (bar top z 641 on the mesh)
# e-stop on the left side face; 3 contact blocks go inside. F2: the left tank fills z 592..678 there, so the e-stop
# moves up to z 720 (head z 700..740, 22 mm over the tank top), still on the left side face.
ESTOP_X, ESTOP_Z = -13.0, (720.0 if F2 else 670.0)

# ---------- placement (mm) ----------
L, W, H = P["relay_aep18024"]["size_mm"]  # 75.5 x 40 x 79, upright as in the photos
RELAY = (8.0, 8.0 + W, -L / 2, L / 2, SHELF_Z + 5, SHELF_Z + 5 + H)
SHELF_BOX = (-70.0, 70.0, -54.5, 54.5, SHELF_Z, SHELF_Z + 5)   # 140 x 109 x 5 mm, drawing in kit/drawings
BOOST = (-28.0, 7.0, -54.0, 11.0, SHELF_Z + 5, SHELF_Z + 5 + 26)  # 65 x 35 x 21 board on 5 mm spacers

# F2 suction tubes (3 x 6 mm), centre lines: each tank outlet barb (x 73, y +-101, z 599) forward to x 80, across the
# front of the front posts to a tee at y 46, up to z 660, back over the shelf (between the relay and the left post)
# into the filter at x 6. Used for the leg check and the renders. Filter to pump inlet is inside the frame.
TUBE_R = 3.0
TUBES = {"tube_left": [(73.0, 101.0, 599.0), (80.0, 101.0, 599.0), (80.0, 46.0, 599.0)],
         "tube_right": [(73.0, -101.0, 599.0), (80.0, -101.0, 599.0), (80.0, 46.0, 599.0)],
         "tube_tee_filter": [(80.0, 46.0, 599.0), (80.0, 46.0, 660.0), (6.0, 46.0, 660.0)]}


def tube_boxes():
    """Axis-aligned boxes of the tube segments (radius TUBE_R)."""
    out = {}
    for n, pts in TUBES.items():
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            lo = [min(u, v) - TUBE_R for u, v in zip(a, b)]
            hi = [max(u, v) + TUBE_R for u, v in zip(a, b)]
            out[f"{n}_{i}"] = (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2])
    return out


def tube_length(pts):
    return sum(float(np.linalg.norm(np.subtract(b, a))) for a, b in zip(pts, pts[1:]))


# Parts placed here; the printed parts and the bought parts around them come from <out>/kit.json (kit.py).
STATIC = [
    ("shelf_acrylic", 91, *SHELF_BOX, "power"),                                    # 140 x 109 x 5 mm acrylic
    ("relay_aep18024", 400, *RELAY, "power"),
    ("estop_contacts", 60, ESTOP_X - 15, ESTOP_X + 15, 22.0, 75.0, ESTOP_Z - 15, ESTOP_Z + 15, "power"),
    ("estop_head", 0, ESTOP_X - 20, ESTOP_X + 20, 75.0, 110.0, ESTOP_Z - 20, ESTOP_Z + 20, "power"),
    ("boost_ham5925", 45, *BOOST, "power"),
    # wiring, connectors, small screws, lumped at the torso centre (GUESS)
    ("wiring_small", 300, -40.0, 40.0, -40.0, 40.0, 600.0, 700.0, "power"),
]
KIT_GROUP = {"lipo": "battery", "rock3c": "compute", "canable": "compute", "buck": "compute", "imu": "compute",
             "pump": "water_system", "filter": "water_system"}
# printed parts of kit.py in the torso layout (the top parts go to TOP), and the containers: kit.py checks them
# with solids, so the box checks here skip them
KIT_TORSO = ("holder_6s_zeee", "pod_spacer_left", "pod_spacer_right", "pod_basket", "pod_cover", "fill_extension",
             "imu_mount", "pod_plate_al2", "pump_holder", "resistor_mount_left", "resistor_mount_right")
CONTAINERS = {"battery_holder", "pod_basket", "pod_cover", "imu_mount", "pod_plate_al2", "pod_spacer_left",
              "pod_spacer_right", "fill_extension", "pump_holder", "resistor_mount_left", "resistor_mount_right"}


def kit_parts():
    """(name, mass g, x0, x1, y0, y1, z0, z1, group, com or None) from <out>/kit.json."""
    kit = json.loads((OUT / "kit.json").read_text())
    rows = [(*r, None) for r in STATIC]
    for n, b in kit["bought"].items():
        if b["mass_g"] or n == "rock3c_plugs":
            g = next((v for k, v in KIT_GROUP.items() if n.startswith(k)), "power")
            rows.append((n, b["mass_g"], *b["box_mm"], g, None))
    for n in KIT_TORSO:
        if n not in kit["parts"]:
            continue                     # pump_holder only with the DWP-385
        p = kit["parts"][n]
        rows.append(("battery_holder" if n.startswith("holder") else n, p["mass_g"] * p.get("qty", 1), *p["box_mm"],
                     "printed", p["com_mm"]))
    clip = kit["parts"]["frame_clip"]   # 8 cable clips on the posts, lumped with the wiring
    rows.append(("frame_clips", clip["mass_g"] * clip["qty"], -40.0, 40.0, -40.0, 40.0, 600.0, 700.0, "printed", None))
    top = [(t["name"], t["mass_g"], tuple(t["com_mm"]), tuple(t["size_mm"])) for t in kit["top"]]
    return rows, top


# Removed from the BHL base link mass (the URDF base includes them): the BHL 6S pack and the Beelink PC.
BHL_REMOVED = [("bhl_battery", 603, (1.5, 11.5, 608.0)), ("bhl_pc", 280, (-0.5, 0.0, 659.0))]
BARE_KG = 11.34267   # bare biped: sum of <mass> in the BHL biped URDF (as sim/payload/models.py)

LEG_PARTS = ("lipo_6s_a", "lipo_6s_b", "battery_holder", "pod_cover", "estop_head", "brake_resistor_1",
             "brake_resistor_2", "pump_dwp385")
CLEAR_MIN = 1.0      # mm between two parts
TORSO_MIN = 1.0      # mm to the kept torso mesh


def removed(p):
    """Torso mesh points that the course robot does not have, or that the layout replaces: the BHL battery and its holder,
    the Beelink PC, the USB board, the shoulders. The PC shelf brackets (bars, tongues) stay: the shelf rests on them."""
    x, y, z = p.T
    bhl_battery = (x > -31) & (x < 34) & (y > -66.5) & (y < 90.5) & (z > 582) & (z < 634)
    bhl_holder = (np.abs(x) < 34) & (np.abs(y) < 31) & (z > 570.5) & (z < 614)
    pc = (np.abs(x) < 52.5) & (np.abs(y) < 59) & (z > 639) & (z < 680)
    usb = (x > 12) & (x < 42) & (np.abs(y) < 70) & (z > 682) & (z < 714)
    shoulders = ((np.abs(y) > 46) & (z > 678) & (np.abs(x) < 50.5)) | ((np.abs(y) > 75.5) & (z > 600))
    return bhl_battery | bhl_holder | pc | usb | shoulders


def torso_points():
    m = trimesh.load(bp.BHL / "meshes/base_visual.stl")
    m.apply_translation([-0.0030965, 0.000264765, 0.675059 - 0.039])
    m.apply_scale(1000)
    p = np.vstack([m.vertices, trimesh.sample.sample_surface(m, 300000, seed=0)[0]])   # seeded: same result each run
    return p[~removed(p)]


def box_mesh(b):
    x0, x1, y0, y1, z0, z1 = b
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0],
                                transform=trimesh.transformations.translation_matrix([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]))


def gap(a, b):
    """Gap between two boxes; negative = overlap depth."""
    d = [max(a[2 * i] - b[2 * i + 1], b[2 * i] - a[2 * i + 1]) for i in range(3)]
    return max(d)


def lump(parts):
    """One box with the mass, centre and the Ixx, Iyy, Izz about that centre of several boxes (dicts in m), for the
    simulation: sim/payload and sim/rl expect one "water" body. Sizes give the same inertia as the parts."""
    m = sum(p["mass_kg"] for p in parts)
    c = sum(p["mass_kg"] * np.array(p["com_m"]) for p in parts) / m
    second = np.zeros(3)                       # sum m (s^2 / 12 + d^2) along each axis
    for p in parts:
        d = np.array(p["com_m"]) - c
        second += p["mass_kg"] * (np.array(p["size_m"]) ** 2 / 12 + d ** 2)
    return {"mass_kg": round(m, 4), "com_m": [round(v, 4) for v in c], "size_m": [round(v, 4) for v in np.sqrt(12 * second / m)]}


def main():
    OUT.mkdir(exist_ok=True)
    if F2:
        tk = json.loads((OUT / "r2.json").read_text())
        r1 = {f"tank_{s}": trimesh.load(OUT / f"tank_{s}.stl") for s in ("left", "right")}
    else:
        tk = json.loads((OUT / "r1f.json").read_text())
        r1 = {"r1f_tank": trimesh.load(OUT / "r1f_tank.stl")}   # the pump (or the filter) is a kit part now
    PARTS, TOP = kit_parts()
    boxes = {n: (x0, x1, y0, y1, z0, z1) for n, _, x0, x1, y0, y1, z0, z1, _, _ in PARTS}
    tubes = tube_boxes() if F2 else {}
    fails = []

    # 1. parts against each other (boxes) and against the tanks (meshes)
    lumped = {"wiring_small", "frame_clips"}
    names = [n for n in boxes if n not in lumped]
    on_shelf = {"shelf_acrylic", "relay_aep18024", "boost_ham5925"}
    on_plate = {"pod_plate_al2", "diode_mbrt400100", "shunt_100a", "rock3c", "rock3c_plugs", "relay_k2"}   # plate or keep-out
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            g = gap(boxes[a], boxes[b])
            if g < CLEAR_MIN and not {a, b} & CONTAINERS and not {a, b} <= on_shelf and not {a, b} <= on_plate \
                    and {a, b} != {"estop_contacts", "estop_head"}:
                fails.append(f"{a} and {b}: gap {g:.1f} mm")
    print("tank clearance:")
    for n, m in r1.items():
        tree = cKDTree(np.vstack([m.vertices, m.sample(40000)]))
        for b in names:
            if b in CONTAINERS | {"shelf_acrylic"}:
                continue  # legs stand on the shelf; the pod plate and cover have a notch and a hole for the neck: kit.py
                          # checks these printed parts against the R1F halves as solids
            bm = box_mesh(boxes[b])
            d = tree.query(np.vstack([bm.vertices, bm.sample(4000)]))[0].min()
            inside = m.contains(bm.vertices).any() if m.is_watertight else False
            if d < 15 or inside:
                print(f"  {n:12s} - {b:18s} {d:5.1f} mm" + ("  (corner inside the tank)" if inside else ""))
            if d < CLEAR_MIN or inside:
                fails.append(f"{n} and {b}: {d:.1f} mm")

    # 2. parts against the kept torso mesh (frame, shelf brackets, hip actuators, rear bump)
    tp = cKDTree(torso_points())
    torso = {}
    for b in names:
        if b in CONTAINERS | {"shelf_acrylic"} or (F2 and b == "pump_dwp385"):
            continue  # these rest on the frame on purpose; kit.py checks the printed meshes (and the pump shape)
        bm = box_mesh(boxes[b])
        d, i = tp.query(np.vstack([bm.vertices, bm.sample(6000)]))
        torso[b] = float(d.min())
        if torso[b] < TORSO_MIN:
            fails.append(f"{b} is {torso[b]:.1f} mm from the torso mesh")
    print("torso clearance (mm):", {k: round(v, 1) for k, v in sorted(torso.items(), key=lambda kv: kv[1])[:8]})

    # 2b. F2 suction tubes: against the parts (boxes), the tanks (the first segment starts at the barb) and the frame
    tube_gaps = {}
    for t, tb in tubes.items():
        bm = box_mesh(tb)
        q = np.vstack([bm.vertices, bm.sample(2000)])
        d = {b: gap(tb, boxes[b]) for b in names if b != "filter_inline"}
        d["torso mesh"] = float(tp.query(q)[0].min())
        if not t.endswith("_0"):
            for n, m in r1.items():
                d[n] = float(cKDTree(m.vertices).query(q)[0].min())
        k = min(d, key=d.get)
        tube_gaps[t] = (k, round(d[k], 1))
        if d[k] < CLEAR_MIN:
            fails.append(f"{t} is {d[k]:.1f} mm from {k}")
    if tubes:
        print("tube clearance (closest part, mm):", tube_gaps)

    # 3. legs: parts outside the frame, over the leg range with the knee limit used by the sims
    pts = {n: np.vstack([box_mesh(boxes[n]).vertices, box_mesh(boxes[n]).sample(3000)]) for n in LEG_PARTS}
    for n, m in (r1.items() if F2 else ()):    # the R2 tanks sit outside the frame
        pts[n] = np.vstack([m.vertices, m.sample(30000)])
    for n, b in tubes.items():
        pts[n] = np.vstack([box_mesh(b).vertices, box_mesh(b).sample(2000)])
    legs = {}
    for label, kmax in (("full URDF range", None), (f"knee <= {bp.KNEE_MAX} rad", bp.KNEE_MAX)):
        worst = bp.sweep(pts, knee_max=kmax)
        legs[label] = {n: (round(float(d), 1) if np.isfinite(d) else None) for n, (d, _, _) in worst.items()}
        print(f"leg clearance, {label}:", legs[label])
    # Over the full URDF range the foot folds up behind the hip (knee near 2.4 rad) and touches the battery ends and
    # the pod. The BHL battery end does the same (0.3 mm at knee 2.44 rad). So the robot needs the knee limit of the sims.
    for n, d in legs[f"knee <= {bp.KNEE_MAX} rad"].items():
        if d is not None and d < bp.LEG_CLEAR_MIN:
            fails.append(f"{n} is {d} mm from a leg (knee <= {bp.KNEE_MAX} rad)")

    # 4. mass and centre of mass
    items = [(n, m / 1000, tuple(c) if c else ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), g)
             for n, m, x0, x1, y0, y1, z0, z1, g, c in PARTS if m > 0]
    items += [(n, m / 1000, c, s, "top") for n, m, c, s in TOP]
    if F2:
        items += [(f"r2_shell_{s}", v["mass_kg"], tuple(u * 1000 for u in v["com_m"]), tuple(u * 1000 for u in v["size_m"]),
                   "water_system") for s, v in tk["shells"].items()]
        # GUESS as R2 round 5: 8 M5 x 55 screws and T-nuts, barbs, tee, 1.3 m of tubing with water
        items += [("tank_hardware_tubing", 0.140, (20.0, 0.0, 640.0), (100.0, 200.0, 60.0), "water_system")]
        wf, wh = lump(list(tk["water_full"].values())), lump(list(tk["water_half"].values()))
    else:
        items += [("r1f_shell", tk["shell"]["mass_kg"], tuple(v * 1000 for v in tk["shell"]["com_m"]),
                   tuple(v * 1000 for v in tk["shell"]["size_m"]), "water_system"),
                  ("tank_hardware_tubing", 0.045, (0.0, 0.0, 745.0), (100.0, 100.0, 50.0), "water_system")]  # GUESS as R1
        wf, wh = tk["water_full"], tk["water_half"]
    water = [("water", wf["mass_kg"], tuple(v * 1000 for v in wf["com_m"]), tuple(v * 1000 for v in wf["size_m"]), "water")]
    removed_m = sum(m for _, m, _ in BHL_REMOVED) / 1000
    removed_c = sum(m * np.array(c) for _, m, c in BHL_REMOVED) / 1000 / removed_m
    add = items + water
    add_m = sum(i[1] for i in add)
    add_c = sum(i[1] * np.array(i[2]) for i in add) / add_m
    groups = {}
    for n, m, c, s, g in add:
        groups[g] = groups.get(g, 0) + m
    print("added mass by group (kg):", {k: round(v, 3) for k, v in groups.items()})
    print(f"added {add_m:.3f} kg at {np.round(add_c, 1)} mm, removed BHL parts {removed_m:.3f} kg at {np.round(removed_c, 1)} mm")

    print(f"total robot: {BARE_KG - removed_m + add_m:.2f} kg (bare biped {BARE_KG:.2f} kg from the BHL model)")
    layout = {"design": DESIGN,
              "note": f"{DESIGN} layout, robot frame in m. Sizes from cad/final/parts.json, positions from cad/final/layout.py. "
                      "F2: 'water' lumps both tanks (inertia-equivalent box).",
              "items": [{"name": n, "mass_kg": round(m, 4), "com_m": [round(v / 1000, 4) for v in c],
                         "size_m": [round(v / 1000, 4) for v in s], "group": g} for n, m, c, s, g in add],
              "bhl_removed": [{"name": n, "mass_kg": m / 1000, "com_m": [v / 1000 for v in c]} for n, m, c in BHL_REMOVED],
              "boxes_mm": boxes, "handles_mm": [HANDLE, (HANDLE[0], HANDLE[1], -HANDLE[3], -HANDLE[2], HANDLE[4], HANDLE[5])],
              "water_half": {"mass_kg": wh["mass_kg"], "com_m": wh["com_m"], "size_m": wh["size_m"]},
              "volume_l": tk["volume_l"],
              "torso_clearance_mm": torso, "leg_clearance_mm": legs, "tube_clearance_mm": tube_gaps, "fails": fails}
    if F2:
        layout["tubes_mm"] = TUBES
        layout["suction_mm"] = {"left_tank_to_filter": round(tube_length(TUBES["tube_left"]) + tube_length(TUBES["tube_tee_filter"])),
                                "right_tank_to_filter": round(tube_length(TUBES["tube_right"]) + tube_length(TUBES["tube_tee_filter"])),
                                "filter_and_filter_to_pump_GUESS": 60}
        layout["water_z_mm"] = tk["water_z_mm"]
        layout["tube_boxes_mm"] = tubes
    (OUT / "layout.json").write_text(json.dumps(layout, indent=1))
    print("\n".join(["FAIL " + f for f in fails]) or "all checks passed")


if __name__ == "__main__":
    main()
