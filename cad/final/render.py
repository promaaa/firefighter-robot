"""Blender renders of the final design F2 (DESIGN=F1: the old design F1), the print kit and the course BOM parts.
F2 views are f2_* and turntable_f2. F1 views are f1_* and turntable_f1.

    TMPDIR=$HOME/.cache PARTS_DIR=out_sts3215 blender -b --factory-startup -P cad/final/render.py -- \
        third_party/berkeley-humanoid-lite-assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite cad/final/renders \
        [f2_front f2_rear f2_cutaway f2_exploded f2_pod f2_holder f2_head turntable_f2]

Printed parts are the STLs of kit/stl (kit.py). Bought parts are boxes from <out>/layout.json (layout.py) and
<out>/kit.json. The handles and the e-stop are simple shapes, sized from the photos (ESTIMATED).
Set F1_CPU=1 to render on the CPU (when the GPU is busy). TMPDIR holds the turntable frames (about 150 MB).
Needs cad/bhl.py, cad/parts/render_module.py with cad/parts/out_sts3215/, and cad/torso_tank/render.py.
"""
import json
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.dirname(HERE)
sys.path[:0] = [CAD, os.path.join(CAD, "parts"), os.path.join(CAD, "torso_tank")]
import bhl  # noqa: E402
import render_module as rm  # noqa: E402  reads <bhl_dir> <out> after "--"
import render as tr  # noqa: E402  cad/torso_tank/render.py: pump(), cut(), still(), turntable()

F2 = os.environ.get("DESIGN", "F2").upper() == "F2"
D = "f2" if F2 else "f1"
OUTD = os.path.join(HERE, "out_f2" if F2 else "out")
LAY = json.load(open(os.path.join(OUTD, "layout.json")))
KIT = json.load(open(os.path.join(OUTD, "kit.json")))
R1F = None if F2 else json.load(open(os.path.join(OUTD, "r1f.json")))
STL = os.path.join(HERE, "kit" if F2 else "kit_f1", "stl")
COLORS = {"lipo_6s": (0.05, 0.05, 0.06), "lipo_3s": (0.08, 0.1, 0.25), "relay_aep18024": (0.02, 0.02, 0.02),
          "shelf_acrylic": None, "rock3c": (0.05, 0.25, 0.12), "canable": (0.6, 0.45, 0.05), "buck_5v": (0.6, 0.08, 0.06),
          "boost_ham5925": (0.6, 0.08, 0.06), "imu_im10a": (0.15, 0.3, 0.7), "diode_mbrt400100": (0.25, 0.25, 0.27),
          "shunt_100a": (0.7, 0.55, 0.3), "meter": (0.03, 0.03, 0.03), "switch": (0.03, 0.03, 0.03),
          "brake_resistor": (0.55, 0.57, 0.6), "relay_k2": (0.05, 0.05, 0.05), "mosfet": (0.1, 0.2, 0.6),
          "estop_contacts": (0.9, 0.4, 0.05)}
TORSO_PRINTS = ["holder_6s_zeee", "pod_spacer_left", "pod_spacer_right", "pod_basket", "pod_cover", "fill_extension",
                "imu_mount", "frame_clip", "pump_holder", "resistor_mount_left", "resistor_mount_right"]
NOT_BOXES = {"wiring_small", "estop_head", "frame_clips", "rock3c_plugs", "battery_holder", "pod_plate_al2",
             "pod_spacer_left", "pod_spacer_right", "pod_basket", "pod_cover", "fill_extension", "imu_mount",
             "pump_holder", "resistor_mount_left", "resistor_mount_right", "pump_dwp385"} | (set() if F2 else {"filter_inline"})
TILT_AX = Vector([rm.INFO["tilt_axis_mm"][i] / 1000 + (0, 0, rm.T)[i] for i in range(3)])
TILT = Matrix.Translation(TILT_AX) @ Matrix.Rotation(math.radians(rm.TILT_DEG), 4, "Y") @ Matrix.Translation(-TILT_AX)

if os.environ.get("F1_CPU"):            # GPU shared with training jobs: render on the CPU with the same calls
    _render = bhl.render

    def _cpu_render(path, res=None, samples=128):
        sc = bpy.context.scene
        sc.render.engine = "CYCLES"
        sc.cycles.device = "CPU"
        sc.cycles.samples = max(16, samples // 3)
        sc.cycles.use_denoising = True
        if res:
            sc.render.resolution_x, sc.render.resolution_y = res
        sc.render.filepath = path
        bpy.ops.render.render(write_still=True)
    bhl.render = _cpu_render


def material(name, M):
    for k, c in COLORS.items():
        if name.startswith(k):
            if c is None:
                return M["acrylic"]
            if k not in M:
                M[k] = bhl.mat(k, c, rough=0.35, metal=0.6 if k in ("diode_mbrt400100", "shunt_100a", "brake_resistor") else 0.0)
            return M[k]
    return M["part"]


def stl(path, m, mw=Matrix.Identity(4)):
    o = tr._import(path, m)
    o.matrix_world = mw @ o.matrix_world
    return o


def parts(M):
    """Bought parts as boxes, printed torso parts, the pod plate, the e-stop and the handles. Returns {name: [objs]}."""
    M["acrylic"] = bhl.mat("acrylic", (0.9, 0.95, 1.0), rough=0.05, transmission=0.9)
    M["alu"] = bhl.mat("alu", (0.75, 0.76, 0.78), rough=0.3, metal=1.0)
    out = {}
    for n, (x0, x1, y0, y1, z0, z1) in LAY["boxes_mm"].items():
        if n in NOT_BOXES:
            continue
        out[n] = [bhl.box(n, ((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000),
                          ((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), material(n, M), bevel=0.0015)]
    for n, b in KIT["bought"].items():             # meter faces and switch buttons on the cover
        x0, x1, y0, y1, z0, z1 = b["box_mm"]
        if n.endswith("_bezel"):
            out[n] = [bhl.box(n, ((x1 - x0) / 1000, (y1 - y0) / 1000, (z1 - z0) / 1000),
                              ((x0 + x1) / 2000, (y0 + y1) / 2000, (z0 + z1) / 2000), M["black"], bevel=0.001)]
        elif n.startswith("switch"):
            d = (z1 - z0 - 3) / 1000
            out[n + "_button"] = [bhl.cyl(n + "_button", d / 2 + 0.003, 0.008, (x0 / 1000 - 0.006, (y0 + y1) / 2000, (z0 + z1) / 2000),
                                          M["red" if "24v" in n else "green"], rot=(0, 90, 0))]
    for n in TORSO_PRINTS:
        if os.path.exists(os.path.join(STL, f"{n}.stl")) and (n != "pump_holder" or KIT["pump"] == "dwp385"):
            out[n] = [stl(os.path.join(STL, f"{n}.stl"), M["part"])]
    if KIT["pump"] == "dwp385":
        out["pump_dwp385"] = [stl(os.path.join(OUTD, "ref_pump_dwp385.stl"), M["black"])]
    out["pod_plate_al2"] = [stl(os.path.join(OUTD, "pod_plate_al2.stl"), M["alu"])]
    out["straps"] = [bhl.tube("strap", [tuple(v / 1000 for v in p) for p in path], 0.003, M["strap"]) for path in KIT["straps_mm"]]
    # e-stop on the left side face, as in the photos: orange bracket, yellow ring, red head
    x, z = LAY["boxes_mm"]["estop_head"][0] / 1000 + 0.02, (LAY["boxes_mm"]["estop_head"][4] + 20) / 1000
    out["estop"] = [bhl.box("estop_bracket", (0.05, 0.006, 0.05), (x, 0.078, z), M["orange"]),
                    bhl.cyl("estop_ring", 0.022, 0.006, (x, 0.084, z), bhl.mat("yellow", (0.95, 0.75, 0.05)), rot=(90, 0, 0)),
                    bhl.cyl("estop_head", 0.02, 0.018, (x, 0.096, z), M["red"], rot=(90, 0, 0))]
    hs = []                                        # carry handles on the side top rails
    for s in (1, -1):
        for xe in (-0.055, 0.055):
            hs.append(bhl.box("handle_block", (0.024, 0.03, 0.022), (xe, s * 0.065, 0.802), M["black"]))
        hs.append(bhl.tube("handle_grip", [(-0.055, s * 0.065, 0.81), (-0.045, s * 0.065, 0.828), (0.045, s * 0.065, 0.828),
                                           (0.055, s * 0.065, 0.81)], 0.012, M["black"]))
    out["handles"] = hs
    return out


def tank(M):
    if F2:                                         # left tank and water, right tank and water, suction tubes
        objs = []
        for side in ("left", "right"):
            objs += [tr._import(os.path.join(OUTD, f"tank_{side}.stl"), M["tank"]),
                     tr._import(os.path.join(OUTD, f"water_{side}.stl"), M["water"])]
        for n, pts in LAY["tubes_mm"].items():
            objs.append(bhl.tube(n, [tuple(v / 1000 for v in p) for p in pts], 0.003, M["hose"]))
        return objs
    return [tr._import(os.path.join(OUTD, "r1f_tank.stl"), M["tank"]),
            tr._import(os.path.join(OUTD, "water_r1f.stl"), M["water"])] + tr.pump(R1F["pump"]["box_mm"], M, +1)


def top(M):
    """ST3215 pan-tilt of cad/parts without its gun plate and camera brackets, deck_f1, head_f1 with the camera,
    the nozzle and the jet hose, tilted like render_module."""
    drop = ("deck", "payload_plate", "cam_usb32", "cam_mlx90640", "ref_cam_usb32", "ref_cam_thermal")
    objs = []
    for o in rm.load_module(M, (0, 0, rm.T), gun=False):
        if o.name in drop:
            bpy.data.objects.remove(o)
        else:
            objs.append(o)
    objs.append(stl(os.path.join(STL, "deck_f1.stl"), M["part"]))
    objs.append(stl(os.path.join(STL, "head_f1.stl"), M["part"], TILT))
    objs.append(stl(os.path.join(OUTD, "ref_camera_b0201.stl"), M["board"], TILT))
    objs.append(stl(os.path.join(OUTD, "ref_nozzle_brass.stl"), M["brass"], TILT))
    objs.append(stl(os.path.join(STL, "hose_clip.stl"), M["part"]))
    tip = Vector([v / 1000 for v in KIT["head"]["nozzle_tip_mm"]])
    x0 = rm.INFO["pan_axis_mm"][0] / 1000
    zn = tip.z
    via = [TILT @ Vector((x0 - 0.008, 0, zn)), TILT @ Vector((x0 - 0.02, 0, zn)), TILT @ Vector((x0 - 0.045, 0, zn - 0.01)),
           Vector((x0 - 0.05, -0.03, rm.T + 0.03)), Vector((0.04, -0.03, rm.T + 0.02)), Vector((0.025, -0.038, rm.T - 0.01))]
    objs.append(bhl.tube("jet_hose", [tuple(v) for v in via], 0.0032, M["hose"]))
    return objs


def scene():
    bhl.clear()
    bhl.studio()
    M = tr.mats(rm.materials())
    M["strap"] = bhl.mat("strap", (0.02, 0.02, 0.02), rough=0.8)
    M["green"] = bhl.mat("green", (0.05, 0.5, 0.1), rough=0.4)
    bpy.context.scene.render.image_settings.file_format = "JPEG"
    bots = rm.robot(M)
    # the course robot has no shoulder brackets, and F1 replaces the BHL battery, holder, PC and USB board
    bhl.delete_faces(bots["base"], lambda p: (abs(p.y) > 0.046 and p.z > 0.678 and abs(p.x) < 0.0505 and p.z < 0.77)
                     or (-0.031 < p.x < 0.034 and -0.0665 < p.y < 0.0905 and 0.582 < p.z < 0.634)
                     or (abs(p.x) < 0.034 and abs(p.y) < 0.031 and 0.5705 < p.z < 0.614)
                     or (abs(p.x) < 0.0525 and abs(p.y) < 0.059 and 0.639 < p.z < 0.680)
                     or (0.012 < p.x < 0.042 and abs(p.y) < 0.07 and 0.682 < p.z < 0.714))
    return M, bots, parts(M), tank(M), top(M)


def move(objs, d):
    for o in objs:
        o.matrix_world = Matrix.Translation(d) @ o.matrix_world


def view(name):
    M, bots, P, tk, tp = scene()
    flat = [o for v in P.values() for o in v]
    body = list(bots.values()) + flat + tk + tp
    if name == D + "_front":
        tr.still(body, name, -35, 12, 70)
    elif name == D + "_rear":
        tr.still(body, name, 140, 14, 70)
    elif name == D + "_cutaway":
        hide = ("pod_cover", "meter", "switch", "frame_clip") if F2 else ("pump", "pod_cover", "meter", "switch", "frame_clip")
        for n in P:                                # (F1: the pump) and the pod cover hide the inside from this side
            if n.startswith(hide):
                for o in P[n]:
                    o.hide_render = True
        tr.cut(bots["base"], lambda p: p.y < -0.02 and p.z > 0.575)
        if F2:
            for o in tk[2:4]:                      # right tank and its water
                o.hide_render = True
        else:
            for o in tk[:2]:
                tr.cut(o, lambda p: p.y < -0.0)
        box = [(x, y, z) for x in (-0.15, 0.13) for y in (-0.15, 0.15) for z in (0.55, 0.98)]
        tr.still(body, name, -115, 16, 70, res=(1400, 1200), box=box, margin=1.02)
    elif name == D + "_exploded":
        move(tp, (0, 0, 0.30))
        if F2:
            move(tk[:2], (0, 0.10, 0))
            move(tk[2:4], (0, -0.10, 0))
        else:
            move(tk, (0, 0, 0.17))
        move(sum((P[n] for n in ("shelf_acrylic", "relay_aep18024", "boost_ham5925", "imu_im10a", "imu_mount",
                                 "estop_contacts")), []), (0, 0, 0.08))
        move(P["lipo_6s_a"], (0, 0.13, 0))
        move(P["lipo_6s_b"], (0, -0.15, 0))
        if F2:                                     # pump and filter up with the shelf, the e-stop out to the left
            move(P.get("pump_holder", []) + P.get("pump_dwp385", []) + P.get("filter_inline", []), (0, 0, 0.08))
            move(P["estop"], (0, 0.08, 0))
        move(sum((P[n] for n in P if n.startswith(("brake_resistor", "resistor_mount"))), []), (0.08, 0, 0))
        move(P["brake_resistor_1"] + P["brake_resistor_2"], (0.04, 0, 0))
        if not F2:
            move(P.get("pump_holder", []), (0, -0.07, 0))
            move(P.get("pump_dwp385", []), (0, -0.12, 0))
        move(sum((P[n] for n in ("pod_plate_al2", "pod_spacer_left", "pod_spacer_right", "diode_mbrt400100", "shunt_100a",
                                 "rock3c")), []), (-0.06, 0, 0))
        move(sum((P[n] for n in P if n.startswith(("pod_basket", "lipo_3s", "canable", "buck"))), []), (-0.12, 0, 0))
        move(P.get("fill_extension", []), (-0.17, 0, 0))
        move(sum((P[n] for n in P if n.startswith(("pod_cover", "meter", "switch"))), []), (-0.24, 0, 0))
        box = [(x, y, z) for x in (-0.40, 0.22) for y in (-0.30, 0.22) for z in (0.50, 1.30)]
        tr.still(body, name, -50, 18, 70, res=(1400, 1700), box=box, margin=1.02)
    elif name == D + "_pod":                         # cover off: plate, basket, boards, fill extension
        for n in P:
            if n.startswith(("pod_cover", "meter", "switch")):
                for o in P[n]:
                    o.hide_render = True
        box = [(x, y, z) for x in (-0.27, -0.07) for y in (-0.09, 0.09) for z in (0.57, 0.80)]
        tr.still(body, name, 155, 14, 70, res=(1400, 1300), box=box, margin=1.04)
    elif name == D + "_holder":                      # right posts cut open above the rails, pump hidden
        tr.cut(bots["base"], lambda p: p.y < -0.045 and 0.5715 < p.z < 0.72)
        if F2:
            for o in tk[2:]:                       # right tank, its water and the tubes hide the holder
                o.hide_render = True
        for n in P:
            if n.startswith(("pump", "pod_", "lipo_3s", "canable", "buck", "mosfet", "meter", "switch", "frame_clip")):
                for o in P[n]:
                    o.hide_render = True
        box = [(x, y, z) for x in (-0.075, 0.095) for y in (-0.085, 0.085) for z in (0.54, 0.65)]
        tr.still(body, name, -32, 22, 60, res=(1400, 1050), box=box, margin=1.10)
    elif name == D + "_head":
        box = [(x, y, z) for x in (-0.075, 0.03) for y in (-0.05, 0.05) for z in (0.865, 0.97)]
        tr.still(body, name, -38, 24, 70, res=(1200, 1200), box=box, margin=1.08)
    elif os.environ.get("F1_CPU"):                 # CPU: fewer, smaller frames
        tr.turntable(body, "turntable_" + D, frames=96, res=(960, 540))
    else:
        tr.turntable(body, "turntable_" + D)


if __name__ == "__main__":
    os.makedirs(rm.OUT, exist_ok=True)
    for v in sys.argv[sys.argv.index("--") + 3:] or [D + "_" + n for n in ("front", "rear", "cutaway", "exploded", "pod",
                                                                          "holder", "head")]:
        view(v)
