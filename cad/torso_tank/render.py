"""Render the torso tanks R1 and R2 on the legs-only BHL with the STS3215 pan-tilt and a nozzle (Blender 5.2, Cycles).

    blender -b --factory-startup -P cad/torso_tank/render.py -- <bhl_dir> cad/torso_tank/renders [views]

Views: r1_cutaway r1_exploded r2_front r2_rear r2_detail turntable_r1 turntable_r2 (default: all stills).
Run body.py first: it writes the STLs and out/assembly.json. The pump and the hoses are simple shapes.
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.dirname(HERE)
os.environ["PARTS_DIR"] = "out_sts3215"
sys.path[:0] = [os.path.join(CAD, "parts"), CAD]
import bhl  # noqa: E402
import render_module as rm  # noqa: E402  cad/parts/render_module.py (reads <bhl_dir> <out> after "--")

OUT_STL = os.path.join(HERE, "out")
OUT = rm.OUT
GEO = json.load(open(os.path.join(OUT_STL, "assembly.json")))["geometry"]
TILT = 8  # deg nose down, as render_module


def stl(name, m, lift=(0, 0, 0)):
    bpy.ops.wm.stl_import(filepath=os.path.join(OUT_STL, f"{name}.stl"))
    o = bpy.context.selected_objects[0]
    o.name = name
    o.data.materials.append(m)
    o.matrix_world = Matrix.Translation(lift) @ Matrix.Scale(0.001, 4)
    bpy.ops.object.shade_auto_smooth(angle=math.radians(30))
    return o


def mats(M):
    M["tank"] = bhl.mat("tank", (0.55, 0.75, 0.95), rough=0.2, transmission=0.7)     # translucent PETG
    M["brass"] = bhl.mat("brass", (0.85, 0.58, 0.28), rough=0.25, metal=1)
    M["hose"] = bhl.mat("hose", (0.85, 0.45, 0.02), rough=0.6)
    return M


def pump(b, m, ports_x, lift=(0, 0, 0)):
    """TCS M400K-V as a 40 x 26 x 25 mm block with two barbs on the +x or -x face."""
    x0, x1, y0, y1, z0, z1 = [v / 1000 for v in b]
    lx, ly, lz = lift
    objs = [bhl.box("pump", (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2 + lx, (y0 + y1) / 2 + ly, (z0 + z1) / 2 + lz),
                    m["black"], bevel=0.003)]
    xf = x1 if ports_x > 0 else x0
    for dy in (-0.006, 0.006):
        objs.append(bhl.cyl("barb", 0.002, 0.009, (xf + ports_x * 0.0045 + lx, (y0 + y1) / 2 + dy + ly, (z0 + z1) / 2 + lz),
                            m["brass"], rot=(0, 90, 0)))
    return objs


def nozzle(M, lift=0.0):
    """Brass nozzle on the payload plate, tilted with it about the tilt axis."""
    ax = [rm.INFO["tilt_axis_mm"][i] / 1000 + (0, 0, rm.T + lift)[i] for i in range(3)]
    e = bpy.data.objects.new("nozzle_tilt", None)
    bpy.context.collection.objects.link(e)
    e.location, e.rotation_euler = ax, (0, math.radians(TILT), 0)
    top = (rm.INFO["heights_mm"]["plate_top"] - rm.INFO["tilt_axis_mm"][2]) / 1000
    a = bhl.cyl("nozzle_body", 0.008, 0.06, (0.075, 0, top + 0.012), M["brass"], rot=(0, 90, 0), parent=e)
    b = bhl.cone("nozzle_tip", 0.008, 0.003, 0.02, (0.115, 0, top + 0.012), M["brass"], rot=(0, 90, 0), parent=e)
    bpy.context.view_layer.update()
    return [a, b], e.matrix_world @ Vector((0.045, 0, top + 0.012))


def scene(gun_module=True):
    bhl.clear()
    bhl.studio()
    M = mats(rm.materials())
    bpy.context.scene.render.image_settings.file_format = "JPEG"
    bots = rm.robot(M)
    objs = rm.load_module(M, (0, 0, rm.T), gun=False) if gun_module else []
    return M, bots, objs


def guard(M):
    """Splash guard of cad/backpack on the front face."""
    return [_import(os.path.join(CAD, "backpack", "out", f"{n}.stl"), M["red"]) for n in ("guard_upper", "guard_lower")]


def _import(path, m):
    bpy.ops.wm.stl_import(filepath=path)
    o = bpy.context.selected_objects[0]
    o.data.materials.append(m)
    o.matrix_world = Matrix.Scale(0.001, 4)
    return o


def r1(M, lift=(0, 0, 0)):
    g = GEO["R1"]
    objs = [stl("r1_tank", M["tank"], lift), stl("water_r1", M["water"], lift)]
    objs += pump(g["pump"], M, +1, lift)
    p, o, n = g["pump"], [v / 1000 for v in g["outlet"]], [v / 1000 for v in g["neck"]]
    lx, ly, lz = lift
    inlet = (p[1] / 1000 + 0.009, 0.006, (p[4] + p[5]) / 2000)
    outlet = (p[1] / 1000 + 0.009, -0.006, (p[4] + p[5]) / 2000)
    sh = lambda q: tuple(a + b for a, b in zip(q, lift))
    objs.append(bhl.tube("feed", [sh(o), sh((o[0] + 0.012, 0.004, o[2] - 0.006)), sh((inlet[0] + 0.012, inlet[1], inlet[2])), sh(inlet)],
                         0.0032, M["hose"]))
    objs.append(bhl.cyl("stopper", 0.010, 0.016, sh((n[0] - 0.006, 0, n[2])), M["red"], rot=(0, 90, 0)))
    return objs, sh(outlet)


def r2(M):
    g = GEO["R2"]
    objs = [stl(n, M["tank"]) for n in ("r2_tank_left", "r2_tank_right")]
    objs += [stl(n, M["water"]) for n in ("water_r2_left", "water_r2_right")]
    objs.append(stl("r2_pump_holder", M["red"]))
    objs += pump(g["pump"], M, +1)
    p = g["pump"]
    inlet = (p[1] / 1000 + 0.009, -0.094, (p[4] + p[5]) / 2000)
    outlet = (p[1] / 1000 + 0.009, -0.106, (p[4] + p[5]) / 2000)
    (lx, ly, lz), (rx, ry, rz) = [[v / 1000 for v in q] for q in g["outlets"]]
    objs.append(bhl.tube("feed_r", [(rx, ry, rz), (rx + 0.012, ry + 0.004, rz - 0.01), (inlet[0] + 0.014, inlet[1], inlet[2] + 0.004), inlet],
                         0.0032, M["hose"]))
    objs.append(bhl.tube("balance", [(lx, ly, lz), (lx + 0.016, ly - 0.01, lz - 0.02), (0.095, 0.05, 0.556), (0.095, -0.05, 0.556),
                                     (inlet[0] + 0.02, inlet[1] + 0.012, inlet[2] + 0.002), (inlet[0] + 0.004, inlet[1] + 0.001, inlet[2])],
                         0.0032, M["hose"]))
    for x, y, z in g["necks"]:
        objs.append(bhl.cyl("stopper", 0.009, 0.014, (x / 1000, y / 1000, z / 1000 + 0.004), M["red"]))
    return objs, outlet


def cut(o, inside):
    """Delete every face of a merged mesh that has a vertex where inside(world point) is true: a cutaway without
    the long sliver faces that a centre test leaves behind."""
    import bmesh
    bpy.context.view_layer.update()
    bm = bmesh.new()
    bm.from_mesh(o.data)
    mw = o.matrix_world
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if any(inside(mw @ v.co) for v in f.verts)], context="FACES")
    bm.to_mesh(o.data)
    bm.free()


def jet_hose(M, start, tip, via):
    return bhl.tube("jet_hose", [start, *via, tip], 0.0032, M["hose"])


def still(objs, name, azim, elev, lens, res=(1200, 1500), margin=1.08, box=None):
    bhl.fit(box or bhl.corners(objs), azim, elev, lens, res, margin=margin)
    bhl.render(os.path.join(OUT, f"{name}.jpg"), samples=96)


def view(name):
    M, bots, mod = scene()
    body = list(bots.values()) + mod
    if name.startswith("r1") or name == "turntable_r1":
        if name == "r1_exploded":
            for o in mod:
                o.matrix_world = Matrix.Translation((0, 0, 0.30)) @ o.matrix_world
            parts, out = r1(M, lift=(0, 0, 0.16))
            noz, tip = nozzle(M, lift=0.30)
            body += parts + noz + guard(M)
            box = [(x, y, z) for x in (-0.15, 0.15) for y in (-0.15, 0.15) for z in (0.50, 1.24)]
            still(body, name, -40, 22, 70, res=(1200, 1600), box=box, margin=1.03)
            return
        parts, out = r1(M)
        noz, tip = nozzle(M)
        hose = jet_hose(M, out, tip, [(out[0] + 0.03, -0.03, out[2]), (0.015, -0.055, 0.72), (0.015, -0.052, 0.80),
                                      (0.03, -0.03, 0.86)])
        if name == "r1_cutaway":  # torso cut open on the camera side (y < 0), above the hips
            cut(bots["base"], lambda p: p.y < -0.02 and p.z > 0.575)
            box = [(x, y, z) for x in (-0.13, 0.13) for y in (-0.15, 0.15) for z in (0.55, 0.96)]
            still(body + parts + noz, name, -125, 18, 70, res=(1400, 1200), box=box, margin=1.02)
            return
        return turntable(body + parts + noz + [hose] + guard(M), "turntable_r1")
    parts, out = r2(M)
    noz, tip = nozzle(M)
    hose = jet_hose(M, out, tip, [(out[0] + 0.012, out[1] - 0.004, out[2] + 0.02), (0.088, -0.09, 0.66), (0.085, -0.085, 0.78),
                                  (0.06, -0.06, 0.85)])
    body += parts + noz + [hose] + guard(M)
    if name == "r2_front":
        still(body, name, -35, 10, 70)
    elif name == "r2_rear":
        still(body, name, -150, 16, 70)
    elif name == "r2_detail":
        box = [(x, y, z) for x in (-0.09, 0.10) for y in (-0.15, 0.15) for z in (0.50, 0.72)]
        still(body, name, -60, 14, 70, res=(1400, 1100), box=box, margin=1.02)
    else:
        turntable(body, "turntable_r2")


def turntable(objs, name, frames=144, fps=24, res=(1280, 720), samples=12):
    """One full turn, as cad/turntable/turntable.py: frames to a temporary folder, ffmpeg to H.264, frame 1 as JPG."""
    sc = bpy.context.scene
    pts = bhl.corners(objs)
    cx, cy = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
    pivot = bpy.data.objects.new("turntable", None)
    bpy.context.collection.objects.link(pivot)
    pivot.location = (cx, cy, 0)
    bpy.context.view_layer.update()
    for o in list(bpy.data.objects):
        if o.parent is None and o.name not in {"floor", "rig", "turntable"} and o.type in ("MESH", "CURVE", "EMPTY"):
            mw = o.matrix_world.copy()
            o.parent = pivot
            o.matrix_world = mw
    ring = [(cx + (x - cx) * math.cos(a) - (y - cy) * math.sin(a), cy + (x - cx) * math.sin(a) + (y - cy) * math.cos(a), z)
            for x, y, z in pts for a in [i * math.pi / 8 for i in range(16)]]
    bhl.fit(ring, -35, 8, 60, res, margin=1.04)
    sc.frame_start, sc.frame_end, sc.render.fps = 1, frames, fps
    bpy.context.preferences.edit.keyframe_new_interpolation_type = "LINEAR"
    pivot.rotation_euler = (0, 0, 0)
    pivot.keyframe_insert("rotation_euler", index=2, frame=1)
    pivot.rotation_euler = (0, 0, 2 * math.pi)
    pivot.keyframe_insert("rotation_euler", index=2, frame=frames + 1)
    sc.render.use_persistent_data = True
    bhl.render(os.path.join(OUT, f"{name}.jpg"), res, samples=samples)
    tmp = tempfile.mkdtemp(prefix=f"tt_{name}_", dir=os.environ.get("TMPDIR"))
    sc.render.image_settings.file_format = "PNG"
    sc.render.filepath = os.path.join(tmp, "f")
    bpy.ops.render.render(animation=True)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", os.path.join(tmp, "f%04d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", "-preset", "slow",
                    os.path.join(OUT, f"{name}.mp4")], check=True)
    shutil.rmtree(tmp)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for v in sys.argv[sys.argv.index("--") + 3:] or ["r1_cutaway", "r1_exploded", "r2_front", "r2_rear", "r2_detail"]:
        view(v)
