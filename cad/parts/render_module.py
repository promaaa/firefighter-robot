"""Blender renders of the top module: assembled, exploded, and on the robot.

blender -b --factory-startup -P cad/parts/render_module.py -- <bhl_dir> <out_dir> [assembled exploded robot]

Reads the STLs and out/assembly.json written by top_module.py (PARTS_DIR=out_sts3215: top_module_sts3215.py).
Uses the helpers in cad/bhl.py. cad/final/render.py imports this file.
"""
import json
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import bhl  # noqa: E402

args = sys.argv[sys.argv.index("--") + 1:]
BHL, OUT = args[0], args[1]
WHICH = args[2:] or ["assembled", "exploded", "robot", "adapters", "concept"]
PARTS = os.path.join(HERE, os.environ.get("PARTS_DIR", "out"))   # PARTS_DIR=out_sts3215 for the STS3215 variant
NAME = os.environ.get("RENDER_NAME", "module_assembled")
INFO = json.load(open(os.path.join(PARTS, "assembly.json")))
T, Z0 = 0.791, 0.039          # torso frame top and foot offset
TILT_DEG = 8                  # nose down a little, to show the tilt axis
LEGS_ONLY = {"arm_%s_%s" % (s, j) for s in ("left", "right") for j in
             ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch", "elbow_roll", "hand_link")}

# exploded offsets in mm (x, y, z) per part
EXPLODE = {
    "ref_pan_servo": (0, 0, -45), "deck": (0, 0, 0), "pan_base": (0, 0, 35), "ref_pan_bearing": (0, 0, 70),
    "pan_yoke": (0, 0, 100), "ref_tilt_servo": (0, 70, 100), "ref_tilt_bearing": (0, -45, 100),
    "tilt_u": (0, 0, 165), "ref_tilt_pin": (0, -95, 165), "payload_plate": (0, 0, 215),
    "cam_usb32": (40, 0, 245), "cam_mlx90640": (40, 0, 245), "ref_cam_usb32": (70, 0, 245),
    "ref_cam_thermal": (70, 0, 245), "ref_gun": (0, 0, 300),
}
TILTING = set(INFO["tilting"]) | {"ref_cam_usb32", "ref_cam_thermal"}


def materials():
    m = bhl.mat
    return dict(
        part=m("part", (0.36, 0.022, 0.016), rough=0.45),       # printed PLA, red
        pla=m("pla", (0.07, 0.07, 0.075), rough=0.42),         # robot
        black=m("black", (0.015, 0.015, 0.017), rough=0.45),
        steel=m("steel", (0.75, 0.75, 0.77), rough=0.3, metal=1),
        gun=m("gun", (0.95, 0.3, 0.02), rough=0.35),
        board=m("board", (0.02, 0.12, 0.05), rough=0.4),
        orange=m("orange", (0.95, 0.3, 0.02), rough=0.35),        # gun stand-in
        yellow=m("yellow", (0.9, 0.42, 0.0), rough=0.4),
        clear=m("clear", (0.9, 0.95, 1.0), rough=0.08, transmission=1),
        water=m("water", (0.25, 0.55, 0.9), rough=0.02, transmission=1),
        red=m("red", (0.55, 0.025, 0.02), rough=0.35, coat=0.3),
    )


def gun_standin(M, base, tilt_deg):
    """The placeholder gun of concept A, same shapes and local positions.
    Its body centre sits GUN_COM_DX from the tilt axis, its bottom on the payload plate."""
    from bhl import box, cone, cyl
    ax = [base[i] + INFO["tilt_axis_mm"][i] / 1000 for i in range(3)]
    tilt = bpy.data.objects.new("gun_tilt", None)
    bpy.context.collection.objects.link(tilt)
    tilt.location, tilt.rotation_euler = ax, (0, math.radians(tilt_deg), 0)
    g = bpy.data.objects.new("gun", None)
    bpy.context.collection.objects.link(g)
    g.parent = tilt
    h = INFO["heights_mm"]
    g.location = (INFO["gun_com_dx_mm"] / 1000 - 0.02, 0, (h["plate_top"] - h["tilt_axis"]) / 1000 + 0.0425)
    box("gun_body", (0.24, 0.07, 0.085), (0.02, 0, 0), M["orange"], bevel=0.016, parent=g)
    box("gun_rear", (0.03, 0.062, 0.07), (-0.11, 0, 0), M["black"], bevel=0.006, parent=g)
    cyl("gun_barrel", 0.02, 0.12, (0.2, 0, 0.005), M["orange"], rot=(0, 90, 0), parent=g)
    cyl("gun_shroud", 0.027, 0.035, (0.155, 0, 0.005), M["yellow"], rot=(0, 90, 0), parent=g)
    cone("gun_tip", 0.02, 0.008, 0.03, (0.275, 0, 0.005), M["black"], rot=(0, 90, 0), parent=g)
    cyl("gun_tank", 0.038, 0.17, (-0.005, 0, 0.078), M["clear"], rot=(0, 90, 0), parent=g)
    cyl("gun_water", 0.034, 0.15, (-0.005, 0, 0.078), M["water"], rot=(0, 90, 0), parent=g)
    for x in (-0.09, 0.08):
        cyl("gun_tank_end", 0.04, 0.012, (x, 0, 0.078), M["black"], rot=(0, 90, 0), parent=g)
    cyl("gun_fill", 0.014, 0.016, (-0.05, 0, 0.118), M["red"], parent=g)
    return [o for o in bpy.data.objects if o.name.startswith("gun_") and o.type == "MESH"]


def ref_mat(name, M):
    if "bearing" in name or "pin" in name:
        return M["steel"]
    if "gun" in name:
        return M["gun"]
    if "cam" in name:
        return M["board"]
    return M["black"]


def load_module(M, base, explode=False, gun=True):
    """Import all parts; base = robot-frame position (m) of the module origin. Returns the objects."""
    ax = Vector([base[i] + INFO["tilt_axis_mm"][i] / 1000 for i in range(3)])
    objs = []
    files = [(n, os.path.join(PARTS, n + ".stl")) for n in INFO["parts"] if n not in INFO["alternatives"]]
    files += [(f[:-4], os.path.join(PARTS, "ref", f)) for f in sorted(os.listdir(os.path.join(PARTS, "ref")))]
    for name, path in files:
        if name == "ref_gun" and not gun:
            continue
        bpy.ops.wm.stl_import(filepath=path)
        o = bpy.context.selected_objects[0]
        o.name = name
        o.data.materials.append(ref_mat(name, M) if name.startswith("ref_") else M["part"])
        bpy.ops.object.shade_auto_smooth(angle=math.radians(30))
        off = EXPLODE.get(name, (0, 0, 0)) if explode else (0, 0, 0)
        mw = Matrix.Translation([base[i] + off[i] / 1000 for i in range(3)]) @ Matrix.Scale(0.001, 4)
        if name in TILTING and not explode:  # turn about the tilt axis (y)
            mw = Matrix.Translation(ax) @ Matrix.Rotation(math.radians(TILT_DEG), 4, "Y") @ Matrix.Translation(-ax) @ mw
        o.matrix_world = mw
        objs.append(o)
    return objs


def robot(M):
    objs, _ = bhl.load_robot(f"{BHL}/urdf/berkeley_humanoid_lite.urdf", f"{BHL}/meshes", dict(bhl.STAND), M["pla"],
                             skip=LEGS_ONLY)
    for o in objs.values():
        o.matrix_world = Matrix.Translation((0, 0, -Z0)) @ o.matrix_world
    bhl.delete_faces(objs["base"], lambda p: abs(p.y) > 0.1 and p.z > 0.62 and not (p.z > 0.77 and abs(p.y) < 0.126))  # shoulder motors, keep the top rails
    return objs


if __name__ == "__main__":
    for view in WHICH:
        bhl.clear()
        bhl.studio()
        M = materials()
        bpy.context.scene.render.image_settings.file_format = "JPEG"
        if view == "concept":   # full concept A robot with the real module and the placeholder gun shape
            TILT_DEG = 6          # same aim as concept A
            bots = robot(M)
            objs = load_module(M, (0, 0, T), gun=False) + gun_standin(M, (0, 0, T), TILT_DEG)
            body = bhl.corners(list(bots.values()) + objs)
            bhl.fit(body, -35, 10, 70, (1600, 2000))
            bhl.render(f"{OUT}/concept_A_real_module.jpg", samples=128)
        elif view == "adapters":  # B head and E valve clamp, side by side on the floor
            objs = []
            z = -INFO["heights_mm"]["u_base"][1] / 1000
            for name, dy in (("head_b", 0.07), ("valve_e", -0.07)):
                bpy.ops.wm.stl_import(filepath=os.path.join(PARTS, name + ".stl"))
                o = bpy.context.selected_objects[0]
                o.data.materials.append(M["part"])
                o.matrix_world = Matrix.Translation((-INFO["pan_axis_mm"][0] / 1000, dy, z)) @ Matrix.Scale(0.001, 4)
                objs.append(o)
            bhl.fit(bhl.corners(objs), -40, 24, 85, (1400, 1000), margin=1.1)
            bhl.render(f"{OUT}/adapters_B_E.jpg", samples=96)
        elif view == "robot":
            robot(M)
            objs = load_module(M, (0, 0, T))
            pts = bhl.corners(objs)
            lo = [(x, y, T - 0.25) for x, y, _ in pts[:8]]
            bhl.fit(pts + lo, -35, 14, 70, (1200, 1500), margin=1.15)
            bhl.render(f"{OUT}/module_on_robot.jpg", samples=96)
            bhl.fit(bhl.corners([bpy.data.objects[n] for n in bpy.data.objects.keys() if n.startswith("bhl_")] + objs),
                    -35, 10, 70, (1200, 1500))
            bhl.render(f"{OUT}/module_on_robot_full.jpg", samples=96)
        else:
            lift = 0.03  # pan servo hangs 24 mm under the deck; keep it above the floor
            objs = load_module(M, (0, 0, lift), explode=view == "exploded", gun=view != "assembled")
            if view == "exploded":
                bpy.data.objects["ref_gun"].hide_render = True
                objs.remove(bpy.data.objects["ref_gun"])
            bhl.fit(bhl.corners(objs), -40, 24, 85, (1400, 1400) if view == "assembled" else (1200, 1600), margin=1.08)
            bhl.render(f"{OUT}/{NAME if view == 'assembled' else 'module_' + view}.jpg", samples=96)
            if view == "assembled" and NAME == "module_assembled":
                bhl.fit(bhl.corners(objs), -140, 24, 85, (1400, 1400), margin=1.08)
                bhl.render(f"{OUT}/module_{view}_rear.jpg", samples=96)
