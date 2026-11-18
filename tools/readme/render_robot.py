"""Blender renders of SOBANG-I from the hardware team's STEP. Colours follow the photos of the real robot.

    python tools/readme/step_to_glb.py cad/SOBANG-I_Full_Assembly.step /tmp/sobang.glb   # needs cadquery-ocp
    blender -b --factory-startup -P tools/readme/render_robot.py -- . /tmp/sobang.glb /tmp/renders [views]

Views: hero_light hero_dark side front_stand rear torso actuator (default: all).
The README uses hero_light, side and rear (docs/img/render-*.jpg, resized to 800 px wide).
The camera module needs the STL files of pull request 7 in cad/. Uses sim/firescenes/bhl.py for the studio and camera.
"""
import math
import os
import re
import sys

import bpy
from mathutils import Matrix, Vector

REPO, GLB, OUT = sys.argv[sys.argv.index("--") + 1:][:3]
VIEWS = sys.argv[sys.argv.index("--") + 4:] or ["hero_light", "hero_dark", "side", "front_stand", "rear", "torso",
                                                 "actuator"]
sys.path.insert(0, os.path.join(REPO, "sim", "firescenes"))
import bhl  # noqa: E402

# STEP frame (after the glTF import): up = -y, robot left = +x, forward = -z. To the robot frame: x fwd, y left, z up.
TO_ROBOT = Matrix(((0, 0, -1, 0), (1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 0, 1)))

# material by part name (first match wins). The part name is the mesh name without the ".001" suffix,
# then the names of its parents.
RULES = [
    (r"Bearing-|5972K278|_Ball|_Shield|_InnerHousing|_OuterHousing", "steel"),
    (r"^Aluminum Extrusion", "alu"),
    (r"^Plate Floor", "acrylic"),
    (r"^Hex Standoff", "brass"),
    (r"^MKS Xdrive", "pcb_black"),
    (r"^Jetson Orin Nano", "jetson"),
    (r"^DC-DC", "pcb_blue"),
    (r"^imu_link|^imu", "pcb_blue"),
    (r"^Battery", "lipo"),
    (r"^AEP18024", "black"),
    (r"^MBRT400100", "diode"),
    (r"^Stator Coil", "copper"),
    (r"^Motor Shell|^Stator|^Rotor Ring|^Rotor Mount", "motor"),
    (r"^Flat Foot", "rubber"),
    (r"^IMU Mount|^Jetson Mount", "black"),
    (r"^Input Shaft|^Output Shaft|^Motor Shaft|^Rotor Shaft|^Cycloidal|^Geometry Stud", "steel"),
    (r".*", "red"),                               # printed PLA: housings, legs, feet mounts, frame brackets, mounts
]


def materials():
    m = bhl.mat
    return dict(
        red=m("pla_red", (0.36, 0.008, 0.01), rough=0.48, coat=0.1),
        motor=m("motor", (0.012, 0.012, 0.014), rough=0.38, metal=0.4),
        black=m("black", (0.015, 0.015, 0.017), rough=0.45),
        rubber=m("rubber", (0.02, 0.02, 0.022), rough=0.75),
        alu=m("alu", (0.82, 0.83, 0.85), rough=0.28, metal=1.0),
        steel=m("steel", (0.75, 0.75, 0.77), rough=0.3, metal=1.0),
        brass=m("brass", (0.85, 0.58, 0.28), rough=0.3, metal=1.0),
        copper=m("copper", (0.8, 0.38, 0.18), rough=0.35, metal=1.0),
        acrylic=m("acrylic", (0.92, 0.96, 1.0), rough=0.04, transmission=1.0),
        pcb_black=m("pcb_black", (0.02, 0.02, 0.025), rough=0.35, coat=0.3),
        pcb_blue=m("pcb_blue", (0.02, 0.12, 0.45), rough=0.35, coat=0.3),
        jetson=m("jetson", (0.05, 0.06, 0.06), rough=0.4, metal=0.5),
        lipo=m("lipo", (0.02, 0.025, 0.03), rough=0.5),
        glass=m("cam_glass", (0.01, 0.015, 0.03), rough=0.05, metal=0.2, coat=1.0),
        diode=m("diode", (0.55, 0.56, 0.58), rough=0.35, metal=0.8),
    )


def names(o):
    out = []
    while o:
        out.append(re.sub(r"\.\d{3}$", "", o.name))
        o = o.parent
    return out


def load(M):
    old = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=GLB)
    new = [o for o in bpy.data.objects if o not in old]
    meshes = [o for o in new if o.type == "MESH"]
    roots = {o for o in new if o.parent is None}
    for r in roots:
        r.matrix_world = TO_ROBOT @ r.matrix_world
    bpy.context.view_layer.update()
    for o in meshes:
        ns = names(o)
        key = next(k for pat, k in RULES if any(re.search(pat, n) for n in (ns[:1] if pat != RULES[0][0] else ns)))
        o.data.materials.clear()
        o.data.materials.append(M[key])
        o["part"] = ns[0]
        o["group"] = "/".join(ns[1:3])
        for p in o.data.polygons:
            p.use_smooth = True
    z0 = min((o.matrix_world @ Vector(c)).z for o in meshes for c in o.bound_box)
    for r in roots:
        r.matrix_world = Matrix.Translation((0, 0, -z0)) @ r.matrix_world
    bpy.context.view_layer.update()
    for o in meshes:                              # smooth shading with sharp edges kept
        bpy.context.view_layer.objects.active = o
    return meshes


# Camera module of PR #7 (STL files, not placed in the STEP). The placement on the top frame is MY ASSUMPTION:
# the tilt base on the front rail at the centre, the Rock 3C adapter on the left half, across the front and rear rails.
# Each entry: file, 3x3 rotation STL -> robot frame, offset (m) from the top centre of the torso, material.
# The tilt base and the camera holder are printed in black (Marc, 2026-11-18).
CAMERA = [
    ("camera_tilt_base.stl", ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (0.06, 0, 0), "black"),
    ("camera_tilt_camera.stl", ((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (0.06, 0, 0), "black"),
    ("rock3c_adapter.stl", ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0.07, 0), "red"),
]


def camera(M, top):
    objs = []
    for f, r, t, k in CAMERA:
        old = set(bpy.data.objects)
        bpy.ops.wm.stl_import(filepath=os.path.join(REPO, "cad", f), global_scale=0.001)
        o = next(x for x in bpy.data.objects if x not in old)
        o.matrix_world = Matrix.Translation((t[0], t[1], top + t[2])) @ Matrix(r).to_4x4() @ o.matrix_world  # keeps the import scale
        o.data.materials.clear()
        o.data.materials.append(M[k])
        o["part"], o["group"] = f, "camera"
        objs.append(o)
    # Arducam B0201 (docs/hardware/electrical.md U10), simple shapes. The holder pins are 28 mm apart and end 8 mm in
    # front of the hinge, so I assume a 32 x 32 mm board with an M12 lens. Sizes are ESTIMATED, not from a CAD file.
    x0, z0 = 0.06 + 0.008, top + 0.0464
    objs += [
        bhl.box("cam_board", (0.0016, 0.032, 0.032), (x0 + 0.0008, 0, z0), M["pcb_black"], bevel=0.0003),
        bhl.box("cam_lens_mount", (0.008, 0.015, 0.015), (x0 + 0.0056, 0, z0), M["black"], bevel=0.0008),
        bhl.cyl("cam_lens", 0.007, 0.012, (x0 + 0.0156, 0, z0), M["black"], rot=(0, 90, 0), bevel=0.0005),
        bhl.cyl("cam_glass", 0.0045, 0.0006, (x0 + 0.0216, 0, z0), M["glass"], rot=(0, 90, 0), bevel=0),
        bhl.box("cam_usb", (0.004, 0.012, 0.008), (x0 - 0.002, 0, z0 - 0.01), M["steel"], bevel=0.0004),
    ]
    for o in objs[-5:]:
        o["part"], o["group"] = o.name, "camera"
    bpy.context.view_layer.update()
    return objs


def light_studio():
    bpy.data.materials["floor"].node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.62, 0.63, 0.65, 1)
    for n, e, c in (("key", 260, (1, 0.98, 0.95)), ("fill", 120, (0.95, 0.97, 1)), ("rim_fire", 120, (1, 0.97, 0.94)),
                    ("rim_cool", 200, (0.95, 0.97, 1)), ("glow", 0, (1, 1, 1))):
        bpy.data.objects[n].data.energy, bpy.data.objects[n].data.color = e, c
    bg = bpy.context.scene.world.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value, bg.inputs["Strength"].default_value = (0.8, 0.82, 0.85, 1), 0.6


def stand(M):
    """The lab's 2020 hanging frame of the 2026-11-17 photo (sizes ESTIMATED from the photo), two straps to the top rails."""
    a, objs, w, d, h = M["alu"], [], 0.32, 0.6, 1.08
    for y in (-w, w):
        objs.append(bhl.box("stand_post", (0.02, 0.02, h), (0.02, y, h / 2), a, bevel=0.001))
        objs.append(bhl.box("stand_foot", (d, 0.02, 0.02), (0.02 - d / 2 + 0.25, y, 0.01), a, bevel=0.001))
    objs.append(bhl.box("stand_base", (0.02, 2 * w, 0.02), (-0.33, 0, 0.01), a, bevel=0.001))
    objs.append(bhl.box("stand_top", (0.02, 2 * w + 0.02, 0.02), (0.02, 0, h + 0.01), a, bevel=0.001))
    for y in (-0.08, 0.08):
        objs.append(bhl.tube("stand_strap", [(0.0, y, h), (-0.005, y, 1.05), (0.0, y, 0.84)], 0.003, M["black"]))
    return objs


def still(objs, name, azim, elev, lens, res, box=None, margin=1.06, samples=256):
    bhl.fit(box or bhl.corners(objs), azim, elev, lens, res, margin=margin)
    bpy.context.scene.view_settings.look = "AgX - Punchy"
    bhl.render(os.path.join(OUT, f"{name}.jpg"), samples=samples)


def view(name):
    bhl.clear()
    bhl.studio()
    bpy.context.scene.render.image_settings.file_format = "JPEG"
    M = materials()
    body = load(M)
    if name != "hero_dark":
        light_studio()
    top = max((o.matrix_world @ Vector(c)).z for o in body for c in o.bound_box)
    print("TOP", top)
    body += camera(M, top)
    if name in ("hero_light", "hero_dark"):
        still(body, name, -32, 10, 85, (1600, 2000))
    elif name == "side":
        still(body, name, -90, 4, 85, (1400, 2000))
    elif name == "front_stand":
        s = stand(M)
        for o in s:
            if o.name.startswith("stand_strap"):
                o.data.splines[0].points[-1].co.z = top
        still(body + s, name, 0, 4, 40, (1500, 2000), box=[(0, y, z) for y in (-0.34, 0.34) for z in (-0.08, 1.1)], margin=1.03)
    elif name == "rear":
        still(body, name, 150, 16, 75, (1600, 2000))
    elif name == "torso":
        tor = [o for o in body if o["group"].startswith("base <1>/Assembly") or o["group"] == "Assembly"
               and o["part"] in ("base <1>",)]
        box = [(x, y, z) for x in (-0.11, 0.11) for y in (-0.15, 0.15) for z in (top - 0.24, top + 0.07)]
        still(body, name, -40, 24, 85, (1600, 1400), box=box, margin=1.08)
    elif name == "actuator":                      # one knee actuator (5010) pulled apart along its axis, on the floor
        act = [o for o in body if "Actuator-5010 <1>/leg_left_knee_pitch <1>" in o["group"] or
               any("Actuator-5010" in n for n in names(o)) and "leg_left_knee_pitch <1>" in names(o)]
        keep = set(act)
        for o in body:
            if o not in keep:
                o.hide_render = True
        c = sum((o.matrix_world.translation for o in act), Vector()) / len(act)
        # axis: the direction of largest spread of part centres is not reliable, use the leg y axis (pitch axis)
        order = {}
        for o in act:
            n = names(o)
            top_part = next((x for x in n if x.startswith(("Housing", "Motor Shell", "MKS", "Cycloidal", "Output Shaft",
                                                            "Input Shaft", "Hex", "Motor Shaft", "Bearing", "MAD"))), n[0])
            order.setdefault(top_part, []).append(o)
        ys = {k: sum((o.matrix_world @ Vector(cc)).y for o in v for cc in o.bound_box) / (8 * len(v)) for k, v in order.items()}
        ymid = sum(ys.values()) / len(ys)
        for k, v in order.items():
            d = (ys[k] - ymid) * 4.0
            for o in v:
                o.matrix_world = Matrix.Translation((-c.x, d - c.y + 0, 0.12 - c.z)) @ o.matrix_world
        bpy.context.view_layer.update()
        still(act, name, -60, 22, 85, (2000, 1400), margin=1.1)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for v in VIEWS:
        view(v)
