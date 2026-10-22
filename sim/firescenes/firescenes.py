"""Synthetic fire scenes for the fire detector: an RGB image, a thermal-like image and labels per sample (see README.md).
The camera pose is the RGB camera of aim/geometry.py CONCEPTS["A"] (an older pan-tilt design), 90 deg HFOV.

blender -b --factory-startup -P sim/firescenes/firescenes.py -- samples FIRST COUNT
blender -b --factory-startup -P sim/firescenes/firescenes.py -- sequence
blender -b --factory-startup -P sim/firescenes/firescenes.py -- sheet
blender -b --factory-startup -P sim/firescenes/firescenes.py -- gunsamples FIRST COUNT   (needs cad/parts/render_module.py and cad/parts/out/)

Sample i always gives the same scene (seed = i).
"""
import json
import math
import os
import random
import shutil
import struct
import subprocess
import sys
import zlib

import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path[:0] = [os.path.join(ROOT, "cad"), os.path.join(ROOT, "aim")]
import bhl  # noqa: E402
import geometry as G  # noqa: E402

OUT = os.path.join(HERE, "out")
TMP = os.path.join(OUT, "tmp")
RES = {"rgb": (1280, 720, 90), "thermal_render": (1280, 960, 56)}  # width, height, HFOV (deg)
LOWRES = {"thermal_256": 5, "thermal_160": 8}  # area-average factor from the 1280x960 thermal render
TILT0 = math.radians(-6)  # concept A gun default tilt (aim/geometry.py), negative = down
SHAKE = math.radians(1.5)  # random camera shake on pan and tilt
KELVIN = 273.15
SPP = {"rgb": 16, "thermal": 8, "thermal_low": 4}  # Cycles samples per pixel; the low-res thermal is averaged anyway


def camera_cfg():
    """Concept A camera from aim/geometry.py, with one intrinsics entry per image."""
    cfg = dict(G.CONCEPTS["A"])
    rgb = cfg["cameras"]["rgb"]
    K = {"rgb": G.intrinsics_from_fov(1280, 720, 90), "thermal_render": G.intrinsics_from_fov(1280, 960, 56),
         "thermal_256": G.intrinsics_from_fov(256, 192, 56), "thermal_160": G.intrinsics_from_fov(160, 120, 56)}
    cfg["cameras"] = {k: dict(rgb, K=v) for k, v in K.items()}
    return cfg


CFG = camera_cfg()


# ---------- small helpers ----------
def png(path, a):
    """Write a uint16 grey (h, w) or uint8 RGB (h, w, 3) array as PNG, standard library only."""
    h, w = a.shape[:2]
    depth, ctype = (16, 0) if a.dtype == np.uint16 else (8, 2)
    raw = b"".join(b"\x00" + r.tobytes() for r in (a.astype(">u2") if depth == 16 else a))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, depth, ctype, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


LUT_X = [0, 0.25, 0.5, 0.75, 1]  # simple black-purple-red-yellow-white colour map for previews
LUT_C = np.array([[0, 0, 0], [80, 20, 120], [200, 50, 40], [250, 180, 30], [255, 255, 255]])


def preview(t_c, lo=15.0, hi=80.0):
    """8-bit colour picture of a temperature image (deg C), fixed range lo..hi."""
    x = np.clip((t_c - lo) / (hi - lo), 0, 1)
    return np.stack([np.interp(x, LUT_X, LUT_C[:, i]) for i in range(3)], -1).astype(np.uint8)


def nodes(m):
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        if n.type != "OUTPUT_MATERIAL":
            nt.nodes.remove(n)
    return nt, nt.nodes["Material Output"]


def textured(name, c, rng):
    """Principled material with a soft noise variation of colour c."""
    m = bhl.mat(name, c, rough=rng.uniform(0.4, 0.9))
    nt = m.node_tree
    noise, ramp = nt.nodes.new("ShaderNodeTexNoise"), nt.nodes.new("ShaderNodeValToRGB")
    noise.inputs["Scale"].default_value = rng.uniform(2, 12)
    ramp.color_ramp.elements[0].color = (*[v * 0.75 for v in c], 1)
    ramp.color_ramp.elements[1].color = (*c, 1)
    nt.links.new(noise.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], nt.nodes["Principled BSDF"].inputs["Base Color"])
    return m


def tag(o, temp_c, fire=0):
    o["temp_c"], o["fire"] = float(temp_c), fire
    return o


def seg_dist(p, a, b):
    ab, ap = b - a, p - a
    s = max(0.0, min(1.0, ap.dot(ab) / max(ab.length_squared, 1e-9)))
    return (ap - ab * s).length


# ---------- scene parts ----------
def teardrop(name, x, y, width, height, t, rng):
    """Flame-shaped mesh with noise displacement. t (s) moves the noise up and changes the height (flicker)."""
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=20, radius=1)
    o = bpy.context.object
    o.name = name
    for v in o.data.vertices:
        z = v.co.z
        k = (1 - z) ** 0.6 if z > 0 else 1.0
        v.co.x, v.co.y, v.co.z = v.co.x * k, v.co.y * k, z * (2.5 if z > 0 else 0.6)
    sz = height / 3.1 * (1 + 0.12 * math.sin(2 * math.pi * rng.uniform(1.5, 3) * t + rng.uniform(0, 6)))
    o.scale = (width / 2, width / 2, sz)
    o.location = (x, y, 0.6 * sz)
    o.visible_shadow = False  # a flame casts no shadow
    e = bpy.data.objects.new(name + "_noise", None)
    bpy.context.collection.objects.link(e)
    e.location = (rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-50, 50) - 1.5 * height * t)
    tex = bpy.data.textures.new(name, "CLOUDS")
    tex.noise_scale = 0.3 * width
    d = o.modifiers.new("flicker", "DISPLACE")
    d.texture, d.texture_coords, d.texture_coords_object, d.strength, d.mid_level = tex, "OBJECT", e, 1.0, 0.5
    bpy.ops.object.shade_smooth()
    return o


def flame(idx, x, y, width, height, t, rng, temp):
    """One fire: a main flame and 1-2 smaller tongues, yellow-to-red emission, an orange point light."""
    m = bpy.data.materials.new(f"flame{idx}")
    nt, out = nodes(m)
    tc, sep, ramp, em = (nt.nodes.new(n) for n in ("ShaderNodeTexCoord", "ShaderNodeSeparateXYZ", "ShaderNodeValToRGB", "ShaderNodeEmission"))
    els = ramp.color_ramp.elements
    els[0].color, els[1].color, els[1].position = (1, 0.85, 0.45, 1), (0.8, 0.1, 0.02, 1), 1.0
    els.new(0.3).color = (1, 0.45, 0.05, 1)
    em.inputs["Strength"].default_value = rng.uniform(2, 6)
    nt.links.new(tc.outputs["Generated"], sep.inputs[0])
    nt.links.new(sep.outputs["Z"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], em.inputs["Color"])
    nt.links.new(em.outputs[0], out.inputs["Surface"])
    parts = [(0, 0, 1, 1)] + [(rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(0.4, 0.6), rng.uniform(0.5, 0.8))
                              for _ in range(rng.randint(1, 2))]
    for k, (dx, dy, fw, fh) in enumerate(parts):
        o = teardrop(f"fire{idx}_{k}", x + dx * width, y + dy * width, width * fw, height * fh, t, rng)
        o.data.materials.append(m)
        o["temp_lo"], o["temp_hi"] = temp
        tag(o, temp[1], idx)
    bpy.ops.object.light_add(type="POINT", location=(x, y, height * 0.6))
    bpy.context.object.data.energy, bpy.context.object.data.color = 300 * width / 0.3, (1, 0.5, 0.15)


def smoke(x, y, width, rng):
    bpy.ops.mesh.primitive_cube_add(size=1, location=(x, y, 0.9))
    o = bpy.context.object
    o.name, o.scale = "smoke", (width * 3 + 0.5, width * 3 + 0.5, 1.6)
    m = bpy.data.materials.new("smoke")
    nt, out = nodes(m)
    vol, noise = nt.nodes.new("ShaderNodeVolumePrincipled"), nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 3
    g = rng.uniform(0.15, 0.5)
    vol.inputs["Color"].default_value = (g, g, g, 1)
    mul = nt.nodes.new("ShaderNodeMath")
    mul.operation, mul.inputs[1].default_value = "MULTIPLY", rng.uniform(0.5, 3)
    nt.links.new(noise.outputs["Fac"], mul.inputs[0])
    nt.links.new(mul.outputs[0], vol.inputs["Density"])
    nt.links.new(vol.outputs[0], out.inputs["Volume"])
    o.data.materials.append(m)
    o["smoke"] = True
    return o


def human(x, y, rng):
    """Standing person from primitives, 34 deg C."""
    cloth = bhl.mat(f"cloth{x:.2f}", [rng.uniform(0.05, 0.6) for _ in range(3)], rough=0.8)
    skin = bhl.mat("skin", rng.choice([(0.8, 0.6, 0.45), (0.55, 0.38, 0.25), (0.3, 0.2, 0.13)]), rough=0.5)
    yaw = rng.uniform(0, 360)
    root = bpy.data.objects.new("human", None)
    bpy.context.collection.objects.link(root)
    parts = [bhl.cyl("leg", 0.075, 0.85, (0, s * 0.1, 0.425), cloth) for s in (-1, 1)]
    parts += [bhl.box("torso", (0.22, 0.4, 0.62), (0, 0, 1.16), cloth, bevel=0.05)]
    parts += [bhl.cyl("arm", 0.05, 0.66, (0, s * 0.26, 1.12), cloth) for s in (-1, 1)]
    parts += [bhl.sphere("head", 0.11, (0, 0, 1.62), skin, scale=(1, 0.9, 1.15))]
    for p in parts:
        p.parent = root
        tag(p, 34.0)
    root.location, root.rotation_euler.z = (x, y, 0), math.radians(yaw)


def lamp(x, y, rng):
    """Floor lamp: pole, glowing shade (80-200 deg C) and a point light."""
    tag(bhl.cyl("lamp_pole", 0.015, 1.4, (x, y, 0.7), bhl.mat("pole", (0.2, 0.2, 0.2))), 25)
    shade = bhl.mat(f"shade{x:.2f}", (1, 0.85, 0.6), emission=(1, 0.8, 0.5), strength=rng.uniform(3, 10))
    tag(bhl.cone("lamp_shade", 0.18, 0.1, 0.22, (x, y, 1.5), shade), rng.uniform(80, 200))
    bpy.ops.object.light_add(type="POINT", location=(x, y, 1.45))
    bpy.context.object.data.energy, bpy.context.object.data.color = rng.uniform(50, 200), (1, 0.85, 0.65)


def red_thing(x, y, rng, ambient):
    c = rng.choice([(0.6, 0.03, 0.02), (0.9, 0.3, 0.02), (0.8, 0.1, 0.05), (0.95, 0.45, 0.05)])
    m = bhl.mat(f"red{x:.2f}", c, rough=0.4)
    kind = rng.choice(["box", "cyl", "cone"])
    if kind == "box":
        s = (rng.uniform(0.3, 0.8), rng.uniform(0.3, 0.8), rng.uniform(0.3, 0.9))
        o = bhl.box("red_box", s, (x, y, s[2] / 2), m, rot=(0, 0, rng.uniform(0, 90)))
    elif kind == "cyl":
        o = bhl.cyl("red_cylinder", 0.09, 0.55, (x, y, 0.275), m)
    else:
        o = bhl.cone("red_cone", 0.17, 0.03, 0.5, (x, y, 0.25), m)
    tag(o, ambient + rng.uniform(-1, 1))
    return kind


def sun_spot(x, y, rng, ambient):
    bpy.ops.mesh.primitive_circle_add(vertices=32, radius=1, fill_type="NGON", location=(x, y, 0.004))
    o = bpy.context.object
    o.name, o.scale = "sun_spot", (rng.uniform(0.3, 0.8), rng.uniform(0.2, 0.5), 1)
    o.rotation_euler.z = rng.uniform(0, 3)
    o.data.materials.append(bhl.mat("spot", (1, 0.95, 0.85), emission=(1, 0.95, 0.85), strength=rng.uniform(1, 3)))
    tag(o, ambient + rng.uniform(8, 20))


def world(color, strength, temp_c):
    w = bpy.data.worlds.new("w")
    bpy.context.scene.world = w
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value, bg.inputs["Strength"].default_value = (*color, 1), strength
    w["temp_c"] = temp_c


def sun(strength, elev, azim, color=(1, 0.97, 0.9)):
    bpy.ops.object.light_add(type="SUN", rotation=(0, math.radians(90 - elev), math.radians(azim)))
    bpy.context.object.data.energy, bpy.context.object.data.color = strength, color


def gun_module(pose, ambient):
    """Concept A top module (cad/parts STLs) and the stand-in gun of cad/parts/render_module.py at the default tilt.
    The USB camera's own mount is left out, because the lens sits inside it."""
    sys.path.insert(0, os.path.join(ROOT, "cad", "parts"))
    import render_module as rm  # reads cad/parts/out/assembly.json
    rm.TILT_DEG = -math.degrees(TILT0)
    mats = rm.materials()
    objs = rm.load_module(mats, (0, 0, G.T), gun=False) + rm.gun_standin(mats, (0, 0, G.T), rm.TILT_DEG)
    root = bpy.data.objects.new("robot", None)
    bpy.context.collection.objects.link(root)
    kept = []
    for o in objs:
        if "cam_usb32" in o.name:
            bpy.data.objects.remove(o)
        else:
            kept.append(tag(o, ambient))
    bpy.context.view_layer.update()
    for o in [bpy.data.objects["gun_tilt"]] + [o for o in kept if o.parent is None]:  # robot frame -> scene
        mw = o.matrix_world.copy()
        o.parent = root
        o.matrix_world = mw
    root.matrix_world = robot_matrix(pose)


# ---------- one scene ----------
def build(seed, t=0.0, robot_dist=None, gun=False):
    """Random scene for this seed. t (s) only changes the flame flicker. Returns the scene description."""
    rng = random.Random(seed)
    bhl.clear()
    sc = bpy.context.scene
    sc.view_settings.view_transform = "AgX"
    outdoor = rng.random() < 0.35
    ambient = rng.uniform(20, 30)
    info = dict(seed=seed, scene="outdoor" if outdoor else "indoor", ambient_c=ambient, distractors=[])
    if outdoor:
        lx, ly = 30.0, 30.0
        light = info["lighting"] = rng.choice(["sun", "overcast", "dusk"])
        ground = rng.choice([(0.12, 0.25, 0.06), (0.3, 0.22, 0.14), (0.12, 0.12, 0.12), (0.45, 0.43, 0.4)])
        tag(bhl.box("ground", (lx * 2, ly * 2, 0.02), (0, 0, -0.01), textured("ground", ground, rng), bevel=0),
            ambient + (rng.uniform(5, 15) if light == "sun" else 0))
        if light == "sun":
            world((0.45, 0.6, 0.9), 1.0, ambient - 40)
            sun(rng.uniform(3, 5), rng.uniform(25, 70), rng.uniform(0, 360))
        elif light == "overcast":
            world((0.7, 0.72, 0.75), 1.5, ambient - 30)
            sun(0.5, 60, rng.uniform(0, 360))
        else:
            world((0.1, 0.1, 0.25), 0.3, ambient - 35)
            sun(0.6, 5, rng.uniform(0, 360), (1, 0.5, 0.25))
    else:
        lx, ly = rng.uniform(3, 6), rng.uniform(2.5, 4.5)  # half sizes
        light = info["lighting"] = rng.choice(["lamps", "window", "dim"])
        floor = rng.choice([(0.5, 0.35, 0.2), (0.6, 0.6, 0.58), (0.25, 0.25, 0.27), (0.7, 0.65, 0.55), (0.35, 0.2, 0.12)])
        wallc = rng.choice([(0.8, 0.8, 0.78), (0.75, 0.7, 0.6), (0.55, 0.6, 0.65), (0.9, 0.88, 0.85)])
        tag(bhl.box("floor", (lx * 2, ly * 2, 0.02), (0, 0, -0.01), textured("floor", floor, rng), bevel=0), ambient - 1)
        wm = textured("wall", wallc, rng)
        for s in (-1, 1):
            tag(bhl.box("wall", (0.1, ly * 2, 2.7), (s * (lx + 0.05), 0, 1.35), wm, bevel=0), ambient)
            tag(bhl.box("wall", (lx * 2, 0.1, 2.7), (0, s * (ly + 0.05), 1.35), wm, bevel=0), ambient)
        world((0.8, 0.8, 0.8), {"lamps": 0.4, "window": 0.6, "dim": 0.04}[light], ambient)
        if light == "lamps":
            for _ in range(rng.randint(2, 4)):
                bpy.ops.object.light_add(type="POINT", location=(rng.uniform(-lx, lx) * 0.8, rng.uniform(-ly, ly) * 0.8, 2.5))
                bpy.context.object.data.energy = rng.uniform(300, 900)
        elif light == "window":
            s = rng.choice((-1, 1))
            win = bhl.mat("window", (0.9, 0.95, 1), emission=(0.9, 0.95, 1), strength=rng.uniform(3, 8))
            tag(bhl.box("window", (rng.uniform(1, 2.5), 0.02, 1.2), (rng.uniform(-lx, lx) * 0.6, s * ly, 1.5), win, bevel=0), ambient + 3)
            sun(rng.uniform(2, 5), rng.uniform(20, 50), 90 if s < 0 else -90)
        else:
            bpy.ops.object.light_add(type="POINT", location=(rng.uniform(-lx, lx) * 0.8, rng.uniform(-ly, ly) * 0.8, 2.0))
            bpy.context.object.data.energy = rng.uniform(20, 80)
    # fires and the robot: fire 0 in the area, robot 0.8-6 m away, facing it +-30 deg
    margin = 0.5
    fx, fy = rng.uniform(-lx + margin, lx - margin) * (0.3 if outdoor else 1), rng.uniform(-ly + margin, ly - margin) * (0.3 if outdoor else 1)
    for _ in range(200):
        d = robot_dist or rng.uniform(0.8, 6.0)
        b = rng.uniform(0, 2 * math.pi)
        rx, ry = fx + d * math.cos(b), fy + d * math.sin(b)
        if abs(rx) < lx - 0.4 and abs(ry) < ly - 0.4:
            break
    else:
        rx, ry = 0.0, 0.0
        d = math.hypot(fx, fy)
    yaw = math.atan2(fy - ry, fx - rx) + (0 if robot_dist else rng.uniform(-0.5, 0.5))
    info["robot_pose_world"] = [rx, ry, yaw]
    robot, f0 = Vector((rx, ry, 0)), Vector((fx, fy, 0))
    spots = [(fx, fy)] + [(rng.uniform(-lx + margin, lx - margin), rng.uniform(-ly + margin, ly - margin)) for _ in range(rng.randint(0, 2))]
    spots = [s for i, s in enumerate(spots) if i == 0 or (Vector((*s, 0)) - robot).length > 0.6]
    info["fires"] = []
    for i, (x, y) in enumerate(spots, 1):
        w = rng.uniform(0.1, 0.6)
        h = w * rng.uniform(1.0, 1.6)
        temp = (rng.uniform(600, 700), rng.uniform(800, 900))
        flame(i, x, y, w, h, t, rng, temp)
        smoky = rng.random() < 0.3
        if smoky:
            smoke(x, y, w, rng)
        info["fires"].append(dict(index=i, base_world=[x, y, 0.0], width_m=w, height_m=h, temp_min_c=temp[0], temp_max_c=temp[1], smoke=smoky))

    def free_spot(clear=0.6):
        for _ in range(100):
            p = Vector((rng.uniform(-lx + 0.4, lx - 0.4), rng.uniform(-ly + 0.4, ly - 0.4), 0))
            if outdoor:
                p = robot + Vector((rng.uniform(-8, 8), rng.uniform(-8, 8), 0))
            if (p - robot).length > clear and seg_dist(p, robot, f0) > clear and all((p - Vector((*s, 0))).length > 0.5 for s in spots):
                return p
        return None

    if not outdoor:  # furniture boxes
        for _ in range(rng.randint(2, 6)):
            p = free_spot()
            if p:
                s = (rng.uniform(0.4, 1.6), rng.uniform(0.4, 1.0), rng.uniform(0.4, 1.9))
                c = rng.choice([(0.45, 0.3, 0.15), (0.2, 0.2, 0.22), (0.7, 0.7, 0.68), (0.3, 0.35, 0.45)])
                tag(bhl.box("furniture", s, (p.x, p.y, s[2] / 2), textured(f"furn{p.x:.2f}", c, rng), rot=(0, 0, rng.uniform(0, 90)), bevel=0.01),
                    ambient + rng.uniform(-1, 1))
    for kind, prob in (("human", 0.5), ("lamp", 0.0 if outdoor else 0.5), ("red", 0.8), ("red", 0.4), ("sun_spot", 0.4)):
        if rng.random() < prob:
            p = free_spot()
            if p is None:
                continue
            if kind == "human":
                human(p.x, p.y, rng)
            elif kind == "lamp":
                lamp(p.x, p.y, rng)
            elif kind == "red":
                kind = "red_" + red_thing(p.x, p.y, rng, ambient)
            else:
                sun_spot(p.x, p.y, rng, ambient)
            info["distractors"].append(kind)
    if gun:
        gun_module(info["robot_pose_world"], ambient)
        info["gun_in_view"] = True
    return info


# ---------- camera and rendering ----------
def robot_matrix(pose, bob=0.0):
    x, y, yaw = pose
    return Matrix.Translation((x, y, bob)) @ Matrix.Rotation(yaw, 4, "Z")


def place_camera(M, pan, tilt, key):
    """Blender camera at the aim/geometry.py pose. Blender looks along -z with +y up; optical frame is z forward, y down."""
    sc = bpy.context.scene
    if sc.camera is None:
        sc.camera = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
        sc.collection.objects.link(sc.camera)
    R, t = G.camera_pose(CFG, key, pan, tilt)
    Rb = R @ np.diag([1.0, -1.0, -1.0])
    sc.camera.matrix_world = M @ Matrix([[*Rb[0], t[0]], [*Rb[1], t[1]], [*Rb[2], t[2]], [0, 0, 0, 1]])
    w, h, hfov = RES[key]
    c = sc.camera.data
    c.sensor_fit, c.angle, c.clip_start, c.clip_end = "HORIZONTAL", math.radians(hfov), 0.02, 200
    sc.render.resolution_x, sc.render.resolution_y = w, h
    return R, t


def render(path, samples, denoise, fmt):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = d.type == "OPTIX"
    sc.cycles.device, sc.cycles.samples, sc.cycles.use_denoising = "GPU", samples, denoise
    s = sc.render.image_settings
    if fmt == "JPEG":
        s.file_format, s.quality = "JPEG", 92
    else:
        s.file_format, s.color_depth, s.exr_codec = "OPEN_EXR", "32", "ZIP"
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)


def read_exr(path):
    img = bpy.data.images.load(path)
    img.colorspace_settings.name = "Non-Color"
    w, h = img.size
    a = np.empty(w * h * 4, np.float32)
    img.pixels.foreach_get(a)
    bpy.data.images.remove(img)
    return a.reshape(h, w, 4)[::-1]


def thermal_materials():
    """Swap every material for an emission shader: red = temperature (K) / 1000, green = 0.1 x fire index, blue = 1 on fires.
    No lights, no smoke."""
    cache, temps = {}, []
    for o in list(bpy.data.objects):
        if o.type == "LIGHT" or o.get("smoke"):
            bpy.data.objects.remove(o)
            continue
        if o.type != "MESH":
            continue
        fire = o.get("fire", 0)
        key = (round(o.get("temp_c", 25.0), 2), fire, o.get("temp_lo"))
        if key not in cache:
            m = bpy.data.materials.new("thermal")
            nt, out = nodes(m)
            em, comb = nt.nodes.new("ShaderNodeEmission"), nt.nodes.new("ShaderNodeCombineColor")
            comb.inputs["Green"].default_value = 0.1 * fire
            comb.inputs["Blue"].default_value = 1.0 if fire else 0.0  # fire coverage of the pixel
            if fire:  # hotter at the bottom of the flame: temp_hi at the base, temp_lo at the tip
                tc, sep, mr = nt.nodes.new("ShaderNodeTexCoord"), nt.nodes.new("ShaderNodeSeparateXYZ"), nt.nodes.new("ShaderNodeMapRange")
                mr.inputs["To Min"].default_value = (o["temp_hi"] + KELVIN) / 1000
                mr.inputs["To Max"].default_value = (o["temp_lo"] + KELVIN) / 1000
                nt.links.new(tc.outputs["Generated"], sep.inputs[0])
                nt.links.new(sep.outputs["Z"], mr.inputs["Value"])
                nt.links.new(mr.outputs["Result"], comb.inputs["Red"])
            else:
                comb.inputs["Red"].default_value = (key[0] + KELVIN) / 1000
            nt.links.new(comb.outputs[0], em.inputs["Color"])
            nt.links.new(em.outputs[0], out.inputs["Surface"])
            cache[key] = m
        o.data.materials.clear()
        o.data.materials.append(cache[key])
        temps.append(key[0] + KELVIN)
    w = bpy.context.scene.world
    bg = w.node_tree.nodes["Background"]
    bg.inputs["Color"].default_value, bg.inputs["Strength"].default_value = ((w["temp_c"] + KELVIN) / 1000, 0, 0, 1), 1.0
    sc = bpy.context.scene
    sc.view_settings.view_transform = "Standard"
    sc.cycles.max_bounces, sc.cycles.filter_width = 0, 1.0
    return temps + [w["temp_c"] + KELVIN]


def boxes(img, n_fires, scale=1):
    """Box (u_min, v_min, u_max, v_max) of each fire in pixels; None when under 4 pixels.
    Blue = share of the pixel covered by fire, green / blue = 0.1 x fire index (edge pixels keep the ratio)."""
    out = []
    cover = img[..., 2]
    code = img[..., 1] / np.maximum(cover, 1e-6)
    for i in range(1, n_fires + 1):
        m = (cover > 0.5) & (np.abs(code - 0.1 * i) < 0.03)
        if m.sum() < 4:
            out.append(None)
            continue
        vs, us = np.nonzero(m)
        out.append([math.floor(us.min() / scale), math.floor(vs.min() / scale), math.ceil((us.max() + 1) / scale), math.ceil((vs.max() + 1) / scale)])
    return out


def kelvin(img):
    return img[..., 0] * 1000.0


def shoot(info, sid, t=0.0, bob=0.0, pan=None, tilt=None, rng=None):
    """Render RGB and thermal images for the built scene and write images and labels. Returns the label dict."""
    rng = rng or random.Random(info["seed"] + 1)
    pan = rng.uniform(-SHAKE, SHAKE) if pan is None else pan
    tilt = TILT0 + rng.uniform(-SHAKE, SHAKE) if tilt is None else tilt
    M = robot_matrix(info["robot_pose_world"], bob)
    R, t_cam = place_camera(M, pan, tilt, "rgb")
    render(f"{OUT}/rgb/{sid}.jpg", SPP["rgb"], True, "JPEG")
    temps = np.array(thermal_materials())
    render(f"{OUT}/tmp/{sid}_full.exr", SPP["thermal"], False, "EXR")
    full = read_exr(f"{OUT}/tmp/{sid}_full.exr")
    # check: the median non-fire pixel is exactly one of the set temperatures (no view transform, no light mixing)
    assert np.abs(temps - np.median(kelvin(full)[full[..., 2] < 1e-4])).min() < 0.05
    place_camera(M, pan, tilt, "thermal_render")
    render(f"{OUT}/tmp/{sid}_low.exr", SPP["thermal_low"], False, "EXR")
    low = read_exr(f"{OUT}/tmp/{sid}_low.exr")
    n = len(info["fires"])
    images = {"rgb": full, "thermal": full}
    label_boxes = {"rgb": boxes(full, n), "thermal": boxes(full, n)}
    dk = np.clip(np.rint(kelvin(full) * 10), 0, 65535).astype(np.uint16)  # deci-kelvin
    png(f"{OUT}/thermal/{sid}.png", dk)
    png(f"{OUT}/preview/thermal/{sid}.png", preview(kelvin(full) - KELVIN))
    for name, f in LOWRES.items():
        h, w = low.shape[0] // f, low.shape[1] // f
        k = kelvin(low).reshape(h, f, w, f).mean(axis=(1, 3))  # area average, like one sensor pixel
        png(f"{OUT}/{name}/{sid}.png", np.clip(np.rint(k * 10), 0, 65535).astype(np.uint16))
        png(f"{OUT}/preview/{name}/{sid}.png", preview(k - KELVIN))
        label_boxes[name] = boxes(low, n, f)
    Minv = M.inverted()
    cam_world = M @ Vector(t_cam)
    fires = []
    for k, fdesc in enumerate(info["fires"]):
        base = Minv @ Vector(fdesc["base_world"])
        centre = base + Vector((0, 0, fdesc["height_m"] / 2))
        fires.append(dict(fdesc, base_robot=list(base), centre_robot=list(centre), distance_m=math.hypot(base.x, base.y),
                          camera_distance_m=((M @ centre) - cam_world).length, boxes={m: b[k] for m, b in label_boxes.items()}))
        del fires[-1]["base_world"]
    K = {k: CFG["cameras"][k]["K"] for k in ("rgb", "thermal_256", "thermal_160")}
    label = dict(id=sid, seed=info["seed"], scene=info["scene"], lighting=info["lighting"], ambient_c=info["ambient_c"],
                 distractors=info["distractors"], gun_in_view=info.get("gun_in_view", False), robot_pose_world=info["robot_pose_world"], robot_bob_m=bob, time_s=t,
                 camera=dict(concept="A", source="aim/geometry.py CONCEPTS['A']['cameras']['rgb']", pan_rad=pan, tilt_rad=tilt,
                             R_robot_from_optical=R.tolist(), t_robot=list(t_cam)),
                 images={"rgb": dict(file=f"rgb/{sid}.jpg", hfov_deg=90, **vars(K["rgb"])),
                         "thermal": dict(file=f"thermal/{sid}.png", unit="decikelvin", hfov_deg=90, **vars(K["rgb"])),
                         **{n: dict(file=f"{n}/{sid}.png", unit="decikelvin", hfov_deg=56, **vars(K[n])) for n in LOWRES}},
                 fires=fires)
    with open(f"{OUT}/labels/{sid}.json", "w") as f:
        json.dump(label, f, indent=1)
    for mod, (w, h) in (("rgb", (1280, 720)), ("thermal", (1280, 720)), ("thermal_256", (256, 192)), ("thermal_160", (160, 120))):
        with open(f"{OUT}/yolo/{mod}/labels/{sid}.txt", "w") as f:
            for fi in fires:
                b = fi["boxes"][mod]
                if b:
                    f.write(f"0 {(b[0] + b[2]) / 2 / w:.6f} {(b[1] + b[3]) / 2 / h:.6f} {(b[2] - b[0]) / w:.6f} {(b[3] - b[1]) / h:.6f}\n")
        src = f"{OUT}/rgb/{sid}.jpg" if mod == "rgb" else f"{OUT}/preview/{mod}/{sid}.png"
        dst = f"{OUT}/yolo/{mod}/images/{sid}{os.path.splitext(src)[1]}"
        if not os.path.exists(dst):
            os.symlink(os.path.relpath(src, os.path.dirname(dst)), dst)
    for p in (f"{OUT}/tmp/{sid}_full.exr", f"{OUT}/tmp/{sid}_low.exr"):
        os.remove(p)
    return label


def dirs():
    for d in ["rgb", "thermal", "labels", "tmp"] + list(LOWRES) + [f"preview/{m}" for m in ["thermal", *LOWRES]] \
            + [f"yolo/{m}/{s}" for m in ["rgb", "thermal", *LOWRES] for s in ("images", "labels")]:
        os.makedirs(f"{OUT}/{d}", exist_ok=True)
    for m in ["rgb", "thermal", *LOWRES]:
        with open(f"{OUT}/yolo/{m}/data.yaml", "w") as f:
            f.write(f"path: {OUT}/yolo/{m}\ntrain: images\nval: images\nnames:\n  0: fire\n")


def samples(first, count):
    for i in range(first, first + count):
        try:
            shoot(build(i), f"{i:06d}")
        except Exception:  # keep the batch going; the README counts the failed samples
            import traceback
            traceback.print_exc()
            print(f"SAMPLE FAILED {i}")


def gun_samples(first, count):
    """Samples with the concept A gun and top module in the scene, gun at the default tilt, no camera shake."""
    for i in range(first, first + count):
        shoot(build(100000 + i, gun=True), f"g{i:05d}", pan=0.0, tilt=TILT0)


def sequence(seconds=10, fps=10):
    """Robot walks 0.5 m/s straight toward fire 1 from 6 m, with 2 Hz camera bobbing. Writes out/sequence/*.mp4."""
    global OUT
    base, OUT = OUT, os.path.join(OUT, "sequence")
    SPP["rgb"] = 8  # 100 frames on a shared GPU
    dirs()
    for seed in range(1000, 1050):  # first indoor scene with room for a 6 m walk
        info = build(seed, robot_dist=6.0)
        f, (x, y, _) = info["fires"][0]["base_world"], info["robot_pose_world"]
        if info["scene"] == "indoor" and abs(math.hypot(f[0] - x, f[1] - y) - 6.0) < 1e-6:
            break
    x0, y0, yaw = info["robot_pose_world"]
    for k in range(seconds * fps):
        t = k / fps
        info = build(seed, t, robot_dist=6.0)
        info["robot_pose_world"] = [x0 + 0.5 * t * math.cos(yaw), y0 + 0.5 * t * math.sin(yaw), yaw]
        step = 2 * math.pi * 2 * t
        shoot(info, f"{k:04d}", t, bob=0.015 * math.sin(step), pan=math.radians(1.0) * math.sin(step / 2),
              tilt=TILT0 + math.radians(1.5) * math.sin(step + 1))
    for mod, src in (("rgb", "rgb/%04d.jpg"), ("thermal", "preview/thermal/%04d.png"),
                     ("thermal_256", "preview/thermal_256/%04d.png"), ("thermal_160", "preview/thermal_160/%04d.png")):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", f"{OUT}/{src}", "-vf",
                        "scale=1280:-2:flags=neighbor", "-pix_fmt", "yuv420p", f"{OUT}/{mod}.mp4"], check=True)
    shutil.rmtree(f"{OUT}/tmp", ignore_errors=True)
    OUT = base


def sheet(n=20, n_gun=4):
    """RGB and thermal side by side for the first n samples and the first n_gun samples with the gun."""
    ids = sorted(f[:-5] for f in os.listdir(f"{OUT}/labels"))
    ids = [i for i in ids if not i.startswith("g")][:n] + [i for i in ids if i.startswith("g")][:n_gun]
    args = [a for i in ids for a in ("-label", f"{i} RGB", f"{OUT}/rgb/{i}.jpg", "-label", f"{i} thermal", f"{OUT}/preview/thermal/{i}.png")]
    subprocess.run(["magick", "montage", "-pointsize", "18"] + args + ["-tile", "4x", "-geometry", "480x270+3+3",
                                                                       f"{OUT}/contact_sheet.jpg"], check=True)


if __name__ == "__main__":
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["samples", "0", "4"]
    dirs()
    if a[0] == "samples":
        samples(int(a[1]), int(a[2]))
    elif a[0] == "sequence":
        if os.path.isdir(f"{OUT}/sequence/labels") and os.listdir(f"{OUT}/sequence/labels"):
            print("out/sequence exists: delete it to make the sequence again")
        else:
            sequence()
    elif a[0] == "gunsamples":
        gun_samples(int(a[1]), int(a[2]))
    elif a[0] == "sheet":
        sheet()
    if a[0] == "samples":
        shutil.rmtree(TMP, ignore_errors=True)
