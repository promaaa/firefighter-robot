"""Blender helpers: place Berkeley Humanoid Lite meshes from its URDF, add primitives, render.

Run inside Blender (bpy). Meshes and URDF come from
https://github.com/HybridRobotics/berkeley-humanoid-lite-assets (CC BY-SA 4.0).
"""
import math
import os
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Matrix, Vector

# Default standing pose from berkeley_humanoid_lite.py (HUMANOID_LITE_BIPED_CFG.init_state)
STAND = {
    "leg_left_hip_pitch_joint": -0.2, "leg_left_knee_pitch_joint": 0.4, "leg_left_ankle_pitch_joint": -0.3,
    "leg_right_hip_pitch_joint": -0.2, "leg_right_knee_pitch_joint": 0.4, "leg_right_ankle_pitch_joint": -0.3,
}


def _tf(el):
    xyz = [float(v) for v in (el.get("xyz") or "0 0 0").split()] if el is not None else [0, 0, 0]
    rpy = [float(v) for v in (el.get("rpy") or "0 0 0").split()] if el is not None else [0, 0, 0]
    return Matrix.Translation(xyz) @ Euler(rpy, "XYZ").to_matrix().to_4x4()


def link_frames(urdf, q):
    """Forward kinematics: {link name: world Matrix} for joint positions q (missing joints = 0)."""
    root = ET.parse(urdf).getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    frames = {}

    def frame(link):
        if link not in frames:
            j = joints.get(link)
            if j is None:
                frames[link] = Matrix.Identity(4)
            else:
                axis = Vector([float(v) for v in (j.find("axis").get("xyz") if j.find("axis") is not None else "0 0 1").split()])
                rot = Matrix.Rotation(q.get(j.get("name"), 0.0), 4, axis) if axis.length > 0 else Matrix.Identity(4)
                frames[link] = frame(j.find("parent").get("link")) @ _tf(j.find("origin")) @ rot
        return frames[link]

    for l in root.findall("link"):
        frame(l.get("name"))
    return root, frames


def load_robot(urdf, mesh_dir, q, mat, skip=(), name="bhl"):
    """Import every visual mesh of the URDF at pose q. Returns {link: object} and the link frames."""
    root, frames = link_frames(urdf, q)
    objs = {}
    for l in root.findall("link"):
        v = l.find("visual")
        if v is None or l.get("name") in skip:
            continue
        f = os.path.join(mesh_dir, os.path.basename(v.find("geometry/mesh").get("filename")))
        bpy.ops.wm.stl_import(filepath=f)
        o = bpy.context.selected_objects[0]
        o.name = f"{name}_{l.get('name')}"
        o.matrix_world = frames[l.get("name")] @ _tf(v.find("origin"))
        o.data.materials.append(mat)
        bpy.ops.object.shade_auto_smooth(angle=math.radians(30))
        objs[l.get("name")] = o
    return objs, frames


# ---------- materials ----------
def mat(name, color, rough=0.5, metal=0.0, transmission=0.0, alpha=1.0, emission=None, strength=0.0, coat=0.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Transmission Weight"].default_value = transmission
    b.inputs["Alpha"].default_value = alpha
    b.inputs["Coat Weight"].default_value = coat
    if emission:
        b.inputs["Emission Color"].default_value = (*emission, 1)
        b.inputs["Emission Strength"].default_value = strength
    return m


# ---------- primitives (all sizes in metres, rotations in degrees) ----------
def _finish(o, name, m, rot, bevel, parent):
    o.name = name
    if rot:
        o.rotation_euler = [math.radians(a) for a in rot]
    if m:
        o.data.materials.append(m)
    if bevel:
        mod = o.modifiers.new("bevel", "BEVEL")
        mod.width, mod.segments, mod.limit_method = bevel, 3, "ANGLE"
    if parent:
        o.parent = parent
    bpy.ops.object.shade_auto_smooth(angle=math.radians(35))
    return o


def box(name, size, loc, m=None, rot=None, bevel=0.004, parent=None):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object
    o.scale = size
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return _finish(o, name, m, rot, bevel, parent)


def cyl(name, r, depth, loc, m=None, rot=None, bevel=0.002, verts=48, parent=None):
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=depth, location=loc, vertices=verts)
    return _finish(bpy.context.object, name, m, rot, bevel, parent)


def cone(name, r1, r2, depth, loc, m=None, rot=None, verts=48, parent=None):
    bpy.ops.mesh.primitive_cone_add(radius1=r1, radius2=r2, depth=depth, location=loc, vertices=verts)
    return _finish(bpy.context.object, name, m, rot, 0, parent)


def sphere(name, r, loc, m=None, scale=(1, 1, 1), parent=None):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=r, location=loc, segments=48, ring_count=24)
    o = bpy.context.object
    o.scale = scale
    return _finish(o, name, m, None, 0, parent)


def tube(name, pts, r, m=None):
    """Hose / cable through points (smooth NURBS path)."""
    c = bpy.data.curves.new(name, "CURVE")
    c.dimensions, c.bevel_depth, c.bevel_resolution, c.use_fill_caps = "3D", r, 6, True
    s = c.splines.new("NURBS")
    s.points.add(len(pts) - 1)
    for p, xyz in zip(s.points, pts):
        p.co = (*xyz, 1)
    s.order_u, s.use_endpoint_u = min(4, len(pts)), True
    o = bpy.data.objects.new(name, c)
    bpy.context.collection.objects.link(o)
    if m:
        o.data.materials.append(m)
    return o


# ---------- scene ----------
def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def studio(size=16):
    """Dark curved backdrop, cool key light and an orange 'fire' glow behind the robot.
    Built for a camera on -y looking +y; fit() turns backdrop and lights to face the real camera."""
    bpy.ops.mesh.primitive_plane_add(size=size, location=(0, 0, 0))
    floor = bpy.context.object
    floor.name = "floor"
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.subdivide(number_cuts=100)
    bpy.ops.object.mode_set(mode="OBJECT")
    for v in floor.data.vertices:  # curve the back edge upward
        if v.co.y > 3:
            t = v.co.y - 3
            v.co.z, v.co.y = t * t * 0.35, 3 + t * 0.8
    floor.data.materials.append(mat("floor", (0.018, 0.019, 0.021), rough=0.5))
    bpy.ops.object.shade_smooth()
    rig = bpy.data.objects.new("rig", None)
    bpy.context.collection.objects.link(rig)

    def area(name, loc, energy, size, color, target=(0, 0, 0.5)):
        bpy.ops.object.light_add(type="AREA", location=loc)
        l = bpy.context.object
        l.name, l.data.energy, l.data.size, l.data.color = name, energy, size, color
        l.rotation_euler = (Vector(target) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        l.parent = rig

    area("key", (1.8, -2.4, 2.6), 150, 1.8, (0.95, 0.97, 1.0))
    area("fill", (-2.6, -1.8, 1.2), 30, 3.0, (0.75, 0.85, 1.0))
    area("rim_fire", (-1.6, 1.6, 1.4), 260, 1.2, (1.0, 0.42, 0.12))
    area("rim_cool", (1.5, 1.7, 2.2), 180, 1.0, (0.65, 0.78, 1.0))
    area("glow", (0, 3.25, 0.08), 160, 2.5, (1.0, 0.35, 0.08), target=(0, 4.6, 1.4))
    w = bpy.data.worlds.new("w")
    bpy.context.scene.world = w
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs["Color"].default_value = (0.02, 0.022, 0.026, 1)
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 1.0
    sc = bpy.context.scene
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"


def fit(coords, azim, elev, lens, res, margin=1.08):
    """Aim the camera from azimuth/elevation (degrees, 0 = robot front) so that all coords fill the frame."""
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = res
    if sc.camera is None:
        bpy.ops.object.camera_add()
        sc.camera = bpy.context.object
    c = sc.camera
    c.data.lens = lens
    a, e = math.radians(azim), math.radians(elev)
    fwd = -Vector((math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)))
    c.rotation_euler = fwd.to_track_quat("-Z", "Y").to_euler()
    bpy.context.view_layer.update()
    co, _ = c.camera_fit_coords(bpy.context.evaluated_depsgraph_get(), [x for v in coords for x in v])
    center = sum((Vector(v) for v in coords), Vector()) / len(coords)
    c.location = co - fwd * ((co - center).dot(-fwd) * (margin - 1))
    theta = math.atan2(-fwd.x, fwd.y)  # turn +y of the backdrop and light rig to the view direction
    for n in ("floor", "rig"):
        bpy.data.objects[n].rotation_euler.z = theta
    return c


def corners(objs):
    """World bounding-box corners of objects."""
    bpy.context.view_layer.update()
    return [tuple(o.matrix_world @ Vector(c)) for o in objs for c in o.bound_box]


def render(path, res=None, samples=128):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = d.type == "OPTIX"
    sc.cycles.device = "GPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = True
    if res:
        sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)


def delete_faces(o, inside):
    """Delete faces whose world-space centroid satisfies inside(Vector). Used to trim parts off merged meshes."""
    import bmesh
    bpy.context.view_layer.update()  # matrix_world is stale after setting location
    bm = bmesh.new()
    bm.from_mesh(o.data)
    mw = o.matrix_world
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if inside(mw @ f.calc_center_median())], context="FACES")
    bm.to_mesh(o.data)
    bm.free()
