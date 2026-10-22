"""Build the MuJoCo models of the Berkeley Humanoid Lite (BHL) biped with the payload of the final design F2.

    uv run python sim/payload/models.py     # writes sim/payload/models/*.xml, then runs the self-check

Needs the BHL assets (scripts/fetch_bhl_assets.sh) and cad/final/out_f2/layout.json (cad/final/layout.py).
Models: bhl_biped (no payload), bhl_F2 (tanks full), bhl_F2_half and bhl_F2_empty.
Robot frame: x forward, y left, z up, feet on the floor, torso upright.
Each part is a rigid body welded to the BHL base, with a solid-box inertia and a visual box (no collision).
"""
import json
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "models")
BHL = os.path.normpath(os.path.join(HERE, "../../third_party/berkeley-humanoid-lite-assets/data/robots/"
                                          "berkeley_humanoid/berkeley_humanoid_lite"))
MESHDIR = os.path.relpath(os.path.join(BHL, "meshes"), OUT)  # relative, so the saved XML files work from models/
Z0 = 0.039  # robot-frame z + Z0 = BHL base-frame z (the CAD robot frame has z = 0 on the floor)
T = 0.791  # top of the BHL torso frame, robot frame
URDF_MASS = {"biped": 11.34267}  # sum of <mass> in the biped URDF file

STAND = {"hip_pitch": -0.2, "knee_pitch": 0.4, "ankle_pitch": -0.3}  # BHL default pose, both legs

# Payload parts: (name, mass kg, centre x y z in the robot frame, box size x y z for the inertia).
# Names that start with "water" are removed in the "_empty" variants.
PARTS, REMOVE = {}, {}


def f2_parts():
    """Final design F2 (cad/final/out_f2/layout.json, two R2 side tanks, water lumped in one inertia-equivalent body):
    all course BOM parts, with the BHL battery and PC taken out of the BHL base link mass (REMOVE)."""
    lay = json.load(open(os.path.normpath(os.path.join(HERE, "../../cad/final/out_f2/layout.json"))))
    full = [(i["name"], i["mass_kg"], tuple(i["com_m"]), tuple(i["size_m"])) for i in lay["items"]]
    wh = lay["water_half"]
    half = [p for p in full if p[0] != "water"] + [("water", wh["mass_kg"], tuple(wh["com_m"]), tuple(wh["size_m"]))]
    rem = [(r["name"], r["mass_kg"], tuple(r["com_m"])) for r in lay["bhl_removed"]]
    return {"F2": full, "F2_half": half}, {"F2": rem, "F2_half": rem}


_f2, _rem = f2_parts()
PARTS.update(_f2)
REMOVE.update(_rem)


def box_inertia(m, s):
    sx, sy, sz = s
    return [m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy)]


def base_spec():
    s = mujoco.MjSpec.from_file(os.path.join(BHL, "mjcf", "berkeley_humanoid_lite_biped.xml"))
    s.meshdir = os.path.join(BHL, "meshes")  # the original says meshdir="assets" and file="merged/..."
    for mesh in s.meshes:
        mesh.file = mesh.file.replace("merged/", "")
    # base_visual.stl is the full-robot torso. The biped MJCF places it at the biped base CoM, 3.7 cm
    # too low (the same error as the biped URDF). Use the full MJCF placement. Visual only.
    g = next(g for g in s.body("base").geoms if g.meshname == "base_visual")
    g.pos = [-0.0030965, 0.000264765, 0.675059]
    s.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05], rgba=[0.3, 0.35, 0.4, 1])
    s.worldbody.add_light(pos=[0, 0, 2.5], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    return s


def build(concept=None, empty=False):
    """MjSpec for 'biped' or a payload of PARTS (empty=True removes the water)."""
    s = base_spec()
    s.modelname = f"bhl_{concept or 'biped'}" + ("_empty" if empty else "")
    if concept in PARTS:
        for name, mass, pos, size in PARTS[concept]:
            if empty and name.startswith("water"):
                continue
            b = s.body("base").add_body(name=f"payload_{name}", pos=[pos[0], pos[1], pos[2] + Z0])
            b.mass, b.ipos, b.inertia, b.explicitinertial = mass, [0, 0, 0], box_inertia(mass, size), True
            rgba = [0.25, 0.55, 0.9, 0.6] if name.startswith("water") else [0.85, 0.2, 0.1, 1]
            b.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[v / 2 for v in size], contype=0, conaffinity=0,
                       group=1, mass=0, rgba=rgba)
    if concept in REMOVE:
        remove_from_base(s, REMOVE[concept])
    return s


def remove_from_base(s, parts):
    """Take point masses (name, kg, robot-frame centre) out of the base link: new mass, centre and inertia."""
    m0 = s.compile()
    i = m0.body("base").id
    m, c = m0.body_mass[i], m0.body_ipos[i].copy()
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, m0.body_iquat[i])
    rot = rot.reshape(3, 3)
    inertia = rot @ np.diag(m0.body_inertia[i]) @ rot.T          # about the old centre, base frame

    def shift(mass, d):  # parallel axis term
        return mass * (d @ d * np.eye(3) - np.outer(d, d))
    pts = [(pm, np.array([p[0], p[1], p[2] + Z0])) for _, pm, p in parts]
    m1 = m - sum(pm for pm, _ in pts)
    c1 = (m * c - sum(pm * p for pm, p in pts)) / m1
    inertia_o = inertia + shift(m, c) - sum(shift(pm, p) for pm, p in pts)  # about the base origin
    inertia1 = inertia_o - shift(m1, c1)
    b = s.body("base")
    b.mass, b.ipos, b.explicitinertial = m1, c1, True
    b.fullinertia = [inertia1[0, 0], inertia1[1, 1], inertia1[2, 2], inertia1[0, 1], inertia1[0, 2], inertia1[1, 2]]


VARIANTS = {"biped": ("biped", False), "F2": ("F2", False), "F2_half": ("F2_half", False), "F2_empty": ("F2", True)}


def path(variant):
    return os.path.join(OUT, f"bhl_{variant}.xml")


def load(variant):
    return mujoco.MjModel.from_xml_path(path(variant))


def write_all():
    os.makedirs(OUT, exist_ok=True)
    from poses import stand_qpos
    for v, (c, empty) in VARIANTS.items():
        s = build(c, empty)
        m = s.compile()
        s.add_key(name="stand", qpos=stand_qpos(m, STAND), ctrl=np.zeros(m.nu))  # feet flat, 1 mm above the floor
        with open(path(v), "w") as f:  # relative mesh path, so the file works from any working directory
            f.write(s.to_xml().replace(f'meshdir="{s.meshdir}', f'meshdir="{MESHDIR}'))


def payload_mass(concept, empty=False):
    return sum(m for n, m, _, _ in PARTS[concept] if not (empty and n.startswith("water")))


def self_check():
    for v, (c, empty) in VARIANTS.items():
        m = load(v)
        expect = URDF_MASS["biped"] + (payload_mass(c, empty) if c in PARTS else 0) - sum(r[1] for r in REMOVE.get(c, []))
        got = m.body_subtreemass[1]
        assert abs(got - expect) < 1e-3, (v, got, expect)
        assert m.nkey == 1 and m.nu == 12, v
    print("self-check models: masses match URDF + payload table, keyframe present")


if __name__ == "__main__":
    write_all()
    self_check()
    for v in VARIANTS:
        print(f"{v:8s} total mass {load(v).body_subtreemass[1]:6.2f} kg  -> {os.path.relpath(path(v), HERE)}")
