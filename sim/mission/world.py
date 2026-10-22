"""Mission world: random scene (room or outdoor), final robot F2 with an ST3215 pan-tilt, fire sprites, water jet and fire model.

Frames: world x, y on the floor, z up, metres. The MuJoCo base frame of the BHL model is 0.039 m above the aim/ robot
frame (feet on the floor): z_base = z_robot + 0.039 (the BHL start drop, sim/sim2sim/README.md).
"""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image
from scipy import ndimage

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(ROOT / "aim")]
from geometry import CONCEPTS, T  # noqa: E402

CFG = CONCEPTS["F2"]  # pan-tilt, camera and nozzle of the final design F2
MODELS = ROOT / "sim" / "payload" / "models"
PARTS = ROOT / "cad" / "parts" / "out_sts3215"
CACHE = Path.home() / "runs" / "firefighter" / "mission" / "cache"
DZ = 0.039  # m, MuJoCo base frame z minus robot frame z
W, H = 640, 480
K = CFG["cameras"]["rgb"]["K"]
FOVY = math.degrees(2 * math.atan(H / 2 / K.fy))
PAN_LIM, TILT_LIM = CFG["pan_limits"], CFG["tilt_limits"]

# ---------------- jet and fire model (ASSUMED values, see README "Fire and water model") ----------------
# Water system and fire model. The mission spec stores the values, so a result file is self-contained.
# "robot": the MuJoCo model sim/payload/models/bhl_<robot>.xml and the tank meshes.
# f2: final design F2, two R2 tanks outside the torso, 0.749 L usable of 0.781 L, 1.0 mm nozzle, drag 0.11 1/m.
#   ASSUMED jet: 0.39 L/min and v0 = 9.2 m/s at 11.1 V, v0 ~ V^0.73 (3S voltage): the gear-pump comparison of
#   sim/water/water.py, which the campaign runs use. The KLP02 estimate is 0.42 L/min and 10.0 m/s (docs/hardware/water.md).
#   Fire model fitted to docs/hardware/water.md section 7: the lowest water rate that beats the regrowth is
#   water_per_area / regrow = 0.19 L/(m^2 s) = 11.5 L/(m^2 min) (Saerdqvist: 9 to 15), and a 15 cm tray takes 13 s of
#   spray at 100 % hits (10 to 30 s). Trays 10 to 20 cm: the flow can handle a 17 to 22 cm square.
DESIGNS = {
    "f2": dict(v0=9.2, drag=0.11, flow=0.39, tank=0.749, tray=(0.10, 0.20), water_per_area=2.5, regrow=13.0, robot="F2"),
}
SPREAD = math.radians(0.8)  # rad, 1 sigma angle spread of the droplets (jet break-up, ASSUMED)
EMIT_HZ, EMIT_N = 100, 4  # droplet groups per second, droplets per group
DROP_DT = 0.005  # s, droplet integration step
G = 9.81


# ---------------- textures ----------------
def _noise(rng, shape, cells, octaves=4):
    """Fractal value noise that tiles, zero mean, about unit amplitude."""
    out = np.zeros(shape)
    for o in range(octaves):
        g = rng.standard_normal((cells[0] * 2 ** o, cells[1] * 2 ** o))
        out += 0.5 ** o * ndimage.zoom(g, (shape[0] / g.shape[0], shape[1] / g.shape[1]), order=3, mode="grid-wrap")
    return out / 1.5


def textures():
    """Procedural PNG textures (made once, cached): name -> path."""
    CACHE.mkdir(parents=True, exist_ok=True)
    names = ("wood", "tiles", "carpet", "grass", "asphalt", "wall")
    paths = {n: CACHE / f"tex_{n}.png" for n in names}
    if all(p.exists() for p in paths.values()):
        return paths
    rng = np.random.default_rng(7)
    n = 512
    yy, xx = np.mgrid[0:n, 0:n] / n
    def save(name, rgb):
        Image.fromarray(np.clip(rgb * 255, 0, 255).astype(np.uint8)).save(paths[name])
    plank = (yy * 8).astype(int)  # 8 planks per tile, each with its own shade and grain
    shade = rng.uniform(0.8, 1.15, 8)[plank]
    grain = 0.08 * np.sin(2 * np.pi * (xx * 3 + 0.4 * _noise(rng, (n, n), (2, 16)) + plank * 0.37) * 6) + 0.05 * _noise(rng, (n, n), (64, 4))
    gap = ((yy * 8) % 1 < 0.02) * 0.35
    save("wood", np.array([0.56, 0.39, 0.24]) * (shade + grain - gap)[..., None])
    grout = (((xx * 4) % 1 < 0.015) | ((yy * 4) % 1 < 0.015)) * 0.25
    save("tiles", np.array([0.78, 0.77, 0.74]) * (1 - grout + 0.03 * _noise(rng, (n, n), (16, 16)))[..., None])
    save("carpet", np.array([0.35, 0.4, 0.48]) * (1 + 0.12 * _noise(rng, (n, n), (64, 64)))[..., None])
    g = _noise(rng, (n, n), (32, 32))
    save("grass", np.stack([0.30 + 0.08 * g, 0.42 + 0.1 * g, 0.18 + 0.04 * g], -1) * (1 + 0.15 * _noise(rng, (n, n), (128, 128)))[..., None])
    save("asphalt", np.array([0.33, 0.33, 0.34]) * (1 + 0.15 * _noise(rng, (n, n), (128, 128)) + 0.05 * _noise(rng, (n, n), (8, 8)))[..., None])
    save("wall", np.array([0.86, 0.85, 0.81]) * (1 + 0.02 * _noise(rng, (n, n), (32, 32)))[..., None])
    return paths


# ---------------- random mission ----------------
FURNITURE = {  # kind: (size x, y, z half sizes in m), colours
    "sofa": (0.45, 1.0, 0.22), "cabinet": (0.25, 0.6, 0.45), "shelf": (0.18, 0.5, 0.9), "bed": (1.0, 0.75, 0.25),
    "table": (0.45, 0.7, 0.38), "chair": (0.22, 0.22, 0.23), "crate": (0.3, 0.3, 0.3), "barrel": (0.28, 0.28, 0.45),
    "car": (2.1, 0.9, 0.7)}
NEUTRAL = [(0.45, 0.3, 0.2), (0.35, 0.35, 0.37), (0.25, 0.3, 0.45), (0.7, 0.66, 0.58), (0.9, 0.9, 0.88), (0.2, 0.2, 0.22),
           (0.3, 0.42, 0.33)]
DISTRACTORS = ("red_box", "cone", "extinguisher", "ball", "poster")


def random_spec(seed, outdoor=None, design="f2", flames="procedural"):
    """One random mission. Everything that changes between runs comes from this dict (and its seed).
    flames: "procedural" (default) or "real" (clips from real fire videos, sprites/SOURCES.md). It uses no random numbers."""
    rng = np.random.default_rng(seed)
    dz = DESIGNS[design]
    outdoor = bool(rng.random() < 0.25) if outdoor is None else outdoor
    lx, ly = (16.0, 16.0) if outdoor else (rng.uniform(7, 10), rng.uniform(6, 8))
    hx, hy = lx / 2, ly / 2
    m = 1.0  # m, start and fire margin to the walls
    start = (rng.uniform(-hx + m, hx - m) * (0.4 if outdoor else 1), rng.uniform(-hy + m, hy - m) * (0.4 if outdoor else 1),
             rng.uniform(-math.pi, math.pi))
    fires, tries = [], 0
    for _ in range(int(rng.integers(1, 4))):
        while tries < 2000:
            tries += 1
            r, a = rng.uniform(1.5, 5.0), rng.uniform(-math.pi, math.pi)
            x, y = start[0] + r * math.cos(a), start[1] + r * math.sin(a)
            if abs(x) < hx - 0.5 and abs(y) < hy - 0.5 and all(math.hypot(x - f["x"], y - f["y"]) > 1.5 for f in fires):
                fires.append(dict(x=x, y=y, d=float(rng.uniform(*dz["tray"])), phase=int(rng.integers(0, 1000))))
                break

    def free(x, y, sx, sy, gap=0.4):
        """Clear of the start, every fire and the line from the start to each fire."""
        def seg_dist(fx, fy):
            px, py, dx, dy = start[0], start[1], fx - start[0], fy - start[1]
            t = np.clip(((x - px) * dx + (y - py) * dy) / (dx * dx + dy * dy), 0, 1)
            return math.hypot(x - px - t * dx, y - py - t * dy)
        rad = math.hypot(sx, sy)
        return (math.hypot(x - start[0], y - start[1]) > rad + 0.9 and abs(x) < hx - sx and abs(y) < hy - sy
                and all(math.hypot(x - f["x"], y - f["y"]) > rad + gap + f["d"] and seg_dist(f["x"], f["y"]) > rad + 0.3 for f in fires))

    furniture = []
    kinds = ["crate", "barrel", "car", "crate"] if outdoor else ["sofa", "cabinet", "shelf", "bed", "table", "chair", "chair"]
    for kind in map(str, rng.permutation(kinds)[: int(rng.integers(2, 5))]):
        sx, sy, sz = FURNITURE[kind]
        for _ in range(100):
            yaw = float(rng.choice([0, math.pi / 2]))
            ex, ey = (sx, sy) if yaw == 0 else (sy, sx)
            if kind in ("sofa", "cabinet", "shelf", "bed") and not outdoor:  # against a wall
                side = rng.integers(4)
                x = (hx - ex - 0.06) * (1 if side == 0 else -1) if side < 2 else rng.uniform(-hx + ex, hx - ex)
                y = (hy - ey - 0.06) * (1 if side == 2 else -1) if side >= 2 else rng.uniform(-hy + ey, hy - ey)
            else:
                x, y = rng.uniform(-hx + ex, hx - ex), rng.uniform(-hy + ey, hy - ey)
            if free(x, y, ex, ey) and all(abs(x - f["x"]) > ex + f["ex"] + 0.3 or abs(y - f["y"]) > ey + f["ey"] + 0.3 for f in furniture):
                furniture.append(dict(kind=kind, x=x, y=y, ex=ex, ey=ey, ez=sz, rgb=NEUTRAL[int(rng.integers(len(NEUTRAL)))]))
                break
    distractors = []
    for kind in map(str, rng.permutation(DISTRACTORS)[: int(rng.integers(1, 4))]):
        for _ in range(100):
            if kind == "poster" and not outdoor:  # red-orange poster on a wall, 0.6-1.6 m high
                side = int(rng.integers(4))
                x = (hx - 0.03) * (1 if side == 0 else -1) if side < 2 else rng.uniform(-hx + 1, hx - 1)
                y = (hy - 0.03) * (1 if side == 2 else -1) if side >= 2 else rng.uniform(-hy + 1, hy - 1)
                distractors.append(dict(kind=kind, x=x, y=y, side=side, z=float(rng.uniform(0.6, 1.6))))
                break
            x, y = rng.uniform(-hx + 0.5, hx - 0.5), rng.uniform(-hy + 0.5, hy - 0.5)
            if kind != "poster" and free(x, y, 0.25, 0.25, 0.3):
                distractors.append(dict(kind=kind, x=x, y=y))
                break
    lamp = None
    if rng.random() < 0.7:  # warm lamp: the bulb height decides if a floor-plane localisation can be fooled
        for _ in range(100):
            x, y = rng.uniform(-hx + 0.4, hx - 0.4), rng.uniform(-hy + 0.4, hy - 0.4)
            if free(x, y, 0.2, 0.2, 0.3):
                lamp = dict(x=x, y=y, z=float(rng.uniform(0.4, 1.7)))
                break
    return dict(
        seed=seed, outdoor=outdoor, room=(lx, ly), start=start, fires=fires, furniture=furniture, distractors=distractors,
        lamp=lamp, floor=str(rng.choice(["grass", "asphalt"] if outdoor else ["wood", "tiles", "carpet"])),
        door=int(rng.integers(4)), light=float(rng.uniform(0.35, 1.0)), sun=(float(rng.uniform(-1, 1)), float(rng.uniform(-1, 1))),
        exposure=float(rng.uniform(0.85, 1.15)),
        # model errors: the true jet, the servo and camera mounting, the odometry. aim/ uses the nominal values.
        # jet after a calibration shot (ASSUMED): exit speed within 3 %, drag within 20 % of the aim/ values
        v0_scale=float(rng.uniform(0.97, 1.03)), drag=float(rng.uniform(0.8, 1.2) * dz["drag"]), recoil=float(rng.uniform(0.3, 1.3)),
        pan_zero=float(np.radians(rng.uniform(-1, 1))), tilt_zero=float(np.radians(rng.uniform(-1, 1))),
        cam_pitch_err=float(np.radians(rng.uniform(-1, 1))),
        odo_scale=float(rng.uniform(0.9, 1.1)), gyro_bias=float(np.radians(rng.uniform(-0.05, 0.05))),
        design=design, flames=flames, drag_aim=dz["drag"], **{k: v for k, v in dz.items() if k not in ("tray", "drag")},
        # supply voltage of the pump: the jet speed and the flow follow it (v0 ~ V^0.73). aim/ gets the matching v0.
        volts=float(rng.uniform(10.5, 12.6)),
    )


# ---------------- MJCF ----------------
def _sub(parent, tag, **kw):
    return ET.SubElement(parent, tag, {k: (" ".join(f"{x:.5g}" for x in v) if isinstance(v, (tuple, list)) else str(v)) for k, v in kw.items()})


def build_xml(spec):
    """bhl_F2.xml (read-only) with the pan-tilt as two joints, the camera, the nozzle, and the scene."""
    tex = textures()
    robot = spec.get("robot", "F2")
    root = ET.parse(MODELS / f"bhl_{robot}.xml").getroot()
    root.find("compiler").set("meshdir", str((MODELS / root.find("compiler").get("meshdir")).resolve()))
    vis = _sub(root, "visual")
    _sub(vis, "headlight", ambient=(0.12, 0.12, 0.12), diffuse=(0.15, 0.15, 0.15), specular=(0, 0, 0))
    _sub(vis, "global", offwidth=1280, offheight=720)
    _sub(vis, "quality", shadowsize=2048, offsamples=4)
    _sub(vis, "map", znear=0.001, zfar=10)
    asset = root.find("asset")
    for n, p in tex.items():
        _sub(asset, "texture", name=n, type="2d", file=str(p))
    sky = (0.55, 0.7, 0.92) if spec["outdoor"] else (0.3, 0.3, 0.32)
    _sub(asset, "texture", name="sky", type="skybox", builtin="gradient", rgb1=sky, rgb2=(0.92, 0.94, 0.97), width=256, height=256)
    rep = 6 if spec["outdoor"] else max(spec["room"]) / 2
    _sub(asset, "material", name="floor", texture=spec["floor"], texrepeat=(rep, rep), reflectance=0.03 if spec["floor"] == "tiles" else 0)
    _sub(asset, "material", name="wall", texture="wall", texrepeat=(4, 1))
    _sub(asset, "material", name="bulb", rgba=(1, 0.72, 0.35, 1), emission=1)
    for n in ("deck", "pan_base", "pan_yoke", "tilt_u"):
        _sub(asset, "mesh", name=f"pt_{n}", file=str((ROOT / "cad/final/out_f2/deck_f1.stl") if n == "deck" else PARTS / f"{n}.stl"), scale=(0.001,) * 3)
    for n in ("pan_servo", "tilt_servo"):
        _sub(asset, "mesh", name=f"pt_{n}", file=str(PARTS / "ref" / f"ref_{n}.stl"), scale=(0.001,) * 3)
    tanks = [ROOT / f"cad/final/out_f2/tank_{s}.stl" for s in ("left", "right")]
    for i, f in enumerate(tanks):
        _sub(asset, "mesh", name=f"tank{i}", file=str(f), scale=(0.001,) * 3)

    world = root.find("worldbody")
    floor = world.find("geom[@name='floor']")
    floor.attrib.pop("rgba", None)
    floor.set("material", "floor")
    lx, ly = spec["room"]
    hx, hy = lx / 2, ly / 2
    if spec["outdoor"]:
        sx, sy = spec["sun"]
        _sub(world, "light", name="sun", directional="true", pos=(0, 0, 10), dir=(sx, sy, -1.2), diffuse=(0.9 * spec["light"] + 0.3,) * 3,
             specular=(0.2, 0.2, 0.2), castshadow="true")
        _sub(world, "light", name="fill", directional="true", pos=(0, 0, 10), dir=(-sx, -sy, -1), diffuse=(0.3, 0.32, 0.38), castshadow="false")
    else:
        wall_h, door_w = 2.4, 0.9
        for i, (cx, cy, ex, ey) in enumerate([(hx, 0, 0.05, hy), (-hx, 0, 0.05, hy), (0, hy, hx, 0.05), (0, -hy, hx, 0.05)]):
            if i == spec["door"]:  # a door opening (0.9 m) in this wall, with the door leaf open at 90 deg
                along = ey if ex == 0.05 else ex
                for s in (-1, 1):
                    half, c = (along - door_w / 2) / 2, (along + door_w / 2) / 2 * s
                    p = (cx, c, wall_h / 2) if ex == 0.05 else (c, cy, wall_h / 2)
                    _sub(world, "geom", type="box", pos=p, size=(0.05, half, wall_h / 2) if ex == 0.05 else (half, 0.05, wall_h / 2), material="wall")
                lintel = (cx, 0, 2.3) if ex == 0.05 else (0, cy, 2.3)
                _sub(world, "geom", type="box", pos=lintel, size=(0.05, door_w / 2, 0.1) if ex == 0.05 else (door_w / 2, 0.05, 0.1), material="wall")
                inward = -np.sign(cx or cy)
                leaf = (cx + inward * 0.45, door_w / 2, 1.0) if ex == 0.05 else (door_w / 2, cy + inward * 0.45, 1.0)
                _sub(world, "geom", type="box", pos=leaf, size=(0.42, 0.02, 1.0) if ex == 0.05 else (0.02, 0.42, 1.0), rgba=(0.55, 0.42, 0.3, 1))
            else:
                _sub(world, "geom", type="box", pos=(cx, cy, wall_h / 2), size=(ex, ey, wall_h / 2), material="wall")
        _sub(world, "geom", name="ceiling", type="box", pos=(0, 0, wall_h + 0.05), size=(hx + 0.2, hy + 0.2, 0.05), rgba=(0.9, 0.9, 0.88, 1),
             group=4, contype=0, conaffinity=0)  # robot camera only (geom group 4), so the follow camera sees in
        li = spec["light"]
        for i, x in enumerate((-hx / 2, hx / 2)):
            _sub(world, "light", name=f"ceiling{i}", pos=(x, 0, wall_h - 0.05), dir=(0, 0, -1), cutoff=80, exponent=1,
                 diffuse=(li, li * 0.97, li * 0.9), specular=(0.1, 0.1, 0.1), attenuation=(1, 0.05, 0.02), castshadow="true" if i == 0 else "false")
    for f in spec["furniture"]:
        _furniture(world, f)
    for dd in spec["distractors"]:
        _distractor(world, dd, hx, hy)
    if spec["lamp"]:
        lp = spec["lamp"]
        _sub(world, "geom", name="lamp_pole", type="cylinder", pos=(lp["x"], lp["y"], lp["z"] / 2), size=(0.015, lp["z"] / 2), rgba=(0.15, 0.15, 0.15, 1))
        _sub(world, "geom", name="lamp_base", type="cylinder", pos=(lp["x"], lp["y"], 0.01), size=(0.12, 0.01), rgba=(0.15, 0.15, 0.15, 1))
        _sub(world, "geom", name="lamp_bulb", type="sphere", pos=(lp["x"], lp["y"], lp["z"]), size=0.07, material="bulb", contype=0, conaffinity=0)
        _sub(world, "light", name="lamp", pos=(lp["x"], lp["y"], lp["z"]), dir=(0, 0, -1), cutoff=170, diffuse=(0.6, 0.45, 0.25),
             attenuation=(0.3, 0.5, 0.5), castshadow="false")
    for i, f in enumerate(spec["fires"]):  # tray with fuel; the flames are sprites (fire_sprites), the light follows the strength
        r = f["d"] / 2
        _sub(world, "geom", name=f"tray{i}", type="cylinder", pos=(f["x"], f["y"], 0.02), size=(r, 0.02), rgba=(0.32, 0.32, 0.33, 1))
        _sub(world, "geom", name=f"fuel{i}", type="cylinder", pos=(f["x"], f["y"], 0.0405), size=(r * 0.9, 0.0005), rgba=(0.08, 0.06, 0.05, 1),
             contype=0, conaffinity=0)
        _sub(world, "light", name=f"fire{i}", pos=(f["x"], f["y"], 0.15 + f["d"]), dir=(0, 0, -1), cutoff=170, diffuse=(1, 0.55, 0.2),
             attenuation=(0.2, 0.3, 1.5), castshadow="false")

    # the robot: recolour the payload boxes (red-orange would look like fire), add the tank, deck and pan-tilt
    base = world.find("body[@name='base']")
    for b in base.findall("body"):
        if b.get("name", "").startswith("payload_"):
            for g in b.findall("geom"):
                g.set("rgba", "0.25 0.25 0.27 1" if "water" not in b.get("name") else "0.3 0.55 0.9 0.5")
                g.set("group", "3")  # hidden: the tank meshes on the side faces show instead
    # replaced by the moving pan-tilt below: the lumped top of the first layout, or the parts of the cad/final kit
    m_top = 0.0
    for n in ("top_module", "nozzle_fitting", "camera_b0201", "deck_f1", "pan_base", "pan_yoke", "tilt_u", "head_f1",
              "ref_pan_servo", "ref_tilt_servo", "ref_pan_bearing", "nozzle_brass", "hose_clips"):
        b = base.find(f"body[@name='payload_{n}']")
        if b is not None:
            m_top += float(b.find("inertial").get("mass"))
            base.remove(b)
    k = m_top / 0.607  # the split below is for the 0.607 kg lumped top: scale it, so the robot keeps its mass
    viz = dict(contype=0, conaffinity=0, group=2, mass=0)
    for i in range(len(tanks)):
        _sub(base, "geom", type="mesh", mesh=f"tank{i}", pos=(0, 0, DZ), rgba=(0.85, 0.85, 0.8, 0.5), **viz)
    _sub(base, "geom", type="mesh", mesh="pt_deck", pos=(0, 0, DZ), rgba=(0.2, 0.2, 0.22, 1), **viz)
    top = T + DZ  # torso top, base frame
    for n in ("pan_base", "pan_servo"):
        _sub(base, "geom", type="mesh", mesh=f"pt_{n}", pos=(0, 0, top), rgba=(0.12, 0.12, 0.13, 1), **viz)
    # fixed part of the top module (deck, pan base, pan servo, bearing): 0.25 kg. Moving: 0.12 kg pan, 0.237 kg tilt.
    # Total 0.607 kg = top module + camera + nozzle (ASSUMED split, from cad/parts/out_sts3215/assembly.json part masses).
    fixed = _sub(base, "body", name="pt_fixed", pos=(-0.035, 0, top + 0.01))
    _sub(fixed, "inertial", pos=(0, 0, 0), mass=0.25 * k, diaginertia=(4e-4, 4e-4, 6e-4))
    px, py, pz = CFG["pivot"]
    piv = (px, py, pz + DZ)
    pan = _sub(base, "body", name="pan", pos=piv)
    _sub(pan, "inertial", pos=(0.0, 0, -0.03), mass=0.12 * k, diaginertia=(1e-4, 1e-4, 8e-5))
    _sub(pan, "joint", name="pan", type="hinge", axis=(0, 0, 1), range=PAN_LIM, damping=0.05, armature=0.003, frictionloss=0.01)
    off = (-px, -py, top - piv[2])  # mesh frame (cad/parts, z from the torso top) to the pan and tilt body frames
    _sub(pan, "geom", type="mesh", mesh="pt_pan_yoke", pos=off, rgba=(0.2, 0.2, 0.22, 1), **viz)
    _sub(pan, "geom", type="mesh", mesh="pt_tilt_servo", pos=off, rgba=(0.12, 0.12, 0.13, 1), **viz)
    tilt = _sub(pan, "body", name="tilt")
    _sub(tilt, "inertial", pos=(0.03, 0, 0.02), mass=0.237 * k, diaginertia=(2e-4, 3e-4, 3e-4))
    _sub(tilt, "joint", name="tilt", type="hinge", axis=(0, -1, 0), range=TILT_LIM, damping=0.05, armature=0.003, frictionloss=0.01)
    _sub(tilt, "geom", type="mesh", mesh="pt_tilt_u", pos=off, rgba=(0.2, 0.2, 0.22, 1), **viz)
    _sub(tilt, "geom", type="box", pos=(0.035, -0.01, 0.027), size=(0.06, 0.045, 0.003), rgba=(0.2, 0.2, 0.22, 1), **viz)  # nozzle and camera plate
    nx, ny, nz = CFG["nozzle"]
    _sub(tilt, "geom", type="capsule", fromto=(0.0, 0, nz, nx - 0.004, ny, nz), size=0.005, rgba=(0.75, 0.6, 0.3, 1), **viz)  # brass nozzle
    _sub(tilt, "site", name="nozzle", pos=(nx, ny, nz))
    c = CFG["cameras"]["rgb"]
    cx, cy, cz = c["pos"]
    _sub(tilt, "geom", type="box", pos=(cx - 0.016, cy, cz), size=(0.003, 0.019, 0.019), rgba=(0.1, 0.35, 0.15, 1), **viz)  # B0201 board
    _sub(tilt, "geom", type="box", pos=(cx - 0.016, cy, (cz + 0.03) / 2), size=(0.003, 0.008, (cz - 0.03) / 2), rgba=(0.2, 0.2, 0.22, 1), **viz)
    p = c["pitch"] + spec["cam_pitch_err"]  # the mounting error: aim/ does not know it
    _sub(tilt, "geom", type="cylinder", pos=(cx - 0.008, cy, cz), size=(0.008, 0.0055), zaxis=(math.cos(p), 0, math.sin(p)), rgba=(0.05, 0.05, 0.05, 1), **viz)
    _sub(tilt, "camera", name="rgb", pos=c["pos"], fovy=f"{FOVY:.4f}", xyaxes=(0, -1, 0, -math.sin(p), 0, math.cos(p)))
    act = root.find("actuator")
    for a in act.findall("general"):  # hw_risks case 7 drive: K 20, D 2, 6 Nm. ctrl = goal - zero error (set by the loop)
        a.attrib.update(biastype="affine", gainprm="20", biasprm="0 -20 -2", forcerange="-6 6", forcelimited="true", ctrllimited="false")
    for j, lim in (("pan", PAN_LIM), ("tilt", TILT_LIM)):  # ST3215 position servo: 2.9 Nm stall torque (30 kg cm at 12 V)
        _sub(act, "position", name=j, joint=j, kp=30, kv=0.3, ctrlrange=lim, forcerange=(-2.9, 2.9))
    key = root.find("keyframe/key")
    key.set("qpos", key.get("qpos") + " 0 0")
    return ET.tostring(root, encoding="unicode")


def _furniture(world, f):
    x, y, ex, ey, ez, rgb = f["x"], f["y"], f["ex"], f["ey"], f["ez"], (*f["rgb"], 1)
    k = f["kind"]
    n = [0]

    def _sub(parent, tag, **kw):  # every furniture geom gets a name "furn..."
        n[0] += 1
        return globals()["_sub"](parent, tag, name=f"furn_{k}_{x:.4f}_{y:.4f}_{n[0]}", **kw)
    if k == "table":  # top and four legs: the floor under it stays visible
        _sub(world, "geom", type="box", pos=(x, y, 2 * ez - 0.02), size=(ex, ey, 0.02), rgba=rgb)
        for sx in (-1, 1):
            for sy in (-1, 1):
                _sub(world, "geom", type="box", pos=(x + sx * (ex - 0.04), y + sy * (ey - 0.04), ez - 0.02), size=(0.02, 0.02, ez - 0.02), rgba=rgb)
    elif k == "barrel":
        _sub(world, "geom", type="cylinder", pos=(x, y, ez), size=(ex, ez), rgba=rgb)
    elif k == "sofa":
        _sub(world, "geom", type="box", pos=(x, y, ez), size=(ex, ey, ez), rgba=rgb)
        bx, by = (x + ex * 0.8 * np.sign(x), y) if ex < ey else (x, y + ey * 0.8 * np.sign(y))
        _sub(world, "geom", type="box", pos=(bx, by, 2 * ez + 0.18), size=(min(ex, 0.1), ey, 0.18) if ex < ey else (ex, min(ey, 0.1), 0.18), rgba=rgb)
    elif k == "car":
        _sub(world, "geom", type="box", pos=(x, y, 0.45), size=(ex, ey, 0.3), rgba=rgb)
        _sub(world, "geom", type="box", pos=(x, y, 1.0), size=(ex * 0.55, ey * 0.95, 0.25), rgba=(0.3, 0.35, 0.4, 1))
        for sx in (-1, 1):
            for sy in (-1, 1):
                ax, ay = (ex * 0.65, ey) if ex > ey else (ex, ey * 0.65)
                _sub(world, "geom", type="cylinder", pos=(x + sx * ax, y + sy * ay, 0.3), size=(0.3, 0.1),
                     zaxis=(0, 1, 0) if ex > ey else (1, 0, 0), rgba=(0.08, 0.08, 0.08, 1))
    else:
        _sub(world, "geom", type="box", pos=(x, y, ez), size=(ex, ey, ez), rgba=rgb)


def _distractor(world, dd, hx, hy):
    x, y, k = dd["x"], dd["y"], dd["kind"]
    if k == "red_box":
        _sub(world, "geom", name="d_red_box", type="box", pos=(x, y, 0.2), size=(0.2, 0.2, 0.2), rgba=(0.75, 0.08, 0.06, 1))
    elif k == "cone":  # traffic cone: orange stack, white band
        for i, (r, z) in enumerate([(0.13, 0.08), (0.1, 0.22), (0.07, 0.36), (0.045, 0.48)]):
            _sub(world, "geom", name=f"d_cone{i}", type="cylinder", pos=(x, y, z), size=(r, 0.07), rgba=(1, 0.8, 0.75, 1) if i == 2 else (0.95, 0.38, 0.05, 1))
    elif k == "extinguisher":
        _sub(world, "geom", name="d_ext", type="capsule", pos=(x, y, 0.3), size=(0.08, 0.2), rgba=(0.8, 0.05, 0.05, 1))
        _sub(world, "geom", name="d_ext_top", type="cylinder", pos=(x, y, 0.62), size=(0.03, 0.05), rgba=(0.1, 0.1, 0.1, 1))
    elif k == "ball":
        _sub(world, "geom", name="d_ball", type="sphere", pos=(x, y, 0.15), size=0.15, rgba=(1, 0.5, 0.05, 1))
    elif k == "poster":  # flat on a wall: red-orange sunset print
        s = dd["side"]
        size = (0.01, 0.4, 0.3) if s < 2 else (0.4, 0.01, 0.3)
        _sub(world, "geom", name="d_poster", type="box", pos=(x, y, dd["z"]), size=size, rgba=(0.95, 0.35, 0.08, 1), contype=0, conaffinity=0)


def distractor_points(spec):
    """World points of the things that can look like fire, for labelling false detections."""
    pts = {"red_box": (0, 0, 0.2), "cone": (0, 0, 0.3), "extinguisher": (0, 0, 0.3), "ball": (0, 0, 0.15)}
    out = {d["kind"]: (d["x"] + pts.get(d["kind"], (0, 0, 0))[0], d["y"], pts[d["kind"]][2] if d["kind"] in pts else d["z"]) for d in spec["distractors"]}
    if spec["lamp"]:
        out["lamp"] = (spec["lamp"]["x"], spec["lamp"]["y"], spec["lamp"]["z"])
    return out


# ---------------- rendering ----------------
def render_rgbd(r):
    """One render pass of a mujoco.Renderer, read RGB and metric depth (along the optical axis), as Renderer.render does.
    Limit: uses the Renderer's private fields to save the second render; fall back to two render() calls if they change."""
    r._gl_context.make_current()
    mujoco.mjr_render(r._rect, r._scene, r._mjr_context)
    rgb = np.empty((r.height, r.width, 3), np.uint8)
    z = np.empty((r.height, r.width), np.float32)
    mujoco.mjr_readPixels(rgb, z, r._rect, r._mjr_context)
    ext = r._model.stat.extent
    near, far = np.float32(r._model.vis.map.znear * ext), np.float32(r._model.vis.map.zfar * ext)
    c = np.float32(-0.5) * (-(far + near) / (far - near)) - np.float32(0.5)
    dcoef = np.float32(-0.5) * (-(np.float32(2) * far * near) / (far - near))
    return np.flipud(rgb).copy(), np.flipud(dcoef / (z.astype(np.float64) + c)).astype(np.float32)


# ---------------- fire sprites ----------------
SPRITE_N, SPRITE_H, SPRITE_W, SPRITE_FPS = 60, 192, 96, 30  # frames per loop (2 s), size, playback rate


def _fft_noise(rng, shape, beta, cutoff):
    """Noise that wraps exactly in both axes: white noise with a 1/f^beta spectrum, low-passed. Unit std."""
    fy, fx = np.fft.fftfreq(shape[0])[:, None], np.fft.fftfreq(shape[1])[None, :]
    f = np.hypot(fy, fx)
    f[0, 0] = 1.0
    spec = np.fft.fft2(rng.standard_normal(shape)) / f ** beta * np.exp(-(f / cutoff) ** 2)
    spec[0, 0] = 0
    out = np.real(np.fft.ifft2(spec))
    return out / out.std()


SPRITES = HERE / "sprites"  # real-fire clips, made by make_sprites.py: one WebP per clip, 60 frames in a 10 x 6 grid


def real_sprites():
    """Flame clips cut from real fire videos (sprites/SOURCES.md). A list with one entry per clip. Each entry is a list of
    mip levels, each (60, h, w, 4) float32 RGBA, row 0 = top, half the size of the level before (256 x 128 px down to 16 x 8).
    composite_fires samples the level closest to the flame size on the screen, like a camera that averages over a pixel."""
    clips = []
    for p in sorted(SPRITES.glob("*.webp")):
        sheet = np.asarray(Image.open(p).convert("RGBA"), np.float32) / 255
        h, w = sheet.shape[0] // 6, sheet.shape[1] // 10
        f = sheet.reshape(6, h, 10, w, 4).transpose(0, 2, 1, 3, 4).reshape(60, h, w, 4)
        pm = np.concatenate([f[..., :3] * f[..., 3:], f[..., 3:]], -1)  # premultiplied: average colour and alpha together
        levels = [f]
        while pm.shape[1] >= 32:
            pm = pm.reshape(60, pm.shape[1] // 2, 2, pm.shape[2] // 2, 2, 4).mean((2, 4))
            levels.append(np.concatenate([pm[..., :3] / np.maximum(pm[..., 3:], 1e-6), pm[..., 3:]], -1).astype(np.float32))
        clips.append(levels)
    assert clips, f"no sprites in {SPRITES}: run sim/mission/make_sprites.py"
    return clips


def flame_sprites(kind="procedural"):
    """kind "real": real_sprites(). kind "procedural" (default):
    Procedural animated flame, (n, h, w, 4) float32 RGBA, row 0 = top. Loops after n frames. Made once, cached.
    Three noise fields scroll up (1, 2 and 4 periods per loop) and bend and erode a flame-shaped heat field,
    which a black-body-like colour map turns into colour and opacity."""
    if kind == "real":
        return real_sprites()
    assert kind == "procedural", kind
    path = CACHE / "flame_sprites_v3.npy"
    if path.exists():
        return np.load(path)
    rng = np.random.default_rng(3)
    n, h, w = SPRITE_N, SPRITE_H, SPRITE_W
    layers = [(_fft_noise(rng, (2 * h, w), 1.2, 0.04), 1), (_fft_noise(rng, (2 * h, w), 1.0, 0.07), 2),
              (_fft_noise(rng, (2 * h, w), 0.8, 0.12), 4)]
    v = (1 - np.arange(h) / (h - 1))[:, None]  # 0 at the bottom, 1 at the top
    u = np.linspace(-1, 1, w)[None, :]
    stops = np.array([[0.0, 0.35, 0.02, 0.0], [0.2, 0.8, 0.14, 0.01], [0.4, 1.0, 0.38, 0.03], [0.6, 1.0, 0.62, 0.12],
                      [0.8, 1.0, 0.85, 0.42], [1.0, 1.0, 0.97, 0.8]])
    out = np.zeros((n, h, w, 4), np.float32)
    for k in range(n):
        ph = k / n
        nz = [np.roll(f, int(round(ph * cyc * 2 * h)), axis=0)[:h] for f, cyc in layers]  # rows move up with time
        puff = 1 + 0.1 * math.sin(2 * math.pi * 6 * ph)  # 3 Hz puffing at 30 fps playback
        vv = v * puff + 0.08 * nz[0] * v
        uu = u + (0.16 * nz[0] + 0.08 * nz[1]) * (0.15 + v) ** 1.2
        width = 0.85 * np.clip(1 - vv, 0, 1) ** 0.9 * (0.75 + 0.25 * np.clip(vv / 0.15, 0, 1))
        heat = np.clip(1 - np.abs(uu / np.maximum(width, 1e-3)) ** 1.6, 0, 1) * np.clip(1 - vv, 0, 1) ** 0.5
        heat = heat - np.clip(vv, 0, 1) ** 1.3 * (0.4 + 0.3 * nz[1] + 0.12 * nz[2])  # tongues and detached pieces at the top
        heat = np.clip(1.4 * heat, 0, 1)
        rgb = np.stack([np.interp(heat, stops[:, 0], stops[:, i]) for i in (1, 2, 3)], -1)
        alpha = np.clip((heat - 0.06) / 0.25, 0, 1) ** 1.3 * 0.97
        out[k] = np.concatenate([rgb, alpha[..., None]], -1)
    np.save(path, out)
    return out


def flame_size(f, s):
    """Flame billboard (half width, height), m, for a fire with tray diameter d at strength s (0..1).
    Height ~ s^0.4 (Heskestad: flame length ~ heat release^0.4), full height 2.5 d (ASSUMED)."""
    return 0.62 * f["d"] * (0.55 + 0.45 * math.sqrt(s)), 2.5 * f["d"] * s ** 0.4


def composite_fires(rgb, depth, cam_pos, cam_mat, fy, fires, strengths, t, sprites, glow=0.6):
    """Draw each burning fire as a vertical billboard that faces the camera, with a per-pixel depth test.
    cam_mat: MuJoCo camera rotation (columns: right, up, back). fy: focal length in px. Returns the image (uint8)
    and, for each fire, (visible flame pixels, visible box (u0, v0, u1, v1) or None).
    sprites: the procedural array, or the real clips (a list): each fire gets clip phase % n, mirrored for every other n."""
    h, w = depth.shape
    img = rgb.astype(np.float32) / 255
    sig = max(2.0, 0.012 * w)  # px, bloom blur
    m = int(3 * sig)
    info = []
    for i, (f, s) in enumerate(zip(fires, strengths)):
        if s <= 0:
            info.append((0, None))
            continue
        hw, fh = flame_size(f, s)
        c = np.array([f["x"], f["y"], 0.03])
        nrm = cam_pos[:2] - c[:2]
        nrm = np.array([*nrm / max(np.linalg.norm(nrm), 1e-6), 0.0])
        right = np.array([-nrm[1], nrm[0], 0.0])
        corners = [c + a * hw * right + b * fh * np.array([0, 0, 1.0]) for a in (-1, 1) for b in (0, 1)]
        pc = [(cam_mat.T @ (p - cam_pos)) for p in corners]
        if all(q[2] > -0.05 for q in pc):
            info.append((0, None))
            continue
        uv = np.array([(w / 2 + fy * q[0] / -q[2], h / 2 - fy * q[1] / -q[2]) for q in pc if q[2] < -0.05])
        u0, v0 = np.floor(uv.min(0)).astype(int) - 1
        u1, v1 = np.ceil(uv.max(0)).astype(int) + 1
        u0, v0, u1, v1 = max(u0, 0), max(v0, 0), min(u1, w - 1), min(v1, h - 1)
        if u1 <= u0 or v1 <= v0:
            info.append((0, None))
            continue
        vv, uu = np.mgrid[v0:v1 + 1, u0:u1 + 1]
        d_cam = np.stack([(uu - w / 2) / fy, -(vv - h / 2) / fy, -np.ones_like(uu, float)], -1)  # camera frame, depth 1
        d_w = d_cam @ cam_mat.T
        den = d_w @ nrm
        tt = ((c - cam_pos) @ nrm) / np.where(np.abs(den) < 1e-9, 1e-9, den)  # = depth along the optical axis
        p = cam_pos + tt[..., None] * d_w
        a = ((p - c) @ right) / hw
        b = (p[..., 2] - c[2]) / fh
        inside = (tt > 0) & (np.abs(a) <= 1) & (b >= 0) & (b <= 1)
        if isinstance(sprites, list):  # real clips: the mip level with about one sprite pixel per screen pixel, or more
            levels = sprites[f["phase"] % len(sprites)]
            k = int(math.log2(max(levels[0].shape[1] / max(np.ptp(uv[:, 1]), 1.0), 1.0)))
            clip = levels[min(k, len(levels) - 1)]
            a = -a if (f["phase"] // len(sprites)) % 2 else a
        else:
            clip = sprites
        frame = clip[(int(t * SPRITE_FPS) + f["phase"]) % len(clip)]
        rows, cols = (1 - b) * (frame.shape[0] - 1), (a + 1) / 2 * (frame.shape[1] - 1)
        samp = np.stack([ndimage.map_coordinates(frame[..., k], [rows, cols], order=1, mode="nearest") for k in range(4)], -1)
        vis = inside & (depth[v0:v1 + 1, u0:u1 + 1] > tt - f["d"] / 2)  # depth test; the tray rim may cover the flame base
        al = np.where(vis, samp[..., 3], 0.0) * min(1.0, 0.6 + 3 * s)  # a dying flame is also dimmer
        crop = img[v0:v1 + 1, u0:u1 + 1]
        layer = samp[..., :3] * al[..., None]
        crop[:] = crop * (1 - al[..., None]) + layer
        if glow:  # bloom: a blurred copy of the flame, added around it (camera glare)
            g0, g1, g2, g3 = max(v0 - m, 0), max(u0 - m, 0), min(v1 + m, h - 1), min(u1 + m, w - 1)
            big = np.zeros((g2 - g0 + 1, g3 - g1 + 1, 3), np.float32)
            big[v0 - g0:v1 - g0 + 1, u0 - g1:u1 - g1 + 1] = layer
            img[g0:g2 + 1, g1:g3 + 1] += glow * ndimage.gaussian_filter(big, (sig, sig, 0), truncate=3.0)
        on = al > 0.1
        n_px = int(on.sum())
        if n_px:
            rr, cc = np.nonzero(on)
            info.append((n_px, (u0 + cc.min(), v0 + rr.min(), u0 + cc.max(), v0 + rr.max())))
        else:
            info.append((0, None))
    return (np.clip(img, 0, 1) * 255).astype(np.uint8), info


# ---------------- water ----------------
class Water:
    """Droplets from the nozzle with gravity and quadratic drag. Each droplet carries an equal share of the flow.
    Hits: a droplet inside a burning fire's cylinder (tray radius, up to half the flame height, at least 5 cm) puts its water
    on that fire. Furniture boxes and walls stop droplets."""

    def __init__(self, spec, boxes, rng):
        self.v0 = jet_v0(spec) * spec["v0_scale"]  # the true jet; aim/ gets jet_v0(spec)
        self.flow = spec["flow"] / 60000 * self.v0 / spec["v0"]  # m^3/s: for a fixed nozzle, flow ~ exit speed
        self.k, self.rng = spec["drag"], rng
        self.boxes = boxes  # (n, 2, 3): min and max corner of each obstacle box
        self.lim = np.array(spec["room"]) / 2
        self.outdoor = spec["outdoor"]
        self.p = np.zeros((0, 3))
        self.v = np.zeros((0, 3))
        self.left = spec["tank"] / 1000  # m^3 in the tank
        self.used = 0.0
        self.landed = []  # (x, y) of droplets that reached the floor, for the stray-water metric
        self.t_emit = 0.0

    def step(self, t, pump_on, nozzle, direction, fires, strengths, dt=DROP_DT):
        """Advance the droplets by dt. Returns the water mass (kg) that hit each fire in this step."""
        hit = np.zeros(len(fires))
        if pump_on and self.left > 0 and t >= self.t_emit:
            self.t_emit = t + 1 / EMIT_HZ
            n = EMIT_N
            dvec = direction + SPREAD * self.rng.standard_normal((n, 3))
            dvec /= np.linalg.norm(dvec, axis=1, keepdims=True)
            sp = self.v0 * (1 + 0.03 * self.rng.standard_normal((n, 1)))
            self.p = np.vstack([self.p, np.repeat(nozzle[None], n, 0) + dvec * sp * self.rng.uniform(0, dt, (n, 1))])
            self.v = np.vstack([self.v, dvec * sp])
            vol = min(self.flow / EMIT_HZ, self.left)
            self.left -= vol
            self.used += vol
        if not len(self.p):
            return hit
        acc = lambda v: np.array([0, 0, -G]) - self.k * np.linalg.norm(v, axis=1, keepdims=True) * v
        vh = self.v + 0.5 * dt * acc(self.v)  # midpoint step: second order, so the 5 ms step matches aim/ballistics
        self.p += vh * dt
        self.v += dt * acc(vh)
        drop = self.flow / EMIT_HZ / EMIT_N * 1000  # kg per droplet
        gone = self.p[:, 2] <= 0
        for i, (f, s) in enumerate(zip(fires, strengths)):
            if s <= 0:
                continue
            zt = max(0.05, 0.5 * flame_size(f, s)[1])
            inn = ~gone & (np.hypot(self.p[:, 0] - f["x"], self.p[:, 1] - f["y"]) < f["d"] / 2) & (self.p[:, 2] < zt)
            hit[i] += drop * inn.sum()
            gone |= inn
        if len(self.boxes):
            gone |= np.any(np.all((self.p[:, None] >= self.boxes[None, :, 0]) & (self.p[:, None] <= self.boxes[None, :, 1]), axis=2), axis=1)
        if not self.outdoor:
            gone |= np.any(np.abs(self.p[:, :2]) > self.lim, axis=1)
        floor = (self.p[:, 2] <= 0) & gone
        self.landed += self.p[floor, :2].tolist()
        self.p, self.v = self.p[~gone], self.v[~gone]
        return hit


def jet_v0(spec):
    """Exit speed at the mission's pump voltage (v0 ~ V^0.73)."""
    return spec["v0"] * (spec["volts"] / 11.1) ** 0.73


def fire_step(fires, strengths, water_kg, dt, spec):
    """Strength: water reduces it (water_per_area kg per m^2 of tray from full), it regrows over regrow s while > 0."""
    out = []
    for f, s, wk in zip(fires, strengths, water_kg):
        if s > 0:
            s = s - wk / (spec["water_per_area"] * math.pi * f["d"] ** 2 / 4) + dt / spec["regrow"]
            s = min(1.0, s) if s > 0 else 0.0
        out.append(s)
    return out


def obstacle_boxes(m):
    """Axis-aligned boxes of the static world geoms (walls, furniture, distractors), for the droplets."""
    out = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != 0 or m.geom_type[g] not in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
            continue
        name = m.geom(g).name
        if name.startswith(("tray", "fuel", "ceiling")) or m.geom_contype[g] == 0 and not name.startswith("d_"):
            continue
        p = m.geom_pos[g]
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            rm = np.zeros(9)
            mujoco.mju_quat2Mat(rm, m.geom_quat[g])
            ext = np.abs(rm.reshape(3, 3)) @ m.geom_size[g]
        else:
            r, hz = m.geom_size[g][0], m.geom_size[g][1]
            ext = np.array([r, r, hz + (r if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CAPSULE else 0)])
        out.append([p - ext, p + ext])
    return np.array(out).reshape(-1, 2, 3)
