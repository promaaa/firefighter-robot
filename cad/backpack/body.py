"""Splash guard (front) and concept B backpack (rear) for the BHL torso.

    uv run --project cad python cad/backpack/body.py

Builds every printed part in the robot frame (mm, x forward, y left, z up, feet on the floor,
torso frame top z = 791), writes STEP + STL to out/, prints volume and mass, then runs the checks:
bed fit, minimum wall parameter, and leg clearance over a grid of hip roll, yaw, pitch and knee angles.
Torso numbers: measured on the BHL mesh base_visual.stl.
"""
import itertools
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from build123d import Box, Cone, Cylinder, Pos, Rot, export_step, export_stl
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

HERE = Path(__file__).parent
OUT = HERE / "out"
BHL = HERE.parent.parent / "third_party/berkeley-humanoid-lite-assets/data/robots/berkeley_humanoid/berkeley_humanoid_lite"

# ---------------- parameters (mm) ----------------
# Torso frame, measured on base_visual.stl (measure.py). 2020 profile, 6 mm slot.
FRONT_X, REAR_X = 70.0, -70.0        # outer faces of the front and rear extrusions
POST_Y = 65.0                        # slot centre of the four posts (posts 20 x 20, z 571..771)
TOP_RAIL_Z, BOT_RAIL_Z = 781.0, 561.0  # slot centre of the top rails (z 771..791) and bottom rails (z 551..571)
REAR_BUMP_X, REAR_BUMP_Z = -74.0, (587.0, 627.0)  # part on the rear face that sticks out 4 mm

M5_CLR, M5_CB_D, M5_CB = 5.5, 10.0, 7.0   # M5 into drop-in T-nuts; counterbore diameter, depth
M4_CLR, M4_CSK = 4.5, 8.6                 # M4 clearance, countersink head diameter (DIN 7991)
INSERT_D, INSERT_L = 5.6, 8.5             # M4 heat-set insert hole (M4 x 8.1 insert)
WALL_MIN = 2.0

# Splash guard: two flat panels on the front face, upper one overlaps the lower one with a drip lip.
G_T = 2.5                    # panel thickness
G_Y = 78.0                   # half width (posts end at y = +-75)
G_Z = (551.0, 675.0, 676.0, 790.0)  # lower z0, lower z1, upper z0, upper z1 (top 1 mm under the deck plate)
LIP_L, LIP_T = 12.0, 2.5     # drip lip length and thickness, 45 deg down and forward
HOOD_Z = 766.0               # hood lip over the vents, just under the top rail
VENT = (4, 24.0, 4.0, 760.0)  # count, length, height, z centre (under the hood)
CABLE = (40.0, 12.0, 579.0)  # cable slot in the lower panel, just above the bottom rail: width, height, z centre

# Backpack: two T-nut brackets on the rear frame, one plate, a tank cradle and a pump bracket.
BR_T = 12.0                  # bracket depth (x): plate sits at REAR_X - BR_T, 8 mm behind the rear bump
BR_UP_Z = (771.0, 791.0)     # upper bracket on the top rail
BR_LO_Z = (638.0, 682.0)     # lower bracket on the posts, above the rear bump
BR_LO_Y = 85.0               # lower bracket half length
TNUT_LO_Z, INS_LO_Z = 650.0, 670.0   # lower bracket: T-nut screws and inserts at different heights
UP_TNUT_Y, UP_INS_Y = 40.0, 68.0
P_T, P_Y, P_Z = 5.0, 108.0, (588.0, 788.0)  # plate thickness, half width, z range

# Tank: Nalgene 32 oz (1 L) wide mouth, 3.5 in x 8.25 in (well.ca listing).
TANK_D, TANK_H, TANK_Y = 89.0, 210.0, 52.0
CR_T = 6.0                   # cradle backing thickness
CR_GAP = 1.0                 # bottle to cradle
CR_WALL = 3.0                # saddle and floor wall
CR_FLOOR_Z = 591.0           # floor bottom; bottle bottom = CR_FLOOR_Z + CR_WALL
SADDLE_DZ = (35.0, 130.0)    # saddle centres above the bottle bottom
SADDLE_H = 35.0
STRAP_W, STRAP_T = 27.0, 3.0  # slot for a 25 mm hook-and-loop strap; channel depth behind the backing
CR_NUT_Z = 700.0             # two more cradle / pump-bracket bolts with nuts on the robot side
BOLT_DY = 22.0               # bolts at TANK_Y +- 22 and PUMP_Y +- 22

# Pump: Seaflo 21 series SFDP1-010-040-21 (Seaflo brochure). Base against the plate, head down.
PUMP_L, PUMP_W, PUMP_H = 169.0, 73.5, 62.0   # length, head width, height above the feet
PUMP_HOLES = (79.0, 55.0)    # foot hole pattern: across (y), along the motor (z); holes 5 mm
PUMP_Y, PUMP_Z = -54.0, 690.0
PB_T, BOSS_D, BOSS_H, PB_Y = 5.0, 12.0, 10.0, 48.0
PB_Z = (652.0, 728.0)

# Print mass estimate: PLA 1.24 g/cm3, 4 perimeters, 5 top/bottom layers, 30 % gyroid.
# Our walls are 2.5 to 12 mm, so we assume the parts print at 70 % of solid (guess).
RHO, FILL = 1.24, 0.70
BED = (220.0, 220.0, 250.0)
LEG_CLEAR_MIN = 10.0
# Proposed software knee limit. Over the full URDF range (knee up to 2.44 rad) the foot and shank fold up behind
# the hips and touch the backpack. Knee <= 1.40 rad gives 23 mm, 1.45 rad gives 11.5 mm (sweep, 2026-10-09).
KNEE_MAX = 1.4


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def hole_x(y, z, d, x0, x1):
    """Cylinder along x from x0 to x1."""
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(d / 2, abs(x1 - x0))


def lip(x_face, z_top, y0, y1):
    """Drip lip: a strip from the panel face going forward and down at 45 deg."""
    c = math.cos(math.radians(45))
    r = LIP_L / 2 - 1.5  # sink 1.5 mm into the panel so the solids fuse
    return Pos(x_face + r * c, (y0 + y1) / 2, z_top - LIP_T / 2 - r * c) * Rot(0, 45, 0) * Box(LIP_L, y1 - y0, LIP_T)


# ---------------- splash guard ----------------
def guard_upper():
    x0, x1 = FRONT_X, FRONT_X + G_T
    p = box(x0, x1, -G_Y, G_Y, G_Z[2], G_Z[3])
    p += lip(x1, HOOD_Z, -G_Y, G_Y)
    p += lip(x1, G_Z[2] + LIP_T, -G_Y, G_Y)   # lip over the joint to the lower panel
    n, vl, vh, vz = VENT
    for i in range(n):
        y = (i - (n - 1) / 2) * (vl + 6)
        p -= box(x0 - 1, x1 + 1, y - vl / 2, y + vl / 2, vz - vh / 2, vz + vh / 2)
    for y, z in ((POST_Y, 720), (-POST_Y, 720), (30, TOP_RAIL_Z), (-30, TOP_RAIL_Z)):
        p -= hole_x(y, z, M5_CLR, x0 - 1, x1 + 1)
    return p


def guard_lower():
    x0, x1 = FRONT_X, FRONT_X + G_T
    p = box(x0, x1, -G_Y, G_Y, G_Z[0], G_Z[1])
    p += lip(x1, G_Z[0] + LIP_T, -G_Y, G_Y)
    w, h, zc = CABLE
    p -= box(x0 - 1, x1 + 1, -w / 2, w / 2, zc - h / 2, zc + h / 2)
    p += lip(x1, zc + h / 2 + 9, -w / 2 - 8, w / 2 + 8)   # short hood over the cable slot
    for y, z in ((POST_Y, 600), (-POST_Y, 600), (30, BOT_RAIL_Z), (-30, BOT_RAIL_Z)):
        p -= hole_x(y, z, M5_CLR, x0 - 1, x1 + 1)
    return p


# ---------------- backpack ----------------
XP0 = REAR_X - BR_T            # plate front face (robot side)
XP1 = XP0 - P_T                # plate rear face
TANK_R = TANK_D / 2
XC1 = XP1 - CR_T               # cradle backing rear face
TANK_X = XC1 - CR_GAP - TANK_R
TANK_Z0 = CR_FLOOR_Z + CR_WALL


def bolt_pts(yc):
    return [(yc + s * BOLT_DY, z) for s in (-1, 1) for z in (INS_LO_Z, CR_NUT_Z)]


def bracket_upper():
    p = box(XP0, REAR_X, -75, 75, *BR_UP_Z)
    for s in (-1, 1):
        p -= hole_x(s * UP_TNUT_Y, TOP_RAIL_Z, M5_CLR, XP0 - 1, REAR_X + 1)
        p -= hole_x(s * UP_TNUT_Y, TOP_RAIL_Z, M5_CB_D, XP0 - 1, XP0 + M5_CB)
        p -= hole_x(s * UP_INS_Y, TOP_RAIL_Z, INSERT_D, XP0 - 1, XP0 + INSERT_L)
    return p


def bracket_lower():
    p = box(XP0, REAR_X, -BR_LO_Y, BR_LO_Y, *BR_LO_Z)
    for s in (-1, 1):
        p -= hole_x(s * POST_Y, TNUT_LO_Z, M5_CLR, XP0 - 1, REAR_X + 1)
        p -= hole_x(s * POST_Y, TNUT_LO_Z, M5_CB_D, XP0 - 1, XP0 + M5_CB)
    for y, z in bolt_pts(TANK_Y) + bolt_pts(PUMP_Y):
        if z == INS_LO_Z:
            p -= hole_x(y, z, INSERT_D, XP0 - 1, XP0 + INSERT_L)
    return p


def plate():
    p = box(XP1, XP0, -P_Y, P_Y, *P_Z)
    for s in (-1, 1):
        p -= hole_x(s * UP_INS_Y, TOP_RAIL_Z, M4_CLR, XP1 - 1, XP0 + 1)
    for y, z in bolt_pts(TANK_Y) + bolt_pts(PUMP_Y):
        p -= hole_x(y, z, M4_CLR, XP1 - 1, XP0 + 1)
    return p


def countersunk(p, y, z, x_head, x_far):
    """M4 countersunk hole (90 deg), head flush at x_head, shank towards x_far."""
    sgn = 1 if x_far > x_head else -1
    p -= hole_x(y, z, M4_CLR, min(x_head, x_far) - 1, max(x_head, x_far) + 1)
    h = (M4_CSK - M4_CLR) / 2
    return p - Pos(x_head + sgn * h / 2, y, z) * Rot(0, 90 * sgn, 0) * Cone(M4_CSK / 2, M4_CLR / 2, h)


def cradle():
    sad = (TANK_Y - TANK_R - CR_GAP - CR_WALL, TANK_Y + TANK_R + CR_GAP + CR_WALL)   # saddle and cup width
    ys = (sad[0] - 8, sad[1] + 8)                                                    # backing width
    slot_dy = TANK_R + CR_GAP + CR_WALL + 2.5                                        # strap slots beside the saddles
    tank_cut = Pos(TANK_X, TANK_Y, TANK_Z0 + 300) * Cylinder(TANK_R + CR_GAP, 600)
    p = box(XC1, XP1, *ys, CR_FLOOR_Z, TANK_Z0 + SADDLE_DZ[1] + SADDLE_H / 2)       # backing
    p += box(TANK_X - 20, XC1 + 0.1, *sad, CR_FLOOR_Z, TANK_Z0 + 15) - tank_cut     # cup wall
    p += box(TANK_X - 20, XC1 + 0.1, *sad, CR_FLOOR_Z, TANK_Z0)                      # floor
    p -= Pos(TANK_X, TANK_Y, CR_FLOOR_Z) * Cylinder(6, 20)                           # drain hole
    for dz in SADDLE_DZ:
        zc = TANK_Z0 + dz
        p += box(TANK_X, XC1 + 0.1, *sad, zc - SADDLE_H / 2, zc + SADDLE_H / 2) - tank_cut
        for s in (-1, 1):
            yc = TANK_Y + s * slot_dy
            p -= box(XC1 - 1, XP1 + 1, yc - 2, yc + 2, zc - STRAP_W / 2, zc + STRAP_W / 2)
        p -= box(XP1 - STRAP_T, XP1 + 1, ys[0] - 1, ys[1] + 1, zc - STRAP_W / 2, zc + STRAP_W / 2)  # strap channel
    for y, z in bolt_pts(TANK_Y):
        p = countersunk(p, y, z, XC1, XP1)
    return p


def pump_bracket():
    xb = XP1 - PB_T
    p = box(xb, XP1, PUMP_Y - PB_Y, PUMP_Y + PB_Y, *PB_Z)
    hy, hz = PUMP_HOLES[0] / 2, PUMP_HOLES[1] / 2
    for sy in (-1, 1):
        for sz in (-1, 1):
            y, z = PUMP_Y + sy * hy, PUMP_Z + sz * hz
            p += hole_x(y, z, BOSS_D, xb - BOSS_H, xb + 0.1)
            p -= hole_x(y, z, INSERT_D, xb - BOSS_H - 1, xb - BOSS_H + INSERT_L)
    for y, z in bolt_pts(PUMP_Y):
        p = countersunk(p, y, z, xb, XP1)
    return p


def tank_ref():
    return Pos(TANK_X, TANK_Y, TANK_Z0 + TANK_H / 2) * Cylinder(TANK_R, TANK_H)


def pump_ref():
    x0 = XP1 - PB_T - BOSS_H
    return box(x0 - PUMP_H, x0, PUMP_Y - PUMP_W / 2, PUMP_Y + PUMP_W / 2, PUMP_Z - PUMP_L / 2, PUMP_Z + PUMP_L / 2)


PARTS = {"guard_upper": guard_upper, "guard_lower": guard_lower, "bracket_upper": bracket_upper,
         "bracket_lower": bracket_lower, "plate": plate, "cradle": cradle, "pump_bracket": pump_bracket}
REFS = {"ref_tank": tank_ref, "ref_pump": pump_ref}  # bought parts, for clearance and renders only


# ---------------- leg kinematics (URDF) ----------------
def _tf(el):
    xyz = [float(v) for v in (el.get("xyz") or "0 0 0").split()] if el is not None else [0, 0, 0]
    rpy = [float(v) for v in (el.get("rpy") or "0 0 0").split()] if el is not None else [0, 0, 0]
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = Rotation.from_euler("xyz", rpy).as_matrix(), xyz
    return t


URDF = ET.parse(BHL / "urdf/berkeley_humanoid_lite.urdf").getroot()
JOINTS = {j.find("child").get("link"): j for j in URDF.findall("joint")}
STAND = {"knee_pitch": 0.4, "ankle_pitch": -0.3}   # default standing angles for joints not swept
SWEPT = ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch")


def frames(q):
    """World frame of every link for joint angles q (missing joints = 0), robot frame in metres before the shift."""
    out = {}

    def frame(link):
        if link not in out:
            j = JOINTS.get(link)
            if j is None:
                out[link] = np.eye(4)
            else:
                rot = np.eye(4)
                ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
                rot[:3, :3] = Rotation.from_rotvec(ax * q.get(j.get("name"), 0.0)).as_matrix()
                out[link] = frame(j.find("parent").get("link")) @ _tf(j.find("origin")) @ rot
        return out[link]

    for l in URDF.findall("link"):
        frame(l.get("name"))
    return out


def link_points(side, n=6000):
    """Surface points of each link of one leg in its own link frame (metres). 6000 points: spacing about 2 mm."""
    pts = {}
    for l in URDF.findall("link"):
        name, v = l.get("name"), l.find("visual")
        if name.startswith(f"leg_{side}_") and v is not None:
            m = trimesh.load(BHL / "meshes" / Path(v.find("geometry/mesh").get("filename")).name)
            m.apply_transform(_tf(v.find("origin")))
            pts[name] = m.sample(n)
    return pts


def leg_cloud(side, pts, q):
    """All points of one leg in the robot frame, mm, with the 0.039 m shift, for joint angles q."""
    fr = frames(q)
    out = [p @ fr[n][:3, :3].T + fr[n][:3, 3] for n, p in pts.items()]
    return (np.vstack(out) - [0, 0, 0.039]) * 1000


def limits(side, j):
    lim = URDF.find(f"joint[@name='leg_{side}_{j}_joint']/limit")
    return float(lim.get("lower")), float(lim.get("upper"))


def lims_of(side, knee_max=None):
    """Sweep ranges: URDF limits, with the knee upper limit cut to knee_max when given."""
    lims = [limits(side, j) for j in SWEPT]
    if knee_max is not None:
        lims[3] = (lims[3][0], min(lims[3][1], knee_max))
    return lims


def pose(side, vals):
    q = {f"leg_{side}_{j}_joint": a for j, a in STAND.items()}
    q.update({f"leg_{side}_{j}_joint": a for j, a in zip(SWEPT, vals)})
    return q


CAP = 60.0  # mm; the sweep reports "more than CAP" for parts that never come closer


def sweep(part_pts, steps=7, refine=5, knee_max=None):
    """Minimum distance from each part to both legs over a grid of hip roll, yaw, pitch and knee angles.
    Coarse grid of steps^4 per leg, then a finer grid of refine^4 around each part's worst pose."""
    trees = {n: cKDTree(p) for n, p in part_pts.items()}
    boxes = {n: (p.min(0), p.max(0)) for n, p in part_pts.items()}
    worst = {n: (np.inf, None, None) for n in trees}  # stays inf when a part is never within CAP of a leg

    def check(side, legpts, vals, names):
        cloud = leg_cloud(side, legpts, pose(side, vals))
        for n in names:
            bound = min(worst[n][0], CAP)
            near = cloud[np.all((cloud > boxes[n][0] - bound) & (cloud < boxes[n][1] + bound), axis=1)]
            if len(near):
                d = trees[n].query(near, distance_upper_bound=bound)[0].min()
                if d < worst[n][0]:
                    worst[n] = (d, side, tuple(round(float(a), 3) for a in vals))

    for side in ("left", "right"):
        legpts = link_points(side)
        lims = lims_of(side, knee_max)
        for vals in itertools.product(*[np.linspace(lo, hi, steps) for lo, hi in lims]):
            check(side, legpts, vals, trees)
    for n in trees:  # refine around the worst coarse pose of each part
        d, side, vals = worst[n]
        if side is None:
            continue
        legpts = link_points(side)
        lims = lims_of(side, knee_max)
        grids = [np.linspace(max(lo, v - (hi - lo) / (steps - 1)), min(hi, v + (hi - lo) / (steps - 1)), refine)
                 for v, (lo, hi) in zip(vals, lims)]
        for v2 in itertools.product(*grids):
            check(side, legpts, v2, [n])
    return worst


def main():
    OUT.mkdir(exist_ok=True)
    x0 = XP1 - PB_T - BOSS_H
    (OUT / "assembly.json").write_text(json.dumps({  # metres, for render.py
        "tank": {"x": TANK_X / 1e3, "y": TANK_Y / 1e3, "z0": TANK_Z0 / 1e3, "r": TANK_R / 1e3, "h": TANK_H / 1e3},
        "pump": {"x": (x0 - PUMP_H / 2) / 1e3, "y": PUMP_Y / 1e3, "z0": (PUMP_Z - PUMP_L / 2) / 1e3,
                 "z1": (PUMP_Z + PUMP_L / 2) / 1e3, "w": PUMP_W / 1e3, "h": PUMP_H / 1e3}}, indent=1))
    total = 0.0
    print(f"{'part':15s} {'vol cm3':>8s} {'solid g':>8s} {'print g':>8s}  bbox mm")
    meshes = {}
    for name, f in {**PARTS, **REFS}.items():
        s = f()
        export_step(s, str(OUT / f"{name}.step"))
        export_stl(s, str(OUT / f"{name}.stl"))
        meshes[name] = trimesh.load(OUT / f"{name}.stl")
        bb = s.bounding_box().size
        dims = sorted([bb.X, bb.Y, bb.Z])
        if name in PARTS:
            v = s.volume / 1000
            g = v * RHO * FILL
            total += g if not name.startswith("guard") else 0
            print(f"{name:15s} {v:8.1f} {v * RHO:8.0f} {g:8.0f}  {bb.X:.0f} x {bb.Y:.0f} x {bb.Z:.0f}")
            assert all(a <= b for a, b in zip(dims, sorted(BED))), f"{name} does not fit the bed"
    pump_g, bottle_g, water_g, hw_g = 570, 180, 946, 120
    print(f"backpack printed parts {total:.0f} g; with pump {pump_g} g, bottle {bottle_g} g (guess), "
          f"hardware + straps + hose {hw_g} g (guess): empty {total + pump_g + bottle_g + hw_g:.0f} g, "
          f"with {water_g} g water {total + pump_g + bottle_g + hw_g + water_g:.0f} g")
    assert min(G_T, LIP_T, CR_WALL, CR_T - STRAP_T, PB_T, P_T, (BOSS_D - INSERT_D) / 2) >= WALL_MIN

    # Leg clearance: hip roll, hip yaw, hip pitch and knee on a 7^4 grid per leg (URDF limits), refined around
    # the worst pose of each part. Ankle at -0.3 rad. The full range is reported; the assert uses knee <= KNEE_MAX.
    pts = {n: np.vstack([m.vertices, m.sample(30000)]) for n, m in meshes.items()}
    for label, kmax in (("full URDF range", None), (f"knee <= {KNEE_MAX} rad", KNEE_MAX)):
        worst = sweep(pts, knee_max=kmax)
        print(f"leg clearance, {label}:")
        for n, (d, side, vals) in sorted(worst.items(), key=lambda kv: kv[1][0]):
            print(f"  {n:15s} " + (f"min {d:5.1f} mm, {side} leg, roll/yaw/pitch/knee = {vals} rad" if side else f"more than {CAP:.0f} mm"))
    for n, (d, side, vals) in worst.items():
        assert d > LEG_CLEAR_MIN, f"{n} is {d:.1f} mm from the {side} leg at roll/yaw/pitch/knee {vals}"
    print(f"checks passed: bed fit, wall >= 2 mm, leg clearance > {LEG_CLEAR_MIN:.0f} mm with knee <= {KNEE_MAX} rad")


if __name__ == "__main__":
    main()
