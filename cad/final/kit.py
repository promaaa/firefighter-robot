"""Print kit of the final design F2: every printed part, the two cut drawings, the print sheet, the hardware list and
the checks. DESIGN=F1 builds the old design F1 (kit_f1/, out/).

    uv run --project cad python cad/final/tank.py                              # first: tanks and the deck
    uv run --project cad --with matplotlib python cad/final/kit.py             # about 30 s

Writes <kit>/stl/*.stl and <kit>/step/*.step (robot frame, mm), <kit>/drawings/ (DXF, PDF, PNG of the acrylic shelf
and the aluminium pod plate), <kit>/print_sheet.md, <kit>/hardware.md and <out>/kit.json (envelopes, masses and
centres for layout.py, render.py and the simulation). <kit>, <out> = kit, out_f2 for F2 and kit_f1, out for F1.
Stops with an AssertionError when a check fails.

Printed parts (parameters below): double battery holder (Zeee or VEGA packs), rear pod (spacers, 3S basket, splash
cover), IMU mount, camera and nozzle head on the tilt U, pump holder, brake resistor mounts, hose clips, frame cable
clips. The tank pieces and the deck come from tank.py. The pan-tilt parts come from cad/parts/top_module_sts3215.py.
The pump holder fits the DWP-385. The team uses the Kamoer KLP02 24 V pump: its holder follows when the pump is measured.
Robot frame: x forward, y left, z up, feet on the floor, torso frame top z = 791.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh
from build123d import CenterOf, Pos, RegularPolygon, Rot, export_step, export_stl, extrude
from scipy.spatial import cKDTree

HERE = Path(__file__).parent
sys.path[:0] = [str(HERE)]
import tank  # noqa: E402  sets up cad/torso_tank/body.py (tank.tt) and cad/parts/top_module.py (tank.tm)
F2 = tank.F2
OUT, KIT = tank.OUT, HERE / ("kit" if F2 else "kit_f1")
import layout  # noqa: E402  torso_points(), the F1 places of the bought parts
import top_module_sts3215 as st  # noqa: E402

tt, tm, bp = tank.tt, tank.tm, tank.tt.bp
P = layout.P                                   # parts.json by id
T = 791.0                                      # torso frame top
BED = bp.BED                                   # 220 x 220 x 250
RHO = 1.27                                     # PETG g/cm3
M5_CLR, M5_CB_D, M4_CLR, M3_CLR, M25_CLR, M2_CLR = 5.5, 10.5, 4.5, 3.4, 2.9, 2.4
M3_NUT_AF, M3_NUT_T = 5.6, 2.6                 # M3 nut pocket (5.5 AF nut + fit)
FIT = 0.3                                      # pocket clearance around a bought part
WALL_MIN = 2.0

# ---------------- torso facts used here (BHL mesh, see README) ----------------
RAIL_TOP = 571.0                               # top face of the bottom rails (front x 50..70, rear, centre y +-10)
TONGUE = (34.0, 50.0, 26.0, 635.0)             # PC shelf bracket tongues: |x| 34..50, |y| <= 26, underside z 635
BUMP = (-74.0, -64.0, -76.0, 74.0, 587.0, 627.0)   # part on the rear face of the BHL torso: no screw heads there
SHELF_T = 5.0
SHELF_Z = tank.SHELF_TOP - SHELF_T             # 641: the acrylic shelf rests on the bracket bars (top z 641)
BRACKET_HOLES = [(sx * 56.0, sy * 9.8) for sx in (-1, 1) for sy in (-1, 1)]   # holes in the bracket bars (mesh)

# ---------------- 1. double battery holder ----------------
# (length along y, width along x, height, pack bottom z, strap centre |y|). Packs lie side by side in x, ends out of
# the side openings between the posts like the BHL pack. VEGA lies on its side (44 wide, 51 high).
PACKS = {"zeee": (138.0, 47.0, 48.5, 584.0, 24.0), "vega": (154.0, 44.0, 51.0, 583.0, 37.0)}
PACK = "zeee"                                  # the pack of the F1 layout
H_DIV, H_CLR, H_WALL, H_WALL_H = 2.0, 0.5, 3.0, 22.0
H_X = 66.0                                     # base reaches over the front and rear bottom rails
STRAP_W, STRAP_T = 20.0, 1.5                   # hook-and-loop strap
H_SCREWS = [(60.0, 0.0), (-60.0, 0.0), (28.0, 0.0), (-28.0, 0.0)]   # M5 into T-nuts: front, rear, centre rail
HIP = (-24.0, 8.0, 43.0, 90.0, 560.0, 575.0)   # hip actuator tops reach z 573 here (mesh); relief under the base


def box(x0, x1, y0, y1, z0, z1):
    return bp.box(x0, x1, y0, y1, z0, z1)


def zcyl(x, y, d, z0, z1):
    return tt.cyl_along("z", (x, y, 0), d, z0, z1)


def xcyl(y, z, d, x0, x1):
    return tt.cyl_along("x", (0, y, z), d, x0, x1)


def ycyl(x, z, d, y0, y1):
    return tt.cyl_along("y", (x, 0, z), d, y0, y1)


def xhex(y, z, af, x0, x1):
    """Hex nut pocket along x from x0 to x1."""
    return Pos(x0, y, z) * (Rot(0, 90, 0) * extrude(RegularPolygon(af / 2, 6, major_radius=False), x1 - x0))


def zhex(x, y, af, z0, z1):
    return Pos(x, y, z0) * extrude(RegularPolygon(af / 2, 6, major_radius=False), z1 - z0)


def holder_geo(v=PACK):
    L, W, H, z0, sy = PACKS[v]
    xo = H_DIV / 2 + 2 * H_CLR + W            # inner face of the outer walls
    by = sy + STRAP_W / 2 + 3                  # half width of the base
    return L, W, H, z0, sy, xo, by


def holder(v=PACK):
    L, W, H, z0, sy, xo, by = holder_geo(v)
    h = box(-H_X, H_X, -by, by, RAIL_TOP, z0)
    for s in (-1, 1):
        h += box(*sorted((s * xo, s * (xo + H_WALL))), -by, by, z0 - 1, z0 + H_WALL_H)        # outer walls
    h += box(-H_DIV / 2, H_DIV / 2, -by, by, z0 - 1, z0 + H_WALL_H)                       # divider
    for s in (-1, 1):                          # strap: tunnel under both packs, exits beside the outer walls
        y0, y1 = s * sy - STRAP_W / 2 - 1, s * sy + STRAP_W / 2 + 1
        h -= box(-xo - H_WALL - 3, xo + H_WALL + 3, y0, y1, RAIL_TOP + 3, RAIL_TOP + 6)
        for sx in (-1, 1):
            h -= box(*sorted((sx * (xo + H_WALL), sx * (xo + H_WALL + 3))), y0, y1, RAIL_TOP + 3, z0 + 1)
    for x, y in H_SCREWS:                      # M5 x 12 into drop-in T-nuts, heads under the pack bottom
        h -= zcyl(x, y, M5_CLR, RAIL_TOP - 1, z0 + 1) + zcyl(x, y, M5_CB_D, RAIL_TOP + 5, z0 + 1)
    for s in (-1, 1):
        x0, x1, y0, y1, a, b = HIP
        h -= box(x0, x1, *sorted((s * y0, s * y1)), a, b)
    return h


def packs(v=PACK):
    L, W, H, z0, sy, xo, by = holder_geo(v)
    c = H_DIV / 2 + H_CLR
    return {"lipo_6s_a": (-c - W, -c, -L / 2, L / 2, z0, z0 + H), "lipo_6s_b": (c, c + W, -L / 2, L / 2, z0, z0 + H)}


def strap_points(v=PACK):
    """Centre line of each strap (under the base, up beside the walls, over both packs), 1 mm spaced, y across."""
    L, W, H, z0, sy, xo, by = holder_geo(v)
    xs = xo - H_CLR                            # pack outer face
    path = [(-xo - H_WALL - 1.5, RAIL_TOP + 4.5), (-xo - H_WALL - 0.75, z0 + H_WALL_H), (-xs - 0.75, z0 + H + 0.75),
            (xs + 0.75, z0 + H + 0.75), (xo + H_WALL + 0.75, z0 + H_WALL_H), (xo + H_WALL + 1.5, RAIL_TOP + 4.5)]
    pts = []
    for (a, b), (c, d) in zip(path, path[1:]):
        n = int(math.hypot(c - a, d - b)) + 1
        pts += [(a + (c - a) * t, b + (d - b) * t) for t in np.linspace(0, 1, n)]
    out = []
    for s in (-1, 1):
        for y in np.linspace(s * sy - STRAP_W / 2, s * sy + STRAP_W / 2, 11):
            out += [(x, y, z) for x, z in pts]
    return np.array(out)


# ---------------- 2. rear pod ----------------
POD_X = -75.0                                  # front face of the 2 mm aluminium plate (rear bump ends at x = -74)
AL_T = 2.0
PX = POD_X - AL_T                              # rear face of the plate, -77
PLATE = (75.0, 588.0, 786.0)                   # half width, bottom z, top z (2.5 mm under the deck)
NOTCH = (17.0, 717.0)                          # open to the top: R1F fill neck (z 735, OD 26) and its extension
POD_SCREWS = [(s * 65.0, z) for s in (-1, 1) for z in (640.0, 770.0)]   # M5 x 12 button head, plate + spacer, T-nut
SPACER = (57.0, 73.0, 630.0, 780.0)            # spacer strips on the rear posts: |y| range, z range (x -75..-70)
# 3S basket: pack on its side (46 high, 30 deep), hung on 4 x M3 above the rear bump
B_BACK, B_LIP, B_FLOOR = 4.0, 4.0, 5.0         # back wall, lip and floor thickness
B_Y, B_Z0 = 72.5, 586.0                        # half width, floor underside
B_BAND = (638.0, 646.0, 6.5)                   # top band of the back wall: z range, thickness (nut pockets)
B_SCREWS = [(y, 642.0) for y in (-52.0, -10.0, 10.0, 52.0)]
B_STRAP_Y = 35.0
L3, H3, D3 = 138.0, 46.0, 30.0                 # 3S pack: length (y), height on its side (z), depth (x)
# electronics on the plate (robot frame boxes); hole patterns
DIODE_Z = 648.0                                # base bottom; base 92.7 x 20.3, 2 holes 80 apart, M6
ROCK_Y, ROCK_Z, ROCK_SO = (-73.0, -17.0), (700.0, 785.0), 10.0   # board y and z range (vertical, ports down), standoffs
SHUNT_Y, SHUNT_Z = (26.0, 66.0), (676.0, 781.0)                  # vertical on the plate, base against the plate
GROMMET = (0.0, 683.0, 20.0)
# boards stuck on the lip of the basket with foam tape (hole patterns not known): (name, y0, y1, z0, z1, height)
LIP_BOARDS = [("canable_1", -69.0, -17.0, 596.0, 612.0, 12.0), ("canable_2", -69.0, -17.0, 616.0, 632.0, 12.0),
              ("buck_5v", -12.0, 13.4, 600.0, 625.4, 10.0),          # Pololu D36V50F5
              ("mosfet_dfr0457", 18.0, 62.0, 598.0, 630.0, 15.0)]    # switch of the DWP-385 pump, 44 x 32 mm
# splash cover: outer box, open at the front and the bottom; hooks over the plate top edge, 2 screws at the bottom
COVER = (-136.0, -75.5, 77.5, 583.0, 788.5)    # x0, x1, |y|, z0, z1
CW = 2.0
METERS = [("meter_hav6001", -44.0, 739.5), ("meter_hav1808", -44.0, 699.5), ("meter_hav1625a", 44.0, 739.5)]
METER_CUT = (45.5, 26.5)                       # panel cut-out, 0.56 inch meters (listing)
SWITCHES = [("switch_pushlock_24v", 0.0, 703.0, 22.3), ("switch_pushlock_12v", 0.0, 768.0, 16.2)]
EXT_OD, EXT_ORING = 24.0, (15.0, 2.0)          # fill extension OD; O-ring ID x cross section in the neck bore

# ---------------- 2b. pump, brake resistors, relay K2 ----------------
PUMP = "dwp385"         # "dwp385": DWP-385 gear pump in its printed holder (F2: on the shelf. F1: on the right side
                        # face, and the R1F pocket holds the inline filter). "m400k": F1 only, TCS M400K-V in the R1F
                        # pocket. The team uses the Kamoer KLP02 24 V pump (size not published): its holder follows.
# DWP-385 (water.md: about 87 mm long, 32 mm motor, 2 ears on the pump head). Other sizes ASSUMED.
DWP_MOTOR = (57.0, 32.0)                       # motor length, diameter
DWP_HEAD = (30.0, 34.0, 48.0)                  # head length, diameter, ear span (ears in the vertical plane)
DWP_AXIS = (-45.0, -97.5, 670.0)               # motor rear end x, axis y and z; head and barbs point forward (+x)
PH_PLATE = (-72.0, 72.0, 646.0, 694.0, 4.0)    # holder plate on the right posts (y -75 face): x, z ranges, thickness
PH_SADDLES = ((-42.0, -34.0), (2.0, 10.0))     # saddle x ranges under the motor
PH_STRAP_X = (-28.0, -8.0)                     # 20 mm strap around the motor, through the plate
# F2: the pump lies on the rear of the middle shelf, along y, head and barbs to the left (+y). Its holder base takes
# the 2 rear M3 screws of the shelf (bracket holes x -56, y +-9.8). The strap runs around the motor and through a
# tunnel under the base.
PUMP2 = (-48.0, -50.0, 703.0)                  # axis x, motor rear end y, axis z. The barbs (z 692, 704) pass over a
                                               # part of the torso mesh at x -69..-51, y 46..52, z 653..689 (rear left)
PH2_BASE = (-66.0, -30.0, -52.0, 14.0, 5.0)    # base x, y ranges, thickness (on the shelf top z 646)
PH2_SADDLES = ((-47.0, -39.0), (-3.0, 5.0))    # saddle y ranges under the motor
PH2_STRAP_Y = (-33.0, -13.0)                   # 20 mm strap tunnel under the base
FILTER2 = (-26.0, 6.0, 24.0, 50.0, 647.0, 672.0)   # inline filter on the shelf, zip-tied (ASSUMED size 32 x 26 x 25)
# brake resistors RARA RH50C4R70J: 71 x 30 x 16 mm with the solder tags (electrical.md), 31 g. Hole pattern ASSUMED
# 39.7 x 21.4 mm, M3 (the RH-50 / HS50 industry pattern). On the front face of the front bottom rail (slot z 561):
# airflow, near the hip-roll drives. On the rear rail they come 7.7 mm from a leg (knee <= 1.4 rad), on the front 41 mm.
RES = (71.0, 30.0, 16.0, 31)
RES_FACE = 70.0                                # rail face x: +70 front bottom rail, -70 rear (fails the leg check)
RES_Y = (-39.0, 39.0)                          # 7 mm between the solder tags of the two resistors
RES_HOLES = (39.7, 21.4)
RM_T = 6.0                                     # resistor mount plate thickness
K2 = (47.5, 75.0, 646.0, 674.0, 35.0)          # relay K2 KKA-K4 with its spade connectors on the pod plate: y, z, depth

# ---------------- 4. camera and nozzle head ----------------
HEAD_DZ = 40.0          # lens axis above the nozzle axis (35 mm or more keeps the nozzle out of the image)
NOZZLE_D, NOZZLE_L = 10.0, 30.0   # nozzle: MK8 1.0 mm brass nozzle on an M6 female x 3 mm barb adapter. The bore
                                  # (10.3) takes a body up to 10 mm round or 8 mm hex; an M3 set screw clamps it. ASSUMED
HOSE_OD = 6.0           # silicone tube 3 x 6 mm
CAM_B = 38.0            # B0201 board
CAM_PATTERNS = (34.0, 28.0)
LENS_WIN = 18.0         # square window for the M12 lens holder, PLACEHOLDER (holder size not published)
VFOV = 2 * math.degrees(math.atan(math.tan(math.radians(60)) * 9 / math.hypot(16, 9)))   # 120 deg diagonal, 16:9

# ---------------- 5. IMU ----------------
IMU = (59.5, 49.5, 11.7)          # IM10A case with flanges: y, z, x (upright, front face forward)
IMU_X = 53.0                      # back face of the case


def plate_outline():
    """Pod plate outline in (y, z), robot frame, counter-clockwise, with the neck notch."""
    yy, z0, z1 = PLATE
    if F2:                                     # no fill neck behind the pod in F2
        return [(-yy, z0), (yy, z0), (yy, z1), (-yy, z1)]
    ny, nz = NOTCH
    return [(-yy, z0), (yy, z0), (yy, z1), (ny, z1), (ny, nz), (-ny, nz), (-ny, z1), (-yy, z1)]


def diode_box():
    return (PX - P["diode_mbrt400100"]["size_mm"][2], PX, -46.35, 46.35, DIODE_Z, DIODE_Z + 20.3)


def plate_holes():
    """(y, z, diameter, label) of every hole in the pod plate."""
    h = [(y, z, M5_CLR, "M5 spacer") for y, z in POD_SCREWS]
    zd = DIODE_Z + 20.3 / 2
    h += [(s * 40.0, zd, 6.5, "M6 diode") for s in (-1, 1)]
    ya, zb = ROCK_Y[0], ROCK_Z[1]
    h += [(y, z, M25_CLR, "M2.5 ROCK 3C") for y in (ya + 3.5, ya + 3.5 + 49) for z in (zb - 3.5, zb - 3.5 - 58)]
    h += [(y, z, M3_CLR, "M3 basket") for y, z in B_SCREWS]
    ys = sum(SHUNT_Y) / 2
    h += [(ys, SHUNT_Z[0] + 7, 5.5, "M5 shunt*"), (ys, SHUNT_Z[1] - 7, 5.5, "M5 shunt*")]
    h += [(GROMMET[0], GROMMET[1], GROMMET[2], "grommet")]
    h += [(y, (K2[2] + K2[3]) / 2, 4.5, "zip tie K2") for y in (K2[0] + 1.5, K2[1] - 2.5)]
    return h


def pod_plate():
    p = extrude(_poly(plate_outline()), AL_T)                 # sketch in XY, extruded along z
    p = Pos(PX, 0, 0) * (Rot(0, 90, 0) * Rot(0, 0, 90) * p)  # (y, z) sketch -> plate in the y-z plane, x PX..POD_X
    for y, z, d, _ in plate_holes():
        p -= xcyl(y, z, d, PX - 1, POD_X + 1)
    return p


def _poly(pts):
    from build123d import Polyline, make_face
    return make_face(Polyline(*pts, close=True))


def pod_spacer(s=1):
    y0, y1, z0, z1 = SPACER
    p = box(POD_X, POD_X + 5, *sorted((s * y0, s * y1)), z0, z1)
    for y, z in POD_SCREWS:
        if y * s > 0:
            p -= xcyl(y, z, M5_CLR, POD_X - 1, POD_X + 6)
    for y, z, d, label in plate_holes():          # room for the nuts of the plate hardware in front: notches
        if y0 - 4 < s * y < y1 + 4 and z0 < z < z1 and "spacer" not in label:
            p -= box(POD_X - 1, POD_X + 6, *sorted((y - s * 3.5, s * (y1 + 1))), z - 3.5, z + 3.5)
    return p


def basket_geo():
    xb = PX - B_BACK                              # basket back wall inner face
    xp0, xp1 = xb - 0.5 - D3, xb - 0.5             # 3S pack x range
    xl = xp0 - 0.5                                # lip inner face
    zf = B_Z0 + B_FLOOR
    return xb, xp0, xp1, xl, zf


def lipo3s_box():
    xb, xp0, xp1, xl, zf = basket_geo()
    return (xp0, xp1, -L3 / 2, L3 / 2, zf, zf + H3)


def pod_basket():
    xb, xp0, xp1, xl, zf = basket_geo()
    zt = zf + H3                                  # pack top
    zb0, zb1, bt = B_BAND
    xr = COVER[0] + CW + 1                        # floor ledge reaches the cover (rear wall inner face - 1)
    b = box(xb, PX, -B_Y, B_Y, B_Z0, zb0)                                   # back wall
    b += box(PX - bt, PX, -B_Y, B_Y, zb0 - 0.01, zb1)                        # top band with the nut pockets
    b += box(xr, PX, -B_Y, B_Y, B_Z0, zf)                                    # floor and the ledge under the cover
    b += box(xl - B_LIP, xl, -B_Y, B_Y, B_Z0, zt)                            # lip; the lip boards sit on its rear
    for s in (-1, 1):
        b += box(xl, xb, *sorted((s * (L3 / 2 + 0.5), s * B_Y)), B_Z0, zf + 20)   # end stops
    for y, z in B_SCREWS:                          # M3 from the plate front, nuts in pockets on the band rear face
        b -= xcyl(y, z, M3_CLR, PX - bt - 1, PX + 1) + xhex(y, z, M3_NUT_AF, PX - bt - 1, PX - bt + M3_NUT_T)
    for y, z in POD_SCREWS:                        # room for the M5 button heads of the plate
        if z < zb1 + 6:
            b -= box(PX - bt - 1, PX + 1, y - 5.5, y + 5.5, z - 5.5, zb1 + 1)
    for s in (-1, 1):                              # strap channels: back wall and lip inner faces, floor slots
        y0, y1 = s * B_STRAP_Y - STRAP_W / 2 - 1, s * B_STRAP_Y + STRAP_W / 2 + 1
        b -= box(xb - 0.1, xb + 2.0, *sorted((y0, y1)), B_Z0 - 1, zb1 + 1)
        b -= box(xl - 2.0, xl + 0.1, *sorted((y0, y1)), B_Z0 - 1, zt + 1)
    for s in (-1, 1):                              # cover screws: M3 from below, nuts in pockets on the ledge top
        b -= zcyl(-125.0, s * 66.0, M3_CLR, B_Z0 - 1, zf + 1) + zhex(-125.0, s * 66.0, M3_NUT_AF, zf - M3_NUT_T, zf + 1)
    for name, y0, y1, z0, z1, hgt in LIP_BOARDS:  # 2 mm rails around each board on the lip rear face
        xr0 = xl - B_LIP
        frame = box(xr0 - 3, xr0, y0 - FIT - 2, y1 + FIT + 2, z0 - FIT - 2, z1 + FIT + 2)
        b += frame - box(xr0 - 4, xr0 + 0.1, y0 - FIT, y1 + FIT, z0 - FIT, z1 + FIT)
    return b


def lip_board_boxes():
    xb, xp0, xp1, xl, zf = basket_geo()
    x = xl - B_LIP
    return {n: (x - 1 - h, x - 1, y0, y1, z0, z1) for n, y0, y1, z0, z1, h in LIP_BOARDS}   # 1 mm foam tape


def pod_cover():
    x0, x1, yy, z0, z1 = COVER
    c = box(x0, x1, -yy, yy, z0, z1) - box(x0 + CW, x1 + 1, -yy + CW, yy - CW, z0 - 1, z1 - CW)
    c -= box(PX, x1 + 1, -yy + CW + 0.01, yy - CW - 0.01, z0 - 1, z1 + 1)   # nothing in front of the plate rear face
    for s in (-1, 1):                              # side walls cover the plate edges up to x = -75.5
        c += box(PX, x1, *sorted((s * (yy - CW), s * yy)), z0, z1)
    for y in (-40.0, 40.0):                        # hooks over the plate top edge (top wall rests on the edge)
        c += box(PX - 1, POD_X + 2.0, y - 10, y + 10, z1 - CW, z1) + box(POD_X, POD_X + 2.0, y - 10, y + 10, PLATE[2] - 8, z1)
    for _, y, z in METERS:
        w, h = METER_CUT[0] + FIT, METER_CUT[1] + FIT
        c -= box(x0 - 1, x0 + CW + 1, y - w / 2, y + w / 2, z - h / 2, z + h / 2)
    for _, y, z, d in SWITCHES:
        c -= xcyl(y, z, d, x0 - 1, x0 + CW + 1)
    if not F2:
        c -= xcyl(0, tt.R1_NECK[0], EXT_OD + 0.6, x0 - 1, x0 + CW + 1)   # F1 fill extension
    for s in (-1, 1):
        for i in range(5):                         # vents high on the side walls
            x = -128.0 + i * 10
            c -= box(x, x + 5, *sorted((s * (yy - CW - 1), s * (yy + 1))), 752.0, 780.0)
        c -= box(-110.0, -86.0, *sorted((s * (yy - CW - 1), s * (yy + 1))), z0 - 1, z0 + 12)   # cable exits
        tab = box(x0, -116.0, *sorted((s * 62.0, s * (yy - 0.01))), z0, z0 + 3)               # screw tabs
        c += tab - zcyl(-125.0, s * 66.0, M3_CLR, z0 - 1, z0 + 4)
    return c


def fill_extension():
    """Tube from the R1F neck to 2 mm past the cover. Its front end plugs 15 mm into the neck bore (19 mm), sealed by
    an O-ring; its shoulder rests on the neck end (x -78). The silicone stopper of R1 goes into the 19 mm rear bore."""
    z, od, idia, xe = tt.R1_NECK
    xr = COVER[0] - 2
    o_id, o_cs = EXT_ORING
    plug = idia - 0.4                             # 18.6 slip fit in the neck bore
    e = xcyl(0, z, plug, xe - 0.01, xe + 15) + xcyl(0, z, EXT_OD, xr, xe)
    e -= xcyl(0, z, plug + 1, xe + 6, xe + 6 + o_cs + 0.2) - xcyl(0, z, o_id + 0.4, xe + 5, xe + 9 + o_cs)   # groove
    e -= xcyl(0, z, 11.0, xe - 1, xe + 16)                                    # fill bore through the plug
    e -= xcyl(0, z, idia, xr - 1, xr + 18)                                    # stopper bore at the rear
    e -= xcyl(0, z, 11.0, xr + 17, xe + 1)
    return e


def dwp385_ref():
    """DWP-385 gear pump: motor, head with ears, two barbs on the front face (ASSUMED shape). F1: along +x on the
    right side face. F2: along +y on the shelf (the F1 shape turned 90 deg about z)."""
    if F2:
        x, y0, z = PUMP2
        return Pos(x, 0, 0) * (Rot(0, 0, 90) * _dwp385(y0, 0.0, z))
    return _dwp385(*DWP_AXIS)


def _dwp385(x0, y, z):
    ml, md = DWP_MOTOR
    hl, hd, ear = DWP_HEAD
    p = xcyl(y, z, md, x0, x0 + ml) + xcyl(y, z, hd, x0 + ml, x0 + ml + hl)
    p += box(x0 + ml, x0 + ml + 4, y - 5, y + 5, z - ear / 2, z + ear / 2)                      # ears
    for dz in (-6.0, 6.0):
        p += xcyl(y, z + dz, 6.2, x0 + ml + hl, x0 + ml + hl + 12)                              # barbs
    return p


def pump_holder_f2():
    """F2: base plate on the middle shelf (2 x M3 through the rear bracket holes), two saddles under the motor, a strap
    tunnel under the base. The 20 mm strap goes around the motor and through the tunnel."""
    xa, xb, ya, yb, t = PH2_BASE
    x, y0, z = PUMP2
    z0 = tank.SHELF_TOP
    h = box(xa, xb, ya, yb, z0, z0 + t)
    r = DWP_MOTOR[1] / 2 + 0.3
    for s0, s1 in PH2_SADDLES:
        h += box(x - r - 2.5, x + r + 2.5, s0, s1, z0 + t - 0.01, z - r + 6) - ycyl(x, z, 2 * r, s0 - 1, s1 + 1)
    h -= box(xa - 1, xb + 1, *PH2_STRAP_Y, z0 - 1, z0 + 2.5)                  # strap tunnel, 2.5 mm roof
    for bx, by in BRACKET_HOLES:
        if bx < 0:
            h -= zcyl(bx, by, M3_CLR, z0 - 1, z0 + t + 1)
    return h


def pump_holder():
    """Plate on the right side faces of the right posts (2 x M5 into T-nuts), two saddles under the motor, a slot
    pair for a 20 mm hook-and-loop strap around the motor. The strap passes behind the plate, in the side opening."""
    xa, xb, za, zb, t = PH_PLATE
    x0, yp, zp = DWP_AXIS
    yf = -75.0                                     # outer side face of the right posts
    h = box(xa, xb, yf - t, yf, za, zb)
    for x in (-60.0, 60.0):
        h -= ycyl(x, zp, M5_CLR, yf - t - 1, yf + 1)
    r = DWP_MOTOR[1] / 2 + 0.3
    for s0, s1 in PH_SADDLES:
        h += box(s0, s1, yp, yf - t + 0.01, zp - r - 3, zp + r + 3) - xcyl(yp, zp, 2 * r, s0 - 1, s1 + 1)
    for sz in (-1, 1):
        zc = zp + sz * (r + 3.5)
        h -= box(PH_STRAP_X[0] - 1, PH_STRAP_X[1] + 1, yf - t - 1, yf + 1, zc - 1.75, zc + 1.75)
    return h


if F2:
    pump_holder = pump_holder_f2  # noqa: F811


def res_boxes():
    """Brake resistor envelopes, robot frame."""
    L, W, H, _ = RES
    s = -1 if RES_FACE < 0 else 1
    xa = RES_FACE + s * RM_T
    return {f"brake_resistor_{i + 1}": (*sorted((xa, xa + s * H)), yc - L / 2, yc + L / 2, 561.0 - W / 2, 561.0 + W / 2)
            for i, yc in enumerate(RES_Y)}


def resistor_mount(i=0):
    """6 mm plate on the bottom rail face: 2 x M5 into T-nuts (heads sunk), 4 x M3 heat-set inserts for the resistor."""
    s = -1 if RES_FACE < 0 else 1
    yc = RES_Y[i]
    xa, xb = sorted((RES_FACE, RES_FACE + s * RM_T))
    m = box(xa, xb, yc - 36.5, yc + 36.5, 545.0, 578.0)
    for dy in (-25.0, 25.0):
        m -= xcyl(yc + dy, 561.0, M5_CLR, xa - 1, xb + 1)
        m -= xcyl(yc + dy, 561.0, M5_CB_D, *sorted((RES_FACE + s * (RM_T - 3.0), RES_FACE + s * (RM_T + 1))))
    for dy in (-RES_HOLES[0] / 2, RES_HOLES[0] / 2):
        for dz in (-RES_HOLES[1] / 2, RES_HOLES[1] / 2):
            m -= xcyl(yc + dy, 561.0 + dz, 4.0, xa - 1, xb + 1)      # through: no thin floor behind the insert
    return m


# ---------------- 4. head ----------------
def head_geo():
    x0 = tm.PAN_X
    zb = T + st.Z_U1                               # top of the tilt U (payload pattern)
    zn = zb + 5 + NOZZLE_D / 2 + 3.5               # nozzle axis
    zc = zn + HEAD_DZ                              # lens axis before the pitch
    xf = x0 + 32.0                                 # camera front face
    return x0, zb, zn, zc, xf


def _pitch(shape, xf, zp):
    return Pos(xf, 0, zp) * Rot(0, tm.CAM_PITCH, 0) * Pos(-xf, 0, -zp) * shape


def _diag(x0, x1, sy, sz, zc, a, b, w):
    """Stadium of width w along x0..x1 from (sy*a, zc+sz*a) to (sy*b, zc+sz*b) in the y-z plane."""
    p0, p1 = (sy * a, zc + sz * a), (sy * b, zc + sz * b)
    n = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    ang = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    mid = ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)
    bar = Pos((x0 + x1) / 2, *mid) * Rot(ang, 0, 0) * Pos(-(x0 + x1) / 2, -mid[0], -mid[1]) * \
        box(x0, x1, mid[0] - n / 2, mid[0] + n / 2, mid[1] - w / 2, mid[1] + w / 2)
    return bar + xcyl(p0[0], p0[1], w, x0, x1) + xcyl(p1[0], p1[1], w, x0, x1)


def head_f1():
    x0, zb, zn, zc, xf = head_geo()
    w, t, d = CAM_B / 2 + 5, 2.5, 26.0             # camera box half size, wall, depth
    h = box(x0 - 28, xf, -w - 3, w + 3, zb, zb + 5)                                  # base
    for x, y in tm.square(tm.PAYLOAD_PATTERN):
        h -= zcyl(x0 + x, y, M4_CLR, zb - 1, zb + 6) + zcyl(x0 + x, y, 9.0, zb + 2.5, zb + 6)
    r = NOZZLE_D / 2 + 3.5
    h += box(x0 + 6, x0 + 30, -r, r, zb + 4.9, zn) + xcyl(0, zn, 2 * r, x0 + 6, x0 + 30)   # nozzle holder
    zp = zc - w                                     # camera box bottom (pivot of the pitch at the front edge)
    wedge = d * math.tan(math.radians(tm.CAM_PITCH)) + 1
    h += box(x0 + 8, xf - 2, -10, 10, zn, zp + 1)                                    # column
    cam = box(xf - d, xf, -w, w, zp - wedge, zp + t)                                 # bottom
    cam += box(xf - t, xf, -w, w, zp - wedge, zp + 2 * w)                            # front wall
    cam += box(xf - d, xf, -w, w, zp + 2 * w - t, zp + 2 * w)                        # roof
    for s in (-1, 1):
        cam += box(xf - d, xf, *sorted((s * (w - t), s * w)), zp - wedge, zp + 2 * w)
    a, b = (p / 2 for p in sorted(CAM_PATTERNS))
    for sy in (-1, 1):                               # one pad per corner on the back of the front wall, with a
        for sz in (-1, 1):                           # diagonal M2 slot that takes the 28 mm and the 34 mm pattern
            cam += _diag(xf - t - 2.5, xf - t + 0.1, sy, sz, zc, a, b, 6.5)
            cam -= _diag(xf - t - 3, xf + 1, sy, sz, zc, a, b, M2_CLR)
    cam -= box(xf - t - 1, xf + 1, -LENS_WIN / 2, LENS_WIN / 2, zc - LENS_WIN / 2, zc + LENS_WIN / 2)
    h += _pitch(cam, xf, zp)
    h -= xcyl(0, zn, NOZZLE_D + 0.3, x0 + 5, x0 + 31)                                # nozzle bore
    h -= zcyl(x0 + 18, 0, 2.5, zn, zn + r + 1)                                       # M3 set screw (thread-forming)
    hr = HOSE_OD / 2 + 0.2 + 2.1                                                     # hose clip at the rear
    h += box(x0 - 24, x0 - 16, -3, 3, zb + 4.9, zn) + xcyl(0, zn, 2 * hr, x0 - 24, x0 - 16)
    h -= xcyl(0, zn, HOSE_OD + 0.4, x0 - 25, x0 - 15) + box(x0 - 25, x0 - 15, -HOSE_OD * 0.35, HOSE_OD * 0.35, zn, zn + hr + 1)
    return h & box(-200, 200, -100, 100, zb, zb + 200)


def head_refs():
    """Camera board with lens and the nozzle in the head pose (tilt = 0), for checks and renders."""
    x0, zb, zn, zc, xf = head_geo()
    w = CAM_B / 2 + 5
    zp = zc - w
    t = 2.5
    board = box(xf - t - 2.5 - 1.6, xf - t - 2.5, -CAM_B / 2, CAM_B / 2, zc - CAM_B / 2, zc + CAM_B / 2)
    lens = xcyl(0, zc, 14.0, xf - t - 2.5, xf + 8)                 # M12 lens and holder, ESTIMATED
    cam = _pitch(board + lens, xf, zp)
    nozzle = xcyl(0, zn, NOZZLE_D, x0 + 4, x0 + 4 + NOZZLE_L) + xcyl(0, zn, 3.8, x0 - 8, x0 + 4.1)
    lens_tip = (xf + 8, zc)                                          # before the pitch
    a = math.radians(tm.CAM_PITCH)
    dx, dz = lens_tip[0] - xf, lens_tip[1] - zp
    lens = (xf + dx * math.cos(a) + dz * math.sin(a), 0.0, zp - dx * math.sin(a) + dz * math.cos(a))
    return {"camera_b0201": cam, "nozzle_brass": nozzle}, lens, (x0 + 4 + NOZZLE_L, 0.0, zn)


# ---------------- 5. IMU mount ----------------
def imu_mount():
    sy, sz, sx = IMU
    y = sy / 2 + FIT
    z0 = SHELF_Z + SHELF_T
    zi = z0 + 4                                    # IMU bottom on the ledge
    xi = IMU_X + sx + FIT                          # IMU front face + fit
    m = box(IMU_X - 3, 68.0, -y - 3, y + 3, z0, zi)                       # foot and ledge
    m += box(IMU_X - 3, IMU_X, -y - 3, y + 3, z0, zi + sz)                # back plate
    for s in (-1, 1):
        m += box(IMU_X, xi + 2.5, *sorted((s * y, s * (y + 3))), z0, zi + 30)        # side walls
        m += box(xi, xi + 2.5, *sorted((s * (y - 4), s * (y + 3))), z0, zi + 30)     # front lips over the flanges
        m -= box(IMU_X - 4, IMU_X + 1, *sorted((s * (y - 1), s * (y + 1.6))), zi + 36, zi + 40)  # zip-tie slot
    for x, yy in BRACKET_HOLES:
        if x > 0:
            m -= zcyl(x, yy, M3_CLR, z0 - 1, zi + 1)
    return m


def imu_box():
    sy, sz, sx = IMU
    zi = SHELF_Z + SHELF_T + 4
    return (IMU_X + FIT / 2, IMU_X + FIT / 2 + sx, -sy / 2, sy / 2, zi, zi + sz)


# ---------------- 6. clips ----------------
HOSE_CLIP_WALL = 2.1


def hose_clip():
    """C-clip for the jet hose with 2 zip-tie slots in its foot. Drawn on the deck in front of the cable slot."""
    x, y, z = 55.0, -25.0, T + tm.DECK_T
    hr = HOSE_OD / 2 + 0.2 + HOSE_CLIP_WALL
    zc = z + 2.0 + hr                                # ring sinks 0.5 mm into the foot (no tangent contact)
    c = box(x - 12, x + 12, y - 6, y + 6, z, z + 2.5) + box(x - 4, x + 4, y - 3, y + 3, z, zc) + ycyl(x, zc, 2 * hr, y - 4, y + 4)
    c -= ycyl(x, zc, HOSE_OD + 0.4, y - 5, y + 5) + box(x - HOSE_OD * 0.35, x + HOSE_OD * 0.35, y - 5, y + 5, zc, zc + hr + 1)
    for sx in (-1, 1):
        c -= box(*sorted((sx * 8.0 + x, sx * 10.0 + x)), y - 3.5, y + 3.5, z - 1, z + 3.5)
    return c


def frame_clip():
    """Cable clip for a bundle up to 12 mm: M5 x 8 into a drop-in T-nut on any 2020 face. Drawn on the right face
    (y = -75) of the rear right post, cables running up and down the post."""
    x, z, yf, r = -60.0, 735.0, -75.0, 6.25        # slot centre on the post face, bundle radius
    c = box(x - 9.5, x + 9.5, yf - 3, yf, z - 6, z + 20)                       # foot on the post face
    c -= ycyl(x, z, M5_CLR, yf - 4, yf + 1)
    yc = yf - 3 - r
    c += zcyl(x, yc, 2 * (r + 2.5), z + 8, z + 20) - zcyl(x, yc, 2 * r, z + 7, z + 21)   # ring above the screw head
    c -= box(x + r - 1.5, x + r + 3.5, yc - 3.75, yc + 3.75, z + 7, z + 21)               # snap-in opening
    return c


# ---------------- print settings ----------------
# name: (qty, infill, orientation, supports). Walls: 4 perimeters (1.6 mm), 5 top and bottom layers.
PRINTS = {
    "r1f_half_left": (1, 1.0, "on the cut face (y = 0)", "only under the 27 mm bridge of the pan notch if it sags"),
    "r1f_half_right": (1, 1.0, "on the cut face (y = 0)", "only under the 27 mm bridge of the pan notch if it sags"),
    "deck_f1": (1, 0.4, "flat, nut pockets up", "none"),
    "pan_base": (1, 0.4, "shelf down", "tree supports under the servo foot"),
    "pan_yoke": (1, 0.4, "disc down", "none"),
    "tilt_u": (1, 0.4, "U base down", "none"),
    "head_f1": (1, 0.3, "base down", "none (the window and the roof are bridges)"),
    "holder_6s_zeee": (1, 0.3, "base down", "none (the strap tunnels are 22 mm bridges)"),
    "holder_6s_vega": (0, 0.3, "base down", "none. Print instead of the Zeee holder if the TA buys VEGA packs"),
    "pod_spacer_left": (1, 1.0, "flat", "none"),
    "pod_spacer_right": (1, 1.0, "flat", "none"),
    "pod_basket": (1, 0.4, "back wall down (x = -77 face on the bed)", "none"),
    "pod_cover": (1, 0.3, "upside down, top wall on the bed", "under the 2 bottom screw tabs"),
    "fill_extension": (1, 1.0, "standing, rear end down", "none"),
    "imu_mount": (1, 0.4, "foot down", "none"),
    "hose_clip": (4, 1.0, "foot down", "none"),
    "frame_clip": (8, 1.0, "foot down (flat face on the bed)", "none"),
    "pump_holder": (1, 0.4, "base down" if F2 else "plate down (outer face on the bed)",
                    "none. It fits the DWP-385 only: wait for the KLP02 holder" if F2 else "none"),
    "tank_left_tub": (1, 1.0, "outer face (y = 124) on the bed", "none"),
    "tank_right_tub": (1, 1.0, "outer face (y = -124) on the bed", "none"),
    "tank_left_plate": (1, 1.0, "glue face (y = 80.5) on the bed, pads up", "none"),
    "tank_right_plate": (1, 1.0, "glue face (y = -80.5) on the bed, pads up", "none"),
    "resistor_mount_left": (1, 0.4, "flat, insert holes up", "none"),
    "resistor_mount_right": (1, 0.4, "flat, insert holes up", "none"),
}
SHELL = 1.6
FLOW = 5.0                         # mm3/s average extrusion rate, PETG 0.4 mm nozzle, 0.2 mm layers (ASSUMED)


def print_estimate(s, infill):
    v, a = s.volume, s.area
    shell = min(v, a * SHELL)
    pv = shell + infill * (v - shell)
    return pv, pv / 1000 * RHO, pv / FLOW * 1.2 / 60 + 10       # printed mm3, g, minutes (+20 % moves, +10 min)


def bought_boxes():
    """Bought parts of the kit area as boxes (robot frame), with masses from parts.json."""
    b = {}
    for n, bx in packs().items():
        b[n] = (bx, P["lipo_6s_zeee"]["mass_g"] if PACK == "zeee" else P["lipo_6s_vega_alt"]["mass_g"])
    b["lipo_3s"] = (lipo3s_box(), P["lipo_3s_dinogy"]["mass_g"])
    b["diode_mbrt400100"] = (diode_box(), P["diode_mbrt400100"]["mass_g"])
    (ya, yb), (za, zb) = ROCK_Y, ROCK_Z
    b["rock3c"] = ((PX - ROCK_SO - 1.6 - 16.0, PX - ROCK_SO + 4.0, ya, yb, za - 2.3, zb), P["rock3c"]["mass_g"])
    b["rock3c_plugs"] = ((PX - ROCK_SO - 20.0, PX - ROCK_SO - 1.6, ya + 2, yb - 2, za - 28.0, za - 2.3), 0)   # keep free
    b["shunt_100a"] = ((PX - 25.0, PX, *SHUNT_Y, *SHUNT_Z), P["shunt_100a"]["mass_g"])
    for n, bx in lip_board_boxes().items():
        b[n] = (bx, P[{"canable_1": "canable_v2_pro", "canable_2": "canable_v2_pro", "buck_5v": "buck_12v_to_5v",
                       "mosfet_dfr0457": "mosfet_dfr0457"}[n]]["mass_g"])
    xin = COVER[0] + CW
    for n, y, z in METERS:
        s = P[n]["size_mm"]
        b[n] = ((xin, xin + s[2], y - METER_CUT[0] / 2, y + METER_CUT[0] / 2, z - METER_CUT[1] / 2, z + METER_CUT[1] / 2),
                P[n]["mass_g"])
        b[n + "_bezel"] = ((COVER[0] - 2.5, COVER[0], y - s[0] / 2, y + s[0] / 2, z - s[1] / 2, z + s[1] / 2), 0)
    for n, y, z, d in SWITCHES:
        s = P[n]["size_mm"]
        b[n] = ((xin, xin + s[2], y - d / 2 - 1.5, y + d / 2 + 1.5, z - d / 2 - 1.5, z + d / 2 + 1.5), P[n]["mass_g"])
    b["imu_im10a"] = (imu_box(), P["imu_im10a"]["mass_g"])
    for n, bx in res_boxes().items():
        b[n] = (bx, RES[3])
    b["relay_k2"] = ((PX - K2[4], PX, K2[0], K2[1], K2[2], K2[3]), P["relay_k2_kka_k4"]["mass_g"])
    if PUMP == "dwp385":
        pb = dwp385_ref().bounding_box()
        b["pump_dwp385"] = (tuple(round(v, 2) for v in (pb.min.X, pb.max.X, pb.min.Y, pb.max.Y, pb.min.Z, pb.max.Z)),
                            P["pump_dwp385"]["mass_g"])
        b["filter_inline"] = (FILTER2 if F2 else tuple(tt.R1_PUMP), 6)   # F1: in the R1F pocket
    else:
        b["pump_m400k"] = (tuple(tt.R1_PUMP), tt.PUMP_G)
    return b


def wall_scan(path, n=8000, q=0.005):
    """Thinnest wall (mm): from surface points, a ray inward to the far side, as cad/parts wall_scan. Returns the
    0.5 % quantile, so that sharp tips and edges (a snap clip tip, a chamfer) do not count as walls."""
    m = trimesh.load(path)
    pts, fi = trimesh.sample.sample_surface_even(m, n, seed=1)
    nrm = m.face_normals[fi]
    o = pts - nrm * 1e-3
    loc, ray, _ = m.ray.intersects_location(o, -nrm, multiple_hits=False)
    return float(np.quantile(np.linalg.norm(loc - o[ray], axis=1), q))


def main():
    for d in ("stl", "step", "drawings"):
        (KIT / d).mkdir(parents=True, exist_ok=True)
    OUT.mkdir(exist_ok=True)
    tanks = ("tank_left_tub", "tank_left_plate", "tank_right_tub", "tank_right_plate") if F2 else \
        ("r1f_half_left", "r1f_half_right")
    assert (OUT / f"{tanks[0]}.stl").exists(), f"run DESIGN={tank.DESIGN} tank.py first"
    lift = Pos(0, 0, T)
    shapes = {
        "pan_base": lift * st.PARTS["pan_base"](), "pan_yoke": lift * st.PARTS["pan_yoke"](),
        "tilt_u": lift * st.PARTS["tilt_u"](), "head_f1": head_f1(),
        "holder_6s_zeee": holder("zeee"), "holder_6s_vega": holder("vega"),
        "pod_spacer_left": pod_spacer(1), "pod_spacer_right": pod_spacer(-1),
        "pod_basket": pod_basket(), "pod_cover": pod_cover(), "fill_extension": fill_extension(),
        "imu_mount": imu_mount(), "hose_clip": hose_clip(), "frame_clip": frame_clip(),
        "pump_holder": pump_holder(), "resistor_mount_left": resistor_mount(1), "resistor_mount_right": resistor_mount(0),
    }
    if PUMP != "dwp385":
        shapes.pop("pump_holder")
    if F2:
        shapes.pop("fill_extension")
    from build123d import import_step
    for n in (*tanks, "deck_f1"):
        shapes[n] = import_step(str(OUT / f"{n}.step"))
    plate = pod_plate()
    info = {"parts": {}, "bought": {}, "checks": {}}
    meshes = {}
    for n, s in shapes.items():
        export_stl(s, str(KIT / "stl" / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.1)
        export_step(s, str(KIT / "step" / f"{n}.step"))
        meshes[n] = trimesh.load(KIT / "stl" / f"{n}.stl")
    export_step(plate, str(KIT / "step" / "pod_plate_al2.step"))
    export_stl(plate, str(OUT / "pod_plate_al2.stl"), tolerance=0.05, angular_tolerance=0.2)   # render only
    refs, lens, tip = head_refs()
    if PUMP == "dwp385":
        export_stl(dwp385_ref(), str(OUT / "ref_pump_dwp385.stl"), tolerance=0.05, angular_tolerance=0.2)
    for n, s in refs.items():
        export_stl(s, str(OUT / f"ref_{n}.stl"), tolerance=0.05, angular_tolerance=0.2)

    # ---------- checks: bed, closed mesh, walls ----------
    print(f"{'part':18s} {'qty':>3s} {'cm3':>6s} {'g':>5s} {'h:min':>6s}  bbox mm            wall")
    for n, s in shapes.items():
        m = meshes[n]
        qty, infill, orient, sup = PRINTS[n]
        assert all(a <= b for a, b in zip(sorted(m.extents), sorted(BED))), f"{n} does not fit the bed"
        assert m.is_watertight, f"{n} STL is not closed"
        assert len(m.split(only_watertight=False)) == 1, f"{n} is more than one body"
        # snap clips: C tips are wedges by design, so the wall parameters count (ring 2.1 or 2.5, foot 2.5 or 3)
        w = {"hose_clip": HOSE_CLIP_WALL, "frame_clip": 2.5}[n] if n in ("hose_clip", "frame_clip") else wall_scan(KIT / "stl" / f"{n}.stl")
        assert w >= WALL_MIN - 0.05, f"{n}: thinnest wall {w:.2f} mm"
        pv, g, t = print_estimate(s, infill)
        c = s.center(CenterOf.MASS)
        bb = s.bounding_box()
        info["parts"][n] = {"qty": qty, "infill": infill, "orientation": orient, "supports": sup,
                            "volume_cm3": round(s.volume / 1000, 1), "mass_g": round(g, 1), "time_min": round(t),
                            "solid_g": round(s.volume / 1000 * RHO, 1), "com_mm": [round(v, 1) for v in (c.X, c.Y, c.Z)],
                            "box_mm": [round(v, 1) for v in (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)],
                            "bbox_mm": [round(v, 1) for v in m.extents], "min_wall_mm": round(w, 2)}
        print(f"{n:18s} {qty:3d} {s.volume / 1000:6.1f} {g:5.0f} {int(t) // 60:3d}:{int(t) % 60:02d}  "
              f"{m.extents[0]:5.1f} x {m.extents[1]:5.1f} x {m.extents[2]:5.1f}  {w:.2f}")
    pa = plate.volume / 1000 * 2.70
    c = plate.center(CenterOf.MASS)
    bb = plate.bounding_box()
    info["parts"]["pod_plate_al2"] = {"qty": 1, "material": "aluminium 2 mm", "mass_g": round(pa, 1),
                                      "com_mm": [round(v, 1) for v in (c.X, c.Y, c.Z)],
                                      "box_mm": [round(v, 1) for v in (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)]}

    # ---------- fit to the bought parts (parameters) ----------
    L, W, H, z0, sy, xo, by = holder_geo()
    fits = {
        "holder bay width - pack width (mm)": round(xo - H_DIV / 2 - W, 2),
        "IMU pocket - case (mm)": FIT,
        "ROCK 3C holes (mm)": "58 x 49 (datasheet 58 x 49)",
        "diode holes (mm)": "80 apart (datasheet 80)",
        "relay holes (mm)": "63.5 x 26 diagonal (datasheet)",
        "camera holes (mm)": "34 x 34 and 28 x 28, M2",
    }
    assert xo - H_DIV / 2 - W >= 2 * H_CLR - 1e-6
    yh = sorted({round(y, 2) for y, z, d, l in plate_holes() if "ROCK" in l})
    zh = sorted({round(z, 2) for y, z, d, l in plate_holes() if "ROCK" in l})
    assert abs(yh[1] - yh[0] - 49) < 1e-6 and abs(zh[1] - zh[0] - 58) < 1e-6
    dz = [y for y, z, d, l in plate_holes() if "diode" in l]
    assert abs(dz[1] - dz[0] - 80) < 1e-6
    info["checks"]["fit"] = fits

    # ---------- the nozzle stays out of the camera image ----------
    ang = math.degrees(math.atan2(lens[2] - tip[2], tip[0] - lens[0]))      # below the forward horizontal, 0..180
    off_axis = abs(ang - tm.CAM_PITCH)                                       # the tip is in the plane y = 0
    assert off_axis > VFOV / 2 + 5, f"nozzle tip is in the camera image ({off_axis:.0f} deg off axis)"
    info["head"] = {"lens_mm": [round(v, 1) for v in lens], "nozzle_tip_mm": [round(v, 1) for v in tip],
                    "camera_pitch_deg": tm.CAM_PITCH, "camera_vfov_deg": round(VFOV, 1),
                    "nozzle_tip_off_axis_deg": round(off_axis, 1), "tilt_axis_mm": [tm.PAN_X, 0.0, T + st.Z_TILT]}

    # ---------- collisions: printed parts and bought parts, solids ----------
    bought = bought_boxes()
    solids = {n: shapes[n] for n in shapes if n not in ("holder_6s_vega", "hose_clip", "frame_clip")}
    solids["pod_plate_al2"] = plate
    for n, (bx, _) in bought.items():
        if not n.endswith("_bezel"):
            solids[n] = box(*bx)
    solids.update({n: s for n, s in refs.items()})
    if PUMP == "dwp385":
        solids["pump_dwp385"] = dwp385_ref()                 # the real shape, not the box
    st_refs = {n: lift * s for n, s in st.refs().items() if n in ("ref_pan_servo", "ref_tilt_servo", "ref_pan_bearing")}
    solids.update(st_refs)
    for n, (cx0, cx1, cy0, cy1, cz0, cz1) in {"relay_aep18024": layout.RELAY, "shelf_acrylic": layout.SHELF_BOX}.items():
        solids[n] = box(cx0, cx1, cy0, cy1, cz0, cz1)
    names = list(solids)
    bbs = {n: solids[n].bounding_box() for n in names}
    overlaps = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            ba, bb_ = bbs[a], bbs[b]
            if ba.min.X > bb_.max.X or bb_.min.X > ba.max.X or ba.min.Y > bb_.max.Y or bb_.min.Y > ba.max.Y \
                    or ba.min.Z > bb_.max.Z or bb_.min.Z > ba.max.Z:
                continue
            r = solids[a] & solids[b]
            v = r.volume if r is not None else 0.0
            if v > 1.0:
                overlaps.append(f"{a} / {b}: {v:.1f} mm3")
    print("solid overlaps:", overlaps or "none")
    info["checks"]["solid_overlaps"] = overlaps
    assert not overlaps, overlaps

    # ---------- torso mesh clearance (contact faces on purpose are left out) ----------
    tp = cKDTree(layout.torso_points())
    keep = {"holder_6s_zeee": lambda p: p[:, 2] > RAIL_TOP + 1.5, "holder_6s_vega": lambda p: p[:, 2] > RAIL_TOP + 1.5,
            "pod_spacer_left": lambda p: p[:, 0] < POD_X + 3.5, "pod_spacer_right": lambda p: p[:, 0] < POD_X + 3.5,
            "imu_mount": lambda p: p[:, 2] > SHELF_Z + SHELF_T + 0.5,
            "pump_holder": (lambda p: p[:, 2] > -1e9) if F2 else (lambda p: p[:, 1] < -75.0 - 1.5),
            "resistor_mount_left": lambda p: np.abs(p[:, 0]) > 71.5, "resistor_mount_right": lambda p: np.abs(p[:, 0]) > 71.5}
    torso = {}
    for n in ("holder_6s_zeee", "holder_6s_vega", "pod_spacer_left", "pod_spacer_right", "pod_basket", "pod_cover",
              "fill_extension", "imu_mount", "head_f1", "pump_holder", "resistor_mount_left", "resistor_mount_right"):
        if n not in meshes:
            continue
        p = np.vstack([meshes[n].vertices, meshes[n].sample(30000)])
        p = p[keep.get(n, lambda q: np.ones(len(q), bool))(p)]
        torso[n] = round(float(tp.query(p)[0].min()), 2)
    if PUMP == "dwp385":                           # the pump shape, not its box (the box is checked in layout.py)
        pm = trimesh.load(OUT / "ref_pump_dwp385.stl")
        torso["pump_dwp385"] = round(float(tp.query(np.vstack([pm.vertices, pm.sample(20000)]))[0].min()), 2)
    for v in PACKS:
        for n, bx in packs(v).items():
            bm = layout.box_mesh(bx)
            torso[f"{n}_{v}"] = round(float(tp.query(np.vstack([bm.vertices, bm.sample(20000)]))[0].min()), 2)
        torso[f"straps_{v}"] = round(float(tp.query(strap_points(v))[0].min()) - STRAP_T / 2, 2)
    print("torso clearance (mm):", torso)
    for n, d in torso.items():
        # straps: soft, they may touch. The exact distance to the bracket tongue (x -50, y 34, z 634) is 0.36 mm
        # (trimesh.proximity); F1 reported 0.5 to 1.0 mm from unseeded samples.
        assert d >= (0.3 if n.startswith("straps") else 1.0), f"{n} is {d} mm from the torso mesh"
    info["checks"]["torso_clearance_mm"] = torso

    # ---------- write ----------
    info["bought"] = {n: {"box_mm": [round(v, 2) for v in bx], "mass_g": g} for n, (bx, g) in bought.items()}
    info["refs"] = {"camera_b0201": {"mass_g": P["camera_b0201"]["mass_g"]}, "nozzle_brass": {"mass_g": 30}}
    # top of the robot for the simulation: printed pan-tilt and head, servos, bearing, camera, nozzle, hose
    top = []

    def add(name, g, s):
        c, b = s.center(CenterOf.MASS), s.bounding_box()
        top.append({"name": name, "mass_g": round(g, 1), "com_mm": [round(v, 1) for v in (c.X, c.Y, c.Z)],
                    "size_mm": [round(v, 1) for v in (b.size.X, b.size.Y, b.size.Z)]})
    for n in ("deck_f1", "pan_base", "pan_yoke", "tilt_u", "head_f1"):
        add(n, info["parts"][n]["mass_g"], shapes[n])
    for n, g in (("ref_pan_servo", 89), ("ref_tilt_servo", 89), ("ref_pan_bearing", 26)):   # ST3215 89 g (Waveshare)
        add(n, g, st_refs[n])
    add("camera_b0201", P["camera_b0201"]["mass_g"], refs["camera_b0201"])
    add("nozzle_brass", 30, refs["nozzle_brass"])                                          # GUESS
    add("hose_clips", 4 * info["parts"]["hose_clip"]["mass_g"], box(tm.PAN_X - 20, tm.PAN_X + 20, -20, 20, T + 20, T + 60))
    add("hose_and_water", 25, box(-40, 40, -45, -25, T - 60, T + 40))                       # GUESS: 0.6 m of tube
    info["top"] = top
    info["pack"] = PACK
    info["pump"] = PUMP
    info["design"] = tank.DESIGN
    L_, W_, H_, z0_, sy_, xo_, by_ = holder_geo()
    xs_ = xo_ - H_CLR
    info["straps_mm"] = [[[round(x, 1), s * sy_, round(z, 1)] for x, z in (
        (-xo_ - H_WALL - 1.5, RAIL_TOP + 4.5), (xo_ + H_WALL + 1.5, RAIL_TOP + 4.5), (xo_ + H_WALL + 0.75, z0_ + H_WALL_H),
        (xs_ + 0.75, z0_ + H_ + 0.75), (-xs_ - 0.75, z0_ + H_ + 0.75), (-xo_ - H_WALL - 0.75, z0_ + H_WALL_H),
        (-xo_ - H_WALL - 1.5, RAIL_TOP + 4.5))] for s in (-1, 1)]                       # for the renders
    info["holder"] = {v: {"pack_bottom_z": PACKS[v][3], "strap_centre_y": PACKS[v][4], "bay_width": round(xo - H_DIV / 2, 2)}
                      for v in PACKS}
    (OUT / "kit.json").write_text(json.dumps(info, indent=1))
    drawings(info)
    sheets(info)
    print("kit checks passed")


# ---------------- drawings ----------------
def _dxf(path, outline, holes, slots=()):
    import ezdxf
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    msp = doc.modelspace()
    msp.add_lwpolyline(outline, close=True, dxfattribs={"layer": "CUT"})
    for u, v, d, _ in holes:
        msp.add_circle((u, v), d / 2, dxfattribs={"layer": "CUT"})
    for u0, u1, v0, v1 in slots:   # rounded slot along u, width v1 - v0
        r = (v1 - v0) / 2
        msp.add_lwpolyline([(u0 + r, v0, 0, 0), (u1 - r, v0, 0, 1), (u1 - r, v1, 0, 0), (u0 + r, v1, 0, 1)],
                           format="xyseb", close=True, dxfattribs={"layer": "CUT"})
    doc.saveas(path)


def _sheet(path, title, outline, holes, slots, notes, origin, ulabel, vlabel):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyBboxPatch, Polygon
    fig = plt.figure(figsize=(11.69, 8.27))                  # A4 landscape
    ax = fig.add_axes([0.04, 0.08, 0.56, 0.84])
    ax.add_patch(Polygon(outline, closed=True, fc="#f2f2f2", ec="#222", lw=1.2))
    ou, ov = origin
    tags = {}
    for i, (u, v, d, label) in enumerate(holes):
        ax.add_patch(Circle((u, v), d / 2, fc="white", ec="#222", lw=0.9))
        tag = chr(65 + i) if i < 26 else f"A{i - 25}"
        tags[tag] = (u, v, d, label)
        ax.text(u + d / 2 + 1, v + d / 2 + 0.5, tag, fontsize=7, color="#b00")
    for u0, u1, v0, v1 in slots:
        ax.add_patch(FancyBboxPatch((u0 + (v1 - v0) / 2, v0), u1 - u0 - (v1 - v0), v1 - v0,
                                    boxstyle=f"round,pad=0,rounding_size={(v1 - v0) / 2 - 0.01}", fc="white", ec="#222"))
    us, vs = [p[0] for p in outline], [p[1] for p in outline]
    ax.annotate("", (min(us), min(vs) - 8), (max(us), min(vs) - 8), arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text((min(us) + max(us)) / 2, min(vs) - 13, f"{max(us) - min(us):.1f}", ha="center", fontsize=8)
    ax.annotate("", (min(us) - 8, min(vs)), (min(us) - 8, max(vs)), arrowprops=dict(arrowstyle="<->", lw=0.8))
    ax.text(min(us) - 13, (min(vs) + max(vs)) / 2, f"{max(vs) - min(vs):.1f}", va="center", rotation=90, fontsize=8)
    ax.plot([ou], [ov], "k+", ms=12)
    ax.text(ou + 1.5, ov + 1.5, "0,0", fontsize=7)
    ax.set_aspect("equal")
    ax.set_xlim(min(us) - 20, max(us) + 10)
    ax.set_ylim(min(vs) - 20, max(vs) + 10)
    ax.axis("off")
    ax.set_title(title, fontsize=11, loc="left")
    tx = fig.add_axes([0.62, 0.08, 0.36, 0.84])
    tx.axis("off")
    rows = [f"{'':3s}{ulabel:>8s}{vlabel:>8s}{'dia':>6s}  use"]
    rows += [f"{t:3s}{u - ou:8.1f}{v - ov:8.1f}{d:6.1f}  {lab}" for t, (u, v, d, lab) in tags.items()]
    rows += [f"slot {u0 - ou:.1f}..{u1 - ou:.1f} x {v0 - ov:.1f}..{v1 - ov:.1f}" for u0, u1, v0, v1 in slots]
    tx.text(0, 1, "\n".join(rows + [""] + notes), va="top", family="monospace", fontsize=7.2)
    fig.savefig(path.with_suffix(".pdf"))
    fig.savefig(path.with_suffix(".png"), dpi=140)
    plt.close(fig)


def shelf_layout():
    """Acrylic shelf (x, y) outline and holes, robot frame."""
    ox, oy = 70.0, 54.5
    outline = [(-ox, -oy), (ox, -oy), (ox, oy), (-ox, oy)]
    holes = [(x, y, M3_CLR, "M3 to bracket bar") for x, y in BRACKET_HOLES]
    xc = sum(layout.RELAY[:2]) / 2
    holes += [(xc + 13.0, -31.75, 5.5, "M5 relay"), (xc - 13.0, 31.75, 5.5, "M5 relay")]
    if not F2:
        holes += [(x, y, M4_CLR, "M4 R1F leg") for x, y, _ in tt.R1_LEGS]
    bx = layout.BOOST
    cx, cy = (bx[0] + bx[1]) / 2, (bx[2] + bx[3]) / 2
    holes += [(cx + sx * 14.5, cy + sy * 29.5, M3_CLR, "M3 boost*") for sx in (-1, 1) for sy in (-1, 1)]
    slots = [(-20.0, 10.0, 44.0, 52.0), (12.0, 36.0, -52.0, -44.0)]
    return outline, holes, slots


def drawings(info):
    out = KIT / "drawings"
    outline, holes, slots = shelf_layout()
    _dxf(str(out / "shelf_acrylic_5mm.dxf"), [(x + 70, y + 54.5) for x, y in outline],
         [(x + 70, y + 54.5, d, l) for x, y, d, l in holes], [(a + 70, b + 70, c + 54.5, d + 54.5) for a, b, c, d in slots])
    _sheet(out / "shelf_acrylic_5mm", "Middle shelf: 5 mm clear acrylic, 1 piece (top view, front = +x to the right)",
           outline, holes, slots,
           ["Origin: rear right corner (x -70, y -54.5).", "u = robot x (forward), v = robot y (left), mm.",
            "Cut the outline and the 2 slots on a laser.", "Drill the holes or cut them on the laser.",
            "* Boost holes: ASSUMED 59 x 29 mm pattern.", "  Measure the board, then drill these 4 last.",
            "Relay: holes on one diagonal. Check which", "  diagonal your relay has before you drill.",
            "M3 bracket holes: the holes of the BHL PC", "  shelf brackets (mesh). Check on the robot.",
            "The shelf rests on the bracket bars (z 641).",
            *(["Rear M3 holes: the pump holder base sits", "  on them (M3 x 20)."] if F2 else [])],
           (-70.0, -54.5), "x", "y")
    yy, z0, _ = PLATE
    ph = plate_holes()
    _dxf(str(out / "pod_plate_al2.dxf"), [(yy - y, z - z0) for y, z in plate_outline()],      # seen from behind,
         [(yy - y, z - z0, d, l) for y, z, d, l in ph])                                      # as the PDF
    _sheet(out / "pod_plate_al2", "Pod plate: 2 mm aluminium (5052 or 6061), 1 piece (seen from behind the robot)",
           [(-y, z) for y, z in plate_outline()], [(-y, z, d, l) for y, z, d, l in ph], [],
           ["Seen from behind: robot left (+y) is on the left.", "Origin: bottom left corner seen from behind.",
            "u = across, v = up from the bottom edge, mm.", "Deburr all edges. No bends.",
            "* Shunt holes: ASSUMED near the ends of a", "  105 mm base. Drill them from the part.",
            "Grommet: 20 mm hole for a rubber grommet.", "M6 diode: thermal paste under the diode.",
            *([] if F2 else ["Notch: R1F fill neck (z 735) and extension."])],
           (-yy, z0), "u", "v")


# ---------------- sheets ----------------
HARDWARE = [  # (item, count, where)
    ("M5 drop-in T-nut, 2020 6 mm slot", 26, "deck 4, holder 4, pod spacers 4, resistor mounts 4, pump holder 2, frame clips 8"),
    ("M5 x 12 socket head screw", 8, "deck 4, holder 4"),
    ("M5 x 12 button head screw", 8, "pod spacers 4, resistor mounts 4"),
    ("M5 x 10 button head screw", 2, "pump holder (DWP-385 only)"),
    ("M5 x 8 button head screw", 8, "frame clips"),
    ("M5 x 12 button head screw + M5 nut + washer", 2, "relay K1 to the shelf, heads below (check the relay foot thickness)"),
    ("M5 x 10 screw + M5 nut + washer", 2, "shunt to the pod plate (ASSUMED size, check the shunt)"),
    ("M6 x 12 screw + M6 nut + 2 washers", 2, "diode to the pod plate, with thermal paste"),
    ("M4 heat-set insert, 5.6 mm hole, 8 mm long", 4, "R1F legs"),
    ("M4 x 12 button head screw", 4, "shelf to the R1F legs, from below"),
    ("M4 x 12 button head screw + M4 nut", 4, "head to the tilt U (nuts in the U pockets)"),
    ("M4 x 16 screw + M4 nut", 4, "pan base to the deck (nuts in the deck pockets)"),
    ("M3 heat-set insert, 4.0 mm hole, 5 mm long", 8, "resistor mounts"),
    ("M3 x 8 screw + washer", 8, "brake resistors to their mounts (ASSUMED 39.7 x 21.4 mm pattern)"),
    ("M3 x 16 screw + M3 nut", 2, "shelf to the rear bracket bar"),
    ("M3 x 20 screw + M3 nut", 2, "IMU mount and shelf to the front bracket bar"),
    ("M3 x 10 screw + M3 nut + nylon spacer 5 mm", 4, "boost converter on the shelf"),
    ("M3 x 12 button head screw + M3 nut", 4, "3S basket to the pod plate (nuts in the band pockets)"),
    ("M3 x 10 screw + M3 nut", 2, "pod cover to the basket ledge, from below"),
    ("M3 x 10 set screw (or thread-forming M3 x 10)", 1, "nozzle adapter in the head"),
    ("M2.5 x 10 male-female brass standoff + M2.5 nut", 4, "ROCK 3C on the pod plate"),
    ("M2.5 x 5 screw", 4, "ROCK 3C to the standoffs"),
    ("M2 x 8 screw + M2 nut", 4, "camera board in the head (34 or 28 mm pattern)"),
    ("Hook-and-loop strap 20 mm x 400 mm", 2, "6S packs in the holder"),
    ("Hook-and-loop strap 20 mm x 250 mm", 2, "3S pack in the basket"),
    ("Hook-and-loop strap 20 mm x 200 mm", 1, "pump in the pump holder"),
    ("Rubber grommet for a 20 mm hole", 1, "pod plate cable pass"),
    ("O-ring 15 x 2 mm (NBR)", 1, "fill extension in the R1F neck bore"),
    ("Silicone stopper for a 19 mm bore, with a vent hole", 1, "rear end of the fill extension"),
    ("Zip tie 2.5 mm", 18, "IMU 1, hose clips 8, relay K2 1, spare 8"),
    ("Double-sided foam tape 1 mm", "0.4 m", "CANable x 2, buck, MOSFET module on the basket lip, relay K2, IMU back"),
    ("Silicone tube 3 x 6 mm", "1 m", "tank to filter to pump to nozzle"),
    ("Thermal paste", "1 g", "diode"),
    ("Aluminium sheet 2 mm, 150 x 198 mm", 1, "pod plate (drawing pod_plate_al2)"),
    ("Clear acrylic 5 mm, 140 x 109 mm", 1, "middle shelf (drawing shelf_acrylic_5mm), from the B4 sheet"),
]


HARDWARE_F2 = [  # F1 list with the R2 tanks: no pump holder on the side face, no R1F legs, no fill extension
    ("M5 drop-in T-nut, 2020 6 mm slot", 32, "deck 4, holder 4, pod spacers 4, resistor mounts 4, tanks 8, frame clips 8"),
    ("M5 x 55 socket head screw + washer", 8, "tanks to the side slots of the posts (4 per tank)"),
    ("M5 x 12 socket head screw", 8, "deck 4, holder 4"),
    ("M5 x 12 button head screw", 8, "pod spacers 4, resistor mounts 4"),
    ("M5 x 8 button head screw", 8, "frame clips"),
    ("M5 x 12 button head screw + M5 nut + washer", 2, "relay K1 to the shelf, heads below (check the relay foot thickness)"),
    ("M5 x 10 screw + M5 nut + washer", 2, "shunt to the pod plate (ASSUMED size, check the shunt)"),
    ("M6 x 12 screw + M6 nut + 2 washers", 2, "diode to the pod plate, with thermal paste"),
    ("M4 x 12 button head screw + M4 nut", 4, "head to the tilt U (nuts in the U pockets)"),
    ("M4 x 16 screw + M4 nut", 4, "pan base to the deck (nuts in the deck pockets)"),
    ("M3 heat-set insert, 4.0 mm hole, 5 mm long", 8, "resistor mounts"),
    ("M3 x 8 screw + washer", 8, "brake resistors to their mounts (ASSUMED 39.7 x 21.4 mm pattern)"),
    ("M3 x 20 screw + M3 nut", 4, "IMU mount and shelf to the front bracket bar 2, pump holder and shelf to the rear bar 2"),
    ("M3 x 10 screw + M3 nut + nylon spacer 5 mm", 4, "boost converter on the shelf"),
    ("M3 x 12 button head screw + M3 nut", 4, "3S basket to the pod plate (nuts in the band pockets)"),
    ("M3 x 10 screw + M3 nut", 2, "pod cover to the basket ledge, from below"),
    ("M3 x 10 set screw (or thread-forming M3 x 10)", 1, "nozzle adapter in the head"),
    ("M2.5 x 10 male-female brass standoff + M2.5 nut", 4, "ROCK 3C on the pod plate"),
    ("M2.5 x 5 screw", 4, "ROCK 3C to the standoffs"),
    ("M2 x 8 screw + M2 nut", 4, "camera board in the head (34 or 28 mm pattern)"),
    ("Brass hose barb M5 x 3 mm", 2, "tank outlets, with PTFE tape"),
    ("Hose tee for 3 mm tube", 1, "joins the 2 tank lines in front of the torso"),
    ("Silicone stopper for a 16 mm bore, with a vent hole", 2, "tank fill necks"),
    ("Hook-and-loop strap 20 mm x 400 mm", 2, "6S packs in the holder"),
    ("Hook-and-loop strap 20 mm x 250 mm", 3, "3S pack in the basket 2, DWP-385 pump holder 1 (see the pump note)"),
    ("Rubber grommet for a 20 mm hole", 1, "pod plate cable pass"),
    ("Zip tie 2.5 mm", 24, "IMU 1, hose clips 8, relay K2 1, filter 2, tank tubes 4, spare 8"),
    ("Double-sided foam tape 1 mm", "0.4 m", "CANable x 2 and buck on the basket lip, relay K2, IMU back"),
    ("Silicone tube 3 x 6 mm", "1.5 m", "tanks to tee to filter to pump 0.6 m, pump to nozzle 0.7 m, spare"),
    ("Thermal paste", "1 g", "diode"),
    ("Aluminium sheet 2 mm, 150 x 198 mm", 1, "pod plate (drawing pod_plate_al2)"),
    ("Clear acrylic 5 mm, 140 x 109 mm", 1, "middle shelf (drawing shelf_acrylic_5mm), from the B4 sheet"),
]
if F2:
    HARDWARE = HARDWARE_F2  # noqa: F811
HARDWARE = HARDWARE + [  # ST3215 pan-tilt (cad/parts/top_module_sts3215.py)
    ("6808-2RS ball bearing, 40 x 52 x 7 mm", 1, "pan axis, in the pan base"),
    ("M3 screw, length to fit (ASSUMED thread)", 12, "pan yoke to the pan servo horn 4, tilt U arms to the 2 tilt servo horns 8"),
    ("M2 screw, length to fit (ASSUMED thread)", 8, "pan servo in the pan base 4, tilt servo to the yoke tabs 4"),
]


def sheets(info):
    rows = ["| Part | Qty | Material | Infill | Orientation on the bed | Supports | Size (mm) | Mass each (g) | Time each (h:min) |",
            "|---|---|---|---|---|---|---|---|---|"]
    tot_g = tot_t = 0
    for n, p in info["parts"].items():
        if n == "pod_plate_al2":
            continue
        e = p["bbox_mm"]
        rows.append(f"| {n} | {p['qty']} | PETG | {int(p['infill'] * 100)} % | {p['orientation']} | {p['supports']} | "
                    f"{e[0]:.0f} x {e[1]:.0f} x {e[2]:.0f} | {p['mass_g']:.0f} | {p['time_min'] // 60}:{p['time_min'] % 60:02d} |")
        tot_g += p["qty"] * p["mass_g"]
        tot_t += p["qty"] * p["time_min"]
    text = ["# Print sheet (generated by kit.py)", "",
            "Status: **planned**. Nothing is printed. Mass and time are estimates from the CAD volume, not from a slicer.", "",
            "**Assumed:** PETG at 1.27 g/cm3, 0.4 mm nozzle, 0.2 mm layers, 4 perimeters (1.6 mm), 5 top and bottom layers, "
            "gyroid infill. The time model is 5 mm3/s of plastic, plus 20 % for moves, plus 10 min per part. "
            "Your slicer gives the real time.", "",
            "The STL files are in the robot frame. Turn each part to the orientation in the table in the slicer.", "",
            *rows, "",
            f"Total for one robot: {tot_g / 1000:.2f} kg of PETG and about {tot_t / 60:.0f} h of printing "
            "(the VEGA holder is not counted).", "",
            ("Print the 4 tank pieces solid (100 %), so that the walls hold water. Glue each plate on its tub with "
             "two-part epoxy. Leak-test each glued tank for 1 hour." if F2 else
             "Print the R1F halves solid (100 %), so that the walls hold water. Leak-test the glued tank for 1 hour.")]
    if F2:
        text[0] = "# Print sheet, final design F2 (generated by kit.py)"
    (KIT / "print_sheet.md").write_text("\n".join(text) + "\n")
    hw = [f"# Hardware list, final design {tank.DESIGN} (generated by kit.py)", "", "Counts are for one robot. Buy 10 % spare of the small screws.", "",
          "| Item | Count | Where |", "|---|---|---|"] + [f"| {a} | {b} | {c} |" for a, b, c in HARDWARE]
    hw += ["", "The pan-tilt also needs the 2 ST3215 servos. The Waveshare drawing gives no thread size for the horn and case "
           "screws. See the pan-tilt steps in `README.md`."]
    if F2:
        hw += ["", "**Pump:** The pump holder and its 250 mm strap fit the DWP-385 pump. The team uses the Kamoer KLP02 24 V "
               "pump with a DMC-101 controller. We will redraw the holder when the KLP02 arrives and is measured."]
    (KIT / "hardware.md").write_text("\n".join(hw) + "\n")


if __name__ == "__main__":
    main()
