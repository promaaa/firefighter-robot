"""Concept A top module: deck plate, pan-tilt for standard servos, payload plate, camera mounts.

    uv run --project cad python cad/parts/top_module.py

Writes STEP + STL per printed part to cad/parts/out/, reference (bought) parts to out/ref/,
and out/assembly.json (tilt axis, masses) for render_module.py. Ends with the printability asserts.

Units mm. Assembly frame = robot frame (x forward, y left, z up) with the origin on the top face
of the torso frame, at its centre (robot z = 0.791 m, feet on the floor).
Every part is exported in this frame, at tilt = 0, so the STLs stack without extra transforms.
"""
import json
import math
from pathlib import Path

from build123d import (Align, Box, CenterOf, Cylinder, Pos, RegularPolygon, Rot, SlotOverall, export_step, export_stl,
                       extrude)

OUT = Path(__file__).parent / "out"

# ---------------- parameters ----------------
# Torso top frame: 2020 profile (measured on base_visual.stl).
RAIL_X = 60.0          # centre line of the front / rear top rails (x = +-60), rails run along y
RAIL_W = 20.0
# Pan axis fore-aft position on the deck (payload simulation, sim/payload/README.md: mass behind the torso
# centre lowers the leg torque). The pan servo body points forward, so its body clears the rear rail
# while PAN_X >= PAN_X_MIN (checked below).
PAN_X = -38.0
M5_CLR, M4_CLR, M3_CLR, M2_CLR, M25_CLR = 5.5, 4.5, 3.4, 2.4, 2.9
M3_PILOT = 2.5         # thread-forming M3 screw into PLA
M4_NUT_AF, M4_NUT_T = 7.0, 3.2
FIT = 0.15             # added to printed bores that take a bought part

# Standard-size servo: DS3218 drawing; body size is the envelope with the MG996R.
# Servo frame: shaft on +z, origin on the shaft axis in the plane of the top face of the mounting flange,
# body length along x, shaft near the +x end.
SV_L, SV_W = 40.9, 20.1          # body length, width (DS3218 40 x 20, MG996R up to 40.9 x 20)
SV_SHAFT = 10.0                  # shaft axis to the +x end of the body (DS3218 drawing)
SV_FL_L, SV_FL_T = 54.5, 3.1     # flange (ears) total length, thickness (DS3218 drawing)
SV_BELOW = 27.8 + SV_FL_T        # flange top face to body bottom (DS3218: bottom to flange underside 27.8)
SV_ABOVE = 40.5 - SV_BELOW       # flange top face to case top (DS3218 body height 40.5)
SV_HOLE_DX, SV_HOLE_DY = 49.5, 10.0   # flange hole spacing (DS3218 drawing)
SV_CLR = 0.4                     # gap around the body in a window
# 25T aluminium disc horn RJX3366: 20 mm disc, 4.5 mm high, 4 x M3 on a 14 mm circle (maker drawing).
# HORN_TOP = horn top face above the flange top. The DS3218 spline top is 45.5 - 30.9 = 14.6 above the flange;
# the horn bore runs through its 4.5 mm height, so the horn top is flush with the spline top (derived).
HORN_D, HORN_TOP, HORN_BC = 20.0, 45.5 - SV_BELOW, 14.0

# Bearings
PAN_BRG = (40.0, 52.0, 7.0)      # 6808-2RS: bore, OD, width
TILT_BRG = (8.0, 22.0, 7.0)      # 608-2RS

# Interface patterns (the "standard" holes other heads use)
PAN_PATTERN = 64.0     # deck <-> pan base, 4 x M4 on a square
PAYLOAD_PATTERN = 40.0 # tilt U <-> payload (gun plate, B head, E valve), 4 x M4 on a square

DECK_T, DECK_Y = 6.0, 130.0
RAIL_SCREW_Y = 55.0              # M5 deck screws at y = +-55, outside the pan base
DECK_SLOT = (15.0, -50.0)        # cable pass-through slot centre x, y
SHELF_T, SHELF = 5.0, 80.0       # pan base flange plate
HOUSING_D = 78.0
LIP = 2.0                        # bearing seat shoulder
TT_T, TT_D = 5.0, 76.0           # turntable disc
CHEEK_T, CHEEK_R = 6.0, 22.0     # servo cheek thickness, radius of its top around the tilt axis
BCHEEK_T = TILT_BRG[2] + 2.5     # bearing cheek
SERVO_GAP = 2.5                  # servo bottom to bearing cheek
ARM_T, ARM_R, U_T, U_GAP = 8.0, 17.0, 6.0, 3.0
GUN_W = 70.0                     # water gun body width
GUN_MASS = 0.9                   # kg with water, GUESS for the CoM estimate
# Gun CoM behind the tilt axis. Payload sim round 2 wants mass near x = -0.06 m and the gun CoM within 0.05 m
# of the pan axis. PAN_X cannot go behind -38.6 mm (rear rail), so the gun moves back instead.
GUN_COM_DX = -22.0
PLATE_T, PLATE_X, PLATE_Y = 5.0, 200.0, 100.0
WING_X = 40.0                    # front wing that carries the camera brackets
STRAP_W = 27.0                   # slot length for 25 mm hook-and-loop straps
# Camera position (camera study): on the gun, the barrel hid the bottom centre of
# the image and the camera saw no floor closer than ~2.6 m. So the cameras sit beside the gun, not on it,
# and look a few degrees below the barrel.
CAM_X = PLATE_X / 2 - WING_X / 2   # foot centre, mm forward of the tilt axis (slots give +-8 mm)
CAM_Y = 57.0                       # side offset of the board centre from the barrel line (USB right, thermal left)
CAM_RISE = 23.0                    # board centre above the payload plate top (lowest value for a 32 mm board)
CAM_PITCH = 16.0                   # deg, camera looks this much below the barrel (camera study round 2)
# Concept B head and concept E valve: adapters on the payload pattern, used instead of the payload plate.
# The camera study found that a nozzle beside the cameras hides 11-18 % of the image, so on B it sits under them.
NOZZLE_D = 10.0                    # nozzle tube or hose-barb outer diameter, PLACEHOLDER until the nozzle is chosen
NOZZLE_OUT = 45.0                  # max. nozzle tip length in front of the head face: tip stays out of the image
CAM_VFOV = 74.0                    # deg, vertical field of view used for that check (90 deg HFOV, 4:3), GUESS
HEAD_CAM_DY = 21.0                 # RGB board centre at -y (right), thermal board centre at +y
HEAD_CAM_DZ = 62.0                 # camera centre above the nozzle axis (camera study round 2: nozzle 5 cm lower)
HEAD_CAM_PITCH = 15.0              # deg, cameras look down relative to the nozzle (camera study round 2)
HEAD_THERMAL = "cam_mlx90640"      # which thermal board the B head takes (a key of CAMS)
RGB_WINDOW, THERMAL_WINDOW = 17.0, 14.0   # lens holder / sensor windows, PLACEHOLDER (lens holder size not found)
# 12 V plastic solenoid valve (concept E): port-to-port length, body width, body height under the coil,
# port axis above the body bottom, inlet/outlet thread outer diameter.
# Adafruit 997 (12 V, G1/2): length 85, body width 33.5, body under the coil about 29 (scaled from the drawing).
# The port axis height is a GUESS (half the body). G1/2 thread outer diameter 20.96.
VALVE_L, VALVE_W, VALVE_H, VALVE_PORT_Z, VALVE_PORT_D = 85.0, 33.5, 29.0, 14.5, 21.0
VALVE_FITTING = 25.0               # outlet fitting length between the valve and the nozzle holder, PLACEHOLDER

BED = (220.0, 220.0, 250.0)
PLA = 1.24                       # g/cm3
SHELL, INFILL = 1.2, 0.40        # 3 walls x 0.4 mm, 40 % gyroid (mass estimate assumption)

# ---------------- derived heights ----------------
Z_SHELF = DECK_T                                 # pan servo flange top face touches the shelf underside
Z_BRG0 = Z_SHELF + HORN_TOP                      # pan horn top = hub bottom = bearing bottom
Z_BRG1 = Z_BRG0 + PAN_BRG[2]                     # housing top
Z_TT = Z_BRG1 + 1.0                              # turntable underside (1 mm over the housing)
Z_TT1 = Z_TT + TT_T
Z_TILT = Z_TT1 + (SV_FL_L / 2 + (SV_L / 2 - SV_SHAFT)) + 1.0   # lowest flange ear stays 1 mm over the turntable
Y_FLANGE = (SV_BELOW + SERVO_GAP + BCHEEK_T + 2.0 - HORN_TOP) / 2  # makes the U symmetric about y = 0
U_IN = 2 * (Y_FLANGE + HORN_TOP)                 # inner width of the U
Z_U0 = Z_TILT + CHEEK_R + U_GAP                  # underside of the U base
Z_U1 = Z_U0 + U_T


def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl(d, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def ycyl(d, y0, y1, x, z):
    """Cylinder along y from y0 to y1."""
    return Pos(x, y0, z) * (Rot(X=-90) * Cylinder(d / 2, y1 - y0, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def hexagon(af, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * extrude(RegularPolygon(af / 2, 6, major_radius=False), z1 - z0)


def slot(length, width, z0, z1, x=0.0, y=0.0, along_y=False):
    s = extrude(SlotOverall(length, width), z1 - z0)
    return Pos(x, y, z0) * (Rot(Z=90) * s if along_y else s)


def square(p):
    return [(sx * p / 2, sy * p / 2) for sx in (-1, 1) for sy in (-1, 1)]


# ---------------- servo (servo frame) ----------------
SV_C = SV_SHAFT - SV_L / 2   # body centre x
SV_HOLES = [(SV_C + sx * SV_HOLE_DX / 2, sy * SV_HOLE_DY / 2) for sx in (-1, 1) for sy in (-1, 1)]


def servo_window(c=SV_CLR, below=SV_BELOW, above=SV_ABOVE):
    return box(SV_C - SV_L / 2 - c, SV_C + SV_L / 2 + c, -SV_W / 2 - c, SV_W / 2 + c, -below - c, above + c)


def servo_model():
    s = box(SV_C - SV_L / 2, SV_C + SV_L / 2, -SV_W / 2, SV_W / 2, -SV_BELOW, SV_ABOVE)
    s += box(SV_C - SV_FL_L / 2, SV_C + SV_FL_L / 2, -SV_W / 2, SV_W / 2, -SV_FL_T, 0)
    s += cyl(6.0, SV_ABOVE, HORN_TOP - 4)                  # spline
    s += cyl(HORN_D, HORN_TOP - 4, HORN_TOP)               # disc horn
    for x, y in SV_HOLES:
        s -= cyl(4.5, -SV_FL_T - 1, 1, x, y)
    return s


def to_tilt(shape):
    """Servo frame -> tilt servo pose: shaft on +y, body hanging below the tilt axis."""
    return Pos(0, Y_FLANGE, Z_TILT) * (Rot(Y=-90) * (Rot(X=-90) * shape))


# ---------------- printed parts ----------------
DECK_X0 = min(-RAIL_X - RAIL_W / 2 - 5, PAN_X - SHELF / 2 - 5)   # deck may reach past the rear rail
DECK_X1 = max(RAIL_X + RAIL_W / 2 + 5, PAN_X + SHELF / 2 + 5)
PAN_X_MIN = -(RAIL_X - RAIL_W / 2) + SV_SHAFT + SV_CLR + 1.0      # servo body 1 mm in front of the rear rail


def deck():
    d = box(DECK_X0, DECK_X1, -DECK_Y / 2, DECK_Y / 2, 0, DECK_T)
    for x in (-RAIL_X, RAIL_X):           # M5 into drop-in T-nuts in the top slot of each rail
        for y in (-RAIL_SCREW_Y, RAIL_SCREW_Y):
            d -= cyl(M5_CLR, -1, DECK_T + 1, x, y)
    for x, y in square(PAN_PATTERN):      # M4 nuts dropped in from the top, under the pan base shelf
        d -= cyl(M4_CLR, -1, DECK_T + 1, PAN_X + x, y)
        d -= hexagon(M4_NUT_AF + FIT, DECK_T - M4_NUT_T - 0.3, DECK_T + 1, PAN_X + x, y)
    # pan servo flange sits in the deck, its body hangs into the torso frame (body points forward)
    d -= box(PAN_X - SV_C - SV_FL_L / 2 - 2, PAN_X - SV_C + SV_FL_L / 2 + 2, -SV_W / 2 - 2, SV_W / 2 + 2, -1, DECK_T + 1)
    d -= slot(40, 12, -1, DECK_T + 1, *DECK_SLOT)   # cable pass-through
    return d


def pan_base():
    z0, zc = Z_SHELF, Z_BRG0 - LIP          # zc = top of the cavity under the bearing seat
    p = box(-SHELF / 2, SHELF / 2, -SHELF / 2, SHELF / 2, z0, z0 + SHELF_T) + cyl(HOUSING_D, z0, Z_BRG1)
    cut = servo_window() + box(SV_C - SV_L / 2 - 1, SV_C + SV_L / 2 + 1, -SV_W / 2 - 1, SV_W / 2 + 1, 0, zc - z0)
    for x, y in SV_HOLES:                                                    # servo screws from below
        cut += cyl(M3_PILOT, -1, 10, x, y)
    p -= Pos(0, 0, z0) * (Rot(Z=180) * cut)                                  # servo body points forward
    p -= cyl(HORN_D + 3, z0 + SHELF_T, zc)                                   # horn turns here
    p -= cyl(PAN_BRG[1] - 5, zc, Z_BRG0 + 1)                                 # shoulder for the outer ring
    p -= cyl(PAN_BRG[1] + FIT, Z_BRG0, Z_BRG1 + 1)                           # bearing seat
    for x, y in square(PAN_PATTERN):
        p -= cyl(M4_CLR, z0 - 1, z0 + SHELF_T + 1, x, y)
    return p


def pan_yoke():
    """Turntable: hub in the 6808 bore, disc, servo cheek and bearing cheek for the tilt axis."""
    t = cyl(PAN_BRG[0] - 0.1, Z_BRG0, Z_BRG1) + cyl(PAN_BRG[0] + 4, Z_BRG1, Z_TT) + cyl(TT_D, Z_TT, Z_TT1)
    for x in (-HORN_BC / 2, HORN_BC / 2):    # 2 x M3 into the disc horn; on y = 0 the servo cheek does not cover them
        t -= cyl(M3_CLR, Z_BRG0 - 1, Z_TT1 + 1, x, 0)
    t -= cyl(6.5, Z_BRG0 - 1, Z_TT1 + 1)                                    # reach the horn centre screw
    # servo cheek: the flange rests on its +y face
    x_half = SV_W / 2 + 6
    ys0, ys1 = Y_FLANGE - SV_FL_T - CHEEK_T, Y_FLANGE - SV_FL_T
    # (the flange sits on y in [Y_FLANGE - SV_FL_T, Y_FLANGE]; the cheek is just below it)
    cheek = box(-x_half, x_half, ys0, ys1, Z_TT1 - 1, Z_TILT) + ycyl(2 * CHEEK_R, ys0, ys1, 0, Z_TILT)
    t += cheek
    t -= to_tilt(servo_window(above=SV_FL_T + SV_CLR))
    for x, y in SV_HOLES:
        t -= to_tilt(Pos(x, y, -SV_FL_T - CHEEK_T - 1) * Cylinder(M3_PILOT / 2, CHEEK_T + 2, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    # bearing cheek on the far side of the servo bottom
    yb1 = Y_FLANGE - SV_BELOW - SERVO_GAP
    yb0 = yb1 - BCHEEK_T
    t += box(-x_half, x_half, yb0, yb1, Z_TT1 - 1, Z_TILT) + ycyl(2 * CHEEK_R, yb0, yb1, 0, Z_TILT)
    t -= ycyl(TILT_BRG[1] + FIT, yb0 - 1, yb0 + TILT_BRG[2], 0, Z_TILT)     # 608 pressed in from -y
    t -= ycyl(TILT_BRG[1] - 4, yb0, yb1 + 1, 0, Z_TILT)                        # lip leaves the inner ring free
    # side ribs tie the two cheeks together
    for sx in (-1, 1):
        t += box(sx * x_half - (3 if sx > 0 else 0), sx * x_half + (0 if sx > 0 else 3), yb0, ys1, Z_TT1 - 1, Z_TT1 + 12)
    return t


def tilt_u():
    y_in = U_IN / 2
    x_half = PAYLOAD_PATTERN / 2 + 8
    u = box(-x_half, x_half, -y_in - ARM_T, y_in + ARM_T, Z_U0, Z_U1)
    for sy in (-1, 1):
        y0, y1 = (y_in, y_in + ARM_T) if sy > 0 else (-y_in - ARM_T, -y_in)
        u += box(-ARM_R, ARM_R, y0, y1, Z_TILT, Z_U1) + ycyl(2 * ARM_R, y0, y1, 0, Z_TILT)
    # +y arm: disc horn screws and a hole for the horn centre screw
    u -= ycyl(6.5, y_in - 1, y_in + ARM_T + 1, 0, Z_TILT)
    for a in range(4):
        dx, dz = HORN_BC / 2 * math.cos(math.radians(45 + 90 * a)), HORN_BC / 2 * math.sin(math.radians(45 + 90 * a))
        u -= ycyl(M3_CLR, y_in - 1, y_in + ARM_T + 1, dx, Z_TILT + dz)
    # -y arm: M8 x 16 hex bolt is the tilt pin; its head sits in a hex pocket on the outside
    u -= ycyl(8.4, -y_in - ARM_T - 1, -y_in + 1, 0, Z_TILT)
    u -= Pos(0, -y_in - ARM_T - 0.01, Z_TILT) * (Rot(X=-90) * extrude(RegularPolygon(13.2 / 2, 6, major_radius=False), 5.5))
    for x, y in square(PAYLOAD_PATTERN):
        u -= cyl(M4_CLR, Z_U0 - 1, Z_U1 + 1, x, y)
        u -= hexagon(M4_NUT_AF + FIT, Z_U0 - 1, Z_U0 + M4_NUT_T, x, y)
    return u


def payload_plate():
    z0, z1 = Z_U1, Z_U1 + PLATE_T
    x1 = PLATE_X / 2
    p = box(-x1, x1, -PLATE_Y / 2, PLATE_Y / 2, z0, z1)
    p += box(x1 - WING_X, x1, -CAM_Y - 22, CAM_Y + 22, z0, z1)              # front wing for the cameras
    wy = GUN_W / 2 + 1
    for sy in (-1, 1):                       # low side walls keep the gun centred
        p += box(-x1 + 15, x1 - WING_X - 5, sy * wy - (0 if sy > 0 else 4), sy * wy + (4 if sy > 0 else 0), z1, z1 + 12)
    for sy in (-1, 1):                       # notch on each wall top: put the gun CoM (with water) over it
        p -= Pos(GUN_COM_DX, sy * (wy + 2), z1 + 12) * (Rot(X=90) * extrude(RegularPolygon(3.0, 3, rotation=-90), 6, both=True))
    for x, y in square(PAYLOAD_PATTERN):     # M4 button heads sit in 2.5 mm counterbores
        p -= cyl(M4_CLR, z0 - 1, z1 + 1, x, y)
        p -= cyl(9.0, z1 - 2.5, z1 + 1, x, y)
    for x in (-45, 45):                      # strap slots just outside the walls, symmetric about the tilt axis
        for sy in (-1, 1):
            p -= slot(STRAP_W + 4, 4.0, z0 - 1, z1 + 1, x, sy * (wy + 8))
    for x in (-60, 60):                      # M4 slots on the centre line for gun clamps
        p -= slot(20, M4_CLR, z0 - 1, z1 + 1, x, 0)
    for sy in (-1, 1):                       # camera bracket slots (M3), fore-aft adjustable
        for dy in (-4, 4):
            p -= slot(24, M3_CLR, z0 - 1, z1 + 1, CAM_X, sy * CAM_Y + dy)
    return p


# Camera boards: (board w along y, board h along z, holes [(y, z) from board centre], hole clearance)
CAMS = {
    "cam_usb32": (32.0, 32.0, square(28.0), M2_CLR),                                 # Arducam UB0234 class
    "cam_mlx90640": (25.4, 17.78, [(sx * 10.16, sz * 6.35) for sx in (-1, 1) for sz in (-1, 1)], M2_CLR),  # Adafruit 4407
    "cam_purethermal": (22.0, 30.0, [(sx * 9.0, sz * 10.755) for sx in (-1, 1) for sz in (-1, 1)], M2_CLR),  # PureThermal 2
}
def cam_mount(w, h, holes, clr, y=0.0, board=False):
    """L bracket: foot with 2 x M3 bolted to the plate slots, upright with stand-offs facing +x,
    leaned forward by CAM_PITCH. board=True returns a dummy board in the same pose (for renders)."""
    z0 = Z_U1 + PLATE_T
    foot = 30.0
    width = max(w, 24.0) + 6
    zc = z0 + CAM_RISE                            # board centre height before the lean
    assert CAM_RISE >= 4 + h / 2 + 3, "board would hit the foot"
    xf = CAM_X + foot / 2
    lean = Pos(xf, 0, z0) * Rot(Y=CAM_PITCH) * Pos(-xf, 0, -z0)
    if board:
        b = box(xf + 4, xf + 5.6, y - w / 2, y + w / 2, zc - h / 2, zc + h / 2)
        return lean * (b + Pos(xf + 5.6, y, zc) * (Rot(Y=90) * Cylinder(6, 10, align=(Align.CENTER, Align.CENTER, Align.MIN))))
    up = box(xf - 4, xf, y - width / 2, y + width / 2, z0, zc + h / 2 + 3)
    for hy, hz in holes:
        up += Pos(xf, y + hy, zc + hz) * (Rot(Y=90) * Cylinder(3.25, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    m = box(CAM_X - foot / 2, xf, y - width / 2, y + width / 2, z0, z0 + 4) + lean * up
    for sy in (-1, 1):                            # gussets
        m += box(CAM_X - foot / 2 + 6, xf - 4, y + sy * (width / 2 - 1.5) - 1.5, y + sy * (width / 2 - 1.5) + 1.5, z0, z0 + 14)
    for dy in (-4.0, 4.0):
        m -= cyl(M3_CLR, z0 - 1, z0 + 5, CAM_X - 4, y + dy)
    for hy, hz in holes:
        m -= lean * (Pos(xf - 5, y + hy, zc + hz) * (Rot(Y=90) * Cylinder(clr / 2, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))))
    return m


def pattern_base(x0, x1, y0, y1, t=5.0):
    """Plate on the U top with the payload pattern; M4 button heads in 2.5 mm counterbores."""
    z0 = Z_U1
    b = box(x0, x1, y0, y1, z0, z0 + t)
    for x, y in square(PAYLOAD_PATTERN):
        b -= cyl(M4_CLR, z0 - 1, z0 + t + 1, x, y)
        b -= cyl(9.0, z0 + t - 2.5, z0 + t + 1, x, y)
    return b


def xbore(d, x0, x1, y, z):
    return Pos(x0, y, z) * (Rot(Y=90) * Cylinder(d / 2, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def head_b():
    """Concept B head: a camera box (RGB and thermal boards side by side) pitched HEAD_CAM_PITCH down,
    on a column that holds the nozzle centred under it. Open back for the hose and cables, roof against drips."""
    t, xf, xb, yw = 3.0, 30.0, -12.0, 42.0      # wall, front face x, back x, half width
    zb = Z_U1 + 5
    zn = zb + NOZZLE_D / 2 + 4                   # nozzle axis
    hb = 32.0 + 2 * 5                            # camera box height
    zp = zn + HEAD_CAM_DZ - hb / 2               # camera box bottom (before the pitch)
    h = pattern_base(-30, xf, -yw, yw)
    h += box(xb, xf, -NOZZLE_D / 2 - 4, NOZZLE_D / 2 + 4, zb, zp)                   # nozzle column
    for sy in (-1, 1):                                                             # side ribs
        h += box(xb, xf, sy * yw - (t if sy > 0 else 0), sy * yw + (0 if sy > 0 else t), zb, zp)
    wedge = (xf - xb) * math.tan(math.radians(HEAD_CAM_PITCH)) + 1                 # fills the gap under the pitched box
    cam = box(xb, xf, -yw, yw, zp - wedge, zp + t)                                 # bottom
    cam += box(xf - t, xf, -yw, yw, zp - wedge, zp + hb)                           # front wall
    cam += box(xb, xf, -yw, yw, zp + hb - t, zp + hb)                              # roof
    for sy in (-1, 1):
        cam += box(xb, xf, sy * yw - (t if sy > 0 else 0), sy * yw + (0 if sy > 0 else t), zp - wedge, zp + hb)
    zc = zp + hb / 2
    boards = [(-HEAD_CAM_DY, CAMS["cam_usb32"], RGB_WINDOW, True), (HEAD_CAM_DY, CAMS[HEAD_THERMAL], THERMAL_WINDOW, False)]
    for y, (w, hh, holes, clr), win, square_win in boards:
        for hy, hz in holes:                                                       # 2 mm stand-offs inside
            cam += xbore(6.5, xf - t - 2, xf - t, y + hy, zc + hz)
            cam -= xbore(clr, xf - t - 3, xf + 1, y + hy, zc + hz)
        cam -= box(xf - t - 1, xf + 1, y - win / 2, y + win / 2, zc - win / 2, zc + win / 2) if square_win \
            else xbore(win, xf - t - 1, xf + 1, y, zc)
    h += Pos(xf, 0, zp) * Rot(Y=HEAD_CAM_PITCH) * Pos(-xf, 0, -zp) * cam
    h -= xbore(NOZZLE_D + 0.3, xb - 1, xf + 1, 0, zn)
    h -= cyl(M3_PILOT, zn, zn + NOZZLE_D / 2 + 5, 10, 0)                            # M3 set screw holds the nozzle
    return h & box(-60, 60, -60, 60, Z_U1, Z_U1 + 200)                             # trim anything under the base


def valve_e():
    """Concept E: cradle for a 12 V solenoid valve (inlet at the back, coil up), nozzle holder in front."""
    yw = VALVE_W / 2 + 0.5
    zb = Z_U1 + 5
    x_out = VALVE_L / 2 + VALVE_FITTING            # nozzle holder back face
    zp = zb + VALVE_PORT_Z
    v = pattern_base(-VALVE_L / 2 - 5, x_out + 8, -yw - 12, yw + 12)
    for sy in (-1, 1):                             # side walls up to the port axis, between the pattern screws
        v += box(-14, 14, sy * yw - (0 if sy > 0 else 4), sy * yw + (4 if sy > 0 else 0), zb, zp)
    for x in (-VALVE_L / 2 + 8, VALVE_L / 2 - 8):   # zip-tie slots, clear of the pattern screws
        for sy in (-1, 1):
            v -= slot(10, 4.0, zb - 6, zb + 1, x, sy * (yw + 8))
    v += box(x_out, x_out + 8, -NOZZLE_D / 2 - 6, NOZZLE_D / 2 + 6, zb, zp + NOZZLE_D / 2 + 6)   # nozzle holder
    v -= xbore(NOZZLE_D + 0.3, x_out - 1, x_out + 9, 0, zp)
    v -= cyl(M3_PILOT, zp, zp + NOZZLE_D / 2 + 7, x_out + 4, 0)
    return v


PARTS = {  # everything but the deck is drawn around the pan axis and moved to x = PAN_X here
    "deck": deck,
    "pan_base": lambda: Pos(PAN_X, 0, 0) * pan_base(),
    "pan_yoke": lambda: Pos(PAN_X, 0, 0) * pan_yoke(),
    "tilt_u": lambda: Pos(PAN_X, 0, 0) * tilt_u(),
    "payload_plate": lambda: Pos(PAN_X, 0, 0) * payload_plate(),
    "cam_usb32": lambda: Pos(PAN_X, 0, 0) * cam_mount(*CAMS["cam_usb32"], y=-CAM_Y),
    "cam_mlx90640": lambda: Pos(PAN_X, 0, 0) * cam_mount(*CAMS["cam_mlx90640"], y=CAM_Y),
    "cam_purethermal": lambda: Pos(PAN_X, 0, 0) * cam_mount(*CAMS["cam_purethermal"], y=CAM_Y),
    "head_b": lambda: Pos(PAN_X, 0, 0) * head_b(),
    "valve_e": lambda: Pos(PAN_X, 0, 0) * valve_e(),
}
ALTERNATIVES = ("cam_purethermal", "head_b", "valve_e")   # not in the concept A assembly
# Print jobs on a 220 x 220 bed: (part, x, y) of the footprint corner, footprint = part bounding box in x and y.
PRINT_JOBS = {
    "J1": [("payload_plate", 5, 5), ("cam_usb32", 5, 170), ("cam_mlx90640", 60, 170)],
    "J2": [("deck", 5, 5), ("tilt_u", 5, 140)],
    "J3": [("pan_base", 5, 5), ("pan_yoke", 92, 5)],
    "J4": [("head_b", 5, 5), ("valve_e", 5, 100)],
}
SUPPORT_G = {"pan_yoke": 4, "head_b": 8}   # support material, GUESS


def check_jobs(shapes, gap=3.0):
    grams = {}
    for job, items in PRINT_JOBS.items():
        rects = []
        for n, x, y in items:
            bb = shapes[n].bounding_box()
            r = (x, y, x + bb.size.X, y + bb.size.Y)
            assert r[2] <= BED[0] and r[3] <= BED[1], f"{job}: {n} leaves the bed"
            for q in rects:
                assert r[0] >= q[2] + gap or q[0] >= r[2] + gap or r[1] >= q[3] + gap or q[1] >= r[3] + gap, f"{job}: {n} overlaps"
            rects.append(r)
        grams[job] = sum(mass(shapes[n])[0] + SUPPORT_G.get(n, 0) for n, _, _ in items)
    return grams


# bought parts for the CoM estimate, kg (GUESSES: typical listing values, not weighed)
REF_MASS = {"ref_pan_servo": 0.060, "ref_tilt_servo": 0.060, "ref_pan_bearing": 0.026, "ref_tilt_bearing": 0.012,
            "ref_tilt_pin": 0.010, "ref_cam_usb32": 0.010, "ref_cam_thermal": 0.005, "ref_gun": GUN_MASS}
TILTING = ["tilt_u", "payload_plate", "cam_usb32", "cam_mlx90640", "cam_purethermal", "ref_gun"]


def refs():
    """Bought parts, for the renders only."""
    bx, bd, bw = PAN_BRG
    tb = TILT_BRG
    yb0 = Y_FLANGE - SV_BELOW - SERVO_GAP - BCHEEK_T
    return {k: Pos(PAN_X, 0, 0) * v for k, v in {
        "ref_pan_servo": Pos(0, 0, Z_SHELF) * (Rot(Z=180) * servo_model()),
        "ref_tilt_servo": to_tilt(servo_model()),
        "ref_pan_bearing": cyl(bd, Z_BRG0, Z_BRG1) - cyl(bx, Z_BRG0 - 1, Z_BRG1 + 1),
        "ref_tilt_bearing": ycyl(tb[1], yb0, yb0 + tb[2], 0, Z_TILT) - ycyl(tb[0], yb0 - 1, yb0 + tb[2] + 1, 0, Z_TILT),
        "ref_tilt_pin": ycyl(8.0, -U_IN / 2 - ARM_T + 5.5, -U_IN / 2 - ARM_T + 21.5, 0, Z_TILT)
        + Pos(0, -U_IN / 2 - ARM_T, Z_TILT) * (Rot(X=-90) * extrude(RegularPolygon(13.0 / 2, 6, major_radius=False), 5.3)),
        "ref_gun": box(GUN_COM_DX - 120, GUN_COM_DX + 120, -GUN_W / 2, GUN_W / 2, Z_U1 + PLATE_T, Z_U1 + PLATE_T + 85),
        "ref_cam_usb32": cam_mount(*CAMS["cam_usb32"], y=-CAM_Y, board=True),
        "ref_cam_thermal": cam_mount(*CAMS["cam_mlx90640"], y=CAM_Y, board=True),
    }.items()}


def mass(shape):
    """PLA mass estimate: solid shell of SHELL mm on every surface, INFILL inside. Grams."""
    v, a = shape.volume / 1000, shape.area / 100              # cm3, cm2
    shell = min(v, a * SHELL / 10)
    return PLA * (shell + INFILL * (v - shell)), PLA * v


def check(shapes, refs):
    import itertools
    for name, s in shapes.items():
        bb = s.bounding_box()
        size = sorted([bb.size.X, bb.size.Y, bb.size.Z])
        assert all(a <= b for a, b in zip(size, sorted(BED))), f"{name} does not fit the bed: {size}"
    # servo window = servo body + clearance on each side
    w = servo_window().bounding_box()
    assert abs(w.size.X - (SV_L + 2 * SV_CLR)) < 1e-6 and abs(w.size.Y - (SV_W + 2 * SV_CLR)) < 1e-6
    assert SV_FL_L > SV_L + 2 * 4, "flange ears must reach past the window"
    nozzle_deg = math.degrees(math.atan2(HEAD_CAM_DZ, NOZZLE_OUT))
    assert nozzle_deg > HEAD_CAM_PITCH + CAM_VFOV / 2, f"B nozzle tip is in the camera image ({nozzle_deg:.0f} deg)"
    assert PAN_X >= PAN_X_MIN, f"pan servo body hits the rear rail: PAN_X must be >= {PAN_X_MIN:.1f}"
    # minimum walls from the parameters (mm)
    walls = {
        "bearing housing": (HOUSING_D - PAN_BRG[1]) / 2,
        "cavity to housing": HOUSING_D / 2 - math.hypot(SV_C - SV_L / 2 - 1, SV_W / 2 + 1),
        "hub ring under horn screws": (PAN_BRG[0] - HORN_BC - M3_CLR) / 2,
        "lip under 608": BCHEEK_T - TILT_BRG[2],
        "nut pocket floor (deck)": DECK_T - M4_NUT_T - 0.3,
        "nut pocket floor (U)": U_T - M4_NUT_T,
        "plate under counterbore": PLATE_T - 2.5,
        "M8 pocket floor": ARM_T - 5.5,
    }
    for k, v in walls.items():
        assert v >= 2.0, f"wall {k} = {v:.2f} mm < 2 mm"
    # no part may overlap another part or a bought part in the assembly (tilt = 0)
    both = {n: s for n, s in {**shapes, **refs}.items() if n not in ALTERNATIVES + ("ref_gun",)}
    for (a, sa), (b, sb) in itertools.combinations(both.items(), 2):
        v = (sa & sb).volume
        assert v < 1e-3, f"{a} overlaps {b} by {v:.2f} mm3"
    return walls


def wall_scan(stl, n=4000):
    """Measured wall: from points on the surface, cast a ray inward and take the distance to the far side (mm)."""
    import numpy as np
    import trimesh
    m = trimesh.load(stl)
    pts, fi = trimesh.sample.sample_surface_even(m, n, seed=1)
    nrm = m.face_normals[fi]
    o = pts - nrm * 1e-3
    loc, ray, _ = m.ray.intersects_location(o, -nrm, multiple_hits=False)
    return float(np.linalg.norm(loc - o[ray], axis=1).min())


if __name__ == "__main__":
    (OUT / "ref").mkdir(parents=True, exist_ok=True)
    shapes = {n: f() for n, f in PARTS.items()}
    info = {"tilt_axis_mm": [PAN_X, 0.0, Z_TILT], "pan_axis_mm": [PAN_X, 0.0], "tilting": TILTING, "alternatives": list(ALTERNATIVES), "parts": {}}
    for n, s in shapes.items():
        export_step(s, str(OUT / f"{n}.step"))
        export_stl(s, str(OUT / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.1)
        bb = s.bounding_box()
        g, g_solid = mass(s)
        info["parts"][n] = {"volume_cm3": round(s.volume / 1000, 1), "mass_g": round(g), "mass_solid_g": round(g_solid),
                            "bbox_mm": [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)]}
        print(f"{n:16s} V={s.volume / 1000:6.1f} cm3  m~{g:5.0f} g (solid {g_solid:4.0f} g)  bbox {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}")
    ref = refs()
    for n, s in ref.items():
        export_stl(s, str(OUT / "ref" / f"{n}.stl"), tolerance=0.05, angular_tolerance=0.2)
    walls = check(shapes, ref)
    info["min_walls_mm"] = {k: round(v, 2) for k, v in walls.items()}
    info["print_jobs_g"] = {k: round(v) for k, v in check_jobs(shapes).items()}
    print("print jobs (g):", info["print_jobs_g"])
    for n in shapes:
        w = wall_scan(OUT / f"{n}.stl")
        info["parts"][n]["min_wall_scan_mm"] = round(w, 2)
        assert w >= 1.95, f"{n}: thinnest wall found {w:.2f} mm"
    info["gun_com_dx_mm"] = GUN_COM_DX
    info["heights_mm"] = {"shelf": Z_SHELF, "pan_bearing": [Z_BRG0, Z_BRG1], "turntable": [Z_TT, Z_TT1],
                          "tilt_axis": Z_TILT, "u_base": [Z_U0, Z_U1], "plate_top": Z_U1 + PLATE_T}
    # centre of mass in the robot frame (m), tilt = 0; printed parts use the estimated mass at their centroid
    pts = {n: (info["parts"][n]["mass_g"] / 1000, s.center(CenterOf.MASS)) for n, s in shapes.items() if n not in ALTERNATIVES}
    pts.update({n: (REF_MASS[n], s.center(CenterOf.MASS)) for n, s in ref.items()})
    for key, sel in (("com_no_gun", [v for n, v in pts.items() if n != "ref_gun"]), ("com_with_gun", list(pts.values()))):
        m = sum(w for w, _ in sel)
        c = [sum(w * (v.X, v.Y, v.Z)[i] for w, v in sel) / m / 1000 for i in range(3)]
        info[key] = {"mass_kg": round(m, 3), "xyz_m": [round(c[0], 3), round(c[1], 3), round(c[2] + 0.791, 3)]}
    print("CoM", info["com_no_gun"], info["com_with_gun"])
    (OUT / "assembly.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info["heights_mm"]), "\nchecks passed")
