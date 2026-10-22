"""Top module variant for two Feetech STS3215 (Waveshare ST3215) serial bus servos.

    uv run --project cad python cad/parts/top_module_sts3215.py

Reuses the deck, payload plate and camera brackets of top_module.py (same frame, same 4 x M4 / 40 mm
payload pattern). Redesigns the pan base, pan yoke and tilt U for a servo without mounting ears: the STS3215
is held by screws into its two horn faces, and the U takes the output horn on one side and the idler horn
on the other side. Writes out_sts3215/ and runs the same kind of checks as top_module.py.
"""
import itertools
import json
import math
from pathlib import Path

from build123d import CenterOf, Pos, Rot, export_step, export_stl

import top_module as tm
from top_module import box, cyl, hexagon, square, ycyl

OUT = Path(__file__).parent / "out_sts3215"

# STS3215 / ST3215, Waveshare 2D drawing and STEP model (https://www.waveshare.com/wiki/ST3215_Servo).
# Servo frame: shaft on +z, origin on the shaft axis in the output case face, body along -x from the shaft.
ST_L, ST_W, ST_H = 45.22, 24.72, 32.0     # body length, width, height between the two horn faces
ST_SHAFT = 10.11                         # shaft axis to the near body end
ST_HORN_D, ST_HORN_H = 19.2, 2.9         # horn disc; height above the case face from the STEP (37.8 - 32) / 2
ST_HORN_BC = 14.0                        # 4 holes on a 14 mm circle (drawing)
ST_TOP_HOLES_X = -(42.86 - ST_SHAFT)     # far holes in the output face (drawing: 18.41 + 24.45 from the near end)
ST_BOT_HOLES_X = -(39.11 - ST_SHAFT)     # far holes in the idler face (drawing: 45.22 - 6.11 from the near end)
ST_HOLES_DY = 20.5                       # hole spacing across the width (drawing, output face; idler face GUESS)
ST_SCREW_CLR = 2.4                       # case screws M2, GUESS (the drawing gives no thread)
ST_CLR = 0.4
# The near holes (18.41 mm from the near end) are only 2.7 mm from the horn edge, so only the far holes are used.

Z0 = tm.DECK_T                           # output face of the pan servo = underside of the pan base shelf
SHELF_T = 5.0
Z_BRG0 = Z0 + SHELF_T                    # the 6808 outer ring sits on the shelf top
Z_BRG1 = Z_BRG0 + tm.PAN_BRG[2]
Z_TT, TT_T = Z_BRG1 + 1.0, 5.0
Z_TT1 = Z_TT + TT_T
HEAD_GAP = 3.5                           # pan horn screw heads under the tilt servo
Z_TILT = Z_TT1 + (ST_L - ST_SHAFT) + ST_CLR + HEAD_GAP
Y_FACE = ST_H / 2                        # tilt servo output face at +y, idler face at -y
U_IN = ST_H + 2 * ST_HORN_H              # between the two horn faces
TAB_T = 2.0                              # yoke tabs on the servo faces; the U arms pass 0.9 mm outside them
ARM_T, ARM_R, U_T = 7.0, 13.0, 6.0     # ARM_T 7 keeps 2 mm beside the M4 nut pockets
Z_U0 = Z_TILT + math.hypot(ST_W / 2, ST_SHAFT) + 3.0   # clear of the servo top corners when tilting
Z_U1 = Z_U0 + U_T
DZ = Z_U1 - tm.Z_U1                      # payload plate and camera brackets move down by this


def to_pan(shape):
    """Servo frame -> pan servo pose: shaft up on the pan axis, output face under the shelf, body forward."""
    return Pos(0, 0, Z0) * (Rot(Z=180) * shape)


def to_tilt(shape):
    """Servo frame -> tilt servo pose: shaft on +y, body hanging below the tilt axis (same turn as top_module)."""
    return Pos(0, Y_FACE, Z_TILT) * (Rot(Y=-90) * (Rot(X=-90) * shape))


def st_body(c=0.0):
    return box(ST_SHAFT - ST_L - c, ST_SHAFT + c, -ST_W / 2 - c, ST_W / 2 + c, -ST_H - c, c)


def st_model():
    return st_body() + cyl(ST_HORN_D, 0, ST_HORN_H) + cyl(ST_HORN_D, -ST_H - ST_HORN_H, -ST_H)


def horn_holes(z0, z1):
    s = cyl(6.5, z0, z1)
    for a in range(4):
        r = math.radians(45 + 90 * a)
        s += cyl(tm.M3_CLR, z0, z1, ST_HORN_BC / 2 * math.cos(r), ST_HORN_BC / 2 * math.sin(r))
    return s


def deck():
    """top_module deck plus a larger opening for the STS3215 body and the end wall of the pan base."""
    x0 = tm.PAN_X - ST_SHAFT - 2
    x1 = tm.PAN_X + (ST_L - ST_SHAFT) + ST_CLR + 3 + 2
    return tm.deck() - box(x0, x1, -ST_W / 2 - 4, ST_W / 2 + 4, -1, tm.DECK_T + 1)


def pan_base():
    """Shelf on the deck pattern, 6808 ring on top. The servo hangs under the shelf: 2 screws from above into
    its output face, 2 screws from below through a foot into its idler face. An end wall joins shelf and foot."""
    s = tm.SHELF
    p = box(-s / 2, s / 2, -s / 2, s / 2, Z0, Z0 + SHELF_T) + cyl(tm.PAN_BRG[1] + 10, Z_BRG0, Z_BRG1)
    p -= cyl(tm.PAN_BRG[1] + tm.FIT, Z_BRG0, Z_BRG1 + 1)                     # bearing seat on the shelf top
    p -= cyl(tm.PAN_BRG[0] + 7, Z_BRG0 - 0.6, Z_BRG0 + 1)                     # inner ring runs free
    p -= cyl(ST_HORN_D + 5, Z0 - 1, Z_BRG0 + 1)                               # horn and hub boss
    for x, y in square(tm.PAN_PATTERN):
        p -= cyl(tm.M4_CLR, Z0 - 1, Z0 + SHELF_T + 1, x, y)
    xe = ST_L - ST_SHAFT + ST_CLR                                             # far end of the body (pan frame, +x)
    zf = Z0 - ST_H                                                            # idler face
    p += box(xe, xe + 3, -ST_W / 2 - 2, ST_W / 2 + 2, zf - 3, Z0 + SHELF_T)   # end wall
    p += box(-ST_BOT_HOLES_X - 5, xe + 3, -ST_W / 2 - 2, ST_W / 2 + 2, zf - 3, zf)   # foot
    p -= box(xe - 1, xe + 4, -8, 8, zf + 6, Z0 - 6)                           # cable window in the end wall
    for sy in (-1, 1):
        p -= cyl(ST_SCREW_CLR, Z0 - 1, Z0 + SHELF_T + 1, -ST_TOP_HOLES_X, sy * ST_HOLES_DY / 2)
        p -= cyl(ST_SCREW_CLR, zf - 4, zf + 1, -ST_BOT_HOLES_X, sy * ST_HOLES_DY / 2)
    return p


def pan_yoke():
    """Turntable: hub in the 6808, boss down to the pan horn, disc, two thin tabs that hold the tilt servo
    by the far holes of its two horn faces."""
    t = cyl(tm.PAN_BRG[0] - 0.1, Z_BRG0, Z_BRG1) + cyl(22.0, Z0 + ST_HORN_H, Z_BRG0)
    t += cyl(tm.PAN_BRG[0] + 4, Z_BRG1, Z_TT) + cyl(tm.TT_D, Z_TT, Z_TT1)
    t -= horn_holes(Z0 + ST_HORN_H - 1, Z_TT1 + 1)
    zt, zb = Z_TILT + ST_TOP_HOLES_X, Z_TILT + ST_BOT_HOLES_X                 # hole heights in each face
    for y0, y1, zh in ((Y_FACE, Y_FACE + TAB_T, zt), (-Y_FACE - TAB_T, -Y_FACE, zb)):
        t += box(-ST_W / 2 - 2, ST_W / 2 + 2, y0, y1, Z_TT1 - 1, zh + ST_SCREW_CLR / 2 + 3)
        for sx in (-1, 1):
            t -= ycyl(ST_SCREW_CLR, y0 - 1, y1 + 1, sx * ST_HOLES_DY / 2, zh)
    return t


def tilt_u():
    """U with both arms on the servo horns (output +y, idler -y); payload pattern on top, nuts from below."""
    y_in, x_half = U_IN / 2, tm.PAYLOAD_PATTERN / 2 + 8
    u = box(-x_half, x_half, -y_in - ARM_T, y_in + ARM_T, Z_U0, Z_U1)
    for y0, y1 in ((y_in, y_in + ARM_T), (-y_in - ARM_T, -y_in)):
        u += box(-ARM_R, ARM_R, y0, y1, Z_TILT, Z_U1) + ycyl(2 * ARM_R, y0, y1, 0, Z_TILT)
        u -= Pos(0, y0 - 1, Z_TILT) * (Rot(X=-90) * horn_holes(0, ARM_T + 2))
    for x, y in square(tm.PAYLOAD_PATTERN):
        u -= cyl(tm.M4_CLR, Z_U0 - 1, Z_U1 + 1, x, y)
        u -= hexagon(tm.M4_NUT_AF + tm.FIT, Z_U0 - 1, Z_U0 + tm.M4_NUT_T, x, y)
    return u


def at_pan(f):
    return lambda: Pos(tm.PAN_X, 0, 0) * f()


PARTS = {
    "deck": deck,
    "pan_base": at_pan(pan_base),
    "pan_yoke": at_pan(pan_yoke),
    "tilt_u": at_pan(tilt_u),
    "payload_plate": lambda: Pos(0, 0, DZ) * tm.PARTS["payload_plate"](),
    "cam_usb32": lambda: Pos(0, 0, DZ) * tm.PARTS["cam_usb32"](),
    "cam_mlx90640": lambda: Pos(0, 0, DZ) * tm.PARTS["cam_mlx90640"](),
}


def refs():
    bd, bx = tm.PAN_BRG[1], tm.PAN_BRG[0]
    r = {
        "ref_pan_servo": to_pan(st_model()),
        "ref_tilt_servo": to_tilt(st_model()),
        "ref_pan_bearing": cyl(bd, Z_BRG0, Z_BRG1) - cyl(bx, Z_BRG0 - 1, Z_BRG1 + 1),
        "ref_gun": Pos(0, 0, DZ) * tm.refs()["ref_gun"],
    }
    r = {k: Pos(tm.PAN_X, 0, 0) * v if k != "ref_gun" else v for k, v in r.items()}
    for k in ("ref_cam_usb32", "ref_cam_thermal"):
        r[k] = Pos(0, 0, DZ) * tm.refs()[k]
    return r


def check(shapes, ref):
    for n, s in shapes.items():
        size = sorted([s.bounding_box().size.X, s.bounding_box().size.Y, s.bounding_box().size.Z])
        assert all(a <= b for a, b in zip(size, sorted(tm.BED))), f"{n} does not fit the bed"
    assert tm.PAN_X - ST_SHAFT - ST_CLR >= -(tm.RAIL_X - tm.RAIL_W / 2) + 1, "pan servo hits the rear rail"
    assert U_IN / 2 - Y_FACE - TAB_T >= 0.5, "tabs touch the U arms"
    both = {n: s for n, s in {**shapes, **ref}.items() if n != "ref_gun"}
    for (a, sa), (b, sb) in itertools.combinations(both.items(), 2):
        v = (sa & sb).volume
        assert v < 1e-3, f"{a} overlaps {b} by {v:.2f} mm3"


if __name__ == "__main__":
    (OUT / "ref").mkdir(parents=True, exist_ok=True)
    shapes = {n: f() for n, f in PARTS.items()}
    ref = refs()
    info = {"tilt_axis_mm": [tm.PAN_X, 0.0, Z_TILT], "pan_axis_mm": [tm.PAN_X, 0.0], "tilting": tm.TILTING,
            "alternatives": [], "parts": {}, "gun_com_dx_mm": tm.GUN_COM_DX}
    for n, s in shapes.items():
        export_step(s, str(OUT / f"{n}.step"))
        export_stl(s, str(OUT / f"{n}.stl"), tolerance=0.02, angular_tolerance=0.1)
        g, g_solid = tm.mass(s)
        w = tm.wall_scan(OUT / f"{n}.stl")
        bb = s.bounding_box()
        info["parts"][n] = {"volume_cm3": round(s.volume / 1000, 1), "mass_g": round(g), "mass_solid_g": round(g_solid),
                            "bbox_mm": [round(bb.size.X, 1), round(bb.size.Y, 1), round(bb.size.Z, 1)], "min_wall_scan_mm": round(w, 2)}
        print(f"{n:14s} V={s.volume / 1000:6.1f} cm3  m~{g:4.0f} g  wall {w:.2f}  bbox {bb.size.X:.0f} x {bb.size.Y:.0f} x {bb.size.Z:.0f}")
        assert w >= 1.95, f"{n}: thinnest wall found {w:.2f} mm"
    for n, s in ref.items():
        export_stl(s, str(OUT / "ref" / f"{n}.stl"), tolerance=0.05, angular_tolerance=0.2)
    check(shapes, ref)
    info["heights_mm"] = {"pan_bearing": [Z_BRG0, Z_BRG1], "turntable": [Z_TT, Z_TT1], "tilt_axis": Z_TILT,
                          "u_base": [Z_U0, Z_U1], "plate_top": Z_U1 + tm.PLATE_T}
    # centre of mass in the robot frame (m), tilt = 0, same method and guessed masses as top_module.py
    ref_mass = dict(tm.REF_MASS, ref_pan_servo=0.089, ref_tilt_servo=0.089)    # STS3215: 89 g (Waveshare)
    pts = {n: (info["parts"][n]["mass_g"] / 1000, s.center(CenterOf.MASS)) for n, s in shapes.items()}
    pts.update({n: (ref_mass[n], s.center(CenterOf.MASS)) for n, s in ref.items()})
    for key, sel in (("com_no_gun", [v for n, v in pts.items() if n != "ref_gun"]), ("com_with_gun", list(pts.values()))):
        m = sum(w for w, _ in sel)
        c = [sum(w * (v.X, v.Y, v.Z)[i] for w, v in sel) / m / 1000 for i in range(3)]
        info[key] = {"mass_kg": round(m, 3), "xyz_m": [round(c[0], 3), round(c[1], 3), round(c[2] + 0.791, 3)]}
    print("CoM", info["com_no_gun"], info["com_with_gun"])
    (OUT / "assembly.json").write_text(json.dumps(info, indent=1))
    print(json.dumps(info["heights_mm"]), "\nchecks passed")
