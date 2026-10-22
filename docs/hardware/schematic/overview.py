"""System overview: one block per part group, power and signal paths. Not a schematic: see sheets 1 and 2."""
from matplotlib.patches import Polygon, Rectangle

from sheetlib import GREY, INK, LW, MM, PROPOSED, Sheet

CW, BW, BH = 50, 40, 15                     # column pitch, box width, box height (mm)
COL = [8 + i * CW for i in range(8)]
ROW = [170, 140, 104, 76, 48, 20]
PANEL = "#f3f3f1"


def box(s, c, r, title, sub, proposed=False, wide=1, tall=1):
    x, y = COL[c], ROW[r] - (tall - 1) * 30
    w, h = BW + (wide - 1) * CW, BH + (tall - 1) * 30
    s.rect(x, y, w, h, lw=0.45 if not proposed else LW, fc="white", ec=PROPOSED if proposed else INK,
           ls=(0, (4, 2)) if proposed else "-", z=2)
    s.text(x + w / 2, y + h / 2 + 1.0, title, size=7.4, ha="center", weight="bold")
    s.text(x + w / 2, y + h / 2 - 3.4, sub, size=5.8, ha="center", color=GREY)
    return x, y, w, h


def mid(b, side):
    x, y, w, h = b
    return {"left": (x, y + h / 2), "right": (x + w, y + h / 2), "top": (x + w / 2, y + h),
            "bottom": (x + w / 2, y)}[side]


def draw(out):
    s = Sheet(406, 200)
    ax = s.ax
    # panels
    ax.add_patch(Polygon([(2, 194), (404, 194), (404, 132), (202, 132), (202, 163), (2, 163)], closed=True,
                         fc=PANEL, ec="#d8d8d4", lw=0.3 * MM, zorder=0))
    ax.add_patch(Rectangle((2, 40), 196, 120, fc=PANEL, ec="#d8d8d4", lw=0.3 * MM, zorder=0))
    ax.add_patch(Rectangle((202, 12), 202, 118, fc=PANEL, ec="#d8d8d4", lw=0.3 * MM, zorder=0))
    s.text(5, 189.5, "MOTOR POWER  ·  2 × 6S LiPo, 22.2 V", size=7.0, weight="bold")
    s.text(5, 155.5, "CONTROL POWER  ·  3S LiPo, 11.1 V", size=7.0, weight="bold")
    s.text(205, 125.5, "COMPUTER, SENSORS AND ACTUATORS", size=7.0, weight="bold")

    packs = box(s, 0, 0, "BT1, BT2", "2 × 6S LiPo 5000 mAh")
    diode = box(s, 1, 0, "F1, F2 · D1", "pack fuses, OR diode")
    k1 = box(s, 2, 0, "K1 main relay", "AEP18024, 80 A")
    legl = box(s, 4, 1, "Left leg A1–A6", "6 × XDrive MINI")
    legr = box(s, 5, 1, "Right leg A7–A12", "6 × XDrive MINI")
    pumpc = box(s, 6, 1, "U7 DMC-101", "pump controller, F14", proposed=True)
    pump = box(s, 7, 1, "M1 pump", "Kamoer KLP02, 24 V")

    bt3 = box(s, 0, 3, "BT3", "3S LiPo 6000 mAh")
    x1 = box(s, 1, 3, "F10 · X1", "fuse, distribution")
    s2 = box(s, 2, 3, "S2 PC POWER", "push-lock switch")
    buck = box(s, 3, 3, "U2 5 V buck", "Pololu D36V50F5")
    boost = box(s, 2, 2, "U1 24 V boost", "HAM5925")
    s1 = box(s, 2, 1, "S1 E-STOP", "3 × NC contacts")
    s3 = box(s, 3, 1, "S3 MOTOR POWER", "push-lock switch")
    k2 = box(s, 0, 4, "K2 servo relay", "KKA-K4, 40 A")

    can1 = box(s, 4, 2, "U4 CANable 1", "isolated USB–CAN")
    can2 = box(s, 5, 2, "U5 CANable 2", "isolated USB–CAN")
    opto = box(s, 6, 2, "U8 opto-isolator", "pump on / off", proposed=True)
    rock = box(s, 4, 3, "U3 RADXA ROCK 3C", "RK3566 · walking policy, fire detector, aiming, safety", wide=3)
    servoa = box(s, 4, 4, "U6 servo adapter", "Waveshare (A)")
    cam = box(s, 5, 4, "U10 camera", "Arducam B0201")
    imu = box(s, 6, 4, "U9 IMU", "IM10A")
    servos = box(s, 4, 5, "M2, M3 pan-tilt", "2 × ST3215, 12 V")

    P = 0.70                                     # power line width (mm)

    def link(a, b, lw=LW, ls="-", arrow=None, label=None, lpos=None, ha="left"):
        s.wire(*([a] + ([lpos] if False else []) + [b]), lw=lw, ls=ls)
        if arrow:
            s.arrow(b[0], b[1], arrow)

    def path(pts, lw=LW, ls="-", arrow=None):
        s.wire(*pts, lw=lw, ls=ls)
        if arrow:
            s.arrow(pts[-1][0], pts[-1][1], arrow)

    # motor power
    path([mid(packs, "right"), mid(diode, "left")], lw=P, arrow="right")
    path([mid(diode, "right"), mid(k1, "left")], lw=P, arrow="right")
    yb = 182.5
    path([mid(k1, "right"), (156, mid(k1, "right")[1]), (156, yb), (mid(pumpc, "top")[0], yb)], lw=P)
    for b in (legl, legr, pumpc):
        s.dot(mid(b, "top")[0], yb)
        path([(mid(b, "top")[0], yb), mid(b, "top")], lw=P, arrow="down")
    s.text(160, yb + 1.6, "+24 V bus  ·  K1 open = drives and pump off", size=5.8, color=GREY)
    path([mid(pumpc, "right"), mid(pump, "left")], lw=P, arrow="right")
    s.text(COL[4] + BW / 2, ROW[1] + 1.6, "brake resistor on hip roll", size=5.0, ha="center", color=GREY)
    s.text(COL[5] + BW / 2, ROW[1] + 1.6, "brake resistor on hip roll", size=5.0, ha="center", color=GREY)

    # control power
    path([mid(bt3, "right"), mid(x1, "left")], lw=P, arrow="right")
    path([mid(x1, "right"), mid(s2, "left")], lw=P, arrow="right")
    path([mid(s2, "right"), mid(buck, "left")], lw=P, arrow="right")
    path([mid(buck, "right"), mid(rock, "left")], lw=P, arrow="right")
    s.text(200.5, mid(buck, "right")[1] + 1.6, "+5 V", size=5.6, ha="center", color=GREY)
    path([mid(s2, "top"), mid(boost, "bottom")], lw=P, arrow="up")
    path([mid(boost, "top"), mid(s1, "bottom")], lw=P, arrow="up")
    s.text(mid(s1, "bottom")[0] + 1.6, 128.0, "+24 V", size=5.6, color=GREY)
    path([(COL[2] + BW, ROW[1] + 9.5), (COL[3], ROW[1] + 9.5)], lw=P, arrow="right")
    xk = mid(s3, "top")[0]
    path([mid(s3, "top"), (xk, 162), (mid(k1, "bottom")[0], 162), mid(k1, "bottom")], ls=(0, (3, 2)), arrow="up")
    s.text(xk + 1.6, 157.6, "K1 coil", size=5.6, color=GREY)
    path([mid(x1, "bottom"), (mid(x1, "bottom")[0], 70), (mid(k2, "top")[0], 70), mid(k2, "top")], lw=P,
         arrow="down")
    path([mid(k2, "right"), mid(servoa, "left")], lw=P, arrow="right")
    s.text(150, mid(k2, "right")[1] + 1.6, "+12 V", size=5.6, ha="center", color=GREY)
    xs = 4.5
    path([(COL[2], ROW[1] + 11.5), (xs, ROW[1] + 11.5), (xs, mid(k2, "left")[1]), mid(k2, "left")],
         ls=(0, (3, 2)), arrow="right")
    s.text(xs + 1.4, 100, "K2 coil", size=5.6, color=GREY)
    ysen, xsen = 97.5, COL[3] - 5
    path([(COL[2] + BW, ROW[1] + 4.0), (xsen, ROW[1] + 4.0), (xsen, ysen), (212, ysen), (212, ROW[3] + BH)],
         ls=(0, (3, 2)), arrow="down")
    s.text(184, ysen + 1.6, "e-stop sense (GPIO 13)", size=5.6, ha="center", color=GREY)

    # computer and signals
    for b, lab in ((can1, "USB"), (can2, "USB"), (opto, "GPIO 11")):
        x = mid(b, "bottom")[0]
        path([(x, ROW[3] + BH), (x, ROW[2])], arrow="up")
        s.text(x + 1.4, ROW[3] + BH + 3.8, lab, size=5.6, color=GREY)
    for b, lab in ((can1, "CAN left"), (can2, "CAN right"), (opto, "on / off")):
        x = mid(b, "top")[0]
        path([(x, ROW[2] + BH), (x, ROW[1])], arrow="up")
        s.text(x + 1.4, ROW[2] + BH + 3.6, lab, size=5.6, color=GREY)
    for b, lab, d in ((servoa, "USB", "down"), (cam, "USB 3", "up"), (imu, "UART4", "up")):
        x = mid(b, "top")[0]
        if d == "down":
            path([(x, ROW[3]), (x, ROW[4] + BH)], arrow="down")
        else:
            path([(x, ROW[4] + BH), (x, ROW[3])], arrow="up")
        s.text(x + 1.4, ROW[4] + BH + 3.6, lab, size=5.6, color=GREY)
    path([mid(servoa, "bottom"), mid(servos, "top")], lw=P, arrow="down")
    s.text(mid(servoa, "bottom")[0] + 1.4, ROW[5] + BH + 3.6, "servo bus", size=5.6, color=GREY)

    # legend
    lx, ly = 8, 30
    s.line([(lx, ly), (lx + 12, ly)], lw=P)
    s.text(lx + 14, ly - 1.0, "power", size=5.8)
    s.line([(lx + 36, ly), (lx + 48, ly)], lw=LW)
    s.text(lx + 50, ly - 1.0, "data or signal", size=5.8)
    s.line([(lx + 82, ly), (lx + 94, ly)], lw=LW, ls=(0, (3, 2)))
    s.text(lx + 96, ly - 1.0, "relay coil, e-stop", size=5.8)
    s.rect(lx + 132, ly - 3, 12, 6, lw=LW, ec=PROPOSED, ls=(0, (4, 2)))
    s.text(lx + 146, ly - 1.0, "proposed", size=5.8)
    s.text(lx, ly - 9.0, "Full wiring: SOB-E-01 (power) and SOB-E-02 (control and communication).", size=5.8,
           color=GREY)
    s.text(lx, ly - 13.4, "Reference designators are the same on all drawings and in harness.csv.", size=5.8,
           color=GREY)
    s.save(out)


if __name__ == "__main__":
    import sys
    draw(sys.argv[1])
