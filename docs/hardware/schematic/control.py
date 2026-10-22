"""Sheet SOB-E-02: control and communication."""
from sheetlib import GREY, LW, LW_PWR, Sheet

REVS = [("A", "2026-10-15", "First wiring plan (final design F1, 12 V pump)"),
        ("B", "2026-10-22", "Final design F2: 24 V pump on the motor bus, new drawing set")]

LEFT = [("A1", 1, "hip roll", "M6C12"), ("A2", 3, "hip yaw", "M6C12"), ("A3", 5, "hip pitch", "M6C12"),
        ("A4", 7, "knee", "M6C12"), ("A5", 11, "ankle pitch", "MAD 5010"), ("A6", 13, "ankle roll", "MAD 5010")]
RIGHT = [("A7", 2, "hip roll", "M6C12"), ("A8", 4, "hip yaw", "M6C12"), ("A9", 6, "hip pitch", "M6C12"),
         ("A10", 8, "knee", "M6C12"), ("A11", 12, "ankle pitch", "MAD 5010"), ("A12", 14, "ankle roll", "MAD 5010")]

U3X, U3Y, U3W, U3H = 150, 122, 46, 140
HDR = {"2  +5V": 120, "4  +5V": 115, "6  GND": 108, "14  GND": 103, "1  3V3": 94, "13  GPIO3_A2": 80,
       "25  GND": 74, "11  GPIO3_A1": 64, "9  GND": 52, "17  3V3": 36, "18  UART4_TX": 30, "16  UART4_RX": 24,
       "20  GND": 18}
USB = {"USB 3.0 host": 120, "USB 2.0 host 1": 100, "USB 2.0 host 2": 80, "USB 2.0 OTG, HOST": 50}
XD = 236                      # left edge of the USB devices
X0 = 280                      # start of the CAN bus lines
BW, GAP = 15.0, 1.8           # drive box width and gap

# positions that sheet 2 refers to
P5V = (110, U3Y + HDR["2  +5V"])
ESTOP = (108, U3Y + HDR["13  GPIO3_A2"])
PUMP = (108, U3Y + HDR["11  GPIO3_A1"])
SERVO = (250, 146)
CAN_L = (X0 + 30, 226)
CAN_R = (X0 + 30, 192)


def can_chain(s, y, drives, side, last_ref):
    """CAN_H / CAN_L bus, one box per drive under it, a 120-ohm terminator at the far end."""
    yh, yl = y, y - 3.0
    x_end = X0 + 6 + len(drives) * (BW + GAP)
    s.wire((X0, yh), (x_end, yh))
    s.wire((X0, yl), (x_end, yl))
    for i, (ref, node, joint, motor) in enumerate(drives):
        bx = X0 + 6 + i * (BW + GAP)
        s.dot(bx + 4.5, yh)
        s.dot(bx + 7.5, yl)
        s.wire((bx + 4.5, yh), (bx + 4.5, y - 9))
        s.wire((bx + 7.5, yl), (bx + 7.5, y - 9))
        s.rect(bx, y - 27, BW, 18, fc="white")
        s.text(bx + BW / 2, y - 13.2, ref, size=6.0, ha="center", weight="bold")
        s.text(bx + BW / 2, y - 16.6, f"node {node}", size=5.0, ha="center")
        s.text(bx + BW / 2, y - 19.8, f"{side} {joint}", size=4.5, ha="center", color=GREY)
        s.text(bx + BW / 2, y - 22.8, motor, size=4.4, ha="center", color=GREY)
        s.text(bx + BW / 2, y - 25.6, "XDrive MINI", size=4.4, ha="center", color=GREY)
    xt = x_end + 2
    s.wire((x_end, yh), (xt, yh))
    s.wire((x_end, yl), (xt - 3, yl))
    s.wire((xt, yh), (xt, yh - 1))
    s.resistor((xt, yh - 1), (xt, yl - 13))
    s.wire((xt, yl - 13), (xt - 3, yl - 13), (xt - 3, yl))
    s.dot(xt - 3, yl)
    s.ref(xt + 2.6, yl - 4.0, last_ref, "120 Ω\njumper on\nthe last drive")
    return x_end


def draw(out, z2):
    s = Sheet()
    s.zone_title(24, 280.5, "ROBOT COMPUTER, CAN BUSES, SENSORS AND PAN-TILT")
    pins = [("left", o, n) for n, o in HDR.items()] + [("right", o, n) for n, o in USB.items()]
    u3 = s.block(U3X, U3Y, U3W, U3H, "U3", "RADXA ROCK 3C", sub="RK3566 · 5 V input only", pins=pins,
                 title_y=U3Y + U3H - 6)
    s.text(U3X + U3W / 2, U3Y + 12.0, "header pins left, USB ports right", size=4.8, ha="center", color=GREY)
    s.text(U3X + U3W / 2, U3Y + 8.4, "GPIO logic 3.3 V, max 3.63 V", size=4.8, ha="center", color=GREY)
    Y = lambda n: u3[n][1]

    # ---- 5 V in: pins 2 + 4, GND: 6 + 14
    s.flag(P5V[0], P5V[1], "+5V_PC", "1/" + z2["plus5v"] + "  U2", direction="left")
    s.wire(P5V, u3["2  +5V"], lw=LW_PWR)
    s.dot(140, Y("2  +5V"))
    s.wire((140, Y("2  +5V")), (140, Y("4  +5V")), u3["4  +5V"], lw=LW_PWR)
    s.text(112.5, P5V[1] + 1.4, "W23 · 2 pairs, 22 AWG", size=5.0, color=GREY)
    s.wire(u3["6  GND"], (130, Y("6  GND")), (130, Y("14  GND")), u3["14  GND"], lw=LW_PWR)
    s.dot(130, Y("14  GND"))
    s.wire((130, Y("14  GND")), (130, Y("14  GND") - 3))
    s.ground(130, Y("14  GND") - 3, "GND_3S", side="left")

    # ---- e-stop sense: R1 pull-up from pin 1 to pin 13, contact S1 31-32 (sheet 2) to pin 25
    s.wire(u3["1  3V3"], (138, Y("1  3V3")))
    s.resistor((138, Y("1  3V3")), (138, Y("13  GPIO3_A2")))
    s.dot(138, Y("13  GPIO3_A2"))
    s.ref(140.0, Y("1  3V3") - 6.0, "R1", "10 kΩ")
    s.wire(u3["13  GPIO3_A2"], ESTOP)
    s.flag(ESTOP[0], ESTOP[1], "ESTOP_SENSE", "1/" + z2["estop"] + "  S1-31", direction="left")
    s.wire(u3["25  GND"], (ESTOP[0], Y("25  GND")))
    s.flag(ESTOP[0], Y("25  GND"), "GND_3S", "1/" + z2["estop"] + "  S1-32", direction="left")
    s.text(110.5, ESTOP[1] + 1.4, "W34 · 0 = released, 1 = pressed", size=5.0, color=GREY)

    # ---- pump on/off: pin 11 -> R8 pull-down, R7 -> opto-isolator U8 (sheet 2)
    y11, y9 = Y("11  GPIO3_A1"), Y("9  GND")
    s.wire(u3["11  GPIO3_A1"], (132, y11))
    s.dot(142, y11)
    s.resistor((142, y11), (142, y9))
    s.dot(142, y9)
    s.ref(139.6, y11 - 5.6, "R8", "10 kΩ", ha="right")
    s.resistor((132, y11), (116, y11))
    s.ref(119.0, y11 - 4.8, "R7", None)
    s.text(125.0, y11 - 4.8, "470 Ω", size=5.0, color=GREY)
    s.wire((116, y11), PUMP)
    s.flag(PUMP[0], PUMP[1], "PUMP_ON", "1/" + z2["pump"] + "  U8-A", direction="left")
    s.wire(u3["9  GND"], (PUMP[0], y9))
    s.flag(PUMP[0], y9, "GND_3S", "1/" + z2["pump"] + "  U8-K", direction="left")
    s.text(110.5, y9 - 4.4, "W33 · R8 holds the pump off during boot", size=5.0, color=GREY)
    s.proposed(64, y9 - 8.0, 84, y11 - y9 + 13.0, tag="PROPOSED")

    # ---- IMU on UART4
    u9 = s.block(52, 132, 34, 30, "U9", "IMU IM10A", sub="230400 baud",
                 pins=[("right", 26, "VCC"), ("right", 20, "RX"), ("right", 14, "TX"), ("right", 8, "GND")],
                 title_y=149)
    s.text(69, 139.0, "/dev/ttyS4 · UART4-M1", size=4.8, ha="center", color=GREY)
    s.text(69, 135.6, "3.3 V supply and logic", size=4.8, ha="center", color=GREY)
    for a, b in (("VCC", "17  3V3"), ("RX", "18  UART4_TX"), ("TX", "16  UART4_RX"), ("GND", "20  GND")):
        s.wire(u9[a], u3[b])
    s.text(96, Y("17  3V3") + 1.4, "W35 · 26 AWG", size=5.0, color=GREY)

    # ---- camera
    cam = s.block(XD, 236, 40, 16, "U10", "CAMERA", sub="Arducam B0201 · IMX291 · UVC",
                  pins=[("left", 6, "USB")], title_y=245.5)
    s.wire(u3["USB 3.0 host"], cam["USB"])
    s.text(204, Y("USB 3.0 host") + 1.4, "W52 · 1 m", size=5.0, color=GREY)
    s.text(XD + 42, 245.0, "640 × 480 MJPEG, 15 fps", size=5.0, color=GREY)
    s.text(XD + 42, 241.6, "/dev/video-fire (udev rule)", size=5.0, color=GREY)

    # ---- CANables and the two leg buses
    for ref, title, port, ybus, chain, side, t1, t2, wn, wc, jog in (
            ("U4", "CANable 1", "USB 2.0 host 1", CAN_L[1], LEFT, "L", "R3", "R4", "W50", "W40", 214),
            ("U5", "CANable 2", "USB 2.0 host 2", CAN_R[1], RIGHT, "R", "R5", "R6", "W51", "W41", 220)):
        by = ybus - 16
        cb = s.block(XD, by, 34, 20, ref, title, sub="2.0 Pro · isolated",
                     pins=[("left", 12, "USB"), ("right", 16, "CAN_H"), ("right", 13, "CAN_L"), ("right", 4, "GND")],
                     title_y=by + 10.5)
        s.wire(u3[port], (jog, Y(port)), (jog, cb["USB"][1]), cb["USB"])
        s.text(204, Y(port) + 1.4, wn, size=5.0, color=GREY)
        s.wire(cb["CAN_H"], (X0, cb["CAN_H"][1]))
        s.wire(cb["CAN_L"], (X0, cb["CAN_L"][1]))
        s.text(XD, by - 3.8, f"termination jumper ON = {t1}, 120 Ω", size=4.8, color=GREY)
        can_chain(s, ybus, chain, side, t2)
        name = "left" if side == "L" else "right"
        s.text(X0 + 2, ybus + 2.2, f"CAN_H / CAN_L · can_{name} · 500 kbit/s · {wc}: twisted pair to JST-GH",
               size=5.0, color=GREY)
        s.wire(cb["GND"], (275, cb["GND"][1]), (275, cb["GND"][1] - 3))
        s.ground(275, cb["GND"][1] - 3)
        s.text(277.0, cb["GND"][1] - 7.0, "GND_M", size=5.6, weight="bold")
        s.text(277.0, cb["GND"][1] - 9.8, "RS1 LOAD", size=4.6, color=GREY)

    # ---- servo adapter and the two ST3215 servos
    sv = s.block(XD, 110, 40, 30, "U6", "SERVO ADAPTER", sub="Waveshare Bus Servo Adapter (A)",
                 pins=[("left", 26, "USB"), ("top", 10, "DC IN"), ("right", 20, "V"), ("right", 14, "DATA"),
                       ("right", 8, "GND")], title_y=129)
    s.text(XD + 20, 117.0, "CH343 · /dev/servo (udev rule)", size=4.6, ha="center", color=GREY)
    s.text(XD + 20, 113.8, "12 V serial bus, 1 Mbit/s", size=4.6, ha="center", color=GREY)
    s.wire(u3["USB 2.0 OTG, HOST"], (226, Y("USB 2.0 OTG, HOST")), (226, sv["USB"][1]), sv["USB"])
    s.text(204, Y("USB 2.0 OTG, HOST") + 1.4, "W53", size=5.0, color=GREY)
    s.wire(sv["DC IN"], (sv["DC IN"][0], SERVO[1]), SERVO, lw=LW_PWR)
    s.flag(SERVO[0], SERVO[1], "+12V_ACT", "1/" + z2["act"] + "  K2, F13")
    s.text(SERVO[0] - 3.0, SERVO[1] + 1.4, "W29", size=5.0, color=GREY, ha="right")
    for k, pn in enumerate(("V", "DATA", "GND")):
        yy = sv[pn][1]
        s.wire(sv[pn], (300, yy))
        s.wire((326, yy), (334, yy))
    for i, (ref, sid, name) in enumerate((("M2", 1, "PAN"), ("M3", 2, "TILT"))):
        mx = 300 + i * 34
        s.rect(mx, 112, 26, 22, fc="white")
        s.text(mx + 13, 127.5, ref, size=6.2, ha="center", weight="bold")
        s.text(mx + 13, 123.4, f"ST3215 · ID {sid}", size=5.0, ha="center")
        s.text(mx + 13, 119.8, name, size=5.0, ha="center", color=GREY)
        s.text(mx + 13, 116.2, "12 V bus servo", size=4.6, ha="center", color=GREY)
    s.text(282, 106.0, "3-wire daisy chain, servo cables", size=5.0, color=GREY)

    notes = [
        "Node ids follow legs/robot.py (JOINTS, BHL numbering). If the drives get other ids, change that table.",
        "The order of the drives along each chain is not fixed. Terminate only the last drive of each chain.",
        "Each CANable joins its CAN GND to GND_M at the RS1 LOAD bolt only (W40, W41).",
        "Do not plug a USB-C supply into the ROCK 3C while the header gives 5 V: both feed the same rail.",
        "Do not use header pins 8 and 10: they carry the boot console. Enable UART4-M1 with rsetup.",
        "gpiochip numbers are not verified: check GPIO3_A1 and GPIO3_A2 with gpiodetect on the board.",
    ]
    s.frame("Control and communication", "SOB-E-02", 2, 2, "B", "2026-10-22", REVS, notes)
    s.save(out)
    return s


if __name__ == "__main__":
    import sys
    draw(sys.argv[1], {"plus5v": "C4", "estop": "E4", "pump": "D6", "act": "E3"})
