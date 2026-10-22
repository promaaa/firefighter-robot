"""Sheet SOB-E-01: power distribution."""
import math

from sheetlib import GREY, LW, LW_BUS, LW_PWR, LW_THIN, PROPOSED, Sheet

REVS = [("A", "2026-10-15", "First wiring plan (final design F1, 12 V pump)"),
        ("B", "2026-10-22", "Final design F2: 24 V pump on the motor bus, new drawing set")]

# positions that other sheets refer to
K1_CONTACT = (132, 258)
K1_COIL = (240, 130)
PLUS5V_FLAG = (182, 156)
PUMP_FLAG = (300, 152)
ACT_FLAG = (150, 92)
ESTOP_FLAGS = (190, 80)


def estop_head(s, x, y_top, y_bottom):
    """Mushroom head and the dashed link through the three contacts of one e-stop button."""
    s.line([(x, y_bottom), (x, y_top)], lw=LW_THIN, ls=(0, (2.0, 1.2)))
    P = [(x + 3.4 * math.cos(a), y_top + 3.4 * math.sin(a)) for a in [i * math.pi / 16 for i in range(17)]]
    s.line(P, lw=LW)
    s.line([(x - 3.4, y_top), (x + 3.4, y_top)], lw=LW)


def draw(out, sheet3_zones):
    s = Sheet()
    z3 = sheet3_zones

    # ================= MOTOR POWER =================
    s.zone_title(24, 280.5, "MOTOR POWER  ·  2 × 6S LiPo, 22.2 V nominal, 25.2 V full")
    RET = 190.0
    s.battery((44, 266), (44, 250), lw=LW_PWR)
    s.ref(50, 260, "BT1", "6S LiPo 5000 mAh\nZeee, EC5")
    s.battery((44, 234), (44, 218), lw=LW_PWR)
    s.ref(50, 228, "BT2", "6S LiPo 5000 mAh\nZeee, EC5")
    s.text(41.0, 264.2, "+", size=6, ha="right")
    s.text(41.0, 232.2, "+", size=6, ha="right")
    s.wire((44, 266), (44, 272), (64, 272), lw=LW_BUS)
    s.wire((44, 234), (44, 240), (64, 240), lw=LW_BUS)
    s.conn(55, 272, "EC5")
    s.conn(55, 240, "EC5")
    for y, f, w in ((272, "F1", "W01"), (240, "F2", "W02")):
        s.fuse((64, y), (80, y), lw=LW_BUS)
        s.ref(67.5, y + 3.6, f, None)
        s.text(74.5, y + 3.6, "20 A", size=5.2, color=GREY)
        s.text(56, y - 4.4, w + " · 12 AWG", size=5.0, color=GREY)
    # D1: dual Schottky, common cathode
    s.wire((80, 272), (88, 272), lw=LW_BUS)
    s.wire((80, 240), (88, 240), lw=LW_BUS)
    s.diode((88, 272), (98, 272), schottky=True, lw=LW_BUS)
    s.diode((88, 240), (98, 240), schottky=True, lw=LW_BUS)
    s.wire((98, 272), (104, 272), (104, 240), (98, 240), lw=LW_BUS)
    s.dot(104, 258)
    s.rect(84, 235.5, 24, 41, lw=LW_THIN, ls=(0, (3, 2)), ec=GREY)
    s.ref(84.5, 231.0, "D1", "MBRT400100\ndual Schottky")
    # K1 contact
    s.wire((104, 258), (124, 258), lw=LW_BUS)
    s.text(106, 259.8, "+24V_BAT", size=5.4, weight="bold")
    s.text(106, 254.0, "W05 · 10 AWG", size=5.0, color=GREY)
    s.contact((124, 258), (140, 258), lw=LW_BUS)
    s.ref(126, 268.0, "K1", "1–2 · AEP18024, 80 A\ncoil: 1/" + s.zone(*K1_COIL))
    # +24V_BUS
    BUSY = 258
    XF14 = 390
    s.wire((140, BUSY), (XF14, BUSY), lw=LW_BUS)
    s.text(146, 260.0, "+24V_BUS   switched by K1, cut by the e-stop", size=5.4, weight="bold")
    for x0, fref, jref, leg, aref, rb, wv, wr in ((166, "F3", "J1", "LEFT LEG", "A1–A6", "RB1", "W06 / W08", "W10"),
                                                  (268, "F4", "J2", "RIGHT LEG", "A7–A12", "RB2", "W07 / W09", "W11")):
        xd = x0 + 14
        s.dot(xd, BUSY)
        s.fuse((xd, BUSY), (xd, 242), lw=LW_BUS)
        s.ref(xd + 3.0, 252.5, fref, "20 A")
        s.text(xd - 2.4, 247.0, wv, size=5.0, ha="right", color=GREY)
        s.text(xd - 2.4, 244.2, "12 AWG", size=5.0, ha="right", color=GREY)
        s.wire((xd, 242), (xd, 226), lw=LW_BUS)
        s.conn(xd, 233, jref + ":1  XT60", vertical=True)
        p = s.block(x0, 196, 58, 30, aref, leg, sub="6 × XDrive MINI, XT30 chain",
                    pins=[("top", 14, "V+"), ("bottom", 14, "V−"), ("right", 21, "AUX+"), ("right", 9, "AUX−")],
                    title_y=211)
        s.text(x0 + 30, 203.0, "hip roll drive AUX: brake resistor", size=4.9, ha="center", color=GREY)
        s.text(x0 + 30, 199.8, "CAN bus: 2/" + z3[leg], size=4.9, ha="center", color=GREY)
        s.wire((xd, 196), (xd, RET), lw=LW_BUS)
        s.conn(xd, 193.2, jref + ":2", vertical=True)
        s.dot(xd, RET)
        xr = x0 + 66
        s.wire(p["AUX+"], (xr, p["AUX+"][1]))
        s.wire(p["AUX−"], (xr, p["AUX−"][1]))
        s.resistor((xr, p["AUX+"][1]), (xr, p["AUX−"][1]))
        s.ref(xr + 3.0, 215.5, rb, "4.7 Ω 50 W\nRH50C4R70J")
        s.text(x0 + 59.5, p["AUX+"][1] + 1.2, wr, size=5.0, color=GREY)
    # return line, shunt and meter
    s.wire((44, 250), (44, 248), (36, 248), (36, RET), lw=LW_BUS)
    s.wire((44, 218), (44, RET), lw=LW_BUS)
    s.dot(44, RET)
    s.text(46.0, 246.2, "–", size=6)
    s.text(46.0, 214.4, "–", size=6)
    s.wire((36, RET), (96, RET), lw=LW_BUS)
    s.resistor((96, RET), (118, RET), lw=LW_BUS)
    s.wire((118, RET), (282, RET), lw=LW_BUS)
    s.ref(99, 183.6, "RS1", "shunt 100 A / 75 mV")
    s.text(93, 191.6, "BATT", size=4.8, ha="right", color=GREY)
    s.text(121, 191.6, "LOAD", size=4.8, color=GREY)
    s.text(52, 191.6, "W03 / W04 · 12 AWG", size=5.0, color=GREY)
    s.ground(150, RET, "GND_M")
    s.dot(150, RET)
    p1 = s.block(122, 204, 32, 17, "P1", "HAV6001", sub="V / A meter",
                 pins=[("left", 12, "V+"), ("bottom", 7, "S+"), ("bottom", 25, "S−")], title_y=213)
    s.wire((104, 240), (104, 228))
    s.dot(104, 240)
    s.fuse((104, 228), (104, 216))
    s.ref(106.5, 225.0, "F5", "2 A")
    s.wire((104, 216), p1["V+"])
    s.wire(p1["S+"], (129, 197), (99.5, 197), (99.5, RET))
    s.wire(p1["S−"], (147, 197), (114.5, 197), (114.5, RET))
    s.dot(99.5, RET)
    s.dot(114.5, RET)
    s.text(131, 198.8, "W12 · 22 AWG", size=5.0, color=GREY)
    # pump feed
    s.dot(XF14, BUSY)
    s.fuse((XF14, BUSY), (XF14, 242), lw=LW_PWR)
    s.ref(XF14 + 3.0, 252.5, "F14", "3 A")
    s.wire((XF14, 242), (XF14, 150), lw=LW_PWR)
    s.text(XF14 + 2.0, 236, "W32 · 18 AWG", size=5.0, color=GREY)

    # ================= CONTROL POWER =================
    s.zone_title(24, 172, "CONTROL POWER  ·  3S LiPo, 11.1 V nominal, 12.6 V full")
    s.battery((34, 152), (34, 136), lw=LW_PWR)
    s.text(31.0, 150.2, "+", size=6, ha="right")
    s.ref(24, 130.5, "BT3", "3S LiPo 6000 mAh\nEC5")
    s.ground(34, 136, "GND_3S")
    s.wire((34, 152), (34, 158), (46, 158), lw=LW_PWR)
    s.conn(41, 158, "EC5")
    s.fuse((46, 158), (60, 158), lw=LW_PWR)
    s.ref(48.0, 161.6, "F10", None)
    s.text(55.5, 161.6, "10 A", size=5.2, color=GREY)
    s.round_symbol((60, 158), (74, 158), "A", r=2.6, lw=LW_PWR)
    s.ref(63.2, 152.4, "P2", "HAV1625a")
    s.wire((74, 158), (83, 158), lw=LW_PWR)
    s.text(40, 152.6, "W20 · 16 AWG", size=5.0, color=GREY)
    s.bar(84, 84, 163, "X1")
    s.text(86.4, 165.0, "+12V_3S · Wago 221-413", size=5.0, color=GREY)

    # computer branch
    YPC = 156
    s.wire((85, YPC), (92, YPC))
    s.fuse((92, YPC), (104, YPC))
    s.ref(93.0, 159.6, "F11", None)
    s.text(100.6, 159.6, "5 A", size=5.2, color=GREY)
    s.contact((106, YPC), (120, YPC), actuator="pushlock", alen=4.0)
    s.wire((104, YPC), (106, YPC))
    s.ref(106, 167.4, "S2", None)
    s.text(112, 167.4, "PC POWER · push-lock", size=5.0, color=GREY)
    s.wire((120, YPC), (150, YPC))
    s.text(122, 157.6, "+12V_SW", size=5.4, weight="bold")
    s.text(122, 152.2, "W21 · 22 AWG", size=5.0, color=GREY)
    # P3 voltmeter on +12V_SW
    s.dot(126, YPC)
    s.wire((126, YPC), (126, 148))
    s.round_symbol((126, 148), (126, 136), "V", r=2.4)
    s.ground(126, 136)
    s.ref(129.5, 144.5, "P3", "HAV1808")
    # U2 buck
    s.dcdc(150, 150, 16, "U2", "5 V buck", "Pololu D36V50F5")
    s.wire((166, YPC), (PLUS5V_FLAG[0], YPC), lw=LW_PWR)
    s.flag(PLUS5V_FLAG[0], YPC, "+5V_PC", "2/" + z3["rock_5v"] + "  ROCK 3C pins 2, 4")
    s.text(168.0, YPC + 1.4, "W23", size=5.0, color=GREY)
    s.ground(158, 150)
    # U1 boost
    s.dot(140, YPC)
    s.wire((140, YPC), (140, 130), (150, 130))
    s.text(141.6, 138, "W24", size=5.0, color=GREY)
    s.dcdc(150, 118, 16, "U1", "24 V boost", "HAM5925 · set 24.0 V, 0.5 A")
    s.ground(158, 118)
    # 24 V control chain: F12, S1 NC1, S3, K1 coil (with RV1 and the S3 lamp H1)
    Y1, Y2, Y3 = 130, 104, 80
    s.wire((166, Y1), (172, Y1))
    s.fuse((172, Y1), (184, Y1))
    s.ref(173.0, 133.6, "F12", None)
    s.text(180.6, 133.6, "2 A", size=5.2, color=GREY)
    s.text(167.0, 125.0, "+24V_CTRL", size=5.0, weight="bold")
    s.wire((184, Y1), (190, Y1))
    s.dot(188, Y1)
    s.contact((190, Y1), (204, Y1), nc=True)
    s.wire((204, Y1), (212, Y1))
    s.contact((212, Y1), (226, Y1), actuator="pushlock", alen=4.0)
    s.ref(212, 141.5, "S3", None)
    s.text(218, 141.5, "MOTOR POWER · push-lock", size=5.0, color=GREY)
    s.text(212, 138.5, "lamp H1 across the K1 coil", size=5.0, color=GREY)
    s.text(206, Y1 + 1.4, "W26", size=5.0, color=GREY)
    s.wire((226, Y1), (230, Y1))
    s.dot(230, Y1)
    s.coil((230, Y1), (250, Y1))
    s.wire((230, Y1), (230, 114))
    s.resistor((230, 122), (250, 122), varistor=True)
    s.lamp((230, 114), (250, 114))
    s.wire((250, Y1), (250, 108))
    s.dot(250, 122)
    s.dot(230, 122)
    s.dot(250, 114)
    s.ground(250, 108)
    s.ref(252.5, 132.5, "K1", "coil 24 V, 136 Ω")
    s.text(252.5, 127.5, "contacts: 1/" + s.zone(*K1_CONTACT), size=5.0, color=GREY)
    s.ref(252.5, 121.0, "RV1", None)
    s.text(259.5, 121.0, "10D390K", size=5.0, color=GREY)
    s.ref(252.5, 113.0, "H1", None)
    s.text(258.5, 113.0, "lamp of S3", size=5.0, color=GREY)
    s.text(231.4, Y1 + 3.6, "W27", size=5.0, color=GREY)
    # NC2 and K2 coil with D2
    s.wire((188, Y1), (188, Y2), (190, Y2))
    s.text(185.0, 117.0, "W38", size=5.0, color=GREY, ha="right")
    s.contact((190, Y2), (204, Y2), nc=True)
    s.wire((204, Y2), (209, Y2))
    s.dot(209, Y2)
    s.coil((209, Y2), (227, Y2))
    s.wire((209, Y2), (209, 97))
    s.diode((227, 97), (209, 97), schottky=True)
    s.wire((227, Y2), (227, 93))
    s.dot(227, 97)
    s.ground(227, 93)
    s.ref(229.5, 106.0, "K2", "coil 24 V")
    s.ref(229.5, 98.0, "D2", "SB560")
    s.text(205.0, Y2 + 1.4, "W39", size=5.0, color=GREY)
    # NC3: e-stop sense to the computer
    s.contact((190, Y3), (204, Y3), nc=True)
    s.flag(190, Y3, "ESTOP_SENSE", "2/" + z3["estop"] + "  pin 13", direction="left")
    s.wire((204, Y3), (208, Y3))
    s.flag(208, Y3, "GND_3S", "2/" + z3["estop"] + "  pin 25")
    estop_head(s, 197.2, 145.0, 78.6)
    s.ref(199.8, 149.2, "S1", None)
    s.text(205.6, 149.2, "E-STOP · A20-E01P · 3 × NC", size=5.0, color=GREY)
    for yy, t in ((Y1, "11–12"), (Y2, "21–22"), (Y3, "31–32")):
        s.text(193.0, yy - 4.4, t, size=4.6, ha="center", color=GREY)
    # K2 contact: +12V_ACT for the pan-tilt servos
    YA = 92
    s.wire((85, YA), (92, YA), lw=LW_PWR)
    s.contact((92, YA), (106, YA), lw=LW_PWR)
    s.ref(92, 99.0, "K2", None)
    s.text(99.0, 99.0, "30–87 · KKA-K4, 40 A", size=5.0, color=GREY)
    s.wire((106, YA), (118, YA), lw=LW_PWR)
    s.text(107.6, YA + 1.4, "+12V_ACT", size=5.4, weight="bold")
    s.fuse((118, YA), (132, YA), lw=LW_PWR)
    s.ref(119.5, YA - 5.4, "F13", None)
    s.text(127.0, YA - 5.4, "10 A", size=5.2, color=GREY)
    s.wire((132, YA), (ACT_FLAG[0], YA), lw=LW_PWR)
    s.conn(141, YA, "DC 5.5 × 2.1", label_side=-1)
    s.flag(ACT_FLAG[0], YA, "+12V_ACT", "2/" + z3["servo"] + "  servo adapter U6")
    s.text(92, 83.6, "W30 / W31 / W29 · 18 AWG", size=5.0, color=GREY)

    # ================= WATER PUMP =================
    s.zone_title(272, 172, "WATER PUMP  ·  24 V")
    s.proposed(270, 74, 138, 92, tag="PROPOSED – CONFIRM WITH THE DMC-101 DATASHEET")
    s.text(XF14 + 2.0, 152.0, "+24V_BUS", size=5.4, weight="bold")
    u7 = s.block(342, 100, 34, 46, "U7", "DMC-101", sub="pump controller",
                 pins=[("right", 40, "V+"), ("right", 30, "V−"), ("right", 16, "M+"), ("right", 6, "M−"),
                       ("left", 24, "CTRL"), ("left", 14, "COM")], title_y=128)
    s.wire((XF14, 150), (XF14, 140), u7["V+"], lw=LW_PWR)
    s.wire(u7["V−"], (381, 130), (381, 124))
    s.ground(381, 124, "GND_M")
    s.wire(u7["M+"], (398, 116), (398, 113), lw=LW_PWR)
    s.round_symbol((398, 113), (398, 99), "M", r=3.0, lw=LW_PWR, sub="=")
    s.wire((398, 99), (398, 94), (392, 94), (392, 106), u7["M−"], lw=LW_PWR)
    s.ref(352, 88.0, "M1", "Kamoer KLP02, 24 V, ≤ 12 W, self-priming diaphragm pump")
    s.text(379, 112.8, "W37", size=5.0, color=GREY)
    u8 = s.block(310, 106, 22, 22, "U8", "OPTO", sub="isolator",
                 pins=[("left", 16, "A"), ("left", 6, "K"), ("right", 16, "C"), ("right", 6, "E")], title_y=119)
    s.wire(u8["C"], (336, 122), (336, 124), u7["CTRL"])
    s.wire(u8["E"], (338, 112), (338, 114), u7["COM"])
    s.text(310, 101.0, "PC817 type. The DMC-101 input", size=4.8, color=GREY)
    s.text(310, 98.2, "type is not known yet (TBC).", size=4.8, color=GREY)
    s.wire(u8["A"], (306, 122), (306, PUMP_FLAG[1]), PUMP_FLAG)
    s.wire(u8["K"], (304, 112), (304, 142), (PUMP_FLAG[0], 142))
    s.flag(PUMP_FLAG[0], PUMP_FLAG[1], "PUMP_ON", "2/" + z3["pump"] + " · pin 11", direction="left")
    s.flag(PUMP_FLAG[0], 142, "GND_3S", "2/" + z3["pump"] + " · pin 9", direction="left")
    s.text(274, 78.0, "U8 keeps GND_M and GND_3S apart. K1 (e-stop) also cuts the pump.", size=5.0,
           color=PROPOSED)

    notes = [
        "Contacts are drawn at rest: e-stop released, push-lock switches open, relays not energised.",
        "Fuses: mini blade, 32 V DC. F1–F4 in MTA 0100335 holders, the others in Coms BE151 holders.",
        "Wire numbers (W01 …) and gauges refer to docs/hardware/harness.csv. Thick lines: 10–12 AWG.",
        "GND_M and GND_3S share no wire. The CAN adapters are isolated (sheet 2).",
        "Set U1 to 24.0 V and its current limit to 0.5 A before you connect the K1 and K2 coils.",
        "RB1, RB2: set brake_resistance = 4.75 Ω on the two hip-roll drives only (electrical.md, 3.4).",
    ]
    s.frame("Power distribution", "SOB-E-01", 1, 2, "B", "2026-10-22", REVS, notes)
    s.save(out)
    return s


if __name__ == "__main__":
    import sys
    draw(sys.argv[1], {"LEFT LEG": "B5", "RIGHT LEG": "D5", "rock_5v": "C2", "estop": "D2", "pump": "D2", "servo": "B7"})
