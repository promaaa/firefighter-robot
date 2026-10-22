"""Drawing helpers for A3 electrical schematics: IEC 60617 style symbols, frame and title block.

All coordinates are in mm on an A3 landscape sheet (420 x 297), origin at the bottom left, y up.
"""
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon, Rectangle, Arc

INK = "#1a1a1a"
GREY = "#6b6b6b"
PROPOSED = "#9a4d00"          # dashed outline and tag of parts that are proposed, not agreed
FONT = "Inter"
W, H = 420.0, 297.0
MM = 1 / 0.3528                # points per mm

LW_BUS = 0.70                  # mm: 10-12 AWG motor power
LW_PWR = 0.50                  # mm: 16-18 AWG
LW = 0.30                      # mm: signal and 22 AWG
LW_THIN = 0.20


class Sheet:
    def __init__(self, w=W, h=H):
        self.fig = plt.figure(figsize=(w / 25.4, h / 25.4))
        self.ax = self.fig.add_axes([0, 0, 1, 1])
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(0, h)
        self.ax.set_aspect("equal")
        self.ax.axis("off")

    # ---------- primitives ----------
    def line(self, pts, lw=LW, color=INK, ls="-", z=2):
        xs, ys = zip(*pts)
        self.ax.plot(xs, ys, color=color, lw=lw * MM, ls=ls, solid_capstyle="butt", dash_capstyle="butt",
                     solid_joinstyle="miter", zorder=z)

    def wire(self, *pts, lw=LW, ls="-", color=INK):
        """Orthogonal wire through the given points."""
        self.line(pts, lw=lw, ls=ls, color=color)

    def dot(self, x, y, r=0.75):
        self.ax.add_patch(Circle((x, y), r, color=INK, zorder=4))

    def ring(self, x, y, r=0.8):
        """Terminal: open circle."""
        self.ax.add_patch(Circle((x, y), r, fc="white", ec=INK, lw=LW * MM, zorder=4))

    def rect(self, x, y, w, h, lw=LW, ls="-", fc="none", ec=INK, z=1):
        self.ax.add_patch(Rectangle((x, y), w, h, fc=fc, ec=ec, lw=lw * MM, ls=ls, zorder=z))

    def text(self, x, y, s, size=7.0, ha="left", va="baseline", weight="normal", color=INK, rot=0, style="normal",
             family=None):
        self.ax.text(x, y, s, fontsize=size, ha=ha, va=va, fontweight=weight, color=color, rotation=rot,
                     fontfamily=family or FONT, fontstyle=style, zorder=5, linespacing=1.25)

    def wlabel(self, x, y, s, ha="center", rot=0):
        """Wire number and gauge, small, above the wire."""
        self.text(x, y + 0.9, s, size=5.2, ha=ha, color=GREY, rot=rot)

    # ---------- 2-terminal symbols ----------
    def _frame(self, p1, p2):
        (x1, y1), (x2, y2) = p1, p2
        L = math.hypot(x2 - x1, y2 - y1)
        ux, uy = (x2 - x1) / L, (y2 - y1) / L
        vx, vy = -uy, ux
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        T = lambda u, v: (cx + u * ux + v * vx, cy + u * uy + v * vy)
        return T, L

    def two(self, p1, p2, body, half, lw=LW, **kw):
        """Leads from p1 and p2 to a body of half-length `half`, drawn by body(T) in local (u along, v left)."""
        T, L = self._frame(p1, p2)
        self.line([p1, T(-half, 0)], lw=lw)
        self.line([T(half, 0), p2], lw=lw)
        body(T, **kw)
        return T

    def _poly(self, T, pts, closed=False, lw=LW, fc="none"):
        P = [T(u, v) for u, v in pts]
        if closed:
            self.ax.add_patch(Polygon(P, closed=True, fc=fc, ec=INK, lw=lw * MM, zorder=3))
        else:
            self.line(P, lw=lw, z=3)

    def fuse(self, p1, p2, lw=LW):
        def b(T):
            self._poly(T, [(-3.2, -1.2), (3.2, -1.2), (3.2, 1.2), (-3.2, 1.2)], closed=True, fc="white")
            self._poly(T, [(-3.2, 0), (3.2, 0)])
        return self.two(p1, p2, b, 3.2, lw=lw)

    def resistor(self, p1, p2, lw=LW, varistor=False):
        def b(T):
            self._poly(T, [(-3.2, -1.2), (3.2, -1.2), (3.2, 1.2), (-3.2, 1.2)], closed=True, fc="white")
            if varistor:
                self._poly(T, [(-4.2, -2.2), (-2.2, -2.2), (3.6, 2.4)], lw=LW_THIN)
        return self.two(p1, p2, b, 3.2, lw=lw)

    def battery(self, p1, p2, lw=LW):
        """Battery of cells; p1 is the + end."""
        def b(T):
            for u in (-1.8, 0.6):
                self._poly(T, [(u, -3.0), (u, 3.0)], lw=LW)            # long plate (+)
                self._poly(T, [(u + 1.2, -1.5), (u + 1.2, 1.5)], lw=1.0)  # short plate (-)
            self._poly(T, [(-0.2, 0), (0.15, 0)], lw=LW_THIN)
        T = self.two(p1, p2, b, 1.8, lw=lw)
        return T

    def diode(self, p1, p2, schottky=False, lw=LW):
        """Anode at p1, cathode at p2."""
        def b(T):
            self._poly(T, [(-1.6, -1.8), (-1.6, 1.8), (1.6, 0)], closed=True, fc=INK)
            self._poly(T, [(1.6, -1.8), (1.6, 1.8)], lw=LW)
            if schottky:
                self._poly(T, [(1.6, 1.8), (2.3, 1.8), (2.3, 1.2)], lw=LW_THIN)
                self._poly(T, [(1.6, -1.8), (0.9, -1.8), (0.9, -1.2)], lw=LW_THIN)
        return self.two(p1, p2, b, 1.6, lw=lw)

    def contact(self, p1, p2, nc=False, lw=LW, actuator=None, alen=6.0, aside=1):
        """Switch contact, drawn at rest. actuator: None, 'push', 'pushlock', 'estop', 'relay'.
        aside: +1 = actuator on the left of the p1->p2 direction, -1 = right."""
        def b(T):
            self._poly(T, [(-3.0, 0), (-2.6, 0)])
            if nc:
                self._poly(T, [(2.6, 0), (2.6, -1.8 * aside)], lw=LW)
                self._poly(T, [(-2.6, 0), (3.2, -1.5 * aside)], lw=LW)
                mid = (0.4, -0.8 * aside)
            else:
                self._poly(T, [(-2.6, 0), (2.4, 2.4 * aside)], lw=LW)
                self._poly(T, [(2.6, 0), (3.0, 0)])
                mid = (0.0, 1.25 * aside)
            if actuator:
                tip = (mid[0], mid[1] + alen * aside)
                self._poly(T, [mid, tip], lw=LW_THIN)
                self.ax.lines[-1].set_linestyle((0, (2.0, 1.2)))
                tu, tv = tip
                if actuator in ("push", "pushlock"):
                    self._poly(T, [(tu - 1.8, tv + 1.2 * aside), (tu - 1.8, tv), (tu + 1.8, tv), (tu + 1.8, tv + 1.2 * aside)])
                    if actuator == "pushlock":
                        self._poly(T, [(tu - 1.2, tv + 2.4 * aside), (tu, tv + 1.4 * aside), (tu + 1.2, tv + 2.4 * aside)], lw=LW_THIN)
                elif actuator == "estop":
                    P = [T(tu + 3.0 * math.cos(a), tv + aside * 3.0 * math.sin(a)) for a in [i * math.pi / 16 for i in range(17)]]
                    self.line(P, lw=LW, z=3)
                    self._poly(T, [(tu - 3.0, tv), (tu + 3.0, tv)])
        return self.two(p1, p2, b, 3.0, lw=lw)

    def coil(self, p1, p2, lw=LW):
        def b(T):
            self._poly(T, [(-2.8, -1.8), (2.8, -1.8), (2.8, 1.8), (-2.8, 1.8)], closed=True, fc="white")
        return self.two(p1, p2, b, 2.8, lw=lw)

    def lamp(self, p1, p2, lw=LW):
        def b(T):
            c = T(0, 0)
            self.ax.add_patch(Circle(c, 2.0, fc="white", ec=INK, lw=LW * MM, zorder=3))
            k = 2.0 / math.sqrt(2)
            self._poly(T, [(-k, -k), (k, k)])
            self._poly(T, [(-k, k), (k, -k)])
        return self.two(p1, p2, b, 2.0, lw=lw)

    def round_symbol(self, p1, p2, letter, r=2.6, lw=LW, sub=None):
        def b(T):
            c = T(0, 0)
            self.ax.add_patch(Circle(c, r, fc="white", ec=INK, lw=LW * MM, zorder=3))
            self.text(c[0], c[1] - 0.15, letter, size=7.5, ha="center", va="center", weight="bold")
            if sub:
                self.text(c[0] + r * 0.62, c[1] - r * 0.62, sub, size=4.8, ha="center", va="center")
        return self.two(p1, p2, b, r, lw=lw)

    # ---------- other symbols ----------
    def ground(self, x, y, name=None, side="right"):
        """IEC earth/ground at (x, y), lead going up."""
        self.line([(x, y), (x, y - 2.0)])
        for i, w in enumerate((3.2, 2.1, 1.0)):
            yy = y - 2.0 - i * 0.9
            self.line([(x - w / 2, yy), (x + w / 2, yy)], lw=LW)
        if name:
            self.text(x + (2.4 if side == "right" else -2.4), y - 3.6, name, size=5.6,
                      ha="left" if side == "right" else "right", weight="bold")

    def bar(self, x, y1, y2, label=None):
        """Distribution bar (terminal block), vertical."""
        self.rect(x - 1.0, y1, 2.0, y2 - y1, lw=LW, fc=INK)
        if label:
            self.text(x, y2 + 1.6, label, size=5.6, ha="center", weight="bold")

    def flag(self, x, y, name, ref, direction="right", width=None):
        """Off-sheet connector: a pentagon with the net name, and the target sheet/zone next to it."""
        w = width or (2.4 + 1.35 * len(name))
        h = 3.6
        if direction == "right":
            P = [(x, y - h / 2), (x + w, y - h / 2), (x + w + 2.2, y), (x + w, y + h / 2), (x, y + h / 2)]
            tx, rx, rha = x + 1.3, x + w + 3.4, "left"
        else:
            P = [(x, y - h / 2), (x - w, y - h / 2), (x - w - 2.2, y), (x - w, y + h / 2), (x, y + h / 2)]
            tx, rx, rha = x - w + 0.2, x - w - 3.4, "right"
        self.ax.add_patch(Polygon(P, closed=True, fc="white", ec=INK, lw=LW * MM, zorder=3))
        self.text(tx, y, name, size=5.4, ha="left", va="center", weight="bold")
        self.text(rx, y, ref, size=5.0, ha=rha, va="center", color=GREY)

    def block(self, x, y, w, h, ref, title, sub=None, pins=(), lw=LW, ls="-", ec=INK, title_y=None, size=7.0):
        """Box with pins. pins: (side, offset_mm, name). Returns {name: (x, y)} of the pin ends on the box edge."""
        self.rect(x, y, w, h, lw=lw, ls=ls, ec=ec, fc="white")
        ty = title_y if title_y is not None else y + h - 4.2
        self.text(x + w / 2, ty, title, size=size, ha="center", weight="bold")
        if sub:
            self.text(x + w / 2, ty - 3.6, sub, size=5.4, ha="center", color=GREY)
        self.text(x, y + h + 1.2, ref, size=6.2, weight="bold")
        out = {}
        for side, off, name in pins:
            if side == "left":
                px, py, ha, tx = x, y + off, "left", x + 1.2
            elif side == "right":
                px, py, ha, tx = x + w, y + off, "right", x + w - 1.2
            elif side == "top":
                px, py, ha, tx = x + off, y + h, "center", x + off
            else:
                px, py, ha, tx = x + off, y, "center", x + off
            if side in ("left", "right"):
                self.text(tx, py, name, size=5.2, ha=ha, va="center")
            elif side == "top":
                self.text(tx, py - 2.6, name, size=5.2, ha=ha, va="center")
            else:
                self.text(tx, py + 2.6, name, size=5.2, ha=ha, va="center")
            out[name] = (px, py)
        return out

    def dcdc(self, x, y, s, ref, title, sub):
        """DC/DC converter: square with a diagonal and the two '=' marks (IEC 06-14 style). Labels above the box."""
        self.rect(x, y, s, s, fc="white")
        self.line([(x, y), (x + s, y + s)], lw=LW_THIN)
        self.text(x + s * 0.27, y + s * 0.70, "=", size=9, ha="center", va="center")
        self.text(x + s * 0.73, y + s * 0.28, "=", size=9, ha="center", va="center")
        self.text(x, y + s + 4.4, ref, size=6.2, weight="bold")
        self.text(x + 7.5, y + s + 4.4, title, size=5.6, weight="bold")
        self.text(x, y + s + 1.4, sub, size=5.0, color=GREY)

    def ref(self, x, y, ref, value=None, ha="left", rot=0, vsize=5.2):
        """Designator (bold) and value under it."""
        self.text(x, y, ref, size=6.2, ha=ha, weight="bold", rot=rot)
        if value:
            for i, v in enumerate(value.split("\n")):
                self.text(x, y - 2.6 - i * 2.4, v, size=vsize, ha=ha, color=GREY, rot=rot)

    def proposed(self, x, y, w, h, tag="PROPOSED – CONFIRM BEFORE BUILD"):
        self.rect(x, y, w, h, lw=LW, ls=(0, (4, 2.5)), ec=PROPOSED, z=0.5)
        self.text(x + 1.6, y + h - 3.6, tag, size=5.4, color=PROPOSED, weight="bold")

    def zone_title(self, x, y, s):
        self.text(x, y, s, size=8.2, weight="bold")

    # ---------- connectors ----------
    def conn(self, x, y, name, vertical=False, label_side=1):
        """Plug (filled pin) and socket (arc) on a wire, IEC 03-03-05 style, with the connector name."""
        import numpy as np
        if not vertical:
            self.ax.add_patch(Rectangle((x - 1.9, y - 0.45), 1.5, 0.9, fc=INK, ec=INK, lw=0, zorder=4))
            a = np.linspace(-np.pi / 2, np.pi / 2, 20)
            self.line(list(zip(x + 0.2 + 1.5 * np.cos(a + np.pi), y + 1.5 * np.sin(a + np.pi))), lw=LW)
            self.ax.add_patch(Rectangle((x - 0.4, y - 0.4), 0.6, 0.8, fc="white", ec="none", zorder=3.5))
            self.text(x, y + 2.6 * label_side - (0.9 if label_side < 0 else 0), name, size=4.9, ha="center", color=GREY,
                      va="bottom" if label_side > 0 else "top")
        else:
            self.ax.add_patch(Rectangle((x - 0.45, y + 0.4), 0.9, 1.5, fc=INK, ec=INK, lw=0, zorder=4))
            a = np.linspace(0, np.pi, 20)
            self.line(list(zip(x + 1.5 * np.cos(a + np.pi), y - 0.2 - 1.5 * np.sin(a + np.pi))), lw=LW)
            self.ax.add_patch(Rectangle((x - 0.4, y - 0.2), 0.8, 0.6, fc="white", ec="none", zorder=3.5))
            self.text(x + 2.4, y - 0.6, name, size=4.9, color=GREY, va="center")

    @staticmethod
    def zone(x, y):
        """Grid zone of a point, e.g. 'C4' (rows A-F from the top, columns 1-8 from the left)."""
        col = min(8, max(1, int((x - 20) / ((W - 30) / 8)) + 1))
        row = "ABCDEF"[min(5, max(0, int((H - 10 - y) / ((H - 20) / 6))))]
        return f"{row}{col}"

    # ---------- frame ----------
    def frame(self, title, number, sheet, sheets, rev, date, revisions, notes):
        # ISO 5457: 20 mm filing margin on the left, 10 mm elsewhere
        x0, y0, x1, y1 = 20, 10, W - 10, H - 10
        self.rect(x0 - 5, y0 - 5, x1 - x0 + 10, y1 - y0 + 10, lw=0.25, ec="#bbbbbb")
        self.rect(x0, y0, x1 - x0, y1 - y0, lw=0.7)
        cols, rows = 8, 6
        cw, rh = (x1 - x0) / cols, (y1 - y0) / rows
        for i in range(cols):
            cx = x0 + (i + 0.5) * cw
            for yy in (y1 + 2.0, y0 - 3.6):
                self.text(cx, yy, str(i + 1), size=6.2, ha="center", color=GREY)
            if i:
                for (ya, yb) in ((y1, y1 + 5), (y0 - 5, y0)):
                    self.line([(x0 + i * cw, ya), (x0 + i * cw, yb)], lw=0.25, color=GREY)
        for j in range(rows):
            cy = y1 - (j + 0.5) * rh
            for xx in (x0 - 2.6, x1 + 2.6):
                self.text(xx, cy, "ABCDEF"[j], size=6.2, ha="center", va="center", color=GREY)
            if j:
                for (xa, xb) in ((x0 - 5, x0), (x1, x1 + 5)):
                    self.line([(xa, y1 - j * rh), (xb, y1 - j * rh)], lw=0.25, color=GREY)
        # title block, bottom right, 180 x 32
        tw, th = 180, 32
        tx, ty = x1 - tw, y0
        self.rect(tx, ty, tw, th, lw=0.7, fc="white", z=1.5)
        self.line([(tx, ty + 20), (x1, ty + 20)], lw=0.35)
        self.line([(tx, ty + 10), (x1, ty + 10)], lw=0.35)
        for xx in (tx + 110, tx + 140):
            self.line([(xx, ty), (xx, ty + 20)], lw=0.35)
        self.text(tx + 2, ty + 27.0, "SOBANG – firefighting humanoid on Berkeley Humanoid Lite legs", size=7.2, weight="bold")
        self.text(tx + 2, ty + 22.4, "Team SOBANG · Seoul National University · ASMR 2026", size=5.6, color=GREY)
        self.text(tx + 2, ty + 14.0, title, size=10.5, weight="bold")
        self.text(tx + 2, ty + 3.6, "Status: preliminary, not built. Sources: docs/hardware/electrical.md, harness.csv",
                  size=5.2, color=GREY)
        for (xx, lab, val) in ((tx + 112, "Drawing", number), (tx + 142, "Rev", rev)):
            self.text(xx, ty + 16.6, lab, size=4.8, color=GREY)
            self.text(xx, ty + 12.0, val, size=8.2, weight="bold")
        self.text(tx + 112, ty + 6.6, "Sheet", size=4.8, color=GREY)
        self.text(tx + 112, ty + 2.4, f"{sheet} of {sheets}", size=7.2, weight="bold")
        self.text(tx + 142, ty + 6.6, "Date · Size", size=4.8, color=GREY)
        self.text(tx + 142, ty + 2.4, f"{date} · A3", size=7.2, weight="bold")
        # revision table above the title block
        ry = ty + th
        self.rect(tx, ry, tw, 4.0 + 3.6 * len(revisions), lw=0.35, fc="white", z=1.5)
        self.text(tx + 2, ry + 1.2 + 3.6 * len(revisions), "Rev   Date          Change", size=5.0, weight="bold",
                  family="Liberation Mono")
        for i, (r, d, c) in enumerate(revisions):
            self.text(tx + 2, ry + 1.2 + 3.6 * (len(revisions) - 1 - i), f"{r:<5} {d}    {c}", size=5.0,
                      family="Liberation Mono")
        # notes, bottom left
        if notes:
            nx, ny = x0 + 3, y0 + 3 + 3.4 * len(notes)
            self.text(nx, ny + 1.0, "NOTES", size=6.2, weight="bold")
            for i, n in enumerate(notes):
                self.text(nx, ny - 3.4 * (i + 1) + 1.0, f"{i + 1}.  {n}", size=5.4)

    def arrow(self, x, y, direction, size=1.8):
        """Filled arrowhead with its tip at (x, y). direction: 'right', 'left', 'up', 'down'."""
        d = {"right": (1, 0), "left": (-1, 0), "up": (0, 1), "down": (0, -1)}[direction]
        ux, uy = d
        vx, vy = -uy, ux
        P = [(x, y), (x - ux * size * 1.6 + vx * size * 0.6, y - uy * size * 1.6 + vy * size * 0.6),
             (x - ux * size * 1.6 - vx * size * 0.6, y - uy * size * 1.6 - vy * size * 0.6)]
        self.ax.add_patch(Polygon(P, closed=True, fc=INK, ec=INK, lw=0, zorder=4))

    def save(self, stem, png_dpi=None):
        """SVG with the text as paths (same look on every computer). PNG only when png_dpi is given."""
        plt.rcParams["svg.fonttype"] = "path"
        plt.rcParams["svg.hashsalt"] = "sobang"          # stable ids: a rebuild without changes gives the same file
        self.fig.savefig(stem + ".svg", metadata={"Date": None})
        if png_dpi:
            self.fig.savefig(stem + ".png", dpi=png_dpi)
        return self.fig
