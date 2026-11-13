"""선택 관절의 실시간 그래프: 각도 / 속도 / 토크. 실측(파랑)·목표(주황 점선)·시뮬(청록).

x축은 '지금부터 몇 초 전'(0 = 지금). 마우스를 올리면 세 그래프에 같은 시각의 세로선과 값이 나온다.
"""
import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt

from .theme import C

pg.setConfigOptions(antialias=True, background=C["surface"], foreground=C["muted"])

LINE_W = 2


def _pen(color, dash=False):
    return pg.mkPen(color, width=LINE_W, style=Qt.DashLine if dash else Qt.SolidLine)


def _nearest(xs, ys, x):
    if xs is None or len(xs) == 0:
        return None
    i = int(np.searchsorted(xs, x))
    i = min(max(i, 0), len(xs) - 1)
    if i > 0 and abs(xs[i - 1] - x) < abs(xs[i] - x):
        i -= 1
    if abs(xs[i] - x) > 0.25:
        return None
    v = ys[i]
    return None if not np.isfinite(v) else float(v)


class JointPlots(pg.GraphicsLayoutWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.window = 15.0
        self._data = {}
        self.ci.layout.setSpacing(4)
        self.ci.setContentsMargins(4, 4, 8, 4)
        specs = [("pos", "각도 °"), ("vel", "속도 °/s"), ("tau", "토크 Nm")]
        self.plots = {}
        for row, (key, label) in enumerate(specs):
            p = self.addPlot(row=row, col=0)
            p.setLabel("left", label, color=C["ink2"])
            p.showGrid(x=True, y=True, alpha=0.12)
            p.setMouseEnabled(x=False, y=False)
            p.setMenuEnabled(False)
            p.hideButtons()
            p.enableAutoRange(axis="y")
            p.setXRange(-self.window, 0, padding=0)
            for ax in ("left", "bottom"):
                p.getAxis(ax).setPen(pg.mkPen(C["axis"]))
                p.getAxis(ax).setTextPen(pg.mkPen(C["muted"]))
            p.getAxis("left").setWidth(52)
            if row < len(specs) - 1:
                p.getAxis("bottom").setStyle(showValues=False)
            else:
                p.setLabel("bottom", "시간 [s]  (0 = 지금)", color=C["ink2"])
            if row > 0:
                p.setXLink(self.plots["pos"])
            self.plots[key] = p
        pp, pv, pt = self.plots["pos"], self.plots["vel"], self.plots["tau"]
        leg = pp.addLegend(offset=(8, 4), labelTextColor=C["ink2"], brush=pg.mkBrush(13, 13, 13, 170))
        leg.setColumnCount(3)
        self.c = {
            "pos_real": pp.plot(pen=_pen(C["real"]), name="실측"),
            "pos_target": pp.plot(pen=_pen(C["target"], dash=True), name="목표"),
            "pos_sim": pp.plot(pen=_pen(C["sim"]), name="시뮬"),
            "vel_real": pv.plot(pen=_pen(C["real"])),
            "vel_sim": pv.plot(pen=_pen(C["sim"])),
            "tau_real": pt.plot(pen=_pen(C["real"])),
            "tau_sim": pt.plot(pen=_pen(C["sim"])),
        }
        self.limits = [pg.InfiniteLine(angle=0, pen=pg.mkPen(C["muted"], width=1, style=Qt.DotLine),
                                       label=lab, labelOpts={"color": C["muted"], "position": 0.88})
                       for lab in ("시뮬 하한", "시뮬 상한")]
        for ln in self.limits:
            pp.addItem(ln, ignoreBounds=True)
            ln.hide()
        # 마우스 세로선과 값
        self.vlines, self.readouts = {}, {}
        for key, p in self.plots.items():
            vl = pg.InfiniteLine(angle=90, pen=pg.mkPen(C["ink2"], width=1, style=Qt.DashLine))
            vl.hide()
            p.addItem(vl, ignoreBounds=True)
            tx = pg.TextItem(color=C["ink"], anchor=(1, 0), fill=pg.mkBrush(13, 13, 13, 200))
            tx.hide()
            p.addItem(tx, ignoreBounds=True)
            self.vlines[key], self.readouts[key] = vl, tx
        self._hover_x = None
        self.scene().sigMouseMoved.connect(self._on_mouse)

    def set_window(self, seconds):
        self.window = float(seconds)
        self.plots["pos"].setXRange(-self.window, 0, padding=0)

    def set_limits(self, lo_deg, hi_deg):
        for ln, v in zip(self.limits, (lo_deg, hi_deg)):
            if v is None:
                ln.hide()
            else:
                ln.setValue(v)
                ln.show()

    def set_tau_label(self, text):
        self.plots["tau"].setLabel("left", text, color=C["ink2"])

    def update_data(self, s):
        """s: App.series() 결과."""
        empty = (np.empty(0), np.empty(0))
        self._data = {}
        if "real" in s:
            x, q, v = s["real"]
            self.c["pos_real"].setData(x, q)
            self.c["vel_real"].setData(x, v)
            self._data["pos_real"], self._data["vel_real"] = (x, q), (x, v)
        else:
            self.c["pos_real"].setData(*empty)
            self.c["vel_real"].setData(*empty)
        if "target" in s:
            x, q = s["target"]
            self.c["pos_target"].setData(x, q, connect="finite")
            self._data["pos_target"] = (x, q)
        else:
            self.c["pos_target"].setData(*empty)
        if "sim" in s:
            x, q, v, tau = s["sim"]
            self.c["pos_sim"].setData(x, q)
            self.c["vel_sim"].setData(x, v)
            self.c["tau_sim"].setData(x, tau)
            self._data["pos_sim"], self._data["vel_sim"], self._data["tau_sim"] = (x, q), (x, v), (x, tau)
        else:
            for k in ("pos_sim", "vel_sim", "tau_sim"):
                self.c[k].setData(*empty)
        if "real_tau" in s:
            x, tau = s["real_tau"]
            self.c["tau_real"].setData(x, tau)
            self._data["tau_real"] = (x, tau)
        else:
            self.c["tau_real"].setData(*empty)
        self._update_readouts()

    # ---- 마우스
    def _on_mouse(self, pos):
        hit = None
        for key, p in self.plots.items():
            if p.sceneBoundingRect().contains(pos):
                hit = p
                break
        if hit is None:
            self._hover_x = None
        else:
            x = hit.vb.mapSceneToView(pos).x()
            self._hover_x = x if -self.window <= x <= 0 else None
        self._update_readouts()

    def leaveEvent(self, ev):
        self._hover_x = None
        self._update_readouts()
        super().leaveEvent(ev)

    def _update_readouts(self):
        x = self._hover_x
        for key, p in self.plots.items():
            vl, tx = self.vlines[key], self.readouts[key]
            if x is None:
                vl.hide()
                tx.hide()
                continue
            vl.setValue(x)
            vl.show()
            unit = {"pos": "°", "vel": "°/s", "tau": "Nm"}[key]
            parts = []
            for name, lab in (("real", "실측"), ("target", "목표"), ("sim", "시뮬")):
                d = self._data.get(f"{key}_{name}")
                v = _nearest(*d, x) if d else None
                if v is not None:
                    parts.append(f"{lab} {v:+.2f}")
            tx.setText(f"{x:.2f} s   " + ("   ".join(parts) if parts else "값 없음") + f"  {unit}")
            (x0, x1), (y0, y1) = p.vb.viewRange()
            tx.setPos(x1, y1)
            tx.show()
