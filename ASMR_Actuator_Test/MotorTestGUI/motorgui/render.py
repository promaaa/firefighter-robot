"""MuJoCo 화면 밖(offscreen) 렌더링과 3D 보기 위젯.

GL 컨텍스트 하나로 두 화면(실물, 시뮬)을 번갈아 그린 뒤 픽셀을 읽어 Qt 위젯에 그림으로 붙인다.
마우스: 왼쪽 끌기 = 회전, 오른쪽 끌기 = 이동, 휠 = 확대/축소, 두 번 클릭 = 기본 시점.
"""
import math

import mujoco
import numpy as np
from PySide6.QtCore import QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath
from PySide6.QtWidgets import QSizePolicy, QWidget

from .theme import C, RGBA

MAX_W, MAX_H = 1920, 1200

VIEW_PRESETS = {
    # 로봇은 +X를 본다. azimuth는 카메라가 바라보는 방향의 각도.
    "비스듬히": dict(azimuth=215.0, elevation=-14.0),
    "정면": dict(azimuth=180.0, elevation=-5.0),
    "왼쪽": dict(azimuth=270.0, elevation=-5.0),
    "오른쪽": dict(azimuth=90.0, elevation=-5.0),
    "뒤": dict(azimuth=0.0, elevation=-5.0),
}


def new_camera(preset="비스듬히"):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, 0.36]
    cam.distance = 1.55
    apply_preset(cam, preset)
    return cam


def apply_preset(cam, preset):
    p = VIEW_PRESETS[preset]
    cam.azimuth = p["azimuth"]
    cam.elevation = p["elevation"]


def copy_camera(src):
    cam = mujoco.MjvCamera()
    cam.type = src.type
    cam.lookat[:] = src.lookat
    cam.distance, cam.azimuth, cam.elevation = src.distance, src.azimuth, src.elevation
    return cam


class OffscreenRenderer:
    def __init__(self, model):
        self.m = model
        model.vis.global_.offwidth = MAX_W
        model.vis.global_.offheight = MAX_H
        self.ctx = mujoco.GLContext(MAX_W, MAX_H)
        self.ctx.make_current()
        self.con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.con)
        self.max_w, self.max_h = self.con.offWidth, self.con.offHeight
        self.opt = mujoco.MjvOption()
        self.opt.frame = mujoco.mjtFrame.mjFRAME_WORLD
        self.pert = mujoco.MjvPerturb()
        # 기하(geom) -> 그 몸체를 움직이는 관절 id
        self.geom_joint = {}
        for g in range(model.ngeom):
            b = int(model.geom_bodyid[g])
            self.geom_joint[g] = int(model.body_jntadr[b]) if model.body_jntnum[b] > 0 else -1
        self._buf = None

    def new_scene(self):
        return mujoco.MjvScene(self.m, maxgeom=3000)

    def render(self, scn, cam, data, w, h, joint_rgba, selected_jid=-1, ghost=None, ghost_rgba=None,
               arrow_data=None):
        """joint_rgba: {joint id: rgba}. 관절이 없는 몸체(몸통)는 base 색."""
        self.ctx.make_current()
        w, h = max(16, min(int(w), self.max_w)), max(16, min(int(h), self.max_h))
        mujoco.mjv_updateScene(self.m, data, self.opt, self.pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
        geom_obj = int(mujoco.mjtObj.mjOBJ_GEOM)
        for i in range(scn.ngeom):
            sg = scn.geoms[i]
            if sg.objtype != geom_obj:
                continue
            g = sg.objid
            if self.m.geom_bodyid[g] == 0:      # 바닥 등 world 몸체
                continue
            jid = self.geom_joint.get(g, -1)
            rgba = joint_rgba.get(jid, RGBA["base"]) if jid >= 0 else RGBA["base"]
            sg.rgba[:] = rgba
            sg.emission = 0.45 if (jid == selected_jid and jid >= 0) else 0.0
        if ghost is not None:
            # 겹쳐 그리기는 본 자세와 4 mm / 1.5도 넘게 다른 링크만 (같은 곳은 투명 0 = 안 보임)
            differs = self._pose_differs(data, ghost)
            n0 = scn.ngeom
            mujoco.mjv_addGeoms(self.m, ghost, self.opt, self.pert, mujoco.mjtCatBit.mjCAT_DYNAMIC, scn)
            for i in range(n0, scn.ngeom):
                sg = scn.geoms[i]
                show = sg.objtype == geom_obj and differs[sg.objid]
                sg.rgba[:] = ghost_rgba if show else (0.0, 0.0, 0.0, 0.0)
                sg.emission = 0.0
        if selected_jid >= 0:
            self._add_axis_arrow(scn, arrow_data if arrow_data is not None else data, selected_jid)
        vp = mujoco.MjrRect(0, 0, w, h)
        mujoco.mjr_render(vp, scn, self.con)
        if self._buf is None or self._buf.shape[:2] != (h, w):
            self._buf = np.empty((h, w, 3), np.uint8)
        mujoco.mjr_readPixels(self._buf, None, vp, self.con)
        return np.ascontiguousarray(self._buf[::-1])

    @staticmethod
    def _pose_differs(a, b, pos_tol=0.004, ang_tol_deg=1.5):
        dp = np.linalg.norm(a.geom_xpos - b.geom_xpos, axis=1)
        ra = a.geom_xmat.reshape(-1, 3, 3)
        rb = b.geom_xmat.reshape(-1, 3, 3)
        tr = np.einsum("nij,nij->n", ra, rb)            # trace(Ra^T Rb)
        cos = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
        return (dp > pos_tol) | (cos < math.cos(math.radians(ang_tol_deg)))

    def _add_geom(self, scn, kind, width, a, b, rgba):
        if scn.ngeom >= scn.maxgeom:
            return None
        g = scn.geoms[scn.ngeom]
        mujoco.mjv_initGeom(g, kind, np.zeros(3), np.zeros(3), np.eye(3).flatten(), np.asarray(rgba, np.float32))
        mujoco.mjv_connector(g, kind, width, np.asarray(a, float), np.asarray(b, float))
        g.emission = 0.3
        scn.ngeom += 1
        return g

    def _add_axis_arrow(self, scn, data, jid):
        """선택 관절의 회전축 화살표 + 양(+) 회전 방향 호 (오른손 법칙)."""
        axis = np.array(data.xaxis[jid])
        n = np.linalg.norm(axis)
        if n < 1e-9:
            return
        axis /= n
        anchor = np.array(data.xanchor[jid])
        col = RGBA["axis_arrow"]
        tip = self._add_geom(scn, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, anchor - 0.04 * axis, anchor + 0.16 * axis, col)
        if tip is not None:
            tip.label = "+"
        # 호: axis = u x v, theta가 커지는 방향이 + 회전
        tmp = np.array([1.0, 0, 0]) if abs(axis[0]) < 0.9 else np.array([0, 1.0, 0])
        u = np.cross(axis, tmp)
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)
        c = anchor + 0.11 * axis
        r = 0.055
        th = np.linspace(0.0, 1.45 * math.pi, 14)
        pts = [c + r * (math.cos(t) * u + math.sin(t) * v) for t in th]
        for a, b in zip(pts[:-1], pts[1:]):
            self._add_geom(scn, mujoco.mjtGeom.mjGEOM_CAPSULE, 0.005, a, b, col)
        t_end = th[-1]
        tangent = -math.sin(t_end) * u + math.cos(t_end) * v
        self._add_geom(scn, mujoco.mjtGeom.mjGEOM_ARROW, 0.008, pts[-1], pts[-1] + 0.028 * tangent, col)


class RobotView(QWidget):
    camera_changed = Signal()
    reset_requested = Signal()

    def __init__(self, title, accent, model, parent=None):
        super().__init__(parent)
        self.title = title
        self.accent = QColor(accent)
        self.model = model
        self.cam = None
        self.scn = None
        self.lines = []           # 아래쪽 안내 글 [(색, 글)]
        self.badge = ""           # 제목 옆 짧은 상태
        self._img = None
        self._last = None
        self._btn = None
        self.setMinimumSize(260, 220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFocusPolicy(Qt.ClickFocus)
        self.setToolTip("왼쪽 끌기: 회전 · 오른쪽 끌기: 이동 · 휠: 확대/축소 · 두 번 클릭: 기본 시점")

    def render_size(self):
        dpr = self.devicePixelRatioF()
        return int(self.width() * dpr), int(self.height() * dpr)

    def set_frame(self, arr):
        h, w = arr.shape[:2]
        img = QImage(arr.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        img.setDevicePixelRatio(self.devicePixelRatioF())
        self._img = img
        self.update()

    def paintEvent(self, ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(C["surface"]))
        if self._img is not None:
            p.drawImage(QRectF(0, 0, self.width(), self.height()), self._img)
        p.setRenderHint(QPainter.Antialiasing)
        # 제목 칩
        f = QFont(self.font())
        f.setPointSizeF(10.5)
        f.setBold(True)
        p.setFont(f)
        text = self.title + (f"   {self.badge}" if self.badge else "")
        tw = p.fontMetrics().horizontalAdvance(text)
        chip = QRectF(10, 10, tw + 34, 28)
        path = QPainterPath()
        path.addRoundedRect(chip, 6, 6)
        p.fillPath(path, QColor(13, 13, 13, 190))
        p.setBrush(self.accent)
        p.setPen(Qt.NoPen)
        p.drawEllipse(QRectF(chip.left() + 10, chip.center().y() - 5, 10, 10))
        p.setPen(QColor(C["ink"]))
        p.drawText(QRectF(chip.left() + 26, chip.top(), tw + 8, chip.height()), Qt.AlignVCenter, text)
        # 아래 안내
        f.setBold(False)
        f.setPointSizeF(9)
        p.setFont(f)
        y = self.height() - 10
        for col, line in reversed(self.lines):
            lw = p.fontMetrics().horizontalAdvance(line)
            r = QRectF(10, y - 22, lw + 16, 22)
            bg = QPainterPath()
            bg.addRoundedRect(r, 5, 5)
            p.fillPath(bg, QColor(13, 13, 13, 170))
            p.setPen(QColor(col))
            p.drawText(r.adjusted(8, 0, 0, 0), Qt.AlignVCenter, line)
            y -= 26
        p.end()

    # ---- 마우스로 카메라
    def mousePressEvent(self, ev):
        self._btn = ev.button()
        self._last = ev.position()

    def mouseReleaseEvent(self, ev):
        self._btn = None

    def mouseMoveEvent(self, ev):
        if self._btn is None or self.cam is None or self.scn is None:
            return
        pos = ev.position()
        dx, dy = pos.x() - self._last.x(), pos.y() - self._last.y()
        self._last = pos
        h = max(1, self.height())
        if self._btn == Qt.LeftButton and not (ev.modifiers() & Qt.ShiftModifier):
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_V
        elif self._btn == Qt.RightButton or (ev.modifiers() & Qt.ShiftModifier):
            action = mujoco.mjtMouse.mjMOUSE_MOVE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        mujoco.mjv_moveCamera(self.model, action, dx / h, dy / h, self.scn, self.cam)
        self.camera_changed.emit()

    def wheelEvent(self, ev):
        if self.cam is None or self.scn is None:
            return
        steps = ev.angleDelta().y() / 120.0
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * steps, self.scn, self.cam)
        self.camera_changed.emit()

    def mouseDoubleClickEvent(self, ev):
        self.reset_requested.emit()
