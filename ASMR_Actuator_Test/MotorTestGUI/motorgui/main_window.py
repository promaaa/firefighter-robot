"""메인 창: 위 = 연결 상태 / 왼쪽 = 실물 3D · 시뮬 3D · 관절 표 / 오른쪽 = 관절 패널."""
import math

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QAction, QKeySequence, QShortcut
from PySide6.QtWidgets import (QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel, QMainWindow, QMessageBox,
                               QPlainTextEdit, QPushButton, QSplitter, QTabWidget, QVBoxLayout, QWidget)

from .render import OffscreenRenderer, RobotView, VIEW_PRESETS, apply_preset, copy_camera, new_camera
from .theme import C, RGBA
from .widgets import JointPanel, JointTable, TopBar

FRAME_MS = 16

HELP_TEXT = """<h3>MotorTestGUI</h3>
<p><b>왼쪽 3D</b>: 실물 로봇. CAN으로 읽은 엔코더 값을 관절 각도로 바꿔 수업 MJCF 모델에 입힌 그림입니다.
<b>오른쪽 3D</b>: MuJoCo 시뮬레이션. 몸통을 고정한 같은 모델이 실물과 <b>같은 목표 궤적</b>을 받아 움직입니다.</p>
<p><b>같게 세팅됐는지 보는 법</b></p>
<ol>
<li>실물 로봇을 받침대에 매달고 전원을 켭니다. 다리를 곧게(시뮬 0° 자세) 둔 상태에서 켜는 것이 좋습니다.</li>
<li>위쪽에서 CAN 어댑터와 <b>포트 1·2</b>를 고르고 <b>연결</b> (CANable 두 개면 둘 다, 하나면 포트 2는 '사용 안 함').
노드 12개가 초록이 되는지 봅니다 (회색 = 보이지 않음). 어느 노드가 어느 버스인지는 자동으로 알아내고 관절 표 node 칸에 나옵니다.</li>
<li>왼쪽 그림이 실물 자세와 같은지 봅니다. 처음에는 영점이 없어 관절이 보라색(미보정)입니다.
'영점·방향 설정' 탭 → <b>모든 관절 지금 = 0°</b> (다리를 곧게 편 자세에서), 또는 <b>범위 훑기</b>
(시작 → 힘을 푼 채 관절을 양쪽 기구 끝까지 → 끝 → 결과 확인 후 적용).</li>
<li>관절을 고르고 <b>토크 켜기</b> → <b>+5°</b>. 노란 화살표(회전축)와 호(+ 방향)를 보고, 실물이 왼쪽 그림처럼
움직였는지 확인합니다. 반대면 힘 풀기 → 방향 −1.</li>
<li>오른쪽 시뮬이 같은 쪽으로 같은 만큼 움직이는지, 그래프에서 실측(파랑)·목표(주황 점선)·시뮬(청록)을 비교합니다.
토크 그래프의 부호가 반대면 '토크 부호(Iq)'를 바꿉니다.</li>
<li>힘을 푼 채 손으로 관절을 끝까지 돌려 관절 표의 '실측 기록 범위'를 '시뮬 범위'와 비교합니다.</li>
<li><b>설정 저장</b> (MotorTestGUI/config/robot_config.json).</li>
</ol>
<p><b>안전</b>: 한 번 명령은 '한 번 최대'(기본 30°)까지, 영점이 있으면 시뮬 관절 범위 안으로 자릅니다.
<b>Esc</b> 또는 빨간 버튼 = 모든 노드 IDLE. 실제 비상 정지는 전원 차단 스위치로 하세요.</p>
<p><b>각도 모호함</b>: 코스 펌웨어는 전원을 켤 때 모터 1회전(관절 24°) 안의 위치만 압니다. 노드가 처음 보일 때
관절이 '기준 자세' ±12° 안에 있다고 보고 각도를 맞추므로, 다른 자세에서 켰다면 기준 자세로 옮긴 뒤
'각도 다시 맞추기'를 누릅니다.</p>
<p><b>3D 조작</b>: 왼쪽 끌기 회전, 오른쪽 끌기 이동, 휠 확대, 두 번 클릭 기본 시점.</p>
"""


class MainWindow(QMainWindow):
    def __init__(self, app, virtual_kwargs=None, autoconnect=False):
        super().__init__()
        self.app = app
        self.virtual_kwargs = virtual_kwargs or {}
        self.setWindowTitle("MotorTestGUI · 실물 ↔ 시뮬 비교")
        self.resize(1680, 980)
        self.renderer = OffscreenRenderer(app.model)
        self.cam = new_camera()
        self.cam_sim = self.cam
        self._frame_n = 0
        self._log_seen = 0
        self._build()
        self._wire()
        self.panel.select(app.cfg.joints[0].name)
        self.table.select(app.cfg.joints[0].name)
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self._frame)
        self.timer.start(FRAME_MS)
        if autoconnect:
            QTimer.singleShot(300, lambda: self.topbar._connect_clicked())

    # ------------------------------------------------------------------ 화면 구성
    def _build(self):
        app = self.app
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        self.topbar = TopBar(app, virtual_buses=int(self.virtual_kwargs.get("buses", 2)))
        root.addWidget(self.topbar)

        main_split = QSplitter(Qt.Horizontal)
        root.addWidget(main_split, 1)

        # 왼쪽: 3D 두 개 + 표
        left = QSplitter(Qt.Vertical)
        views_box = QWidget()
        vb = QVBoxLayout(views_box)
        vb.setContentsMargins(6, 6, 6, 0)
        vb.setSpacing(4)
        bar = QHBoxLayout()
        bar.addWidget(QLabel("시점"))
        for name in VIEW_PRESETS:
            b = QPushButton(name)
            b.clicked.connect(lambda _=False, n=name: self._preset(n))
            bar.addWidget(b)
        self.sync_cam = QCheckBox("두 화면 시점 같이")
        self.sync_cam.setChecked(True)
        self.ghost_target = QCheckBox("왼쪽에 목표 자세 겹치기")
        self.ghost_target.setChecked(True)
        self.ghost_real = QCheckBox("오른쪽에 실물 자세 겹치기")
        self.ghost_real.setChecked(True)
        bar.addSpacing(10)
        for w in (self.sync_cam, self.ghost_target, self.ghost_real):
            bar.addWidget(w)
        bar.addStretch(1)
        bar.addWidget(QLabel("시뮬 대기 관절"))
        self.idle_mode = QComboBox()
        self.idle_mode.addItem("실물 자세 따라가기", "follow")
        self.idle_mode.addItem("힘 풀림 (중력)", "passive")
        self.idle_mode.setToolTip("목표가 없는(토크를 켜지 않은) 관절을 시뮬에서 어떻게 둘지.\n"
                                  "따라가기: 시뮬이 실물 각도를 PD로 따라감 (명령 전에 두 화면을 맞춰 둠)\n"
                                  "힘 풀림: actuator를 끄고 중력·마찰만 (실물이 힘 풀린 채 매달린 모습과 비교)")
        bar.addWidget(self.idle_mode)
        self.sync_btn = QPushButton("시뮬 ← 실물 자세")
        self.sync_btn.setToolTip("시뮬 관절 위치를 지금 실측 각도로 바로 옮깁니다")
        self.reset_btn = QPushButton("시뮬 초기화")
        bar.addWidget(self.sync_btn)
        bar.addWidget(self.reset_btn)
        vb.addLayout(bar)

        vsplit = QSplitter(Qt.Horizontal)
        self.view_real = RobotView("실물 로봇 · 엔코더", C["real"], app.model)
        self.view_sim = RobotView("시뮬레이션 · MuJoCo", C["sim"], app.model)
        self.view_real.cam, self.view_real.scn = self.cam, self.renderer.new_scene()
        self.view_sim.cam, self.view_sim.scn = self.cam, self.renderer.new_scene()
        vsplit.addWidget(self.view_real)
        vsplit.addWidget(self.view_sim)
        vb.addWidget(vsplit, 1)
        left.addWidget(views_box)

        tabs = QTabWidget()
        self.table = JointTable(app)
        tabs.addTab(self.table, "관절 표")
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        tabs.addTab(self.log_view, "기록")
        left.addWidget(tabs)
        left.setStretchFactor(0, 3)
        left.setStretchFactor(1, 1)
        left.setSizes([600, 330])
        main_split.addWidget(left)

        self.panel = JointPanel(app)
        self.panel.setMinimumWidth(500)
        main_split.addWidget(self.panel)
        main_split.setStretchFactor(0, 3)
        main_split.setStretchFactor(1, 2)
        main_split.setSizes([1100, 580])
        self.setCentralWidget(central)

        # 메뉴
        m = self.menuBar().addMenu("파일")
        a = QAction("설정 저장", self)
        a.setShortcut(QKeySequence.Save)
        a.triggered.connect(self._save)
        m.addAction(a)
        a = QAction("설정 다른 이름으로 저장…", self)
        a.triggered.connect(lambda: self._save(ask=True))
        m.addAction(a)
        m.addSeparator()
        a = QAction("종료", self)
        a.triggered.connect(self.close)
        m.addAction(a)
        h = self.menuBar().addMenu("도움말")
        a = QAction("사용법", self)
        a.setShortcut(QKeySequence.HelpContents)
        a.triggered.connect(lambda: QMessageBox.information(self, "사용법", HELP_TEXT))
        h.addAction(a)
        self.statusBar().showMessage(f"모델: {app.mjcf_path.name} · {app.model_note} · 설정: {app.cfg.path}")

    def _wire(self):
        tb, app = self.topbar, self.app
        tb.connect_requested.connect(self._connect)
        tb.disconnect_requested.connect(self._disconnect)
        tb.estop_requested.connect(self._estop)
        tb.node_clicked.connect(self._select)
        self.table.joint_clicked.connect(self._select)
        self.panel.joint_changed.connect(lambda n: self.table.select(n))
        self.panel.message.connect(lambda text, err: self.statusBar().showMessage(text, 8000))
        self.panel.save_btn.clicked.connect(self._save)
        self.sync_cam.toggled.connect(self._sync_toggled)
        self.idle_mode.currentIndexChanged.connect(lambda _: setattr(app, "sim_idle_mode",
                                                                     self.idle_mode.currentData()))
        self.sync_btn.clicked.connect(app.sync_sim_to_real)
        self.reset_btn.clicked.connect(app.reset_sim)
        for v in (self.view_real, self.view_sim):
            v.reset_requested.connect(lambda: self._preset("비스듬히"))
        sc = QShortcut(QKeySequence(Qt.Key_Escape), self)
        sc.setContext(Qt.ApplicationShortcut)
        sc.activated.connect(self._estop)

    # ------------------------------------------------------------------ 동작
    def _select(self, name):
        self.panel.select(name)
        self.table.select(name)

    def _preset(self, name):
        apply_preset(self.cam, name)
        if self.cam_sim is not self.cam:
            apply_preset(self.cam_sim, name)

    def _sync_toggled(self, on):
        if on:
            self.cam_sim = self.cam
        else:
            self.cam_sim = copy_camera(self.cam)
        self.view_sim.cam = self.cam_sim

    def _connect(self, iface, channels, bitrate):
        if iface == "slcan" and (not channels or not channels[0]):
            QMessageBox.warning(self, "CAN 연결", "포트 1의 COM 포트를 찾지 못했습니다.\n"
                                "CANable을 꽂고 ↻를 누르거나 포트를 직접 입력하세요.")
            return
        err = self.app.connect(iface, channels, bitrate, self.virtual_kwargs if iface == "virtual" else None)
        if err:
            QMessageBox.warning(self, "CAN 연결", err)

    def _disconnect(self):
        if self.app.any_real_enabled():
            r = QMessageBox.question(self, "연결 끊기", "토크가 켜진 관절이 있습니다. 모두 힘을 풀고 연결을 끊을까요?")
            if r != QMessageBox.Yes:
                return
        self.app.disconnect()

    def _estop(self):
        self.app.idle_all()
        self.statusBar().showMessage("모든 노드 IDLE 명령, 모든 목표 지움", 8000)

    def _save(self, ask=False):
        path = None
        if ask:
            path, _ = QFileDialog.getSaveFileName(self, "설정 저장", str(self.app.cfg.path), "JSON (*.json)")
            if not path:
                return
        p = self.app.cfg.save(path)
        self.app.dirty = False
        self.app.log(f"설정 저장: {p}")
        self.statusBar().showMessage(f"설정 저장: {p}", 8000)

    # ------------------------------------------------------------------ 매 프레임
    def _frame(self):
        app = self.app
        app.update()
        self._frame_n += 1
        n = self._frame_n
        self._render_views(real=(n % 2 == 0))
        if n % 3 == 1:
            self.panel.refresh_plots()
        if n % 6 == 0:
            self.topbar.refresh()
            self.panel.refresh()
            while app.link.alerts:
                text = app.link.alerts.popleft()
                self.panel._show(text, True)
                self.statusBar().showMessage(text, 15000)
        if n % 12 == 3:
            self.table.refresh()
            self._drain_logs()

    def _joint_colors(self, real_view):
        app = self.app
        out = {}
        counts = {"ok": 0, "uncal": 0, "offline": 0, "error": 0, "nomodel": 0}
        for name, info in app.infos.items():
            if info is None:
                counts["nomodel"] += 1
                continue
            if real_view:
                st = app.status(name)
                counts[st] += 1
                out[info.jid] = {"ok": RGBA["real_ok"], "uncal": RGBA["uncal"], "error": RGBA["error"],
                                 "offline": RGBA["offline"]}[st]
            else:
                out[info.jid] = RGBA["sim_ok"]
        return out, counts

    def _render_views(self, real=True):
        app = self.app
        info = app.infos.get(self.panel.name)
        sel = info.jid if info is not None else -1
        t = app.now()
        has_target = bool(app.cmdr.active_names())
        any_real = any(app.real_q(n) is not None for n in app.cfg.names)

        colors, counts = self._joint_colors(True)
        v = self.view_real
        if real and v.isVisible() and v.width() > 20:
            w, h = v.render_size()
            ghost = app.target_data if (self.ghost_target.isChecked() and has_target) else None
            v.set_frame(self.renderer.render(v.scn, v.cam, app.real_data, w, h, colors, sel, ghost,
                                             RGBA["ghost_target"]))
            total = len(app.cfg.joints)
            lines = []
            if not app.link.connected:
                lines.append((C["muted"], "실물 연결 안 됨: 모든 관절을 0° 자세로 그림"))
            else:
                lines.append((C["ink2"], f"연결 {total - counts['offline'] - counts['nomodel']}/{total} · "
                                         f"미보정 {counts['uncal']} · 에러 {counts['error']}"))
                if counts["offline"]:
                    lines.append((C["muted"], "반투명 회색 = 응답 없는 관절 (0°로 그림)"))
                if counts["uncal"]:
                    lines.append((C["uncal"], "보라 = 영점 없음 (각도가 시뮬 기준이 아님)"))
            if ghost is not None:
                lines.append((C["target"], "주황 반투명 = 목표 자세 (실측과 다른 링크만)"))
            v.lines = lines
            j = app.cfg.joint(self.panel.name)
            q = app.real_q(self.panel.name)
            v.badge = f"{j.short} {'—' if q is None else f'{math.degrees(q):+.1f}°'}"

        v = self.view_sim
        if not real and v.isVisible() and v.width() > 20:
            w, h = v.render_size()
            scolors, _ = self._joint_colors(False)
            ghost = app.real_data if (self.ghost_real.isChecked() and any_real) else None
            v.set_frame(self.renderer.render(v.scn, v.cam, app.sim.d, w, h, scolors, sel, ghost, RGBA["ghost_real"]))
            mode = "실물 자세 따라가기" if app.sim_idle_mode == "follow" else "힘 풀림(중력)"
            lines = [(C["ink2"], f"몸통 고정 · dt {app.sim.dt * 1000:.0f} ms · 목표 없는 관절: {mode}")]
            if ghost is not None:
                lines.append((C["real"], "파랑 반투명 = 실물 자세 (시뮬과 다른 링크만)"))
            v.lines = lines
            qs = app.sim.q(self.panel.name)
            v.badge = f"{app.cfg.joint(self.panel.name).short} {'—' if qs is None else f'{math.degrees(qs):+.1f}°'}"

    def _drain_logs(self):
        logs = list(self.app.logs) + list(self.app.link.logs)
        logs.sort(key=lambda r: r[0])
        new = [r for r in logs if r[0] > self._log_seen]
        for t, text in new:
            self.log_view.appendPlainText(f"[{t:8.2f} s] {text}")
        if new:
            self._log_seen = new[-1][0]

    # ------------------------------------------------------------------ 종료
    def closeEvent(self, ev):
        app = self.app
        if app.link.running and app.any_real_enabled():
            r = QMessageBox.question(self, "종료", "토크가 켜진 관절이 있습니다. 모두 힘을 풀고 종료할까요?")
            if r != QMessageBox.Yes:
                ev.ignore()
                return
        if app.dirty:
            r = QMessageBox.question(self, "종료", "영점·방향 설정이 바뀌었습니다. 저장할까요?",
                                     QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
            if r == QMessageBox.Cancel:
                ev.ignore()
                return
            if r == QMessageBox.Save:
                self._save()
        self.timer.stop()
        if app.link.running:
            app.disconnect()
        ev.accept()
