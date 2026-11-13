"""화면 구성 요소: 위쪽 연결 상태 줄, 관절 표, 오른쪽 관절 패널."""
import math

from PySide6.QtCore import QTimer, Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QAbstractItemView, QButtonGroup, QCheckBox, QComboBox, QDialog, QDialogButtonBox,
                               QDoubleSpinBox, QFrame,
                               QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QMessageBox, QSizePolicy,
                               QPushButton, QRadioButton, QScrollArea, QSplitter, QTableWidget, QTableWidgetItem,
                               QTabWidget, QToolButton, QVBoxLayout, QWidget)

from . import protocol as P
from .config import TURN_TO_JOINT_DEG, driver_joint_gains
from .plots import JointPlots
from .theme import C, dot_icon

CANABLE_VIDPID = (0x16D0, 0x117E)


def fmt_deg(q, digits=2):
    return "—" if q is None else f"{math.degrees(q):+.{digits}f}°"


def _vline():
    f = QFrame()
    f.setFrameShape(QFrame.VLine)
    f.setStyleSheet(f"color: {C['border']};")
    return f


def _label(text="", name=None, tip=None):
    lb = QLabel(text)
    if name:
        lb.setObjectName(name)
    if tip:
        lb.setToolTip(tip)
    return lb


def _scroll(w):
    """공간이 모자라면 버튼을 찌그러뜨리지 않고 스크롤."""
    sa = QScrollArea()
    sa.setWidget(w)
    sa.setWidgetResizable(True)
    sa.setFrameShape(QFrame.NoFrame)
    sa.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    return sa


def joint_state(app, name):
    """(색, 짧은 글) - 연결 표시와 표에서 같이 쓴다."""
    j = app.cfg.joint(name)
    ns = app.link.nodes[j.node]
    now = app.now()
    if not app.link.connected:
        return C["off"], "연결 안 됨"
    if not ns.online(now):
        return (C["critical"], "응답 끊김") if ns.ever_seen else (C["off"], "없음")
    if ns.axis_error:
        return C["critical"], "에러"
    if ns.state == P.STATE_CLOSED_LOOP:
        return C["real"], "위치 제어"
    if ns.state == P.STATE_IDLE:
        return C["good"], "힘 풀림"
    return C["warning"], P.state_text(ns.state)


# ====================================================================== 위쪽 연결 상태
class TopBar(QFrame):
    connect_requested = Signal(str, object, int)     # 어댑터 종류, 포트 목록(1~2개), 속도
    NONE2 = "(사용 안 함)"
    disconnect_requested = Signal()
    estop_requested = Signal()
    node_clicked = Signal(str)

    IFACES = [("slcan", "slcan (CANable 출고 펌웨어, COM)"), ("gs_usb", "gs_usb (candleLight)"),
              ("pcan", "pcan"), ("virtual", "가상 로봇 (장치 없이 연습)")]

    def __init__(self, app, virtual_buses=2):
        super().__init__()
        self.app = app
        self.virtual_buses = virtual_buses
        self.setObjectName("topbar")
        self.setStyleSheet(f"#topbar {{ background: {C['surface']}; border-bottom: 1px solid {C['border']}; }}")
        h = QHBoxLayout(self)
        h.setContentsMargins(10, 6, 10, 6)
        h.setSpacing(10)

        # 연결 상태
        box = QVBoxLayout()
        box.setSpacing(0)
        self.state_lbl = QLabel()
        f = self.state_lbl.font()
        f.setPointSizeF(12)
        f.setBold(True)
        self.state_lbl.setFont(f)
        self.state_detail = _label("", "muted")
        self.state_detail.setMaximumWidth(330)
        box.addWidget(self.state_lbl)
        box.addWidget(self.state_detail)
        h.addLayout(box)
        h.addWidget(_vline())

        # 어댑터 선택
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(3)
        self.iface = QComboBox()
        for key, text in self.IFACES:
            self.iface.addItem(text, key)
        self.channel = QComboBox()
        self.channel.setEditable(True)
        self.channel.setMinimumWidth(140)
        self.channel.setToolTip("포트 1: CANable 하나 (예: 왼다리 버스)")
        self.channel2 = QComboBox()
        self.channel2.setEditable(True)
        self.channel2.setMinimumWidth(140)
        self.channel2.setToolTip("포트 2: 두 번째 CANable (예: 오른다리 버스). 하나만 쓰면 '사용 안 함'.\n"
                                 "어느 노드가 어느 버스에 있는지는 연결하면 자동으로 알아냅니다.")
        self.refresh_btn = QToolButton()
        self.refresh_btn.setText("↻")
        self.refresh_btn.setToolTip("COM 포트 목록 다시 읽기")
        self.bitrate = QComboBox()
        for b in (500000, 250000, 1000000):
            self.bitrate.addItem(f"{b // 1000} kbps", b)
        self.connect_btn = QPushButton("연결")
        self.connect_btn.setObjectName("primary")
        self.connect_btn.setMinimumWidth(70)
        grid.addWidget(_label("CAN 어댑터", "muted"), 0, 0)
        grid.addWidget(self.iface, 0, 1, 1, 4)
        grid.addWidget(_label("포트 1·2", "muted"), 1, 0)
        grid.addWidget(self.channel, 1, 1)
        grid.addWidget(self.channel2, 1, 2)
        grid.addWidget(self.refresh_btn, 1, 3)
        grid.addWidget(self.bitrate, 1, 4)
        grid.addWidget(self.connect_btn, 0, 5, 2, 1)
        h.addLayout(grid)
        h.addWidget(_vline())

        # 노드 표시
        ng = QGridLayout()
        ng.setHorizontalSpacing(3)
        ng.setVerticalSpacing(3)
        self.leds = {}
        left = [j for j in app.cfg.joints if "_left_" in j.name]
        right = [j for j in app.cfg.joints if "_right_" in j.name]
        for row, (side, js) in enumerate((("L", left), ("R", right))):
            ng.addWidget(_label(side, "muted"), row, 0)
            for col, j in enumerate(js):
                b = QToolButton()
                b.setText(str(j.node))
                b.setFixedSize(34, 22)
                b.clicked.connect(lambda _=False, n=j.name: self.node_clicked.emit(n))
                ng.addWidget(b, row, col + 1)
                self.leds[j.name] = b
        nbox = QVBoxLayout()
        nbox.setSpacing(1)
        legend = _label("노드  ● 힘 풀림  ● 위치 제어  ● 에러·끊김  ● 없음", "muted",
                        "초록: 연결됨·IDLE / 파랑: 위치 제어 / 빨강: 에러 또는 응답 끊김 / 회색: 한 번도 안 보임. "
                        "누르면 그 관절을 고릅니다.")
        legend.setText(f"노드  <span style='color:{C['good']}'>●</span> 힘 풀림  "
                       f"<span style='color:{C['real']}'>●</span> 위치 제어  "
                       f"<span style='color:{C['critical']}'>●</span> 에러·끊김  "
                       f"<span style='color:{C['off']}'>●</span> 없음")
        nbox.addWidget(legend)
        nbox.addLayout(ng)
        h.addLayout(nbox)
        h.addWidget(_vline())

        # 통계
        sbox = QVBoxLayout()
        sbox.setSpacing(0)
        self.vbus_lbl = _label("Vbus —", "value")
        self.stats_lbl = _label("", "muted")
        sbox.addWidget(self.vbus_lbl)
        sbox.addWidget(self.stats_lbl)
        h.addLayout(sbox)
        h.addStretch(1)

        self.estop = QPushButton("■ 모두 힘 풀기 (Esc)")
        self.estop.setObjectName("estop")
        self.estop.setToolTip("모든 노드에 IDLE 명령을 보내고 모든 목표를 지웁니다.\n"
                              "실제 비상 정지는 전원 차단 스위치로 하세요 (수업 Real Robot Safety).")
        h.addWidget(self.estop)

        self.iface.currentIndexChanged.connect(self._iface_changed)
        self.refresh_btn.clicked.connect(lambda: self.refresh_ports(keep=True))
        self.connect_btn.clicked.connect(self._connect_clicked)
        self.estop.clicked.connect(self.estop_requested.emit)

        cfg = app.cfg
        i = self.iface.findData(cfg.can_interface)
        self.iface.setCurrentIndex(max(0, i))
        bi = self.bitrate.findData(cfg.can_bitrate)
        self.bitrate.setCurrentIndex(max(0, bi))
        self._iface_changed()

    def _iface_changed(self):
        kind = self.iface.currentData()
        self.channel.clear()
        self.channel2.clear()
        self.channel.setEnabled(kind != "virtual")
        self.channel2.setEnabled(kind != "virtual")
        self.refresh_btn.setEnabled(kind == "slcan")
        self.bitrate.setEnabled(kind != "virtual")
        if kind == "slcan":
            self.refresh_ports(keep=False)
        elif kind == "gs_usb":
            self.channel.addItem("0", "0")
            self.channel2.addItem(self.NONE2, "")
            self.channel2.addItem("1", "1")
        elif kind == "pcan":
            self.channel.addItem("PCAN_USBBUS1", "PCAN_USBBUS1")
            self.channel2.addItem(self.NONE2, "")
            self.channel2.addItem("PCAN_USBBUS2", "PCAN_USBBUS2")
        else:
            two = self.virtual_buses == 2
            self.channel.addItem("(가상 버스 1)" if two else "(장치 없음)", "")
            self.channel2.addItem("(가상 버스 2)" if two else "(장치 없음)", "")

    def refresh_ports(self, keep=True):
        """COM 포트 목록을 다시 채운다.

        keep=True(↻ 버튼): 지금 고른 포트를 그대로 둔다 ('사용 안 함'도 유지).
        keep=False(처음·어댑터 바꿀 때): 저장된 포트, 없으면 CANable을 찾은 순서대로 포트 1·2에 넣는다.
        """
        cfg = self.app.cfg
        if keep:
            want1, want2 = self._channel_value(self.channel), self._channel_value(self.channel2)
        else:
            want1, want2 = cfg.can_channel, cfg.can_channel2
        auto = not keep and not cfg.can_channel
        self.channel.clear()
        self.channel2.clear()
        self.channel2.addItem(self.NONE2, "")
        try:
            from serial.tools import list_ports
            ports = sorted(list_ports.comports(), key=lambda p: p.device)
        except Exception:  # noqa: BLE001
            ports = []
        canables = []
        for p in ports:
            is_canable = (p.vid, p.pid) == CANABLE_VIDPID or "canable" in (p.description or "").lower()
            label = f"{p.device}  {'CANable' if is_canable else (p.description or '')}"
            self.channel.addItem(label, p.device)
            self.channel2.addItem(label, p.device)
            if is_canable:
                canables.append(p.device)
        if not ports:
            self.channel.addItem("(COM 포트 없음)", "")

        def pick(combo, dev):
            i = combo.findData(dev) if dev else -1
            if i >= 0:
                combo.setCurrentIndex(i)
            return i >= 0

        if not pick(self.channel, want1) and canables:
            pick(self.channel, canables[0])
        first = self._channel_value(self.channel)
        if auto:
            rest = [c for c in canables if c != first]
            if rest:
                pick(self.channel2, rest[0])
        elif want2 and want2 != first:
            pick(self.channel2, want2)

    def _channel_value(self, combo):
        data = combo.currentData()
        text = combo.currentText().strip()
        if data is not None and combo.itemText(combo.currentIndex()) == text:
            return data
        if not text or text == self.NONE2:
            return ""
        return text.split()[0]

    def _connect_clicked(self):
        if self.app.link.running:
            self.disconnect_requested.emit()
            return
        ch1, ch2 = self._channel_value(self.channel), self._channel_value(self.channel2)
        chans = [ch1] + ([ch2] if ch2 else [])
        self.connect_requested.emit(self.iface.currentData(), chans, int(self.bitrate.currentData()))

    def refresh(self):
        app, link = self.app, self.app.link
        now = app.now()
        st = link.state
        col = {"connected": C["good"], "connecting": C["warning"], "error": C["critical"]}.get(st, C["muted"])
        text = {"connected": "연결됨", "connecting": "연결 중…", "error": "오류",
                "disconnected": "연결 안 됨"}[st]
        if st == "connected" and link.virtual is not None:
            text = "연결됨 (가상)"
        self.state_lbl.setText(f"<span style='color:{col}'>●</span> 실물 로봇 {text}")
        self.state_detail.setText(link.message if st != "connected" else link.descr)
        self.state_detail.setToolTip(link.message)
        running = link.running
        self.connect_btn.setText("끊기" if running else "연결")
        for w in (self.iface, self.bitrate, self.refresh_btn):
            w.setEnabled(not running and (w is self.iface or self.iface.currentData() != "virtual"))
        self.channel.setEnabled(not running and self.iface.currentData() != "virtual")
        self.channel2.setEnabled(not running and self.iface.currentData() != "virtual")
        for name, b in self.leds.items():
            colr, short = joint_state(app, name)
            j = app.cfg.joint(name)
            ns = link.nodes[j.node]
            fg = "#ffffff" if colr not in (C["off"],) else C["muted"]
            b.setStyleSheet(f"QToolButton {{ background: {colr}; color: {fg}; border-radius: 4px; border: none;"
                            f" font-weight: bold; }}")
            err = P.decode_bits(ns.axis_error, P.AXIS_ERRORS)
            bi = link.bus_of(j.node)
            where = (f" · 버스 {bi + 1} ({link.channels[bi]})"
                     if link.n_buses > 1 and bi is not None and bi < len(link.channels) else "")
            tip = f"{j.short} · node {j.node} · {j.motor}{where}\n{short}"
            if ns.vbus is not None and ns.online(now):
                tip += f" · Vbus {ns.vbus:.2f} V"
            if err:
                tip += f"\naxis error: {err}"
            b.setToolTip(tip)
        vb = [ns.vbus for ns in link.nodes.values() if ns.vbus is not None and ns.online(now)]
        self.vbus_lbl.setText(f"Vbus {min(vb):.2f} V" + (f" ~ {max(vb):.2f}" if max(vb) - min(vb) > 0.2 else "")
                              if vb else "Vbus —")
        if st == "connected":
            s = link.stats
            lats = [ns.latency for ns in link.nodes.values() if ns.latency is not None and ns.online(now)]
            lat = f" · 응답 {1000 * sum(lats) / len(lats):.1f} ms" if lats else ""
            n_on = sum(ns.online(now) for ns in link.nodes.values())
            unk = f" · 설정 밖 node {sorted(link.unknown)}" if link.unknown else ""
            loads = s.get("loads") or [s["load"]]
            if len(loads) > 1:
                cnt = [sum(1 for n, b in link.node_bus.items() if b == i and n in link.nodes
                           and link.nodes[n].online(now)) for i in range(len(loads))]
                busl = " · ".join(f"버스{i + 1} {100 * x:.0f}% ({cnt[i]}개)" for i, x in enumerate(loads))
            else:
                busl = f"버스 약 {100 * loads[0]:.0f}%"
            self.stats_lbl.setText(f"노드 {n_on}/{len(link.nodes)} · 폴링 {s['poll_hz']:.0f} Hz · "
                                   f"수신 {s['rx_fps']:.0f}/s · {busl}{lat}"
                                   + (f" · 송신 실패 {s['tx_err']}" if s['tx_err'] else "") + unk)
        else:
            self.stats_lbl.setText("")


# ====================================================================== 관절 표
class JointTable(QTableWidget):
    joint_clicked = Signal(str)
    COLS = ["관절", "node", "상태", "실측", "목표", "시뮬", "실측−시뮬", "시뮬 범위", "실측 기록 범위", "영점",
            "방향", "Iq [A]", "알림"]

    def __init__(self, app):
        super().__init__(len(app.cfg.joints), len(self.COLS))
        self.app = app
        self.setHorizontalHeaderLabels(self.COLS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        self.setAlternatingRowColors(True)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.horizontalHeader().setStretchLastSection(True)
        self._cache = {}
        self._sized = False
        self.verticalHeader().setDefaultSectionSize(22)
        tips = {6: "실측 각도 − 시뮬 각도. 같은 목표를 받았는데 크게 다르면 방향·영점·게인·마찰을 확인",
                8: "이번에 기록한 실측 최소~최대. 손으로 관절을 끝까지 돌려 보면 실물 범위와 시뮬 범위를 비교할 수 있음",
                9: "영점(zero_turn)이 있으면 '보정', 없으면 '미보정'(절대 각도 명령 불가)"}
        for c, t in tips.items():
            self.horizontalHeaderItem(c).setToolTip(t)
        self.rows = {}
        for r, j in enumerate(app.cfg.joints):
            self.rows[j.name] = r
            for c in range(len(self.COLS)):
                it = QTableWidgetItem("")
                if 3 <= c <= 11:
                    it.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.setItem(r, c, it)
            self.item(r, 0).setText(j.short)
            self.item(r, 1).setText(str(j.node))
            info = app.infos.get(j.name)
            if info is None:
                self.item(r, 7).setText("모델에 없음")
            elif info.lo is not None:
                self.item(r, 7).setText(f"{math.degrees(info.lo):+.0f} ~ {math.degrees(info.hi):+.0f}°")
            else:
                self.item(r, 7).setText("제한 없음")
        self.itemSelectionChanged.connect(self._sel)
        self._block = False

    def _sel(self):
        if self._block:
            return
        rows = self.selectionModel().selectedRows()
        if rows:
            self.joint_clicked.emit(self.app.cfg.joints[rows[0].row()].name)

    def select(self, name):
        self._block = True
        self.selectRow(self.rows[name])
        self._block = False

    def _set(self, r, c, text, icon_color=None, tip=None):
        """바뀐 칸만 고친다 (setText마다 표 전체를 다시 재지 않게)."""
        key = (text, icon_color, tip)
        if self._cache.get((r, c)) == key:
            return
        self._cache[(r, c)] = key
        it = self.item(r, c)
        it.setText(text)
        if icon_color is not None:
            it.setIcon(dot_icon(icon_color))
        if tip is not None:
            it.setToolTip(tip)

    def refresh(self):
        app = self.app
        t = app.now()
        blank = C["surface"]
        for j in app.cfg.joints:
            r = self.rows[j.name]
            col, short = joint_state(app, j.name)
            self._set(r, 2, short, col)
            bi = app.link.bus_of(j.node)
            if app.link.n_buses > 1 and bi is not None:
                self._set(r, 1, f"{j.node} · 버스{bi + 1}", None, f"버스 {bi + 1} ({app.link.channels[bi]})"
                          if bi < len(app.link.channels) else None)
            else:
                self._set(r, 1, str(j.node))
            qr = app.real_q(j.name)
            qt = app.cmdr.target_q(j.name, t)
            qs = app.sim.q(j.name)
            self._set(r, 3, fmt_deg(qr, 1))
            self._set(r, 4, fmt_deg(qt, 1))
            self._set(r, 5, fmt_deg(qs, 1))
            notes = []
            if qr is not None and qs is not None:
                diff = math.degrees(qr - qs)
                # 영점이 없으면 실측 각도가 시뮬 기준이 아니므로 차이도 의미가 없다 → 경고하지 않음
                warn = j.calibrated and abs(diff) > 3.0
                tip = "3° 넘게 다름" if warn else ("" if j.calibrated else "영점이 없어 비교 의미 없음")
                self._set(r, 6, f"{diff:+.1f}°", C["warning"] if warn else (blank if j.calibrated else C["muted"]), tip)
                if warn:
                    notes.append(f"실물·시뮬 {diff:+.0f}° 차이")
            else:
                self._set(r, 6, "—", blank, "")
            lo, hi = app.range_rec[j.name]
            info = app.infos.get(j.name)
            if math.isfinite(lo):
                out = info is not None and info.lo is not None and j.calibrated and not (
                    info.in_range(lo, math.radians(2)) and info.in_range(hi, math.radians(2)))
                self._set(r, 8, f"{math.degrees(lo):+.1f} ~ {math.degrees(hi):+.1f}°",
                          C["serious"] if out else blank, "시뮬 관절 범위를 2° 넘게 벗어난 적 있음" if out else "")
                if out:
                    notes.append("시뮬 범위 밖 → 방향·영점 확인")
            else:
                self._set(r, 8, "—", blank, "")
            self._set(r, 9, "보정" if j.calibrated else "미보정", C["good"] if j.calibrated else C["uncal"])
            self._set(r, 10, f"{j.direction:+d}")
            ns = app.link.nodes[j.node]
            on = ns.online(t)
            self._set(r, 11, "—" if ns.iq is None or not on else f"{ns.iq:+.2f}")
            if on and not j.calibrated:
                notes.insert(0, "영점 없음")
            if app.sweep_active and on:
                sp = app.sweep_span(j.name) or 0.0
                if j.stop_lo_deg is None and j.stop_hi_deg is None:
                    notes.insert(0, f"기록 폭 {sp:.1f}° (기구 끝 없음)")
                elif j.stop_lo_deg is None or j.stop_hi_deg is None:
                    notes.insert(0, f"기록 폭 {sp:.1f}° (한쪽 끝만)")
                else:
                    notes.insert(0, f"기록 폭 {sp:.1f}° / 기구 {j.stop_hi_deg - j.stop_lo_deg:.1f}°")
            if ns.axis_error and on:
                notes.insert(0, P.decode_bits(ns.axis_error, P.AXIS_ERRORS))
            if info is None:
                notes.append("MJCF에 관절 없음")
            self._set(r, 12, " · ".join(notes))
        if not self._sized:
            self.resizeColumnsToContents()
            for c, w in ((1, 74), (3, 62), (4, 62), (5, 62), (6, 80), (8, 120), (11, 60)):
                self.setColumnWidth(c, max(self.columnWidth(c), w))
            self._sized = True


CAL_HELP = """<b>영점·방향 확인 순서</b>
<ol>
<li>실물 로봇을 받침대에 매달고, 다리를 곧게 펴서 시뮬 0° 자세(오른쪽 화면의 처음 자세)와 같게 둡니다.</li>
<li>'모든 관절 지금 = 0°' (또는 관절마다 '지금 자세 = 0°' → '영점 정하기').</li>
<li>관절을 고르고 명령 탭에서 '토크 켜기' → '+5°'. 3D 화면의 노란 화살표가 회전축, 노란 호가 + 회전 방향입니다
(오른손 법칙: 엄지 = 화살표, 손가락이 감기는 쪽 = +).</li>
<li>실물이 왼쪽 그림과 같은 쪽으로 움직였으면 방향이 맞습니다. 반대로 움직였으면 '힘 풀기' → 방향 −1.
0° 자세에서 영점을 잡았다면 영점은 다시 잡지 않아도 됩니다.</li>
<li>힘을 푼 채 관절을 손으로 끝까지 돌려 관절 표의 '실측 기록 범위'를 '시뮬 범위'와 비교합니다.
기구 끝을 기준으로 영점을 잡으려면 '범위 훑기'(시작 → 양쪽 끝까지 → 끝)를 씁니다.</li>
<li>'설정 저장'.</li>
</ol>
<p><b>전원 켤 때 기준 자세</b>: 코스 펌웨어는 전원을 켤 때 모터 1회전(관절 24°) 안의 위치만 압니다. 그래서 노드가
처음 보일 때 관절이 기준 자세 ±12° 안에 있다고 보고 24° 단위를 정합니다. 다른 자세에서 전원을 켰다면 관절을
기준 자세 근처로 옮기고 '각도 다시 맞추기'를 누르세요.</p>
<p><b>토크 부호(Iq)</b>: 드라이버 motor.config.direction (-1이면 Iq 부호가 실제 토크와 반대). BasicTest/data의
setup 백업에서 읽은 값이 기본입니다. 실물과 시뮬 토크 그래프의 부호가 반대면 바꿉니다.</p>
<p><b>보호 정지</b>: 토크가 켜진 관절의 목표-실측 차이나 전류가 정한 시간 넘게 크면 그 관절만 IDLE로 둡니다.
방향이 반대로 설정된 관절은 화면 각도로는 명령을 따르는 것처럼 보이다가 기구 끝에 막혀 큰 전류로 밀게 됩니다.</p>"""


SWEEP_HELP = """<b>가동범위 훑어서 영점 잡기</b>
<ol>
<li>로봇을 매달고 모든 관절의 힘을 풉니다 (토크가 켜진 관절이 있으면 시작되지 않습니다).</li>
<li>'시작'을 누릅니다. 연결된 모든 관절의 엔코더 최소·최대를 기록합니다.</li>
<li>관절마다 손으로 <b>양쪽 기구 끝까지</b> 천천히 돌리고, 끝에서 가볍게 한 번씩 댑니다. 여러 관절을 이어서 해도 됩니다.
관절 표 알림 칸에 '기록 폭 / 기구 폭'이 나옵니다. 빨리 돌리면 모터가 전압을 만드니 천천히.</li>
<li>돌리는 관절 말고 다른 관절은 곧게 편 채로 둡니다 (다른 링크에 먼저 닿으면 기구 끝 각도와 달라집니다).
hip yaw는 두 다리를 곧게 두면 반대쪽 발목에 먼저 닿으니, 반대쪽 다리를 벌려 두고 돌립니다.</li>
<li>다리를 곧게 펴 두고 '끝 → 영점 계산'. 결과 창의 '지금 각도'가 0° 근처인지 봅니다.
크게 다르면(예: 무릎이 −140°) 방향이 반대입니다.</li>
<li>적용할 관절을 확인하고 '체크한 관절에 영점 적용' → '설정 저장'.</li>
</ol>
<p><b>기구 끝 각도</b>: 수업 모델 v1 메쉬 충돌 검사 값입니다. knee·ankle은 시뮬 범위 끝과 같고, hip pitch는 시뮬 범위보다
19~24° 바깥, hip yaw는 한쪽 끝만(몸통 프레임), hip roll은 기구 끝이 없어 이 방법을 쓸 수 없습니다('지금 자세 = 0°'로).
실물에서 다르게 나오면 config/robot_config.json의 stop_lo_deg / stop_hi_deg를 고칩니다.</p>
<p><b>계산</b>: 엔코더 최소·최대가 기구 끝 각도라고 보고 영점을 구합니다. 양 끝이 다 있으면 두 끝에서 구한 영점의 평균을
씁니다 (손으로 밀어 넘어간 양이 양쪽에서 상쇄됨). 기록 폭이 기구 끝 사이 폭과 3° 넘게 다르면 기본으로 적용하지 않습니다.</p>
<p><b>방향</b>: 방향이 틀리면 폭은 맞아도 각도가 거꾸로 나옵니다. 방향을 먼저 확인하거나, 끝낼 때 다리를 곧게 펴 두고
'지금 각도'를 보세요.</p>"""


class SweepDialog(QDialog):
    """가동범위 기록 결과: 관절별 영점 후보를 보여 주고, 체크한 관절만 적용."""
    BASIS = {"avg": "양 끝 평균", "lo": "하한 끝", "hi": "상한 끝"}
    STATUS_COLOR = {"ok": "good", "warn": "warning", "one": "warning", "bad": "critical"}

    def __init__(self, results, parent=None):
        super().__init__(parent)
        self.setWindowTitle("가동범위 기록 결과 → 영점")
        self.results = results
        v = QVBoxLayout(self)
        head = _label("체크한 관절만 영점이 바뀝니다. 기구 끝 각도는 각 관절 설정의 stop_lo_deg / stop_hi_deg "
                      "(기본: 수업 모델 v1 메쉬 충돌 검사)입니다.<br>"
                      "'지금 각도'는 새 영점으로 본 지금 자세입니다. 다리를 곧게 펴고 끝냈다면 0° 근처여야 합니다 "
                      "(크게 다르면 방향 확인).")
        head.setWordWrap(True)
        v.addWidget(head)
        cols = ["적용", "관절", "기구 끝", "기록 폭 / 기구 폭", "판정", "기준", "지금 각도", "각도 바뀜"]
        tb = QTableWidget(len(results), len(cols))
        tb.setHorizontalHeaderLabels(cols)
        tb.verticalHeader().setVisible(False)
        tb.setSelectionMode(QAbstractItemView.NoSelection)
        tb.setEditTriggers(QAbstractItemView.NoEditTriggers)
        tb.horizontalHeaderItem(7).setToolTip("지금 영점 대비 각도 읽음이 바뀌는 양 (미보정이면 —)")
        self.tb = tb
        self.combos = {}
        fmt = lambda x: "없음" if x is None else f"{x:+.2f}°"
        for r, res in enumerate(results):
            chk = QTableWidgetItem()
            if res["cands"]:
                chk.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
                chk.setCheckState(Qt.Checked if res["apply"] else Qt.Unchecked)
            else:
                chk.setFlags(Qt.NoItemFlags)
            tb.setItem(r, 0, chk)
            tb.setItem(r, 1, QTableWidgetItem(f"{res['short']}  ·  node {res['node']}"))
            tb.setItem(r, 2, QTableWidgetItem(f"{fmt(res['stop_lo'])} ~ {fmt(res['stop_hi'])}"))
            span = "—" if res["span"] is None else f"{res['span']:.1f}°"
            if res["span"] is not None and res["model_span"] is not None:
                span += f" / {res['model_span']:.1f}°"
            tb.setItem(r, 3, QTableWidgetItem(span))
            st = QTableWidgetItem(res["msg"])
            st.setForeground(QColor(C[self.STATUS_COLOR.get(res["status"], "muted")]))
            tb.setItem(r, 4, st)
            if res["cands"]:
                cb = QComboBox()
                for key in ("avg", "lo", "hi"):
                    if key in res["cands"]:
                        cb.addItem(self.BASIS[key], key)
                cb.setCurrentIndex(max(0, cb.findData(res["basis"])))
                cb.currentIndexChanged.connect(lambda _=0, r=r: self._update_row(r))
                tb.setCellWidget(r, 5, cb)
                self.combos[r] = cb
            else:
                tb.setItem(r, 5, QTableWidgetItem("—"))
            self._update_row(r)
        tb.resizeColumnsToContents()
        tb.horizontalHeader().setStretchLastSection(True)
        v.addWidget(tb, 1)
        bb = QDialogButtonBox()
        ok = bb.addButton("체크한 관절에 영점 적용", QDialogButtonBox.AcceptRole)
        ok.setObjectName("primary")
        bb.addButton("적용 안 함", QDialogButtonBox.RejectRole)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        v.addWidget(bb)
        self.resize(1080, 560)

    def _basis(self, r):
        cb = self.combos.get(r)
        return None if cb is None else cb.currentData()

    def _update_row(self, r):
        res = self.results[r]
        key = self._basis(r)
        q = res["q_now"].get(key)
        sh = res["shift"].get(key)
        self.tb.setItem(r, 6, QTableWidgetItem("—" if q is None else f"{q:+.1f}°"))
        self.tb.setItem(r, 7, QTableWidgetItem("—" if sh is None else f"{sh:+.2f}°"))

    def choices(self):
        out = {}
        for r, res in enumerate(self.results):
            it = self.tb.item(r, 0)
            key = self._basis(r)
            if key is not None and it.checkState() == Qt.Checked:
                out[res["name"]] = res["cands"][key]
        return out


# ====================================================================== 오른쪽 관절 패널
class JointPanel(QWidget):
    joint_changed = Signal(str)
    message = Signal(str, bool)       # 글, 오류인지

    def __init__(self, app):
        super().__init__()
        self.app = app
        self.name = app.cfg.joints[0].name
        v = QVBoxLayout(self)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(6)

        # ---- 관절 선택
        top = QHBoxLayout()
        lab = QLabel("관절 선택")
        f = lab.font()
        f.setBold(True)
        f.setPointSizeF(10.5)
        lab.setFont(f)
        self.combo = QComboBox()
        for j in app.cfg.joints:
            self.combo.addItem(f"{j.short}   ·   node {j.node}   ·   {j.motor}", j.name)
        self.combo.setFont(f)
        self.prev_btn, self.next_btn = QToolButton(), QToolButton()
        self.prev_btn.setText("◀")
        self.next_btn.setText("▶")
        top.addWidget(lab)
        top.addWidget(self.combo, 1)
        top.addWidget(self.prev_btn)
        top.addWidget(self.next_btn)
        v.addLayout(top)

        # ---- 현재 상태
        gb = QGroupBox("현재 상태")
        g = QGridLayout(gb)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(2)
        self.val = {}

        def add(row, col, key, title, tip=None):
            g.addWidget(_label(title, "muted", tip), row, col * 2)
            lb = _label("—", "value", tip)
            lb.setTextInteractionFlags(Qt.TextSelectableByMouse)
            g.addWidget(lb, row, col * 2 + 1)
            self.val[key] = lb

        add(0, 0, "state", "축 상태")
        add(0, 1, "err", "에러")
        add(1, 0, "real", "실측 각도", "엔코더 값을 영점·방향으로 바꾼 관절 각도 (시뮬 기준)")
        add(1, 1, "target", "목표")
        add(2, 0, "sim", "시뮬 각도")
        add(2, 1, "diff", "실측 − 시뮬")
        add(3, 0, "vel", "속도 실/시 °/s")
        add(3, 1, "tau", "토크 실/시 Nm", "실측 토크 = 방향 × 토크 부호 × Iq × kt × 15 (마찰 포함, 추정)")
        add(4, 0, "iq", "Iq 측정/명령 A")
        add(4, 1, "raw", "엔코더 원시", "드라이버 pos_estimate [motor turn]와 정수 turn 보정 k")
        add(5, 0, "range", "시뮬 범위")
        add(5, 1, "cal", "영점 · 방향")
        self.gains = _label("", "muted")
        self.gains.setWordWrap(True)
        g.addWidget(self.gains, 6, 0, 1, 4)
        g.setColumnStretch(1, 1)
        g.setColumnStretch(3, 1)
        split = QSplitter(Qt.Vertical)
        split.setChildrenCollapsible(False)
        v.addWidget(split, 1)
        split.addWidget(gb)

        # ---- 그래프
        pbox = QWidget()
        pv = QVBoxLayout(pbox)
        pv.setContentsMargins(0, 0, 0, 0)
        pv.setSpacing(2)
        pbar = QHBoxLayout()
        pbar.addWidget(_label("실시간 그래프", "muted"))
        pbar.addStretch(1)
        pbar.addWidget(_label("보기", "muted"))
        self.window_combo = QComboBox()
        for s in (5, 15, 30, 60):
            self.window_combo.addItem(f"{s} s", s)
        self.window_combo.setCurrentIndex(1)
        pbar.addWidget(self.window_combo)
        pv.addLayout(pbar)
        self.plots = JointPlots()
        self.plots.setMinimumHeight(200)
        pv.addWidget(self.plots, 1)
        split.addWidget(pbox)

        # ---- 명령 / 영점 탭
        self.tabs = QTabWidget()
        self.tabs.addTab(_scroll(self._build_cmd_tab()), "명령")
        self.tabs.addTab(_scroll(self._build_cal_tab()), "영점·방향 설정")
        self.tabs.setMinimumHeight(170)
        split.addWidget(self.tabs)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setStretchFactor(2, 0)
        split.setSizes([215, 470, 265])
        self.msg = _label("", "msg_ok")
        self.msg.setWordWrap(True)
        self.msg.setMinimumHeight(18)
        v.addWidget(self.msg)
        self._msg_timer = QTimer(self)
        self._msg_timer.setSingleShot(True)
        self._msg_timer.timeout.connect(lambda: self.msg.setText(""))

        self.combo.currentIndexChanged.connect(lambda i: self._select(self.combo.itemData(i)))
        self.prev_btn.clicked.connect(lambda: self.combo.setCurrentIndex((self.combo.currentIndex() - 1)
                                                                         % self.combo.count()))
        self.next_btn.clicked.connect(lambda: self.combo.setCurrentIndex((self.combo.currentIndex() + 1)
                                                                         % self.combo.count()))
        self.window_combo.currentIndexChanged.connect(lambda _: self.plots.set_window(self.window_combo.currentData()))
        self._select(self.name)

    # ------------------------------------------------------------------ 명령 탭
    def _build_cmd_tab(self):
        w = QWidget()
        g = QGridLayout(w)
        g.setVerticalSpacing(6)
        app = self.app
        self.btn_on = QPushButton("토크 켜기")
        self.btn_on.setObjectName("primary")
        self.btn_on.setToolTip("CAN으로 위치 제어(closed loop) 진입. 지금 위치를 목표로 잡으므로 튀지 않습니다.")
        self.btn_off = QPushButton("힘 풀기")
        self.btn_clear = QPushButton("에러 지우기")
        self.send_real = QCheckBox("실물에 보내기")
        self.send_real.setChecked(True)
        self.send_real.setToolTip("끄면 명령이 시뮬에만 갑니다 (실물 연결 전 미리 보기).\n"
                                  "'힘 풀기'와 '모두 힘 풀기'는 이 설정과 상관없이 실물에 보냅니다.")
        row = QHBoxLayout()
        for b in (self.btn_on, self.btn_off, self.btn_clear):
            row.addWidget(b)
        row.addStretch(1)
        row.addWidget(self.send_real)
        g.addLayout(row, 0, 0, 1, 2)

        # 상대 이동
        g.addWidget(_label("상대 이동", "muted"), 1, 0)
        rel = QHBoxLayout()
        rel.setSpacing(3)
        self.jog_btns = []
        for d in (-10, -5, -1):
            b = QPushButton(f"{d}°")
            b.clicked.connect(lambda _=False, d=d: self._do(app.move_rel, self.name, d))
            rel.addWidget(b)
            self.jog_btns.append(b)
        self.rel_spin = QDoubleSpinBox()
        self.rel_spin.setRange(-180, 180)
        self.rel_spin.setDecimals(1)
        self.rel_spin.setSingleStep(1.0)
        self.rel_spin.setValue(3.0)
        self.rel_spin.setSuffix(" °")
        self.rel_go = QPushButton("이동")
        rel.addSpacing(6)
        rel.addWidget(self.rel_spin)
        rel.addWidget(self.rel_go)
        rel.addSpacing(6)
        for d in (1, 5, 10):
            b = QPushButton(f"+{d}°")
            b.clicked.connect(lambda _=False, d=d: self._do(app.move_rel, self.name, d))
            rel.addWidget(b)
            self.jog_btns.append(b)
        g.addLayout(rel, 1, 1)

        # 절대 이동
        g.addWidget(_label("절대 이동", "muted"), 2, 0)
        ab = QHBoxLayout()
        self.abs_spin = QDoubleSpinBox()
        self.abs_spin.setRange(-180, 180)
        self.abs_spin.setDecimals(1)
        self.abs_spin.setSuffix(" °")
        self.abs_go = QPushButton("이 각도로")
        self.abs_zero = QPushButton("0°로")
        self.abs_now = QPushButton("지금 각도 넣기")
        self.abs_now.setToolTip("입력 칸에 지금 실측(없으면 시뮬) 각도를 넣습니다")
        ab.addWidget(self.abs_spin)
        ab.addWidget(self.abs_go)
        ab.addWidget(self.abs_zero)
        ab.addWidget(self.abs_now)
        ab.addStretch(1)
        g.addLayout(ab, 2, 1)

        # 속도·제한
        g.addWidget(_label("이동 설정", "muted"), 3, 0)
        st = QHBoxLayout()
        self.speed = QDoubleSpinBox()
        self.speed.setRange(1, 180)
        self.speed.setValue(app.cfg.max_speed_dps)
        self.speed.setSuffix(" °/s")
        self.speed.setToolTip("최소 저크 궤적의 최고 속도. 같은 궤적을 실물과 시뮬에 줍니다.")
        self.max_step = QDoubleSpinBox()
        self.max_step.setRange(1, 180)
        self.max_step.setValue(app.cfg.max_step_deg)
        self.max_step.setSuffix(" °")
        self.max_step.setToolTip("한 번 명령으로 움직일 수 있는 최대 각도")
        self.margin = QDoubleSpinBox()
        self.margin.setRange(0, 30)
        self.margin.setValue(app.cfg.limit_margin_deg)
        self.margin.setSuffix(" °")
        self.margin.setToolTip("영점이 있을 때 목표를 시뮬 관절 범위에서 이만큼 안쪽으로 자름")
        st.addWidget(_label("속도", "muted"))
        st.addWidget(self.speed)
        st.addWidget(_label("한 번 최대", "muted"))
        st.addWidget(self.max_step)
        st.addWidget(_label("범위 여유", "muted"))
        st.addWidget(self.margin)
        st.addStretch(1)
        g.addLayout(st, 3, 1)

        g.addWidget(_label("보호 정지", "muted",
                           "토크가 켜진 관절의 목표-실측 차이나 전류가 정한 시간 넘게 크면 그 관절만 IDLE로 둡니다.\n"
                           "방향이 반대로 설정되면 화면 각도는 명령을 따르는 것처럼 보이다가 기구 끝에 막혀\n"
                           "전류 한계(18 A)까지 밀 수 있어서 넣은 보호입니다."), 4, 0)
        gr = QHBoxLayout()
        self.guard_err = QDoubleSpinBox()
        self.guard_err.setRange(2, 90)
        self.guard_err.setValue(app.cfg.guard_err_deg)
        self.guard_err.setSuffix(" °")
        self.guard_cur = QDoubleSpinBox()
        self.guard_cur.setRange(1, 18)
        self.guard_cur.setDecimals(1)
        self.guard_cur.setValue(app.cfg.guard_current_a)
        self.guard_cur.setSuffix(" A")
        gr.addWidget(_label("목표-실측 오차", "muted"))
        gr.addWidget(self.guard_err)
        gr.addWidget(_label(f"{app.cfg.guard_err_s:g} s 넘게 · |Iq|", "muted"))
        gr.addWidget(self.guard_cur)
        gr.addWidget(_label(f"{app.cfg.guard_current_s:g} s 넘게", "muted"))
        gr.addStretch(1)
        g.addLayout(gr, 4, 1)
        self.guard_err.valueChanged.connect(lambda x: setattr(app.cfg, "guard_err_deg", x))
        self.guard_cur.valueChanged.connect(lambda x: setattr(app.cfg, "guard_current_a", x))

        self.btn_on.clicked.connect(lambda: self._do(app.enable, self.name))
        self.btn_off.clicked.connect(lambda: self._do(app.disable, self.name))
        self.btn_clear.clicked.connect(lambda: self._do(app.clear_errors, self.name))
        self.rel_go.clicked.connect(lambda: self._do(app.move_rel, self.name, self.rel_spin.value()))
        self.abs_go.clicked.connect(lambda: self._do(app.move_abs, self.name, self.abs_spin.value()))
        self.abs_zero.clicked.connect(lambda: self._do(app.move_abs, self.name, 0.0))
        self.abs_now.clicked.connect(self._fill_abs_now)
        self.send_real.toggled.connect(lambda on: setattr(app, "send_real", on))
        self.speed.valueChanged.connect(lambda x: setattr(app.cfg, "max_speed_dps", x))
        self.max_step.valueChanged.connect(lambda x: setattr(app.cfg, "max_step_deg", x))
        self.margin.valueChanged.connect(lambda x: setattr(app.cfg, "limit_margin_deg", x))
        return w

    def _fill_abs_now(self):
        q = self.app.real_q(self.name)
        if q is None:
            q = self.app.sim.q(self.name)
        if q is not None:
            self.abs_spin.setValue(math.degrees(q))

    # ------------------------------------------------------------------ 영점 탭
    def _build_cal_tab(self):
        w = QWidget()
        g = QGridLayout(w)
        g.setVerticalSpacing(6)
        app = self.app

        g.addWidget(_label("방향", "muted", "q = 방향 × (모터 위치 − 영점) / 15. 실물이 그림과 반대로 움직이면 바꿉니다."),
                    0, 0)
        d = QHBoxLayout()
        self.dir_grp = QButtonGroup(w)
        self.dir_p, self.dir_m = QRadioButton("+1"), QRadioButton("−1")
        self.dir_grp.addButton(self.dir_p, 1)
        self.dir_grp.addButton(self.dir_m, 2)   # Qt에서 id -1은 자동 번호라 2를 씀
        d.addWidget(self.dir_p)
        d.addWidget(self.dir_m)
        d.addSpacing(20)
        d.addWidget(_label("토크 부호(Iq)", "muted",
                           "드라이버 motor.config.direction. -1인 보드는 Iq 부호가 실제 토크와 반대 "
                           "(BasicTest/bench/odrv_usb.torque_sign). 그래프의 실측 토크에만 쓰입니다."))
        self.ts_grp = QButtonGroup(w)
        self.ts_p, self.ts_m = QRadioButton("+1"), QRadioButton("−1")
        self.ts_grp.addButton(self.ts_p, 1)
        self.ts_grp.addButton(self.ts_m, 2)
        d.addWidget(self.ts_p)
        d.addWidget(self.ts_m)
        self.ts_src = _label("", "muted")
        d.addWidget(self.ts_src)
        d.addStretch(1)
        g.addLayout(d, 0, 1)

        g.addWidget(_label("영점", "muted"), 1, 0)
        z = QHBoxLayout()
        z.addWidget(_label("지금 자세 ="))
        self.zero_spin = QDoubleSpinBox()
        self.zero_spin.setRange(-180, 180)
        self.zero_spin.setDecimals(2)
        self.zero_spin.setSuffix(" °")
        z.addWidget(self.zero_spin)
        self.zero_btn = QPushButton("영점 정하기")
        self.zero_btn.setObjectName("primary")
        self.zero_btn.setToolTip("지금 실측 위치를 왼쪽 칸의 관절 각도로 정의합니다 (토크가 꺼져 있어야 함)")
        z.addWidget(self.zero_btn)
        z.addStretch(1)
        g.addLayout(z, 1, 1)
        g.addWidget(_label("빠른 값", "muted"), 2, 0)
        qz = QHBoxLayout()
        self.q0_btn = QPushButton("0°")
        self.qlo_btn = QPushButton("시뮬 하한")
        self.qhi_btn = QPushButton("시뮬 상한")
        for b in (self.q0_btn, self.qlo_btn, self.qhi_btn):
            b.setToolTip("칸에 값만 넣습니다. 관절을 그 자세로 둔 뒤 '영점 정하기'")
            qz.addWidget(b)
        qz.addStretch(1)
        g.addLayout(qz, 2, 1)

        g.addWidget(_label("전원 켤 때", "muted"), 3, 0)
        bz = QHBoxLayout()
        bz.addWidget(_label("기준 자세", tip="노드가 처음 보일 때 관절이 이 각도 ±12° 안에 있다고 보고 24° 주기 모호함을 풉니다"))
        self.boot_spin = QDoubleSpinBox()
        self.boot_spin.setRange(-180, 180)
        self.boot_spin.setDecimals(1)
        self.boot_spin.setSuffix(" °")
        bz.addWidget(self.boot_spin)
        self.rewrap_btn = QPushButton("각도 다시 맞추기")
        self.rewrap_btn.setToolTip("지금 관절이 기준 자세 ±12° 안에 있을 때 누르면 24° 단위 오차를 고칩니다")
        self.clear_zero_btn = QPushButton("영점 지우기")
        bz.addWidget(self.rewrap_btn)
        bz.addWidget(self.clear_zero_btn)
        bz.addStretch(1)
        g.addLayout(bz, 3, 1)

        g.addWidget(_label("범위 훑기", "muted", "힘을 푼 채 관절을 양쪽 기구 끝까지 손으로 돌려 영점을 잡습니다"), 4, 0)
        sw = QHBoxLayout()
        self.sweep_start_btn = QPushButton("시작")
        self.sweep_start_btn.setToolTip("연결된 모든 관절의 엔코더 최소·최대 기록을 시작합니다 (토크가 모두 꺼져 있어야 함)")
        self.sweep_end_btn = QPushButton("끝 → 영점 계산")
        self.sweep_end_btn.setObjectName("primary")
        self.sweep_end_btn.setToolTip("기록을 멈추고, 기구 끝 각도로 관절별 영점을 계산해 결과 창을 엽니다")
        self.sweep_cancel_btn = QPushButton("취소")
        sweep_help = QPushButton("?")
        sweep_help.setFixedWidth(30)
        sweep_help.setToolTip(SWEEP_HELP)
        sweep_help.clicked.connect(lambda: QMessageBox.information(self, "가동범위 훑어서 영점 잡기", SWEEP_HELP))
        for b in (self.sweep_start_btn, self.sweep_end_btn, self.sweep_cancel_btn, sweep_help):
            sw.addWidget(b)
        sw.addStretch(1)
        g.addLayout(sw, 4, 1)
        self.sweep_status = _label("", "muted")
        self.sweep_status.setWordWrap(True)
        self.sweep_status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)   # 긴 글 때문에 탭이 넓어지지 않게
        g.addWidget(self.sweep_status, 5, 1)

        allrow = QHBoxLayout()
        self.zero_all_btn = QPushButton("모든 관절 지금 = 0°")
        self.zero_all_btn.setToolTip("연결된 모든 관절의 지금 자세를 0°로 정합니다 (다리 곧게, 발 평평)")
        self.rewrap_all_btn = QPushButton("모두 다시 맞추기")
        self.rewrap_all_btn.setToolTip("연결된 모든 관절이 기준 자세 ±12° 안에 있다고 보고 24° 단위를 다시 고릅니다")
        self.save_btn = QPushButton("설정 저장")
        self.save_btn.setObjectName("primary")
        more = QPushButton("확인 순서 ?")
        more.setToolTip(CAL_HELP)
        more.clicked.connect(lambda: QMessageBox.information(self, "영점·방향 확인 순서", CAL_HELP))
        allrow.addWidget(self.zero_all_btn)
        allrow.addWidget(self.rewrap_all_btn)
        allrow.addWidget(more)
        allrow.addStretch(1)
        allrow.addWidget(self.save_btn)
        g.addLayout(allrow, 6, 0, 1, 2)
        g.setRowStretch(7, 1)
        self.sweep_start_btn.clicked.connect(self._sweep_start)
        self.sweep_end_btn.clicked.connect(self._sweep_end)
        self.sweep_cancel_btn.clicked.connect(self._sweep_cancel)

        self.dir_grp.idClicked.connect(lambda i: self._do(app.set_direction, self.name, 1 if i == 1 else -1))
        self.ts_grp.idClicked.connect(lambda i: app.set_torque_sign(self.name, 1 if i == 1 else -1))
        self.zero_btn.clicked.connect(lambda: self._do(app.set_angle_here, self.name, self.zero_spin.value()))
        self.q0_btn.clicked.connect(lambda: self.zero_spin.setValue(0.0))
        self.qlo_btn.clicked.connect(lambda: self._set_zero_spin_limit(0))
        self.qhi_btn.clicked.connect(lambda: self._set_zero_spin_limit(1))
        self.boot_spin.valueChanged.connect(lambda x: app.set_boot_ref(self.name, x))
        self.rewrap_btn.clicked.connect(lambda: app.rewrap([self.name]))
        self.rewrap_all_btn.clicked.connect(lambda: app.rewrap())
        self.clear_zero_btn.clicked.connect(lambda: self._do(app.clear_zero, self.name))
        self.zero_all_btn.clicked.connect(self._zero_all)
        return w

    def _set_zero_spin_limit(self, which):
        info = self.app.infos.get(self.name)
        if info is None or info.lo is None:
            self._show("시뮬 모델에 이 관절의 범위가 없습니다", True)
            return
        self.zero_spin.setValue(math.degrees(info.lo if which == 0 else info.hi))
        self._show("관절을 그 끝까지 손으로 돌려 둔 상태에서 '영점 정하기'를 누르세요", False)

    def _zero_all(self):
        r = QMessageBox.question(self, "모든 관절 영점",
                                 "연결된 모든 관절의 지금 자세를 0°로 정합니다.\n"
                                 "로봇이 시뮬 0° 자세(다리 곧게, 발 평평)에 있나요?")
        if r != QMessageBox.Yes:
            return
        done, skipped = self.app.zero_all_here()
        self._show(f"영점 설정 {len(done)}개" + (f", 건너뜀 {len(skipped)}개 (연결 없음 또는 토크 켜짐)"
                                              if skipped else ""), False)

    def _sweep_start(self):
        if self._do(self.app.sweep_start):
            return
        self._show("기록 중: 힘을 푼 채 관절을 손으로 양쪽 기구 끝까지 천천히 돌린 뒤 '끝 → 영점 계산'", False)

    def _sweep_end(self):
        res = self.app.sweep_finish()
        dlg = SweepDialog(res, self.window())
        if dlg.exec() != QDialog.Accepted:
            self._show("가동범위 기록 결과를 적용하지 않았습니다 (영점 그대로)", False)
            return
        choices = dlg.choices()
        if not choices:
            self._show("체크한 관절이 없어 영점을 바꾸지 않았습니다", False)
            return
        done = self.app.apply_sweep(choices)
        self._show(f"영점 설정 {len(done)}개 ({', '.join(done)}). '설정 저장'을 눌러야 파일에 남습니다", False)

    def _sweep_cancel(self):
        self.app.sweep_cancel()
        self._show("가동범위 기록 취소 (영점 그대로)", False)

    def _refresh_sweep(self, connected, use_real):
        app = self.app
        active = app.sweep_active
        if active and not connected:
            app.sweep_cancel()
            active = False
            self._show("연결이 끊겨 가동범위 기록을 취소했습니다", True)
        self.sweep_start_btn.setEnabled(connected and not active)
        self.sweep_end_btn.setEnabled(active)
        self.sweep_cancel_btn.setEnabled(active)
        if active and use_real:
            self.btn_on.setEnabled(False)
        j = app.cfg.joint(self.name)
        if active:
            sp = app.sweep_span(self.name)
            if j.stop_lo_deg is None and j.stop_hi_deg is None:
                ends = "기구 끝 없음"
            elif j.stop_lo_deg is None or j.stop_hi_deg is None:
                ends = "한쪽 끝만"
            else:
                ends = f"기구 {j.stop_hi_deg - j.stop_lo_deg:.1f}°"
            self.sweep_status.setText(f"<span style='color:{C['warning']}'>● 기록 중 {app.now() - app.sweep_t0:.0f} s</span>"
                                      f" · {j.short} 폭 {0.0 if sp is None else sp:.1f}° / {ends}"
                                      " (나머지는 관절 표 알림 칸)")
        elif connected:
            self.sweep_status.setText("힘 푼 채 '시작' → 관절을 양쪽 기구 끝까지 → 다리 펴고 '끝'")
        else:
            self.sweep_status.setText("실물 로봇에 연결하면 쓸 수 있습니다")

    # ------------------------------------------------------------------ 공통
    def _do(self, fn, *args):
        err = fn(*args)
        if err:
            self._show(err, True)
        return err

    def _show(self, text, error):
        self.msg.setObjectName("msg_err" if error else "msg_ok")
        self.msg.setStyleSheet(f"color: {C['serious'] if error else C['ink2']};")
        self.msg.setText(("⚠ " if error else "") + text)
        self._msg_timer.start(8000)
        self.message.emit(text, error)

    def select(self, name):
        i = self.combo.findData(name)
        if i >= 0 and i != self.combo.currentIndex():
            self.combo.setCurrentIndex(i)

    def _select(self, name):
        self.name = name
        app = self.app
        j = app.cfg.joint(name)
        info = app.infos.get(name)
        app.link.selected_node = j.node
        if info is not None and info.lo is not None:
            self.plots.set_limits(math.degrees(info.lo), math.degrees(info.hi))
        else:
            self.plots.set_limits(None, None)
        kp_d, kd_d = driver_joint_gains(j.motor)
        if info is not None:
            tl = "제한 없음" if not math.isfinite(info.tau_lim) else f"{info.tau_lim:.2f} Nm"
            match = abs(info.kp - kp_d) / kp_d < 0.05 and abs(info.kd - kd_d) / kd_d < 0.05
            mark = "같음" if match else f"<span style='color:{C['warning']}'>다름</span>"
            self.gains.setText(f"게인 · 시뮬 Kp {info.kp:.1f} / Kd {info.kd:.2f} / 한계 {tl}<br>"
                               f"게인 · 드라이버 Kp {kp_d:.1f} / Kd {kd_d:.2f} → {mark}")
            self.gains.setToolTip(f"시뮬: {info.gain_src}\n드라이버: 코스 xdrive_config (setup_driver.py가 저장하는 값). "
                                  "CAN Simple 0.5.1로는 게인을 읽을 수 없어 저장값을 보여 줍니다.")
        else:
            self.gains.setText("시뮬 모델에 이 관절이 없습니다")
        self._sync_cal_widgets()
        self.joint_changed.emit(name)

    def _sync_cal_widgets(self):
        j = self.app.cfg.joint(self.name)
        for grp, val in ((self.dir_grp, j.direction), (self.ts_grp, j.torque_sign)):
            grp.blockSignals(True)
            grp.button(1 if val > 0 else 2).setChecked(True)
            grp.blockSignals(False)
        self.ts_src.setText("(백업)" if j.torque_sign_src.startswith("백업") else f"({j.torque_sign_src})")
        self.ts_src.setToolTip(j.torque_sign_src)
        self.boot_spin.blockSignals(True)
        self.boot_spin.setValue(j.boot_ref_deg)
        self.boot_spin.blockSignals(False)

    def refresh(self):
        """5~10 Hz: 글자 값."""
        app = self.app
        name = self.name
        j = app.cfg.joint(name)
        ns = app.link.nodes[j.node]
        t = app.now()
        col, short = joint_state(app, name)
        self.val["state"].setText(f"<span style='color:{col}'>●</span> {short}")
        errs = [x for x in (P.decode_bits(ns.axis_error, P.AXIS_ERRORS),
                            ("motor " + P.decode_bits(ns.motor_error, P.MOTOR_ERRORS)) if ns.motor_error else "",
                            ("encoder " + P.decode_bits(ns.encoder_error, P.ENCODER_ERRORS))
                            if ns.encoder_error else "") if x]
        self.val["err"].setText(", ".join(errs) if errs else "없음")
        self.val["err"].setStyleSheet(f"color: {C['critical']};" if errs else "")
        qr, qt, qs = app.real_q(name), app.cmdr.target_q(name, t), app.sim.q(name)
        self.val["real"].setText(fmt_deg(qr))
        self.val["target"].setText(fmt_deg(qt) + ("  (이동 중)" if app.cmdr.get(name) and
                                                   app.cmdr.get(name).moving(t) else ""))
        self.val["sim"].setText(fmt_deg(qs))
        self.val["diff"].setText("—" if qr is None or qs is None else
                                 f"{math.degrees(qr - qs):+.2f}°" + ("" if j.calibrated else " (영점 없음)"))
        vr, vs = app.real_qd(name), app.sim.qd(name)
        self.val["vel"].setText(f"{'—' if vr is None else f'{math.degrees(vr):+.1f}'} / "
                                f"{'—' if vs is None else f'{math.degrees(vs):+.1f}'}")
        tr, ts = app.real_tau(name), app.sim.tau.get(name)
        self.val["tau"].setText(f"{'—' if tr is None else f'{tr:+.2f}'} / {'—' if ts is None else f'{ts:+.2f}'}")
        on = ns.online(t)
        self.val["iq"].setText("—" if ns.iq is None or not on else f"{ns.iq:+.2f} / {ns.iq_set:+.2f}")
        info = app.infos.get(name)
        self.val["range"].setText("모델에 없음" if info is None else "제한 없음" if info.lo is None else
                                  f"{math.degrees(info.lo):+.1f} ~ {math.degrees(info.hi):+.1f}°")
        cal = "보정" if j.calibrated else f"<span style='color:{C['uncal']}'>미보정</span>"
        self.val["cal"].setText(f"{cal} · {j.direction:+d}")
        self.val["cal"].setToolTip(f"zero_turn {j.zero_turn:.5f} turn" if j.calibrated else "영점 없음")
        self.val["raw"].setText("—" if ns.pos is None or not on else f"{ns.pos:.3f} t  k{ns.k:+d}")

        connected = app.link.connected
        use_real = app.send_real and connected
        tg = app.cmdr.get(name)
        holding = tg is not None
        self.btn_on.setEnabled((on or not use_real) and not (use_real and ns.state == P.STATE_CLOSED_LOOP and
                                                              holding))
        self.btn_clear.setEnabled(connected and on)
        can_move = holding or not use_real
        for b in self.jog_btns + [self.rel_go]:
            b.setEnabled(can_move)
        abs_ok = can_move and (j.calibrated or not use_real)
        for b in (self.abs_go, self.abs_zero):
            b.setEnabled(abs_ok)
            b.setToolTip("" if abs_ok else "영점이 없거나 토크가 꺼져 있습니다")
        busy = holding and tg.real
        for b in (self.zero_btn, self.clear_zero_btn, self.dir_p, self.dir_m):
            b.setEnabled(not busy)
        self.zero_btn.setEnabled(not busy and on)
        if self.send_real.isChecked() != app.send_real:
            self.send_real.setChecked(app.send_real)
        self._refresh_sweep(connected, use_real)

    def refresh_plots(self):
        self.plots.update_data(self.app.series(self.name, self.plots.window))
