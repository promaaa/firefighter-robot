"""CAN 버스 입출력 스레드 (CAN Simple, fw 0.5.1).

0.5.1은 엔코더 값을 주기적으로 보내지 않아서 매 주기 RTR로 요청한다. 요청을 한꺼번에 보내고 응답은
arbitration ID(node << 5 | cmd)로 나눠 받는다 (하나씩 기다리지 않음).

주기마다:
  - 움직이는 궤적이 있는 관절: Set Input Pos (궤적 끝 + 0.1 s까지)
  - 연결된 노드: Get Encoder Estimates (poll_hz), Get Iq (선택 관절은 매번, 나머지는 1/5), Vbus (2 s마다)
  - 연결 안 된 노드: 0.5 s마다 한 번 Get Encoder Estimates로 찔러 봄 (heartbeat가 꺼진 보드 대비)
  - axis error가 있으면 motor/encoder error 요청

GUI 스레드는 send_*()로 큐에 명령을 넣는다. 이 스레드는 루프마다(최대 5 ms) 큐를 비운다.

CAN 버스는 1개 또는 2개 (CANable 두 개: 보통 왼다리·오른다리 버스). 한 스레드가 두 버스를 번갈아 받는다.
어느 노드가 어느 버스에 있는지는 그 노드의 프레임(heartbeat 등)이 들어온 버스로 알아내고(node_bus),
그 노드에 보내는 명령은 그 버스로만 보낸다. 아직 모르는 노드, 응답이 끊긴 노드 찾기, 모두 힘 풀기는 모든 버스로.

연결해도 토크를 켜지 않는다 (요청 RTR만 보냄). closed loop는 '토크 켜기'를 눌렀을 때만 보낸다.
연결할 때(또는 보드가 끊겼다 다시 나타났을 때) 이미 IDLE이 아닌 보드가 있으면 (다른 스크립트가 켜 둔 채
끝났거나 프로그램이 비정상 종료된 경우) 이 프로그램이 켠 것이 아니므로 IDLE로 돌린다.
"""
import math
import queue
import struct
import threading
import time
from collections import deque
from types import SimpleNamespace

import can

from bench import can_simple as cs

from . import protocol as P

HIST_LEN = 3000            # 50 Hz로 60 s
OFFLINE_S = 0.6            # 이 시간 동안 아무것도 안 오면 연결 끊김
NEW_SESSION_GAP = 1.5      # 이보다 오래 끊겼다 다시 오면 재부팅으로 보고 k(정수 turn)를 다시 고름
VIRTUAL_CHANNEL = "motor_test_gui_virtual"


class NodeState:
    def __init__(self, node):
        self.node = node
        self.reset()

    def reset(self):
        self.pos = None          # [motor turn]
        self.vel = None          # [motor turn/s]
        self.t_enc = -1e9
        self.iq_set = None
        self.iq = None
        self.t_iq = -1e9
        self.vbus = None
        self.t_vbus = -1e9
        self.axis_error = 0
        self.state = None
        self.t_hb = -1e9
        self.motor_error = 0
        self.encoder_error = 0
        self.k = 0               # 정수 turn 보정
        self.new_session = True
        self.ever_seen = False
        self.state_checked = False   # 이번 세션 첫 heartbeat에서 '모르는 토크'를 확인했는지
        self.session = 0             # 끊겼다 다시 나타날 때마다 +1 (재부팅이면 pos_estimate가 새로 시작)
        self.rec_pmin = math.inf     # 가동범위 기록: pos_estimate 최소/최대 [motor turn]
        self.rec_pmax = -math.inf
        self.rec_n = 0
        self.latency = None      # 엔코더 RTR 왕복 [s]
        self._t_req = None
        self.enc_hist = deque(maxlen=HIST_LEN)   # (t, pos, vel)
        self.iq_hist = deque(maxlen=HIST_LEN)    # (t, iq_set, iq)

    def last_seen(self):
        return max(self.t_hb, self.t_enc, self.t_iq, self.t_vbus)

    def online(self, now):
        return now - self.last_seen() < OFFLINE_S

    def closed_loop(self, now):
        return self.online(now) and self.state == P.STATE_CLOSED_LOOP


class CanLink:
    def __init__(self, cfg, commander, clock):
        self.cfg = cfg
        self.cmdr = commander
        self.clock = clock
        self.nodes = {j.node: NodeState(j.node) for j in cfg.joints}
        self.unknown = {}                 # 설정에 없는 node -> 마지막 heartbeat 시각
        self.state = "disconnected"       # disconnected | connecting | connected | error
        self.message = "연결 안 됨"
        self.descr = ""
        self.selected_node = None
        self.stats = {"rx_fps": 0.0, "tx_fps": 0.0, "load": 0.0, "loads": [], "tx_err": 0, "poll_hz": 0.0}
        self.buses = []                   # python-can Bus 목록 (1~2개)
        self.channels = []                # 버스별 포트 이름
        self.node_bus = {}                # node -> 버스 번호 (0부터). 그 노드 프레임이 들어온 버스
        self._node_bus_t = {}             # (node, 버스) -> 마지막으로 받은 시각 (양쪽 버스에서 보이는지 확인용)
        self._dup_warned = set()
        self.logs = deque(maxlen=1000)    # (t, text)
        self.virtual = None
        self._q = queue.Queue()
        self._thread = None
        self._stop = threading.Event()
        self._enable_req = {}             # node -> 요청 시각
        self._err_since = {}              # 보호 정지 타이머 (node -> 시작 시각)
        self._cur_since = {}
        self.alerts = deque(maxlen=50)    # GUI가 눈에 띄게 보여 줄 경고
        self.recording = False            # 가동범위 기록 중 (엔코더 응답마다 최소/최대 갱신)
        self.rec_session = {}             # 기록 시작 때 node별 session
        self._reset_counters()

    # ------------------------------------------------------------------ GUI 쪽
    def log(self, text):
        self.logs.append((self.clock(), text))

    @property
    def connected(self):
        return self.state == "connected"

    @property
    def running(self):
        return self._thread is not None and self._thread.is_alive()

    @property
    def n_buses(self):
        return len(self.buses)

    def bus_of(self, node):
        """그 노드가 있는 버스 번호 (0부터, 아직 모르면 None)."""
        return self.node_bus.get(node)

    def start(self, interface, channels, bitrate, poll_hz, virtual_kwargs=None):
        """channels: 포트 이름 목록 (1~2개). 문자열 하나도 받는다."""
        if self.running:
            return
        if isinstance(channels, str):
            channels = [channels]
        channels = list(channels) or [""]
        for ns in self.nodes.values():
            ns.reset()
        self.recording = False
        self.unknown.clear()
        self.node_bus.clear()
        self._node_bus_t.clear()
        self._dup_warned.clear()
        self._stop.clear()
        self.state, self.message = "connecting", f"{interface} 여는 중..."
        self._thread = threading.Thread(target=self._run, name="can-link", daemon=True,
                                        args=(interface, channels, int(bitrate), float(poll_hz), virtual_kwargs))
        self._thread.start()

    def stop(self, timeout=3.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
        self._thread = None

    def send_enable(self, node):
        self._q.put(("enable", node))

    def send_disable(self, node):
        self._q.put(("disable", node))

    def send_clear_errors(self, node):
        self._q.put(("clear", node))

    def send_idle_all(self):
        self._q.put(("idle_all", None))

    def start_record(self):
        """가동범위 기록 시작. 초기화는 이 스레드에서 해서 받는 쪽과 엇갈리지 않게."""
        self._q.put(("rec_start", None))

    def stop_record(self):
        self.recording = False

    # ------------------------------------------------------------------ 스레드
    def _reset_counters(self):
        self._rx = self._tx = self._bits = self._ticks = 0
        self._bits_bus = [0] * max(1, len(self.buses))
        self._tx_err = 0

    def _open(self, interface, channels, bitrate, virtual_kwargs):
        """-> (bus 목록, 포트 이름 목록, 설명)."""
        if interface == "virtual":
            from .virtual_robot import VirtualRobot
            vk = dict(virtual_kwargs or {})
            n = max(1, min(2, int(vk.pop("buses", 2))))
            chans = [f"{VIRTUAL_CHANNEL}_{i + 1}" for i in range(n)]
            self.virtual = VirtualRobot(chans, **vk)
            self.virtual.start()
            buses = [can.Bus(interface="virtual", channel=ch, receive_own_messages=False) for ch in chans]
            names = ["가상1", "가상2"][:n]
            split = " · 버스 2개 (왼다리/오른다리)" if n == 2 else ""
            return buses, names, f"가상 로봇 (노드 {len(self.virtual.axes)}개{split}, 장치 없음)"
        buses = []
        for i, ch in enumerate(channels):
            ns = SimpleNamespace(interface=interface, channel=ch or None, bitrate=bitrate, node=None,
                                 sim_motor="6512")
            try:
                buses.append(cs.open_bus(ns))
            except (Exception, SystemExit) as e:  # noqa: BLE001 - find_slcan_port는 포트를 못 고르면 sys.exit
                for b in buses:
                    try:
                        b.shutdown()
                    except Exception:  # noqa: BLE001
                        pass
                which = f"포트 {i + 1} ({ch or '자동'})" if len(channels) > 1 else (ch or "자동 선택")
                hint = "  (COM 포트를 직접 고르세요)" if isinstance(e, SystemExit) else ""
                raise RuntimeError(f"{which}: {e}{hint}")
        names = [ch or "(자동)" for ch in channels]
        return buses, names, f"{interface} {' + '.join(names)} · {bitrate // 1000} kbps"

    def _run(self, interface, channels, bitrate, poll_hz, virtual_kwargs):
        try:
            buses, names, descr = self._open(interface, channels, bitrate, virtual_kwargs)
        except Exception as e:  # noqa: BLE001 - 어떤 어댑터 오류든 GUI에 보여 준다
            self.state, self.message = "error", f"연결 실패: {e}"
            self.log(self.message)
            self._stop_virtual()
            return
        self.buses, self.channels = buses, names
        self.descr = descr
        self.state, self.message = "connected", f"연결됨: {descr}"
        self.log(self.message)
        self._reset_counters()
        rate = bitrate if interface != "virtual" else 500000.0
        rot = 0
        dt = 1.0 / max(1.0, poll_hz)
        tick = 0
        next_tick = time.perf_counter()
        t_stat = time.perf_counter()
        send_fail_run = 0
        try:
            while not self._stop.is_set():
                self._drain_queue()
                now_p = time.perf_counter()
                if now_p >= next_tick:
                    ok = self._tick(tick)
                    send_fail_run = 0 if ok else send_fail_run + 1
                    if send_fail_run > 50:
                        raise RuntimeError("CAN 송신이 계속 실패합니다 (어댑터 연결 확인)")
                    tick += 1
                    self._ticks += 1
                    next_tick += dt
                    if next_tick < now_p:
                        next_tick = now_p + dt
                if now_p - t_stat >= 1.0:
                    span = now_p - t_stat
                    loads = [b / span / rate for b in self._bits_bus]
                    self.stats = {"rx_fps": self._rx / span, "tx_fps": self._tx / span,
                                  "load": max(loads), "loads": loads,
                                  "tx_err": self._tx_err, "poll_hz": self._ticks / span}
                    self._rx = self._tx = self._bits = self._ticks = 0
                    self._bits_bus = [0] * len(buses)
                    t_stat = now_p
                # 받기: 모든 버스에서 쌓인 프레임을 다 꺼내고, 없으면 한 버스씩 돌아가며 최대 2 ms 기다림
                got = False
                for i, b in enumerate(buses):
                    msg = b.recv(0)
                    while msg is not None:
                        got = True
                        self._handle(msg, i)
                        msg = b.recv(0)
                if not got:
                    timeout = max(0.0, min(next_tick - time.perf_counter(), 0.005 if len(buses) == 1 else 0.002))
                    rot = (rot + 1) % len(buses)
                    msg = buses[rot].recv(timeout)
                    if msg is not None:
                        self._handle(msg, rot)
        except Exception as e:  # noqa: BLE001
            self.state, self.message = "error", f"CAN 통신 오류: {e}"
            self.log(self.message)
        finally:
            try:
                self._drain_queue()      # 끊기 직전에 넣은 IDLE 명령도 보낸다
            except Exception:  # noqa: BLE001
                pass
            for b in buses:
                try:
                    b.shutdown()
                except Exception:  # noqa: BLE001
                    pass
            self.buses = []
            self._stop_virtual()
            if self.state != "error":
                self.state, self.message = "disconnected", "연결 끊음"
                self.log("연결 끊음")

    def _stop_virtual(self):
        if self.virtual is not None:
            self.virtual.stop()
            self.virtual = None

    # ---- 보내기
    def _targets(self, node, all_buses):
        idx = self.node_bus.get(node)
        if all_buses or idx is None or idx >= len(self.buses):
            return range(len(self.buses))
        return (idx,)

    def _tx_frame(self, msg, idx, bits):
        try:
            self.buses[idx].send(msg)
            self._tx += 1
            self._bits += bits
            self._bits_bus[idx] += bits
            return True
        except (can.CanError, OSError, ValueError):
            self._tx_err += 1
            return False

    def _send(self, node, cmd, data=b"", all_buses=False):
        """그 노드가 있는 버스로 보낸다 (모르는 노드거나 all_buses면 모든 버스로)."""
        msg = can.Message(arbitration_id=cs.arb_id(node, cmd), data=data.ljust(8, b"\x00"), is_extended_id=False)
        ok = True
        for i in self._targets(node, all_buses):
            ok &= self._tx_frame(msg, i, P.frame_bits(8))
        return ok

    def _rtr(self, node, cmd, all_buses=False):
        msg = can.Message(arbitration_id=cs.arb_id(node, cmd), is_remote_frame=True, dlc=8, is_extended_id=False)
        ok = True
        for i in self._targets(node, all_buses):
            ok &= self._tx_frame(msg, i, P.frame_bits(8, rtr=True))
        return ok

    def _drain_queue(self):
        while True:
            try:
                kind, node = self._q.get_nowait()
            except queue.Empty:
                return
            if kind == "idle_all":
                targets = set(self.nodes) | set(self.unknown)
                for _ in range(2):   # 한 번 놓쳐도 되게 두 번, 노드가 어느 버스에 있든 닿게 모든 버스로
                    for n in targets:
                        self._send(n, cs.SET_AXIS_STATE, struct.pack("<I", P.STATE_IDLE), all_buses=True)
                self.log(f"모든 노드 IDLE 명령 ({len(targets)}개, 버스 {len(self.buses)}개)")
            elif kind == "enable":
                ns = self.nodes.get(node)
                if ns is None or ns.pos is None:
                    self.log(f"node {node}: 위치를 아직 못 읽어서 closed loop를 켜지 않음")
                    continue
                if ns.axis_error:
                    self._send(node, cs.CLEAR_ERRORS)
                self._send(node, cs.SET_CONTROLLER_MODES, struct.pack("<ii", P.CONTROL_POSITION,
                                                                       P.INPUT_PASSTHROUGH))
                self._send(node, cs.SET_INPUT_POS, P.pack_input_pos(ns.pos))
                self._send(node, cs.SET_AXIS_STATE, struct.pack("<I", P.STATE_CLOSED_LOOP))
                self._enable_req[node] = self.clock()
            elif kind == "disable":
                self._send(node, cs.SET_AXIS_STATE, struct.pack("<I", P.STATE_IDLE))
            elif kind == "rec_start":
                for ns in self.nodes.values():
                    ns.rec_pmin, ns.rec_pmax, ns.rec_n = math.inf, -math.inf, 0
                self.rec_session = {n: ns.session for n, ns in self.nodes.items()}
                self.recording = True
            elif kind == "clear":
                self._send(node, cs.CLEAR_ERRORS)
                ns = self.nodes.get(node)
                if ns is not None:
                    ns.motor_error = ns.encoder_error = 0

    def _tick(self, tick):
        now = self.clock()
        ok = True
        # 1) 목표 스트리밍
        for j in self.cfg.joints:
            tg = self.cmdr.get(j.name)
            if tg is None or not tg.real:
                continue
            ns = self.nodes[j.node]
            if not ns.online(now):
                if now - tg.t_created > 1.0:
                    self.cmdr.clear(j.name)
                    self.log(f"{j.short}: node {j.node} 응답이 끊겨 목표를 지움")
                continue
            if ns.state != P.STATE_CLOSED_LOOP:
                if now - max(tg.t_created, self._enable_req.get(j.node, -1e9)) > 1.5:
                    self.cmdr.clear(j.name)
                    err = P.decode_bits(ns.axis_error, P.AXIS_ERRORS)
                    self.log(f"{j.short}: 축이 위치 제어가 아님 ({P.state_text(ns.state)}"
                             f"{', ' + err if err else ''}) -> 목표를 지움")
                continue
            if self._guard(j, ns, tg, now):
                continue
            if now <= tg.t_end + 0.1:
                p = j.turn_from_q(tg.eval(now), ns.k)
                ok &= self._send(j.node, cs.SET_INPUT_POS, P.pack_input_pos(p))
        # 2) 요청
        sel = self.selected_node
        for ns in self.nodes.values():
            n = ns.node
            if ns.online(now):
                ns._t_req = time.perf_counter()
                ok &= self._rtr(n, cs.GET_ENCODER_ESTIMATES)
                if n == sel or (tick + n) % 5 == 0:
                    self._rtr(n, cs.GET_IQ)
                if (tick + 7 * n) % 100 == 0:
                    self._rtr(n, cs.GET_VBUS)
                if ns.axis_error and (tick + n) % 25 == 0:
                    self._rtr(n, cs.GET_MOTOR_ERROR)
                    self._rtr(n, cs.GET_ENCODER_ERROR)
            elif (tick + n) % 25 == 0:      # 응답 없는 노드는 모든 버스에서 찾아봄 (배선을 옮겼을 수도 있음)
                self._rtr(n, cs.GET_ENCODER_ESTIMATES, all_buses=True)
        return ok

    def _guard(self, j, ns, tg, now):
        """보호 정지: 추종 오차나 전류가 정해진 시간 넘게 크면 그 관절만 IDLE. 멈췄으면 True.

        방향이 반대로 설정된 관절은 화면 각도로는 명령을 잘 따르는 것처럼 보이다가 기구 끝에 막혀
        전류 한계(18 A)까지 밀게 된다. 그 경우를 여기서 끊는다.
        """
        cfg = self.cfg
        n = j.node
        if ns.pos is None:
            return False
        err = abs(tg.eval(now) - j.q_from_turn(ns.pos, ns.k))
        reason = None
        if err > math.radians(cfg.guard_err_deg):
            t0 = self._err_since.setdefault(n, now)
            if now - t0 > cfg.guard_err_s:
                reason = (f"목표와 실측 차이 {math.degrees(err):.1f}°가 {cfg.guard_err_s:g} s 넘게 "
                          f"{cfg.guard_err_deg:g}°보다 큼")
        else:
            self._err_since.pop(n, None)
        if reason is None and ns.iq is not None and now - ns.t_iq < 0.5 and abs(ns.iq) > cfg.guard_current_a:
            t0 = self._cur_since.setdefault(n, now)
            if now - t0 > cfg.guard_current_s:
                reason = f"전류 {ns.iq:+.1f} A가 {cfg.guard_current_s:g} s 넘게 {cfg.guard_current_a:g} A보다 큼"
        elif ns.iq is not None and abs(ns.iq) <= cfg.guard_current_a:
            self._cur_since.pop(n, None)
        if reason is None:
            return False
        self._send(n, cs.SET_AXIS_STATE, struct.pack("<I", P.STATE_IDLE))
        self.cmdr.clear(j.name)
        self._err_since.pop(n, None)
        self._cur_since.pop(n, None)
        text = (f"⚠ {j.short} (node {n}) 보호 정지: {reason}. 방향이 반대이거나 기구 끝·걸림에 막혔을 수 있습니다.")
        self.log(text)
        self.alerts.append(text)
        return True

    def _idle_unowned(self, node, state, label):
        """연결 직후 처음 본 보드가 IDLE이 아니면 IDLE로 돌린다 (프로그램을 켜자마자 힘이 들어가 있지 않게)."""
        if state is None or state == P.STATE_IDLE:
            return
        for _ in range(2):   # 한 번 놓쳐도 되게 두 번
            self._send(node, cs.SET_AXIS_STATE, struct.pack("<I", P.STATE_IDLE))
        text = (f"{label}: 처음 응답했을 때 이미 '{P.state_text(state)}' 상태여서 힘을 풀었습니다 (IDLE). "
                f"다른 프로그램이 토크를 켜 둔 채 끝났거나, 통신이 끊겼다 돌아온 경우입니다.")
        self.log(text)
        self.alerts.append(text)

    # ---- 받기
    def _learn_bus(self, node, idx, now):
        """그 노드 프레임이 들어온 버스를 기억. 두 버스에서 같은 노드가 보이면 한 번 경고."""
        self._node_bus_t[(node, idx)] = now
        if len(self.buses) > 1:
            other = self._node_bus_t.get((node, 1 - idx))
            if other is not None and now - other < 1.0 and node not in self._dup_warned:
                first = not self._dup_warned
                self._dup_warned.add(node)
                if first:     # 눈에 띄는 경고는 한 번만, 나머지 노드는 기록에만
                    text = (f"⚠ node {node}의 응답이 버스 1과 버스 2 양쪽에서 들어옵니다. CANable 두 개가 같은 "
                            f"버스에 물렸거나, 두 버스에 같은 node ID 보드가 있습니다. 배선과 node ID를 확인하세요.")
                    self.alerts.append(text)
                else:
                    text = f"node {node}도 버스 1·2 양쪽에서 보임"
                self.log(text)
        prev = self.node_bus.get(node)
        self.node_bus[node] = idx
        if prev is not None and prev != idx and node not in self._dup_warned:
            self.log(f"node {node}: 버스 {prev + 1} → 버스 {idx + 1}로 옮겨짐")

    def _handle(self, msg, idx=0):
        if msg.is_error_frame or msg.is_remote_frame or msg.is_extended_id:
            return
        self._rx += 1
        bits = P.frame_bits(len(msg.data))
        self._bits += bits
        if idx < len(self._bits_bus):
            self._bits_bus[idx] += bits
        node, cmd = cs.split_id(msg.arbitration_id)
        now = self.clock()
        self._learn_bus(node, idx, now)
        ns = self.nodes.get(node)
        if ns is None:
            if cmd == cs.HEARTBEAT:
                if node not in self.unknown:
                    self.log(f"설정에 없는 node {node}의 heartbeat (관절 표에 없는 보드)")
                    if len(msg.data) >= 5:
                        _, st = P.parse_heartbeat(bytes(msg.data))
                        self._idle_unowned(node, st, f"설정에 없는 node {node}")
                self.unknown[node] = now
            return
        data = bytes(msg.data)
        if now - ns.last_seen() > NEW_SESSION_GAP:
            ns.new_session = True
            ns.state_checked = False
            ns.session += 1
            j = self.cfg.by_node(node)
            if ns.ever_seen:
                self.log(f"{j.short} (node {node}) 다시 연결됨: 재부팅이면 지금 자세로 각도를 다시 맞춤")
            ns.ever_seen = True
        if cmd == cs.HEARTBEAT and len(data) >= 5:
            ns.axis_error, ns.state = P.parse_heartbeat(data)
            ns.t_hb = now
            if not ns.state_checked:
                ns.state_checked = True
                j = self.cfg.by_node(node)
                tg = self.cmdr.get(j.name)
                if tg is None or not tg.real:      # 이 프로그램이 켠 토크가 아님
                    self._idle_unowned(node, ns.state, f"{j.short} (node {node})")
        elif cmd == cs.GET_ENCODER_ESTIMATES and len(data) >= 8:
            pos, vel = P.parse_two_floats(data)
            if ns.new_session:
                j = self.cfg.by_node(node)
                ns.k = j.unwrap_k(pos)
                ns.new_session = False
            ns.pos, ns.vel, ns.t_enc = pos, vel, now
            ns.enc_hist.append((now, pos, vel))
            if self.recording:
                if pos < ns.rec_pmin:
                    ns.rec_pmin = pos
                if pos > ns.rec_pmax:
                    ns.rec_pmax = pos
                ns.rec_n += 1
            if ns._t_req is not None:
                ns.latency = time.perf_counter() - ns._t_req
                ns._t_req = None
        elif cmd == cs.GET_IQ and len(data) >= 8:
            ns.iq_set, ns.iq = P.parse_two_floats(data)
            ns.t_iq = now
            ns.iq_hist.append((now, ns.iq_set, ns.iq))
        elif cmd == cs.GET_VBUS and len(data) >= 4:
            ns.vbus = P.parse_float(data)
            ns.t_vbus = now
        elif cmd == cs.GET_MOTOR_ERROR and len(data) >= 4:
            ns.motor_error = P.parse_motor_error(data)
        elif cmd == cs.GET_ENCODER_ERROR and len(data) >= 4:
            ns.encoder_error = P.parse_u32(data)
