"""ODrive fw 0.5.1 CAN Simple 프로토콜 (python-can).

프레임 ID = node_id << 5 | cmd_id (표준 11비트). Get 계열은 RTR 요청을 보내면 같은 ID로 응답한다.
0.5.1은 위치를 주기적으로 보내 주지 않으므로 매 주기 요청해야 한다. 근거: 펌웨어 can_simple.cpp.
"""
import struct
import sys
import time

import can

from . import params

HEARTBEAT = 0x001
ESTOP = 0x002
GET_MOTOR_ERROR = 0x003
GET_ENCODER_ERROR = 0x004
SET_AXIS_STATE = 0x007
GET_ENCODER_ESTIMATES = 0x009
SET_CONTROLLER_MODES = 0x00B
SET_INPUT_POS = 0x00C
SET_INPUT_VEL = 0x00D
SET_INPUT_TORQUE = 0x00E
SET_VEL_LIMIT = 0x00F
GET_IQ = 0x014
REBOOT = 0x016
GET_VBUS = 0x017
CLEAR_ERRORS = 0x018

CANABLE_SLCAN_VIDPID = (0x16D0, 0x117E)


def arb_id(node, cmd):
    return (node << 5) | cmd


def split_id(arbitration_id):
    return arbitration_id >> 5, arbitration_id & 0x1F


def add_args(parser, node_required=True):
    parser.add_argument("--interface", default="slcan", choices=["slcan", "gs_usb", "pcan", "virtual"],
                        help="slcan: 출고 펌웨어(COM 포트) / gs_usb: candleLight / pcan / virtual: 시뮬레이션")
    parser.add_argument("--channel", default=None,
                        help="slcan은 COM 포트(예: COM5, 비우면 자동 탐색), gs_usb는 장치 번호(0), pcan은 PCAN_USBBUS1")
    parser.add_argument("--bitrate", type=int, default=params.CAN_BITRATE)
    parser.add_argument("--node", type=int, required=node_required, help="CAN node ID (06_can_check.py scan으로 확인)")
    parser.add_argument("--sim-motor", default="6512", choices=sorted(params.MOTORS),
                        help="--interface virtual일 때 시뮬레이션할 모터")


def find_slcan_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    hits = [p for p in ports if (p.vid, p.pid) == CANABLE_SLCAN_VIDPID or "canable" in (p.description or "").lower()]
    if len(hits) == 1:
        return hits[0].device
    print("COM 포트 목록:")
    for p in ports:
        vidpid = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "----:----"
        print(f"  {p.device:8s} {vidpid}  {p.description}")
    sys.exit("CANable(slcan) COM 포트를 하나로 정하지 못했습니다. --channel COMx 로 지정하세요.")


def open_bus(args):
    """args.interface / channel / bitrate로 python-can Bus를 연다. virtual이면 시뮬레이션 노드도 띄운다."""
    try:
        return _open_bus(args)
    except (can.CanError, OSError, ValueError) as e:
        hint = {"slcan": "COM 포트 번호, 다른 프로그램(시리얼 터미널 등)이 포트를 잡고 있지 않은지",
                "gs_usb": "candleLight 펌웨어인지, Windows 장치 관리자에서 WinUSB 드라이버인지(00_env_check.py)",
                "pcan": "PEAK 드라이버 설치와 채널 이름"}.get(args.interface, "")
        raise RuntimeError(f"CAN 어댑터 열기 실패 ({args.interface}): {e}\n  확인: {hint}")


def _open_bus(args):
    kind = args.interface
    if kind == "virtual":
        from .sim import start_virtual_can
        channel = args.channel or "bench"
        start_virtual_can(channel, args.node if args.node is not None else 1, args.sim_motor)
        print(f"[SIM] 가상 CAN '{channel}'에 시뮬레이션 노드를 띄웠습니다.")
        return can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
    if kind == "slcan":
        channel = args.channel or find_slcan_port()
        print(f"slcan {channel}, {args.bitrate} bps")
        return can.Bus(interface="slcan", channel=channel, bitrate=args.bitrate)
    if kind == "gs_usb":
        # candleLight는 libusb-1.0이 필요하다. ODrive(USB)가 쓰는 libusb0과 섞이지 않게 이 프로세스에서만 로드.
        import libusb_package
        import usb.backend.libusb1
        usb.backend.libusb1.get_backend(find_library=lambda _: libusb_package.get_library_path())
        index = int(args.channel or 0)
        print(f"gs_usb #{index}, {args.bitrate} bps")
        return can.Bus(interface="gs_usb", channel=index, index=index, bitrate=args.bitrate)
    if kind == "pcan":
        channel = args.channel or "PCAN_USBBUS1"
        print(f"pcan {channel}, {args.bitrate} bps")
        return can.Bus(interface="pcan", channel=channel, bitrate=args.bitrate)
    raise ValueError(kind)


class Node:
    """한 축(node)과 주고받기."""

    def __init__(self, bus, node_id, timeout=0.05):
        self.bus = bus
        self.node = node_id
        self.timeout = timeout
        self.heartbeat = None      # (axis_error, axis_state)
        self.heartbeat_time = None
        self.last_rtt = None

    # ---- 보내기 ----
    def send(self, cmd, data=b""):
        data = data.ljust(8, b"\x00")
        self.bus.send(can.Message(arbitration_id=arb_id(self.node, cmd), data=data, is_extended_id=False))

    def set_axis_state(self, state):
        self.send(SET_AXIS_STATE, struct.pack("<I", state))

    def set_controller_modes(self, control_mode, input_mode):
        self.send(SET_CONTROLLER_MODES, struct.pack("<ii", control_mode, input_mode))

    def set_input_pos(self, pos, vel_ff=0.0, torque_ff=0.0):
        """pos [turn], vel_ff [turn/s], torque_ff [Nm]. FF는 0.001 단위 int16."""
        v = max(-32768, min(32767, int(round(vel_ff * 1000))))
        t = max(-32768, min(32767, int(round(torque_ff * 1000))))
        self.send(SET_INPUT_POS, struct.pack("<fhh", pos, v, t))

    def set_input_torque(self, torque):
        self.send(SET_INPUT_TORQUE, struct.pack("<f", torque))

    def set_vel_limit(self, vel_limit):
        self.send(SET_VEL_LIMIT, struct.pack("<f", vel_limit))

    def clear_errors(self):
        self.send(CLEAR_ERRORS)

    def estop(self):
        self.send(ESTOP)

    # ---- 받기 ----
    def _note(self, msg):
        """수신 중 지나가는 heartbeat를 기록."""
        if msg is None or msg.is_remote_frame or msg.is_extended_id:
            return
        node, cmd = split_id(msg.arbitration_id)
        if node == self.node and cmd == HEARTBEAT and len(msg.data) >= 8:
            self.heartbeat = struct.unpack("<II", bytes(msg.data[:8]))
            self.heartbeat_time = time.perf_counter()

    def drain(self):
        while True:
            msg = self.bus.recv(0)
            if msg is None:
                return
            self._note(msg)

    def request(self, cmd, timeout=None):
        """RTR 요청 -> 응답 data(bytes). 시간 초과면 TimeoutError."""
        timeout = self.timeout if timeout is None else timeout
        self.drain()
        want = arb_id(self.node, cmd)
        t0 = time.perf_counter()
        self.bus.send(can.Message(arbitration_id=want, is_remote_frame=True, dlc=8, is_extended_id=False))
        deadline = t0 + timeout
        while True:
            left = deadline - time.perf_counter()
            if left <= 0:
                raise TimeoutError(f"node {self.node} cmd 0x{cmd:03X} 응답 없음 ({timeout * 1000:.0f} ms)")
            msg = self.bus.recv(left)
            self._note(msg)
            if msg is not None and msg.arbitration_id == want and not msg.is_remote_frame:
                self.last_rtt = time.perf_counter() - t0
                return bytes(msg.data)

    def get_encoder_estimates(self):
        """(pos [turn], vel [turn/s])"""
        return struct.unpack("<ff", self.request(GET_ENCODER_ESTIMATES)[:8])

    def get_iq(self):
        """(Iq_setpoint, Iq_measured) [A]"""
        return struct.unpack("<ff", self.request(GET_IQ)[:8])

    def get_vbus(self):
        return struct.unpack("<f", self.request(GET_VBUS)[:4])[0]

    def get_motor_error(self):
        return struct.unpack("<I", self.request(GET_MOTOR_ERROR)[:4])[0]

    def get_encoder_error(self):
        return struct.unpack("<I", self.request(GET_ENCODER_ERROR)[:4])[0]

    def wait_heartbeat(self, timeout=0.5):
        """다음 heartbeat (axis_error, axis_state). 없으면 None."""
        t_prev = self.heartbeat_time
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self._note(self.bus.recv(max(0.0, deadline - time.perf_counter())))
            if self.heartbeat_time != t_prev:
                return self.heartbeat
        return None

    def wait_state(self, state, timeout=2.0):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            hb = self.wait_heartbeat(deadline - time.perf_counter())
            if hb and hb[1] == state:
                return True
            if hb and hb[0]:
                return False
        return False


def scan(bus, duration=1.5):
    """heartbeat를 듣고 {node_id: (axis_error, axis_state, 횟수)}."""
    found = {}
    deadline = time.perf_counter() + duration
    while time.perf_counter() < deadline:
        msg = bus.recv(max(0.0, deadline - time.perf_counter()))
        if msg is None or msg.is_remote_frame or msg.is_extended_id:
            continue
        node, cmd = split_id(msg.arbitration_id)
        if cmd == HEARTBEAT and len(msg.data) >= 8:
            err, state = struct.unpack("<II", bytes(msg.data[:8]))
            n = found.get(node, (0, 0, 0))[2] + 1
            found[node] = (err, state, n)
    return found
