#!/usr/bin/env python3
"""MotorTestGUI ⇄ Radxa ⇄ CANable 중계 (CAN 프레임을 그대로 주고받음).

PC의 GUI가 SSH로 이 파일을 Radxa에 복사하고 실행한다 (motorgui/radxa_link.py). 연결은 SSH 터널을 거친다:
  GUI ──TCP(127.0.0.1)──ssh -L──> Radxa의 유닉스 소켓 ──> 이 프로그램 ──> CANable(들) ──> 드라이버

표준 라이브러리만 쓴다 (Radxa에 패키지를 설치하지 않음). Python 3.9 이상, Linux.
CAN 장치:
  --slcan auto            CANable 출고 펌웨어(slcan)를 /dev/ttyACM*로 직접 연다 (USB VID:PID 16d0:117e, 기본)
  --slcan /dev/ttyACM0,/dev/ttyACM1
  --socketcan can0,can1   SocketCAN (candleLight 펌웨어나 slcand). 비트레이트는 ip link로 미리 설정
  --pycan virtual:a,b     python-can으로 열기 (PC에서 시험할 때만)

안전:
  - 이 프로그램은 스스로 상태 명령을 보내지 않는다. 예외는 '모두 힘 풀기(IDLE)'뿐:
    GUI 연결이 끊기거나, GUI에서 아무것도 안 온 지 --watchdog 초(기본 0.5 s)가 지나면
    지금까지 본 모든 node에 IDLE(Set Axis State 1)을 두 번 보낸다.
  - --exit-on-stdin-eof: SSH가 끊기면(표준 입력이 닫히면) IDLE을 보내고 끝난다.
  - CAN 장치는 TIOCEXCL로 독점해서 다른 프로그램과 동시에 열리지 않는다.

통신 (작은 끝, 메시지 = 머리 <BH>(종류, 길이) + 내용):
  1 FRAME    양방향  <BBBxI8s> 버스 번호, 플래그(1 확장 ID, 2 RTR), DLC, ID, 데이터 8바이트
  2 PING     GUI→    <dI> 보낸 시각, 번호   →  3 PONG 같은 내용
  4 INFO     →GUI    JSON (연결 직후: 버스 목록, 비트레이트, 버전, IMU 지원 여부)
  5 LOG      →GUI    UTF-8 글
  6 IMU      →GUI    JSON 한 줄 (imu_reader.py 출력 그대로, 끝나면 {"type": "exit", ...})
  7 IMU_CMD  GUI→    JSON {"cmd": "start", "opts": {sensor, bus, port, baud, axes, out_hz}} 또는 {"cmd": "stop"}

IMU: GUI가 IMU_CMD start를 보내면 같은 폴더의 imu_reader.py를 별도 프로세스로 띄운다 (BMI088이면 numpy·smbus2가
그 프로세스에만 필요). IMU가 멈추거나 오류가 나도 CAN 중계와 감시(IDLE)에는 영향이 없다. GUI 연결이 끊기면 IMU도 끈다.
"""
import argparse
import glob
import json
import os
import re
import select
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque

VERSION = "2026-11-23"
CANABLE_IDS = {("16d0", "117e")}
SLCAN_BITRATE = {10000: b"0", 20000: b"1", 50000: b"2", 100000: b"3", 125000: b"4", 250000: b"5",
                 500000: b"6", 750000: b"7", 1000000: b"8"}
HDR = struct.Struct("<BH")
FRAME = struct.Struct("<BBBxI8s")
PING = struct.Struct("<dI")
T_FRAME, T_PING, T_PONG, T_INFO, T_LOG, T_IMU, T_IMU_CMD = 1, 2, 3, 4, 5, 6, 7
F_EXT, F_RTR = 1, 2
SET_AXIS_STATE, STATE_IDLE = 0x007, 1
IMU_READER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imu_reader.py")
AXIS_RE = re.compile(r"^[+-]?[xyz]$")


def log(text):
    print(f"[bridge] {text}", file=sys.stderr, flush=True)


# ---------------------------------------------------------------------- CAN 장치
class SlcanTty:
    """CANable 출고 펌웨어(slcan)를 직렬 장치로 직접 다룬다."""

    kind = "slcan"

    def __init__(self, path, bitrate):
        import fcntl
        import termios
        import tty
        self.name = os.path.basename(path)
        self.path = path
        self._lock = threading.Lock()
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)      # 다른 프로그램이 같이 열지 못하게
        except OSError:
            pass
        tty.setraw(self.fd)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        code = SLCAN_BITRATE.get(int(bitrate))
        if code is None:
            raise ValueError(f"slcan이 지원하지 않는 비트레이트: {bitrate}")
        self._write(b"C\r")
        time.sleep(0.05)
        self._write(b"S" + code + b"\r")
        self._write(b"O\r")
        self.serial = _usb_attr(path, "serial")

    def _write(self, data):
        with self._lock:
            view = memoryview(data)
            while view:
                n = os.write(self.fd, view)
                view = view[n:]

    def send(self, can_id, flags, dlc, data):
        dlc = max(0, min(8, dlc))
        if flags & F_EXT:
            head = (b"R" if flags & F_RTR else b"T") + b"%08X%d" % (can_id & 0x1FFFFFFF, dlc)
        else:
            head = (b"r" if flags & F_RTR else b"t") + b"%03X%d" % (can_id & 0x7FF, dlc)
        body = b"" if flags & F_RTR else data[:dlc].hex().upper().encode()
        self._write(head + body + b"\r")

    def run(self, on_frame, stop):
        buf = b""
        while not stop.is_set():
            r, _, _ = select.select([self.fd], [], [], 0.2)
            if not r:
                continue
            chunk = os.read(self.fd, 4096)
            if not chunk:
                raise OSError("장치가 닫힘")
            buf += chunk
            while True:
                i = buf.find(b"\r")
                j = buf.find(b"\x07")
                if i < 0 and j < 0:
                    break
                k = min(x for x in (i, j) if x >= 0)
                line, buf = buf[:k], buf[k + 1:]
                fr = parse_slcan(line)
                if fr is not None:
                    on_frame(*fr)
            if len(buf) > 4096:
                buf = b""

    def close(self):
        try:
            self._write(b"C\r")
        except OSError:
            pass
        try:
            os.close(self.fd)
        except OSError:
            pass


def parse_slcan(line):
    """slcan 한 줄 -> (id, flags, dlc, data8) 또는 None."""
    if not line:
        return None
    c = line[:1]
    try:
        if c in (b"t", b"r"):
            can_id, dlc, p, flags = int(line[1:4], 16), int(line[4:5]), 5, 0
        elif c in (b"T", b"R"):
            can_id, dlc, p, flags = int(line[1:9], 16), int(line[9:10]), 10, F_EXT
        else:
            return None
        dlc = min(dlc, 8)
        if c in (b"r", b"R"):
            return can_id, flags | F_RTR, dlc, bytes(8)
        data = bytes.fromhex(line[p:p + 2 * dlc].decode())
        return can_id, flags, dlc, data.ljust(8, b"\x00")
    except (ValueError, IndexError):
        return None


class SocketCan:
    kind = "socketcan"
    _FMT = struct.Struct("=IB3x8s")

    def __init__(self, ifname, bitrate=None):
        self.name = ifname
        self.serial = None
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.settimeout(0.2)

    def send(self, can_id, flags, dlc, data):
        cid = can_id & (0x1FFFFFFF if flags & F_EXT else 0x7FF)
        if flags & F_EXT:
            cid |= socket.CAN_EFF_FLAG
        if flags & F_RTR:
            cid |= socket.CAN_RTR_FLAG
        self.sock.send(self._FMT.pack(cid, dlc, data[:8].ljust(8, b"\x00")))

    def run(self, on_frame, stop):
        while not stop.is_set():
            try:
                raw = self.sock.recv(16)
            except socket.timeout:
                continue
            cid, dlc, data = self._FMT.unpack(raw)
            if cid & socket.CAN_ERR_FLAG:
                continue
            flags = (F_EXT if cid & socket.CAN_EFF_FLAG else 0) | (F_RTR if cid & socket.CAN_RTR_FLAG else 0)
            on_frame(cid & (0x1FFFFFFF if flags & F_EXT else 0x7FF), flags, dlc, data)

    def close(self):
        self.sock.close()


class PyCan:
    """python-can으로 연다 (PC에서 시험할 때). 예: virtual:a,b  slcan:COM16"""

    kind = "pycan"

    def __init__(self, interface, channel, bitrate):
        import can
        self._can = can
        self.name = f"{interface}:{channel}"
        self.serial = None
        kw = {"receive_own_messages": False} if interface == "virtual" else {"bitrate": bitrate}
        self.bus = can.Bus(interface=interface, channel=channel, **kw)

    def send(self, can_id, flags, dlc, data):
        self.bus.send(self._can.Message(arbitration_id=can_id, is_extended_id=bool(flags & F_EXT),
                                        is_remote_frame=bool(flags & F_RTR), dlc=dlc,
                                        data=None if flags & F_RTR else data[:dlc]))

    def run(self, on_frame, stop):
        while not stop.is_set():
            m = self.bus.recv(0.2)
            if m is None or m.is_error_frame:
                continue
            flags = (F_EXT if m.is_extended_id else 0) | (F_RTR if m.is_remote_frame else 0)
            on_frame(m.arbitration_id, flags, m.dlc, bytes(m.data).ljust(8, b"\x00"))

    def close(self):
        self.bus.shutdown()


def _usb_attr(dev_path, attr):
    """/dev/ttyACM0 -> 그 USB 장치의 sysfs 속성 (idVendor 등). 없으면 None."""
    base = os.path.realpath(f"/sys/class/tty/{os.path.basename(dev_path)}/device")
    for d in (base, os.path.dirname(base), os.path.dirname(os.path.dirname(base))):
        p = os.path.join(d, attr)
        if os.path.isfile(p):
            try:
                with open(p) as f:
                    return f.read().strip()
            except OSError:
                return None
    return None


def find_canables():
    out = []
    for dev in sorted(glob.glob("/dev/ttyACM*")):
        vid, pid = _usb_attr(dev, "idVendor"), _usb_attr(dev, "idProduct")
        if (vid, pid) in CANABLE_IDS:
            out.append(dev)
    return out


# ---------------------------------------------------------------------- IMU (별도 프로세스)
def imu_args(opts):
    """GUI 설정 → imu_reader.py 인자. 알려진 값만 받는다."""
    sensor = opts.get("sensor")
    if sensor not in ("bmi088", "im10a", "fake"):
        raise ValueError(f"모르는 IMU 종류: {sensor!r}")
    axes = [str(s).strip() for s in opts.get("axes", ["+x", "+y", "+z"])]
    if len(axes) != 3 or not all(AXIS_RE.match(s) for s in axes):
        raise ValueError(f"IMU 축 설정이 이상합니다: {axes}")
    out_hz = min(200.0, max(1.0, float(opts.get("out_hz", 50.0))))
    # '-x'처럼 마이너스로 시작하는 값을 argparse가 옵션으로 읽지 않게 한 덩어리로 넘긴다
    args = ["--sensor", sensor, "--json", "--out-hz", f"{out_hz:g}", "--axes=" + ",".join(axes)]
    if sensor == "bmi088":
        args += ["--bus", str(int(opts.get("bus", 3)))]
    elif sensor == "im10a":
        port = str(opts.get("port", "/dev/ttyS4")).strip()
        if not re.match(r"^/dev/[A-Za-z0-9_./-]+$", port):
            raise ValueError(f"IMU 포트 이름이 이상합니다: {port!r}")
        args += ["--port", port, "--baud", str(int(opts.get("baud", 230400)))]
    return args


class ImuProc:
    def __init__(self, send):
        self.send = send                  # send(종류, 내용): 지금 연결된 GUI로
        self.proc = None
        self.lock = threading.Lock()

    def start(self, opts):
        args = imu_args(opts)
        self.stop()
        if not os.path.exists(IMU_READER):
            raise RuntimeError(f"{IMU_READER}가 없습니다 (GUI가 복사하지 못함)")
        p = subprocess.Popen([sys.executable, "-u", IMU_READER, *args], stdin=subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        with self.lock:
            self.proc = p
        tail = deque(maxlen=6)
        threading.Thread(target=self._err, args=(p, tail), daemon=True, name="imu-err").start()
        threading.Thread(target=self._out, args=(p, tail), daemon=True, name="imu-out").start()
        log(f"IMU 시작: {' '.join(args)} (pid {p.pid})")

    def _out(self, p, tail):
        for raw in iter(p.stdout.readline, b""):
            line = raw.strip()
            if line:
                self.send(T_IMU, line)
        rc = p.wait()
        with self.lock:
            current = self.proc is p
            if current:
                self.proc = None
        if current:                        # 일부러 끈 것이 아니면 GUI에 알림
            time.sleep(0.05)               # stderr 마지막 줄을 tail에 받을 시간
            self.send(T_IMU, json.dumps({"type": "exit", "code": rc, "msg": " / ".join(tail)}).encode())
            log(f"IMU 프로그램 끝남 (코드 {rc}) {' / '.join(tail)}")

    def _err(self, p, tail):
        for raw in iter(p.stderr.readline, b""):
            line = raw.decode(errors="replace").strip()
            if line:
                tail.append(line)
                self.send(T_LOG, f"[IMU] {line}".encode())

    def stop(self):
        with self.lock:
            p, self.proc = self.proc, None
        if p is not None and p.poll() is None:
            if os.name == "nt":       # PC 시험용: 가상환경 python.exe는 진짜 python을 자식으로 띄우므로 같이 끔
                subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], capture_output=True)
            else:
                p.terminate()
            try:
                p.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                p.kill()
            log("IMU 끔")


# ---------------------------------------------------------------------- 중계
class Bridge:
    def __init__(self, devices, bitrate, watchdog_s):
        self.devices = devices
        self.bitrate = bitrate
        self.watchdog_s = watchdog_s
        self.stop = threading.Event()
        self.client = None
        self.client_lock = threading.Lock()
        self.last_rx = 0.0
        self.tripped = False
        self.seen = [set() for _ in devices]        # 버스별로 본 node
        self.dev_err = [None] * len(devices)
        self.counts = {"rx_can": 0, "tx_can": 0, "tx_err": 0}
        self.imu = ImuProc(self.send_client)

    def send_client(self, typ, payload):
        c = self.client
        if c is not None:
            try:
                c.send_msg(typ, payload)
            except OSError:
                pass

    def imu_command(self, payload):
        try:
            d = json.loads(payload.decode())
            if d.get("cmd") == "start":
                self.imu.start(d.get("opts") or {})
            elif d.get("cmd") == "stop":
                self.imu.stop()
                self.send_client(T_IMU, json.dumps({"type": "stopped"}).encode())
            else:
                raise ValueError(f"모르는 IMU 명령: {d.get('cmd')!r}")
        except Exception as e:  # noqa: BLE001 - 설정 오류를 GUI에 보여 줌
            self.send_client(T_IMU, json.dumps({"type": "error", "msg": f"IMU 시작 실패: {e}"}).encode())

    # -- CAN에서 받음
    def on_frame(self, bus, can_id, flags, dlc, data):
        self.counts["rx_can"] += 1
        if not flags & F_EXT:
            self.seen[bus].add(can_id >> 5)
        c = self.client
        if c is not None:
            try:
                c.send_msg(T_FRAME, FRAME.pack(bus, flags, dlc, can_id, data))
            except OSError:
                pass

    def dev_loop(self, i):
        dev = self.devices[i]
        try:
            dev.run(lambda *fr: self.on_frame(i, *fr), self.stop)
        except Exception as e:  # noqa: BLE001 - USB가 빠지는 등
            self.dev_err[i] = str(e)
            log(f"버스 {i + 1} ({dev.name}) 수신 중단: {e}")
            c = self.client
            if c is not None:
                try:
                    c.send_msg(T_LOG, f"⚠ Radxa: 버스 {i + 1} ({dev.name}) 수신 중단: {e}".encode())
                except OSError:
                    pass

    # -- 모두 힘 풀기
    def idle_all(self, why):
        n = 0
        for _ in range(2):
            for i, dev in enumerate(self.devices):
                for node in sorted(self.seen[i]):
                    try:
                        dev.send((node << 5) | SET_AXIS_STATE, 0, 8, struct.pack("<I", STATE_IDLE).ljust(8, b"\x00"))
                        n += 1
                    except OSError:
                        pass
        log(f"모두 힘 풀기 ({why}): IDLE 프레임 {n}개")
        return n

    def watchdog_loop(self):
        while not self.stop.wait(0.05):
            if self.client is not None and not self.tripped and time.monotonic() - self.last_rx > self.watchdog_s:
                self.tripped = True
                n = self.idle_all(f"GUI에서 {self.watchdog_s:g} s 동안 아무것도 안 옴")
                c = self.client
                if c is not None:
                    try:
                        c.send_msg(T_LOG, f"⚠ Radxa 감시: GUI 응답이 {self.watchdog_s:g} s 끊겨 모두 힘 풀기 "
                                          f"(IDLE {n}개)".encode())
                    except OSError:
                        pass

    # -- GUI 연결 하나
    def serve_client(self, conn):
        cl = Client(conn)
        with self.client_lock:
            if self.client is not None:
                cl.send_msg(T_LOG, "다른 GUI가 이미 연결되어 있습니다".encode())
                conn.close()
                return
            self.client = cl
        self.last_rx = time.monotonic()
        self.tripped = False
        info = {"version": VERSION, "bitrate": self.bitrate, "watchdog_s": self.watchdog_s, "host": socket.gethostname(),
                "buses": [{"name": d.name, "kind": d.kind, "serial": d.serial} for d in self.devices],
                "imu": os.path.exists(IMU_READER)}
        cl.send_msg(T_INFO, json.dumps(info).encode())
        log(f"GUI 연결됨 (버스 {len(self.devices)}개)")
        try:
            for typ, payload in cl.messages():
                self.last_rx = time.monotonic()
                if self.tripped:
                    self.tripped = False
                if typ == T_FRAME and len(payload) == FRAME.size:
                    bus, flags, dlc, can_id, data = FRAME.unpack(payload)
                    if bus < len(self.devices):
                        if not flags & F_EXT:
                            self.seen[bus].add(can_id >> 5)
                        try:
                            self.devices[bus].send(can_id, flags, dlc, data)
                            self.counts["tx_can"] += 1
                        except OSError as e:
                            self.counts["tx_err"] += 1
                            if self.counts["tx_err"] % 100 == 1:
                                cl.send_msg(T_LOG, f"⚠ Radxa: 버스 {bus + 1} 송신 실패: {e}".encode())
                elif typ == T_PING:
                    cl.send_msg(T_PONG, payload)
                elif typ == T_IMU_CMD:
                    threading.Thread(target=self.imu_command, args=(payload,), daemon=True, name="imu-cmd").start()
        except OSError:
            pass
        finally:
            with self.client_lock:
                self.client = None
            try:
                conn.close()
            except OSError:
                pass
            log("GUI 연결 끊김")
            self.idle_all("GUI 연결 끊김")
            self.imu.stop()

    def close(self):
        self.stop.set()
        for d in self.devices:
            d.close()


class Client:
    def __init__(self, conn):
        self.conn = conn
        self._wlock = threading.Lock()
        if conn.family in (socket.AF_INET, socket.AF_INET6):
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def send_msg(self, typ, payload):
        data = HDR.pack(typ, len(payload)) + payload
        with self._wlock:
            self.conn.sendall(data)

    def messages(self):
        buf = b""
        while True:
            chunk = self.conn.recv(65536)
            if not chunk:
                return
            buf += chunk
            while len(buf) >= HDR.size:
                typ, n = HDR.unpack_from(buf)
                if len(buf) < HDR.size + n:
                    break
                yield typ, buf[HDR.size:HDR.size + n]
                buf = buf[HDR.size + n:]


def open_devices(a):
    devs = []
    try:
        if a.socketcan:
            for name in a.socketcan.split(","):
                devs.append(SocketCan(name.strip()))
        elif a.pycan:
            iface, chans = a.pycan.split(":", 1)
            for ch in chans.split(","):
                devs.append(PyCan(iface, ch.strip(), a.bitrate))
        else:
            paths = find_canables() if a.slcan == "auto" else [p.strip() for p in a.slcan.split(",") if p.strip()]
            for p in paths:
                devs.append(SlcanTty(p, a.bitrate))
    except Exception:
        for d in devs:
            d.close()
        raise
    return devs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unix", help="이 유닉스 소켓에서 GUI를 기다림 (SSH 터널용)")
    ap.add_argument("--listen", help="HOST:PORT 에서 TCP로 기다림 (시험용. 같은 네트워크 누구나 접속 가능하니 주의)")
    ap.add_argument("--slcan", default="auto")
    ap.add_argument("--socketcan", default="")
    ap.add_argument("--pycan", default="")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--watchdog", type=float, default=0.5)
    ap.add_argument("--exit-on-stdin-eof", action="store_true")
    ap.add_argument("--allow-empty", action="store_true", help="CAN 장치가 없어도 시작 (통신 시험용)")
    a = ap.parse_args()
    if not a.unix and not a.listen:
        ap.error("--unix 또는 --listen 이 필요합니다")

    try:
        devs = open_devices(a)
    except Exception as e:  # noqa: BLE001
        print(f"ERROR CAN 장치 열기 실패: {e}", flush=True)
        log(f"CAN 장치 열기 실패: {e}")
        return 2
    if not devs and not a.allow_empty:
        print("ERROR Radxa에 CANable(USB 16d0:117e)이 없습니다. USB 연결을 확인하세요", flush=True)
        return 3
    br = Bridge(devs, a.bitrate, a.watchdog)
    for i in range(len(devs)):
        threading.Thread(target=br.dev_loop, args=(i,), daemon=True, name=f"can{i}").start()
    threading.Thread(target=br.watchdog_loop, daemon=True, name="watchdog").start()

    if a.unix:
        try:
            os.unlink(a.unix)
        except FileNotFoundError:
            pass
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(a.unix)
        os.chmod(a.unix, 0o600)
        where = a.unix
    else:
        host, port = a.listen.rsplit(":", 1)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, int(port)))
        where = f"{host}:{srv.getsockname()[1]}"
    srv.listen(2)
    srv.settimeout(0.5)

    def stdin_watch():
        try:
            while sys.stdin.buffer.read(1):
                pass
        except (OSError, ValueError):
            pass
        log("표준 입력 닫힘 (SSH 끊김) → 끝냄")
        br.stop.set()

    if a.exit_on_stdin_eof:
        threading.Thread(target=stdin_watch, daemon=True, name="stdin").start()

    ready = {"where": where, "buses": [d.name for d in devs], "version": VERSION}
    print("READY " + json.dumps(ready), flush=True)
    log(f"대기 중: {where}, 버스 {[d.name for d in devs]}, 비트레이트 {a.bitrate}")
    try:
        while not br.stop.is_set():
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            threading.Thread(target=br.serve_client, args=(conn,), daemon=True, name="client").start()
    except KeyboardInterrupt:
        pass
    finally:
        br.idle_all("중계 끝남")
        br.imu.stop()
        br.close()
        srv.close()
        if a.unix:
            try:
                os.unlink(a.unix)
            except OSError:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
