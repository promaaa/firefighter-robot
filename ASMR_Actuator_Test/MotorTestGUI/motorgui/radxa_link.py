"""Radxa 경유 CAN: GUI의 CAN 프레임을 LAN(SSH 터널)으로 Radxa의 중계 프로그램(radxa/can_bridge.py)에 보낸다.

연결 순서 (host가 SSH 별칭이나 주소일 때, 예: 'radxa'):
  1. ssh로 중계 프로그램을 Radxa의 ~/motor_bridge/에 복사 (매번 덮어씀: GUI와 같은 판을 씀)
  2. ssh -L 127.0.0.1:<빈 포트>:<Radxa 유닉스 소켓> 로 중계 프로그램 실행 → 'READY' 줄을 기다림
  3. 127.0.0.1:<포트>로 TCP 접속 → INFO(버스 목록) 받음
host가 'tcp://IP:PORT'면 이미 --listen으로 실행 중인 중계 프로그램에 바로 접속한다 (시험용).

CanLink는 버스마다 RadxaBus(python-can BusABC)를 받으므로 폴링·보호 정지·보행 테스트가 그대로 동작한다.
SSH가 끊기거나 GUI가 멈추면 Radxa 쪽 감시(0.5 s)가 모든 node를 IDLE로 둔다.
"""
import json
import os
import queue
import random
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import can

BRIDGE_SRC = Path(__file__).resolve().parents[1] / "radxa" / "can_bridge.py"
REMOTE_DIR = "~/motor_bridge"
HDR = struct.Struct("<BH")
FRAME = struct.Struct("<BBBxI8s")
PING = struct.Struct("<dI")
T_FRAME, T_PING, T_PONG, T_INFO, T_LOG = 1, 2, 3, 4, 5
F_EXT, F_RTR = 1, 2
PING_S = 0.1
SSH_OPTS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=2",
            "-o", "ServerAliveCountMax=3"]
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

HINT = ("확인: Radxa 전원·랜선, 노트북 '이더넷' 주소가 192.168.137.1인지(인터넷 연결 공유), "
        "PowerShell에서 'ssh radxa hostname'이 되는지 (CLAUDE.md '연결이 안 될 때')")


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class RadxaConnection:
    def __init__(self, host, bitrate, log=None, timeout=15.0, extra_args=""):
        self.host = (host or "radxa").strip()
        self.bitrate = int(bitrate)
        self._log = log or (lambda s: None)
        self.timeout = timeout
        self.extra_args = extra_args
        self.proc = None
        self.sock = None
        self.info = {}
        self.queues = []
        self.dead = None                     # 끊긴 이유 (None이면 살아 있음)
        self.rtt_ms = None
        self.logs = deque(maxlen=200)
        self._wlock = threading.Lock()
        self._released = set()
        self._ready = threading.Event()
        self._ready_line = None
        self._stderr = deque(maxlen=30)
        self._seq = 0
        self._stop = threading.Event()

    # ------------------------------------------------------------------ 시작
    def start(self):
        try:
            if self.host.startswith("tcp://"):
                h, p = self.host[6:].rsplit(":", 1)
                self._connect((h, int(p)))
            else:
                self._deploy()
                self._start_ssh()
            self._read_info()
        except Exception:
            self.close()
            raise
        self.queues = [queue.SimpleQueue() for _ in self.info.get("buses", [])]
        threading.Thread(target=self._reader, daemon=True, name="radxa-rx").start()
        threading.Thread(target=self._pinger, daemon=True, name="radxa-ping").start()
        return self

    def _ssh(self, args, **kw):
        return subprocess.run(["ssh", *SSH_OPTS, self.host, *args], capture_output=True, timeout=25,
                              creationflags=NO_WINDOW, **kw)

    def _deploy(self):
        """중계 프로그램을 Radxa에 복사 (scp 대신 ssh + cat: 한 번에)."""
        src = BRIDGE_SRC.read_bytes()
        t0 = time.perf_counter()
        r = self._ssh([f"mkdir -p {REMOTE_DIR} && cat > {REMOTE_DIR}/can_bridge.py && python3 --version"], input=src)
        if r.returncode != 0:
            err = (r.stderr or b"").decode(errors="replace").strip()
            raise RuntimeError(f"Radxa({self.host})에 SSH로 접속하지 못했습니다: {err or r.returncode}\n{HINT}")
        self._log(f"Radxa({self.host})에 중계 프로그램 복사 ({len(src)} 바이트, "
                  f"{(r.stdout or b'').decode(errors='replace').strip()}, {time.perf_counter() - t0:.1f} s)")

    def _start_ssh(self):
        lport = _free_port()
        sock_path = f"/tmp/motortestgui_{os.getpid()}_{random.randrange(1 << 30):x}.sock"
        remote = (f"exec python3 -u {REMOTE_DIR}/can_bridge.py --unix {sock_path} --bitrate {self.bitrate} "
                  f"--exit-on-stdin-eof {self.extra_args}").strip()
        cmd = ["ssh", *SSH_OPTS, "-o", "ExitOnForwardFailure=yes", "-L", f"127.0.0.1:{lport}:{sock_path}",
               self.host, remote]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     creationflags=NO_WINDOW)
        threading.Thread(target=self._read_stdout, daemon=True, name="radxa-stdout").start()
        threading.Thread(target=self._read_stderr, daemon=True, name="radxa-stderr").start()
        if not self._ready.wait(self.timeout) or not (self._ready_line or "").startswith("READY"):
            tail = " / ".join(self._stderr) or (self._ready_line or "응답 없음")
            msg = (self._ready_line or "")[6:] if (self._ready_line or "").startswith("ERROR") else tail
            raise RuntimeError(f"Radxa 중계 프로그램이 시작되지 않았습니다: {msg}\n{HINT}")
        ready = json.loads(self._ready_line[6:])
        self._log(f"Radxa 중계 시작: CAN {ready.get('buses')} (판 {ready.get('version')})")
        deadline = time.perf_counter() + 5.0
        while True:          # ssh 터널이 열리는 데 조금 걸릴 수 있음
            try:
                self._connect(("127.0.0.1", lport))
                return
            except OSError:
                if time.perf_counter() > deadline:
                    raise
                time.sleep(0.1)

    def _read_stdout(self):
        for raw in iter(self.proc.stdout.readline, b""):
            line = raw.decode(errors="replace").strip()
            if line.startswith(("READY", "ERROR")) and not self._ready.is_set():
                self._ready_line = line
                self._ready.set()
            elif line:
                self._log(f"[Radxa] {line}")
        if not self._ready.is_set():
            self._ready.set()
        self._mark_dead("Radxa 중계 프로그램이 끝났습니다 (SSH 끊김)")

    def _read_stderr(self):
        for raw in iter(self.proc.stderr.readline, b""):
            line = raw.decode(errors="replace").strip()
            if line:
                self._stderr.append(line)
                self.logs.append(line)
                self._log(f"[Radxa] {line}")

    def _connect(self, addr):
        s = socket.create_connection(addr, timeout=5.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(None)
        self.sock = s

    def _read_info(self):
        self.sock.settimeout(5.0)
        typ, payload = self._recv_msg_blocking()
        while typ == T_LOG:
            self._log(f"[Radxa] {payload.decode(errors='replace')}")
            typ, payload = self._recv_msg_blocking()
        if typ != T_INFO:
            raise RuntimeError("Radxa 중계 프로그램 응답이 올바르지 않습니다")
        self.info = json.loads(payload.decode())
        self.sock.settimeout(None)          # INFO 뒤에 이미 받은 프레임은 self._buf에 남아 _reader가 이어 씀

    def _recv_msg_blocking(self):
        buf = getattr(self, "_buf", b"")
        while True:
            if len(buf) >= HDR.size:
                typ, n = HDR.unpack_from(buf)
                if len(buf) >= HDR.size + n:
                    self._buf = buf[HDR.size + n:]
                    return typ, buf[HDR.size:HDR.size + n]
            chunk = self.sock.recv(65536)
            if not chunk:
                raise RuntimeError("Radxa 중계 프로그램이 연결을 닫았습니다 (다른 GUI가 연결 중일 수 있음)")
            buf += chunk

    # ------------------------------------------------------------------ 주고받기
    def _reader(self):
        buf = self._buf
        try:
            while not self._stop.is_set():
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= HDR.size:
                    typ, n = HDR.unpack_from(buf)
                    if len(buf) < HDR.size + n:
                        break
                    payload = buf[HDR.size:HDR.size + n]
                    buf = buf[HDR.size + n:]
                    if typ == T_FRAME and n == FRAME.size:
                        bus, flags, dlc, can_id, data = FRAME.unpack(payload)
                        if bus < len(self.queues):
                            rtr = bool(flags & F_RTR)
                            self.queues[bus].put(can.Message(
                                arbitration_id=can_id, is_extended_id=bool(flags & F_EXT), is_remote_frame=rtr,
                                dlc=dlc, data=None if rtr else data[:dlc], timestamp=time.time(), channel=bus))
                    elif typ == T_PONG and n == PING.size:
                        t_sent, _ = PING.unpack(payload)
                        rtt = (time.perf_counter() - t_sent) * 1000.0
                        self.rtt_ms = rtt if self.rtt_ms is None else 0.8 * self.rtt_ms + 0.2 * rtt
                    elif typ == T_LOG:
                        text = payload.decode(errors="replace")
                        self.logs.append(text)
                        self._log(text if text.startswith("⚠") else f"[Radxa] {text}")
        except OSError as e:
            self._mark_dead(f"Radxa 연결 끊김: {e}")
            return
        self._mark_dead("Radxa 연결이 닫혔습니다")

    def _pinger(self):
        while not self._stop.wait(PING_S):
            self._seq += 1
            try:
                self._send(T_PING, PING.pack(time.perf_counter(), self._seq))
            except OSError:
                return

    def _send(self, typ, payload):
        if self.dead:
            raise can.CanOperationError(self.dead)
        data = HDR.pack(typ, len(payload)) + payload
        with self._wlock:
            self.sock.sendall(data)

    def send_frame(self, bus, msg):
        flags = (F_EXT if msg.is_extended_id else 0) | (F_RTR if msg.is_remote_frame else 0)
        data = b"" if msg.is_remote_frame else bytes(msg.data)
        try:
            self._send(T_FRAME, FRAME.pack(bus, flags, msg.dlc, msg.arbitration_id, data.ljust(8, b"\x00")))
        except OSError as e:
            self._mark_dead(f"Radxa 송신 실패: {e}")
            raise can.CanOperationError(str(e)) from e

    def _mark_dead(self, why):
        if self.dead is None and not self._stop.is_set():
            self.dead = why
            self._log(f"⚠ {why}")

    # ------------------------------------------------------------------ 버스 / 닫기
    def buses(self):
        return [RadxaBus(self, i, b.get("name", f"bus{i}")) for i, b in enumerate(self.info.get("buses", []))]

    def release(self, idx):
        self._released.add(idx)
        if len(self._released) >= len(self.queues):
            self.close()

    def close(self):
        self._stop.set()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        p = self.proc
        if p is not None:
            try:
                p.stdin.close()             # 원격: 표준 입력 닫힘 → IDLE 보내고 끝냄
            except OSError:
                pass
            try:
                p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                p.terminate()
                try:
                    p.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    p.kill()
            self.proc = None


class RadxaBus(can.BusABC):
    """Radxa 중계의 버스 하나 (CanLink가 쓰는 python-can Bus와 같은 모양)."""

    def __init__(self, conn, idx, name):
        self._conn = conn
        self._idx = idx
        self._q = conn.queues[idx]
        self.channel_info = f"Radxa:{name}"
        super().__init__(channel=f"radxa:{name}")

    def send(self, msg, timeout=None):
        self._conn.send_frame(self._idx, msg)

    def _recv_internal(self, timeout):
        if self._conn.dead:
            raise can.CanOperationError(self._conn.dead)
        try:
            if timeout is None:
                while True:
                    try:
                        return self._q.get(timeout=0.2), False
                    except queue.Empty:
                        if self._conn.dead:
                            raise can.CanOperationError(self._conn.dead) from None
            if timeout <= 0:
                return self._q.get_nowait(), False
            return self._q.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def shutdown(self):
        super().shutdown()
        self._conn.release(self._idx)


if __name__ == "__main__":       # 간단 점검: python -m motorgui.radxa_link radxa
    c = RadxaConnection(sys.argv[1] if len(sys.argv) > 1 else "radxa", 500000, log=print,
                        extra_args=" ".join(sys.argv[2:])).start()
    print("info", c.info)
    time.sleep(1.5)
    print("rtt ms", c.rtt_ms)
    c.close()
