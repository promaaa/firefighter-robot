#!/usr/bin/env python3
"""Radxa에서 IMU를 읽어 몸통 자세(쿼터니언)와 각속도를 내보낸다.

센서 세 가지를 같은 모양으로 낸다 (몸통 좌표: x 앞, y 왼쪽, z 위. 학습 모델과 같음):
  --sensor bmi088  Seeed Grove 6-Axis (BMI088, 수업 BOM), I2C. 센서는 가속도·자이로만 내므로 자세는 여기서 계산한다.
                   계산은 학습 레포(SOBANG_Locomotion)의 bmi088.py를 쓴다 (같은 폴더의 사본, 커밋 db6d83a에서
                   소프트 리셋 쓰기의 NACK 오류를 무시하도록 한 곳만 고침): Mahony 필터, 400 Hz 목표(I2C 100 kHz에서는
                   약 300 Hz), 처음 2 s 가만히 두고 자이로 바이어스·수평을 잡음.
                   필요: numpy, smbus2 (sudo apt install python3-numpy python3-smbus2).
  --sensor im10a   Hiwonder IM10A (팀 전장 설계), UART. 센서가 계산한 자세(쿼터니언 0x59, 없으면 오일러각 0x53)와
                   자이로(0x52)를 읽어 몸통 좌표로 돌리기만 한다. 표준 라이브러리만 쓴다.
                   --setup: 한 번만. 팀 레포 legs/imu.py --setup과 같은 설정(자이로 + 쿼터니언, 200 Hz, 230400 baud)을
                   센서에 저장한다.
  --sensor fake    장치 없이 시험: roll·pitch가 천천히 흔들리는 가짜 값.

--axes: 몸통 x, y, z 방향에 놓인 IMU 축 (보드에 찍힌 X, Y, Z). 예: 보드를 z축으로 90° 돌려 달았으면 +y -x +z.
축 점검 (로봇을 똑바로 세우고 시작, BMI088은 처음 2 s 가만히):
  똑바로              gravity ≈ (0, 0, -1)
  앞으로 숙임         gravity x > 0, 숙이는 동안 gyro y > 0
  오른쪽으로 기울임   gravity y < 0, 기울이는 동안 gyro x > 0
  왼쪽으로 돎         gyro z > 0
yaw는 지자기 센서를 쓰지 않으면 시간이 지나며 틀어진다. 보행 정책은 중력 방향(roll·pitch)과 각속도만 쓴다.

실행 예 (Radxa에서):
  python3 imu_reader.py --sensor bmi088 --bus 3 --axes +x +y +z      # 사람이 읽는 출력
  python3 imu_reader.py --sensor im10a --port /dev/ttyS4
  python3 imu_reader.py --sensor im10a --port /dev/ttyS4 --setup     # IM10A 처음 한 번
MotorTestGUI의 CAN 중계(can_bridge.py)는 --json으로 실행해 한 줄에 JSON 하나씩 받는다:
  {"type": "info", "sensor": ..., "detail": ...}                         처음 한 번
  {"t": 초, "state": "calibrating"|"ok"|"stale", "q": [w, x, y, z], "w": [rad/s x3], "a": [m/s² x3] 또는 null,
   "hz": 센서 갱신 빈도}
  {"type": "error", "msg": ...}                                         시작 실패 (끝 코드 2)
"""
import argparse
import json
import math
import os
import select
import struct
import sys
import threading
import time

GRAVITY = 9.80665
HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------- 회전 (표준 라이브러리만)
def axes_matrix(axes):
    """axes[i] = 몸통 축 i 방향의 IMU 축 ('+x', '-y' 등) → v_body = M v_imu 인 3x3 (bmi088.axes_to_matrix와 같음)."""
    m = [[0.0] * 3 for _ in range(3)]
    for row, name in enumerate(axes):
        name = name.strip()
        sign = -1.0 if name.startswith("-") else 1.0
        axis = name.lstrip("+-")
        if axis not in ("x", "y", "z"):
            raise ValueError(f"축 이름이 이상합니다: {name!r} (예: +x, -y)")
        m[row]["xyz".index(axis)] = sign
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if abs(det - 1.0) > 1e-6:
        raise ValueError(f"축 {list(axes)}은 오른손 좌표가 아닙니다 (한 축이 겹치거나 부호 하나를 바꿔야 함)")
    return m


def mat_vec(m, v):
    return [m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def quat_to_mat(q):
    """(w, x, y, z), 몸체 → 세계."""
    w, x, y, z = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def mat_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = [(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s]
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = [(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s]
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = [(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s]
    if q[0] < 0.0:
        q = [-c for c in q]
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q]


def euler_to_mat(roll, pitch, yaw):
    """Z-Y-X (yaw, pitch, roll) [rad] → 몸체 → 세계."""
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def roll_pitch(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def gravity_body(q):
    """세계의 중력 방향 (0, 0, -1)을 몸통 좌표로 (똑바로 서면 (0, 0, -1))."""
    r = quat_to_mat(q)
    return [-r[2][0], -r[2][1], -r[2][2]]


# ---------------------------------------------------------------------- 가짜 IMU
class FakeSource:
    name = "fake"

    def __init__(self, calibration_s=2.0):
        self.t0 = time.monotonic()
        self.calibration_s = calibration_s
        self.detail = "가짜 값: roll ±6° (5 s 주기), pitch ±4° (7.7 s 주기)"
        self.error = None

    def start(self):
        self.t0 = time.monotonic()

    @staticmethod
    def angles(t):
        w1, w2 = 2 * math.pi * 0.20, 2 * math.pi * 0.13
        roll, pitch = math.radians(6.0) * math.sin(w1 * t), math.radians(4.0) * math.sin(w2 * t + 0.7)
        return roll, pitch, math.radians(6.0) * w1 * math.cos(w1 * t), math.radians(4.0) * w2 * math.cos(w2 * t + 0.7)

    def sample(self):
        t = time.monotonic() - self.t0
        if t < self.calibration_s:
            return {"state": "calibrating", "q": None, "w": None, "a": None, "hz": 0.0}
        roll, pitch, droll, dpitch = self.angles(t)
        q = mat_to_quat(euler_to_mat(roll, pitch, 0.0))
        g = gravity_body(q)
        # 몸통 각속도 ≈ (droll, dpitch) (작은 각). 가속도 = 비력 (서 있으면 +g 위)
        return {"state": "ok", "q": q, "w": [droll, dpitch * math.cos(roll), -dpitch * math.sin(roll)],
                "a": [-GRAVITY * c for c in g], "hz": 200.0}

    def stop(self):
        pass


# ---------------------------------------------------------------------- BMI088 (학습 레포 bmi088.py)
class Bmi088Source:
    name = "bmi088"

    def __init__(self, bus, axes, rate_hz=400.0, calibration_s=2.0):
        try:
            import numpy as np
        except ImportError:
            raise RuntimeError("numpy가 없습니다. Radxa에서: sudo apt install python3-numpy") from None
        try:
            import smbus2  # noqa: F401
        except ImportError:
            raise RuntimeError("smbus2가 없습니다. Radxa에서: sudo apt install python3-smbus2 "
                               "(없으면 python3 -m pip install --user --break-system-packages smbus2)") from None
        if HERE not in sys.path:
            sys.path.insert(0, HERE)
        import bmi088
        self.np, self.mod = np, bmi088
        dev = f"/dev/i2c-{bus}"
        if not os.path.exists(dev):
            raise RuntimeError(f"{dev}가 없습니다. 헤더 I2C를 켜야 합니다 (sudo rsetup → Overlays에서 I2C{bus} 켜고 "
                               "재부팅). 있는 버스: " + (", ".join(sorted(p for p in os.listdir("/dev")
                                                                        if p.startswith("i2c-"))) or "없음"))
        if not os.access(dev, os.R_OK | os.W_OK):
            raise RuntimeError(f"{dev}에 접근 권한이 없습니다: sudo usermod -aG i2c $USER 후 다시 로그인")
        probe = bmi088.Bmi088(bus)          # 여기서 찾아 두면 오류 글이 분명함 (실제 읽기는 Bmi088Imu 스레드)
        self.detail = (f"/dev/i2c-{bus}, 가속도 0x{probe.acc_address:02x}, 자이로 0x{probe.gyro_address:02x}, "
                       f"{rate_hz:g} Hz, Mahony kp 2.0 ki 0.02, 축 {' '.join(axes)}")
        probe.close()
        self.thread_error = None
        old_hook = threading.excepthook

        def hook(args):
            self.thread_error = f"{args.exc_type.__name__}: {args.exc_value}"
            old_hook(args)

        threading.excepthook = hook
        self.imu = bmi088.Bmi088Imu(bus=bus, axes=tuple(axes), rate_hz=rate_hz, calibration_s=calibration_s)
        self.error = None

    def start(self):
        self.imu.run_forever()

    def sample(self):
        imu, np = self.imu, self.np
        if imu.thread is not None and not imu.thread.is_alive():
            self.error = f"BMI088 읽기가 멈췄습니다 ({self.thread_error or 'I2C 오류'})"
            return None
        if not imu.is_calibrated.is_set():
            return {"state": "calibrating", "q": None, "w": None, "a": None, "hz": 0.0}
        q = imu.quaternion.astype(np.float64)
        q = (q / np.linalg.norm(q)).tolist()
        return {"state": "ok", "q": q, "w": np.deg2rad(imu.angular_velocity.astype(np.float64)).tolist(),
                "a": imu.acceleration.astype(np.float64).tolist(), "hz": float(imu.loop_rate)}

    def stop(self):
        self.imu.stop()


# ---------------------------------------------------------------------- IM10A (WitMotion 표준 프로토콜)
ACC, GYRO, ANGLE, QUAT = 0x51, 0x52, 0x53, 0x59
BAUDS = (230400, 9600, 115200, 460800, 921600)


class Tty:
    def __init__(self, path, baud):
        import fcntl
        import termios
        import tty
        self.path = path
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)      # 다른 프로그램과 동시에 열지 않음
        except OSError:
            pass
        tty.setraw(self.fd)
        self.set_baud(baud)

    def set_baud(self, baud):
        import termios
        speed = getattr(termios, f"B{int(baud)}", None)
        if speed is None:
            raise ValueError(f"지원하지 않는 baud: {baud}")
        a = termios.tcgetattr(self.fd)
        a[2] = (a[2] | termios.CLOCAL | termios.CREAD) & ~(termios.CSTOPB | getattr(termios, "CRTSCTS", 0))
        a[4] = a[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.baud = int(baud)

    def read(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def write(self, data):
        while data:
            try:
                n = os.write(self.fd, data)
                data = data[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.1)

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


def parse_frames(buf):
    """WitMotion 프레임: 0x55, 종류, int16 x4 (작은 끝), 합계 (앞 10바이트 합의 하위 바이트).
    → ([(종류, (v0, v1, v2, v3))...], 남은 바이트)."""
    out, i = [], 0
    while len(buf) - i >= 11:
        if buf[i] != 0x55 or sum(buf[i:i + 10]) & 0xFF != buf[i + 10]:
            i += 1
            continue
        out.append((buf[i + 1], struct.unpack_from("<4h", buf, i + 2)))
        i += 11
    return out, buf[i:]


def find_baud(ser, first, wait_s=0.6):
    """first부터 몇 가지 baud를 돌려 보며 올바른 프레임이 들어오는 것을 찾는다."""
    for baud in [int(first)] + [b for b in BAUDS if b != int(first)]:
        ser.set_baud(baud)
        buf, n, end = b"", 0, time.monotonic() + wait_s
        while time.monotonic() < end:
            frames, buf = parse_frames(buf + ser.read(0.05))
            n += len(frames)
        if n >= 3:
            return baud
    return None


class Im10aSource:
    name = "im10a"

    def __init__(self, port, baud, axes):
        self.m = axes_matrix(axes)
        self.mt = transpose(self.m)
        self.port, self.want_baud, self.axes = port, int(baud), list(axes)
        if not os.path.exists(port):
            raise RuntimeError(f"{port}가 없습니다. 헤더 UART를 켜야 합니다 (UART4: sudo rsetup → Overlays에서 "
                               "rk3568-uart4-m1 켜고 재부팅). 팀 배선: IMU TX → 16번 핀, IMU RX → 18번 핀")
        if not os.access(port, os.R_OK | os.W_OK):
            raise RuntimeError(f"{port}에 접근 권한이 없습니다: sudo usermod -aG dialout $USER 후 다시 로그인")
        self.ser = Tty(port, baud)
        self.baud = find_baud(self.ser, baud)
        if self.baud is None:
            self.ser.close()
            raise RuntimeError(f"{port}에서 IM10A 응답이 없습니다 ({', '.join(map(str, BAUDS))} baud 모두). "
                               "배선(IMU TX → Radxa RX), 3.3 V 전원, 포트 이름을 확인하세요")
        note = "" if self.baud == self.want_baud else f" (설정 {self.want_baud}와 다름"
        if self.baud == 9600:
            note += ", 출고 설정 → --setup으로 200 Hz·230400 baud 권장"
        if note:
            note += ")"
        self.detail = f"{port}, {self.baud} baud{note}, 축 {' '.join(axes)}"
        self.lock = threading.Lock()
        self.last = {}            # 종류 → (값, 받은 시각)
        self.rate = 0.0
        self.error = None
        self.stop_ev = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True, name="im10a")
        self.thread.start()

    def _run(self):
        buf, count, t_count = b"", 0, time.monotonic()
        try:
            while not self.stop_ev.is_set():
                frames, buf = parse_frames(buf + self.ser.read(0.05))
                now = time.monotonic()
                with self.lock:
                    for typ, v in frames:
                        self.last[typ] = (v, now)
                        if typ == QUAT or (typ == ANGLE and QUAT not in self.last):
                            count += 1
                if now - t_count >= 1.0:
                    self.rate, count, t_count = count / (now - t_count), 0, now
        except OSError as e:
            self.error = f"IM10A 읽기 중단: {e}"

    def sample(self):
        if self.error:
            return None
        now = time.monotonic()
        with self.lock:
            last = dict(self.last)
        quat, angle = last.get(QUAT), last.get(ANGLE)
        att = quat if quat is not None and now - quat[1] < 0.3 else angle if angle is not None and now - angle[1] < 0.3 else None
        if att is None:
            stale = bool(last)
            return {"state": "stale" if stale else "starting", "q": None, "w": None, "a": None, "hz": self.rate}
        if att is quat:
            qi = [c / 32768.0 for c in quat[0]]
            n = math.sqrt(sum(c * c for c in qi)) or 1.0
            r_wi = quat_to_mat([c / n for c in qi])
        else:
            roll, pitch, yaw = (math.radians(c / 32768.0 * 180.0) for c in angle[0][:3])
            r_wi = euler_to_mat(roll, pitch, yaw)
        q = mat_to_quat(mat_mul(r_wi, self.mt))             # 몸통 → 세계 = (IMU → 세계)(몸통 → IMU)
        w = a = None
        if GYRO in last and now - last[GYRO][1] < 0.3:
            w = mat_vec(self.m, [math.radians(c / 32768.0 * 2000.0) for c in last[GYRO][0][:3]])
        if ACC in last and now - last[ACC][1] < 0.3:
            a = mat_vec(self.m, [c / 32768.0 * 16.0 * GRAVITY for c in last[ACC][0][:3]])
        return {"state": "ok", "q": q, "w": w, "a": a, "hz": self.rate}

    def stop(self):
        self.stop_ev.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.ser.close()


def im10a_setup(port, baud_now=None):
    """팀 레포 legs/imu.py setup()과 같은 순서: 잠금 풀기 → 출력 내용(자이로 + 쿼터니언) → 200 Hz → 저장 →
    230400 baud → (새 baud에서) 저장."""
    ser = Tty(port, baud_now or 9600)
    try:
        found = find_baud(ser, baud_now or 9600)
        if found is None:
            raise RuntimeError(f"{port}에서 IM10A 응답이 없습니다 (배선·전원 확인)")
        print(f"IM10A 응답: {found} baud", flush=True)

        def write(reg, value):
            ser.write(struct.pack("<BBBH", 0xFF, 0xAA, reg, value))
            time.sleep(0.1)

        write(0x69, 0xB588)                  # 잠금 풀기: FF AA 69 88 B5
        write(0x02, 1 << 2 | 1 << 9)         # 출력: 각속도(비트 2), 쿼터니언(비트 9)
        write(0x03, 0x0B)                    # 200 Hz
        write(0x00, 0x0000)                  # 저장
        write(0x69, 0xB588)
        write(0x04, 0x07)                    # 230400 baud (센서가 바로 바꿈)
        ser.set_baud(230400)
        write(0x00, 0x0000)                  # 새 baud에서 한 번 더 저장 (BHL robot/imu.py와 같음)
        ok = find_baud(ser, 230400) == 230400
    finally:
        ser.close()
    return ok


# ---------------------------------------------------------------------- 실행
AXIS_TOKEN = ("+x", "-x", "+y", "-y", "+z", "-z", "x", "y", "z")


def join_axes_argv(argv):
    """'--axes +y -x +z'를 '--axes=+y,-x,+z'로 바꾼다. argparse는 '-x'를 옵션 이름으로 읽기 때문."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] == "--axes":
            vals, j = [], i + 1
            while j < len(argv) and len(vals) < 3 and argv[j] in AXIS_TOKEN:
                vals.append(argv[j])
                j += 1
            out.append("--axes=" + ",".join(vals))
            i = j
        else:
            out.append(argv[i])
            i += 1
    return out


def open_source(a):
    axes = a.axes.replace(",", " ").split()
    if len(axes) != 3:
        raise ValueError(f"--axes에는 축 세 개가 필요합니다 (받은 값: {axes})")
    axes_matrix(axes)
    if a.sensor == "fake":
        return FakeSource(a.calibration)
    if a.sensor == "bmi088":
        return Bmi088Source(a.bus, axes, a.rate, a.calibration)
    return Im10aSource(a.port, a.baud, axes)


def emit(obj):
    sys.stdout.write(json.dumps(obj, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def rnd(v, n):
    return None if v is None else [round(float(c), n) for c in v]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sensor", choices=("bmi088", "im10a", "fake"), required=True)
    ap.add_argument("--bus", type=int, default=3, help="BMI088: I2C 버스 번호 (/dev/i2c-N)")
    ap.add_argument("--port", default="/dev/ttyS4", help="IM10A: 직렬 포트 (팀 설계: UART4 = /dev/ttyS4)")
    ap.add_argument("--baud", type=int, default=230400, help="IM10A: 먼저 시도할 baud (안 되면 다른 값도 찾아봄)")
    ap.add_argument("--axes", default="+x,+y,+z",
                    help="몸통 x, y, z 방향의 IMU 축. '--axes +y -x +z' 또는 '--axes=+y,-x,+z'")
    ap.add_argument("--rate", type=float, default=400.0, help="BMI088: 읽기 빈도 [Hz]")
    ap.add_argument("--calibration", type=float, default=2.0, help="BMI088: 처음 가만히 두는 시간 [s]")
    ap.add_argument("--out-hz", type=float, default=50.0, help="--json 출력 빈도 [Hz]")
    ap.add_argument("--json", action="store_true", help="한 줄에 JSON 하나 (CAN 중계용)")
    ap.add_argument("--setup", action="store_true", help="IM10A: 200 Hz·230400 baud·자이로+쿼터니언으로 한 번 설정")
    ap.add_argument("--duration", type=float, default=0.0, help="이 시간 [s] 뒤 끝냄 (0 = 계속)")
    a = ap.parse_args(join_axes_argv(sys.argv[1:]))

    if a.setup:
        if a.sensor != "im10a":
            ap.error("--setup은 --sensor im10a에서만 씁니다")
        ok = im10a_setup(a.port)
        print("설정 완료: 200 Hz, 230400 baud, 자이로 + 쿼터니언" if ok else
              "설정을 보냈지만 230400 baud에서 응답을 확인하지 못했습니다. 센서 전원을 껐다 켜고 다시 확인하세요")
        return 0 if ok else 1

    try:
        src = open_source(a)
        src.start()
    except Exception as e:  # noqa: BLE001 - 시작 실패 이유를 그대로 알림
        if a.json:
            emit({"type": "error", "msg": str(e)})
        else:
            print(f"오류: {e}", file=sys.stderr)
        return 2
    if a.json:
        emit({"type": "info", "sensor": src.name, "detail": src.detail, "pid": os.getpid()})
    else:
        print(f"{src.name}: {src.detail}")
        if src.name == "bmi088":
            print(f"처음 {a.calibration:g} s는 로봇을 똑바로 세우고 가만히 두세요 (자이로 바이어스·수평)")
    period = 1.0 / max(1.0, a.out_hz if a.json else 10.0)
    t_end = time.monotonic() + a.duration if a.duration > 0 else None
    next_t = time.monotonic()
    try:
        while t_end is None or time.monotonic() < t_end:
            s = src.sample()
            if s is None:
                msg = src.error or "센서 읽기 중단"
                if a.json:
                    emit({"type": "error", "msg": msg})
                else:
                    print(f"\n오류: {msg}", file=sys.stderr)
                return 3
            if a.json:
                emit({"t": round(time.monotonic(), 4), "state": s["state"], "q": rnd(s["q"], 5),
                      "w": rnd(s["w"], 4), "a": rnd(s["a"], 3), "hz": round(s["hz"], 1)})
            elif s["q"] is None:
                print(f"{s['state']} …", end="\r", flush=True)
            else:
                g = gravity_body(s["q"])
                roll, pitch = roll_pitch(s["q"])
                w = s["w"] or [math.nan] * 3
                amag = math.sqrt(sum(c * c for c in s["a"])) / GRAVITY if s["a"] else math.nan
                print(f"roll {math.degrees(roll):+6.1f}° pitch {math.degrees(pitch):+6.1f}° | gravity "
                      f"{g[0]:+.3f} {g[1]:+.3f} {g[2]:+.3f} | gyro °/s {math.degrees(w[0]):+7.2f} "
                      f"{math.degrees(w[1]):+7.2f} {math.degrees(w[2]):+7.2f} | |a| {amag:5.3f} g | "
                      f"{s['hz']:5.1f} Hz   ", end="\r", flush=True)
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
            if next_t < time.monotonic() - 1.0:
                next_t = time.monotonic()
    except (KeyboardInterrupt, BrokenPipeError):
        pass
    finally:
        src.stop()
        if not a.json:
            print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
