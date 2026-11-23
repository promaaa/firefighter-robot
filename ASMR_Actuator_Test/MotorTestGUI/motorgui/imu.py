"""IMU 자세: 받아 둔 값(ImuFeed), 가상 IMU, 자세 계산, 3D 그림 기울이기.

자세 q = (w, x, y, z), 몸통 → 세계. 몸통 좌표는 x 앞, y 왼쪽, z 위 (학습 모델과 같음).
Radxa에서는 radxa/imu_reader.py가 계산·변환해서 보내고 (BMI088: bmi088.py의 Mahony 필터, IM10A: 센서가 낸 자세),
GUI는 받은 값을 그대로 쓴다. 그림에는 roll·pitch만 반영한다 (yaw는 지자기 없이는 시간이 지나며 틀어짐).
"""
import math
import threading
import time
from collections import deque

import numpy as np

SENSOR_SHORT = {"bmi088": "BMI088", "im10a": "IM10A", "fake": "가상", "virtual": "가상", "none": "없음"}
AXIS_CHOICES = ["+x", "-x", "+y", "-y", "+z", "-z"]
STALE_S = 0.5                 # 이보다 오래된 값은 그림에 쓰지 않음
HIST_LEN = 6000               # roll·pitch 기록 (50 Hz면 2분)


# ---------------------------------------------------------------------- 자세 계산
def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def roll_pitch_yaw(q):
    """Z-Y-X 오일러각 [rad]."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def tilt_matrix(roll, pitch):
    """yaw를 뺀 자세: Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def gravity_body(q):
    """세계 중력 방향 (0, 0, -1)을 몸통 좌표로. 정책 관측 6–8과 같은 값 (똑바로 서면 (0, 0, -1))."""
    return -quat_to_mat(q)[2]


def axes_valid(axes):
    """몸통 x, y, z 방향의 IMU 축 세 개가 오른손 회전인지."""
    m = np.zeros((3, 3))
    try:
        for row, name in enumerate(axes):
            m[row, "xyz".index(name.lstrip("+-"))] = -1.0 if name.startswith("-") else 1.0
    except (ValueError, IndexError):
        return False
    return abs(np.linalg.det(m) - 1.0) < 1e-6


def tilt_data(model, data, rot, pivot):
    """mj_kinematics를 마친 data의 위치·방향을 pivot을 중심으로 rot만큼 돌린다 (world 몸체에 붙은 바닥 등은 그대로).

    그림 전용: 물리 계산에는 쓰지 않는다. mjv_updateScene이 읽는 값(몸체·기하·사이트·관절축·조명)만 바꾼다.
    """
    def pos(arr, mask):
        arr[mask] = (arr[mask] - pivot) @ rot.T + pivot

    def mat(arr, mask):
        arr[mask] = (rot @ arr[mask].reshape(-1, 3, 3)).reshape(-1, 9)

    bodies = np.arange(model.nbody) != 0
    pos(data.xpos, bodies)
    mat(data.xmat, bodies)
    pos(data.xipos, bodies)
    mat(data.ximat, bodies)
    if model.ngeom:
        g = model.geom_bodyid != 0
        pos(data.geom_xpos, g)
        mat(data.geom_xmat, g)
    if model.nsite:
        s = model.site_bodyid != 0
        pos(data.site_xpos, s)
        mat(data.site_xmat, s)
    if model.njnt:
        data.xanchor[:] = (data.xanchor - pivot) @ rot.T + pivot
        data.xaxis[:] = data.xaxis @ rot.T
    if model.nlight:
        li = model.light_bodyid != 0
        pos(data.light_xpos, li)
        data.light_xdir[li] = data.light_xdir[li] @ rot.T


# ---------------------------------------------------------------------- 받아 둔 값
class ImuFeed:
    """IMU 표본을 받아 두는 곳. Radxa 중계(radxa_link)나 가상 IMU가 채우고, GUI가 읽는다. 스레드 안전."""

    def __init__(self):
        self.lock = threading.Lock()
        self.hist = deque(maxlen=HIST_LEN)
        self.reset()

    def reset(self, state="off", text=""):
        with self.lock:
            self.state = state       # off | starting | calibrating | ok | stale | error
            self.text = text
            self.sensor = ""
            self.detail = ""
            self.q = self.w = self.a = None
            self.hz = 0.0
            self.t_rx = None
            self.n = 0
            self.hist.clear()

    def set_state(self, state, text="", sensor=None):
        with self.lock:
            self.state, self.text = state, text
            if sensor is not None:
                self.sensor = sensor

    def on_message(self, d, t_rx=None):
        """imu_reader.py JSON 한 줄 (dict)."""
        t_rx = time.perf_counter() if t_rx is None else t_rx
        kind = d.get("type")
        if kind == "info":
            with self.lock:
                self.sensor, self.detail = d.get("sensor", ""), d.get("detail", "")
                self.state, self.text = "starting", ""
            return
        if kind == "error":
            self.set_state("error", d.get("msg") or "오류")
            return
        if kind == "exit":            # 바로 앞에 이유(error)를 받았으면 그 글을 남긴다
            with self.lock:
                if self.state == "error" and self.text:
                    return
            self.set_state("error", d.get("msg") or f"IMU 프로그램이 끝났습니다 (코드 {d.get('code')})")
            return
        if kind == "stopped":
            self.set_state("off", "IMU 끔")
            return
        self.push(d.get("q"), d.get("w"), d.get("a"), float(d.get("hz") or 0.0), d.get("state", "ok"), t_rx)

    def push(self, q, w, a, hz, state, t_rx=None):
        t_rx = time.perf_counter() if t_rx is None else t_rx
        with self.lock:
            self.state, self.hz = state, hz
            if state in ("ok", "calibrating"):
                self.text = ""
            if q is not None:
                q = np.asarray(q, float)
                n = np.linalg.norm(q)
                if n > 0.5:
                    self.q = q / n
                    self.w = None if w is None else np.asarray(w, float)
                    self.a = None if a is None else np.asarray(a, float)
                    self.t_rx = t_rx
                    self.n += 1
                    r, p, _ = roll_pitch_yaw(self.q)
                    self.hist.append((t_rx, math.degrees(r), math.degrees(p)))

    def snapshot(self):
        with self.lock:
            age = None if self.t_rx is None else time.perf_counter() - self.t_rx
            return {"state": self.state, "text": self.text, "sensor": self.sensor, "detail": self.detail,
                    "q": None if self.q is None else self.q.copy(), "w": self.w, "a": self.a, "hz": self.hz,
                    "age": age, "n": self.n}

    def fresh_q(self):
        """그림에 쓸 수 있는 최신 자세 (없거나 오래됐으면 None)."""
        with self.lock:
            if self.q is None or self.t_rx is None or time.perf_counter() - self.t_rx > STALE_S:
                return None
            return self.q.copy()

    def series(self, window):
        """(지금부터 몇 초 전, roll°, pitch°) 배열."""
        now = time.perf_counter()
        with self.lock:
            rows = [r for r in self.hist if now - r[0] <= window]
        if not rows:
            return np.zeros(0), np.zeros(0), np.zeros(0)
        a = np.asarray(rows)
        return a[:, 0] - now, a[:, 1], a[:, 2]


# ---------------------------------------------------------------------- 가상 IMU (GUI '가상 로봇'용)
class FakeImu:
    """radxa/imu_reader.py --sensor fake와 같은 움직임 (roll ±6°, pitch ±4°)을 50 Hz로 feed에 넣는다."""

    def __init__(self, feed, calibration_s=2.0, hz=50.0):
        self.feed, self.calibration_s, self.period = feed, calibration_s, 1.0 / hz
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self.feed.reset("starting")
        self.feed.set_state("starting", "", sensor="virtual")
        with self.feed.lock:
            self.feed.detail = "가상 로봇의 가짜 IMU: roll ±6° (5 s 주기), pitch ±4° (7.7 s 주기)"
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="fake-imu")
        self._thread.start()

    def _run(self):
        t0 = time.perf_counter()
        while not self._stop.wait(self.period):
            t = time.perf_counter() - t0
            if t < self.calibration_s:
                self.feed.push(None, None, None, 0.0, "calibrating")
                continue
            w1, w2 = 2 * math.pi * 0.20, 2 * math.pi * 0.13
            roll, pitch = math.radians(6.0) * math.sin(w1 * t), math.radians(4.0) * math.sin(w2 * t + 0.7)
            dr, dp = math.radians(6.0) * w1 * math.cos(w1 * t), math.radians(4.0) * w2 * math.cos(w2 * t + 0.7)
            r = tilt_matrix(roll, pitch)
            q = _mat_to_quat(r)
            g = -r[2]
            self.feed.push(q, [dr, dp * math.cos(roll), -dp * math.sin(roll)], -9.80665 * g, 200.0, "ok")

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self.feed.reset("off")


def _mat_to_quat(m):
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0
    if w < 1e-6:
        raise ValueError("작은 기울기용")
    return np.array([w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)])
