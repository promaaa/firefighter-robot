"""보행 테스트 (규칙 기반): 하네스에 매단 로봇이 두 다리로 걷는 듯한 동작을 한다.

목적은 "보행 동작을 할 하드웨어 세팅이 끝났음"을 보이는 것이다. 균형이나 지면 반력은 다루지 않는다.

동작 순서 (각 관절의 목표는 commander의 GaitTarget 하나로, 실물 스트리밍·시뮬·보호 정지가 기존과 같은 길로 읽는다):
  1. 토크 켜기: 12개 관절을 지금 자세 그대로 closed loop
  2. 준비 자세로: 무릎을 굽혀 엉덩이 아래에 발을 둔 자세로 최소 저크 이동
  3. 걷기 ("저벅저벅"): 발목 위치 궤적을 정하고 2링크 역기구학으로 고관절·무릎 각도를 구한다.
     디딤 구간에는 발이 땅에 붙은 것처럼 일정한 높이에서 뒤로 미끄러지고 (발바닥 평평),
     흔듦 구간에는 발을 들어 앞으로 옮겨 내려놓는다. 발목은 뒤꿈치 착지 → 평평 → 뒤꿈치 들기 → 발끝 밀기.
     두 다리는 반 주기 차이. 진폭(보폭·발 높이·좌우·발목)은 처음 몇 주기에 걸쳐 0에서 키움
  4. 멈추기: 진폭을 한 주기에 걸쳐 0으로 줄인 뒤 처음 자세로 돌아감 (설정에 따라 그 자리에서 모두 힘 풀기)

관절 목표는 모두 시뮬(MJCF) 관절 기준 각도다. 실물로는 CAN 링크가 설정 파일의 방향(direction)·영점으로 바꿔 보낸다.

비상: '모두 힘 풀기'(Esc)는 App.idle_all()로 바로 IDLE을 보내고 목표를 지운다. 보행 쪽은 그 사실만 기록한다.
보호 정지·응답 끊김·에러로 한 관절이라도 목표가 사라지면 보행을 중단하고 모든 관절 힘을 푼다.

매 실행을 logs/walk_YYYYMMDD_HHMMSS/에 자동 기록한다:
  meta.json (설정·영점·게인·연결), events.txt (단계, 경고, CAN 기록), samples.csv (관절별 목표·실측·시뮬·전류 50 Hz),
  summary.json (관절별 최대·RMS 추종 오차와 전류, 끝난 이유).
"""
import csv
import json
import math
import os
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import mujoco

from .commander import move_duration
from .config import driver_joint_gains
from . import protocol as P

LOG_DIR = Path(__file__).resolve().parents[1] / "logs"
TAIL_S = 2.0               # 끝나거나 중단된 뒤에도 이만큼 더 기록 (힘이 풀리는 모습)
ENABLE_TIMEOUT_S = 2.5     # 이 안에 12개가 closed loop가 안 되면 중단
STALE_S = 0.06             # 이보다 오래된 엔코더 값은 추종 오차 통계에서 뺌
ANKLE_ROLL_PER_HIP_ROLL = 0.707   # 고관절 롤 축이 45° 기울어 발바닥이 sin45만큼 기울어짐 → 발목 롤로 같은 부호만큼 되돌림
KNEE_MIN_DEG = 4.0         # 역기구학에서 다리를 이보다 더 펴지 않음 (특이점 피함)
KNEE_MAX_DEG = 120.0
BOB_M = 0.005              # 디딤 한가운데 다리를 조금 더 폄 (몸이 오르내리는 느낌) [m, 진폭 배율 곱함]
HEEL_STRIKE_DEG = 6.0      # 착지 직전 발끝 들기 [deg × 발목 쓰기]
PUSH_OFF_DEG = 12.0        # 디딤 끝 뒤꿈치 들기(발끝 밀기) [deg × 발목 쓰기]
SOFT_DEG = 4.0             # 관절 범위 끝 이만큼 안쪽부터 부드럽게 눌러 범위를 넘지 않게
SWING_DORSI_DEG = 12.0     # 흔듦 구간에 발목을 이보다 더 젖히지 않고 발끝이 처지게 둠 (발목 한계에 눌리지 않게)
WARN_DPS = 180.0           # 미리보기에서 관절 최고 속도가 이보다 크면 경고
MAX_DPS = 300.0            # 이보다 크면 시작·변경을 막음

# 해부학 방향(고관절 굽힘 = 다리 앞으로, 무릎 굽힘, 발목 = 발끝 들기 를 +)에서 MJCF 관절 각도로 가는 부호.
# 수업 Robot Model v1을 MuJoCo 순기구학으로 확인 (2026-11-17): +x = 앞, +y = 왼쪽.
#   왼 고관절 피치 +20° → 발 +x / 오른 −20°가 같은 쪽, 왼 무릎 −30° → 발 뒤로 / 오른 +30°,
#   왼 발목 +10° → 발끝 들림 / 오른 −10°. 고관절 롤은 두 다리 모두 +이면 발이 +y로 감.
ROLE_SIGN = {
    ("left", "hip_pitch"): +1, ("right", "hip_pitch"): -1,
    ("left", "knee_pitch"): -1, ("right", "knee_pitch"): +1,
    ("left", "ankle_pitch"): +1, ("right", "ankle_pitch"): -1,
}

STAGE_KO = {"enabling": "토크 켜는 중", "approach": "준비 자세로 이동", "ramp": "걷기 (진폭 키우는 중)",
            "walk": "걷는 중", "stopping": "멈추는 중 (진폭 줄임)", "return": "처음 자세로 돌아가는 중",
            "done": "끝", "aborted": "중단", "idle": "대기"}


@dataclass
class WalkParams:
    steps_per_min: float = 50.0    # 걸음 속도: 1분에 몇 걸음 (한 발 = 한 걸음). 한 주기(왼발·오른발) = 120 / 이 값 [s]
    stride_cm: float = 10.0        # 보폭: 디딤 동안 발이 엉덩이 아래에서 앞→뒤로 미끄러지는 거리
    lift_cm: float = 3.0           # 흔듦 구간에 발을 드는 높이
    crouch_deg: float = 25.0       # 기본 자세의 무릎 굽힘 (클수록 보폭·발 높이 여유가 큼)
    duty: float = 0.6              # 디딤 비율: 한 다리가 땅을 딛는 시간 / 주기 (0.5보다 크면 두 발이 같이 딛는 구간)
    ankle_use: float = 0.6         # 발목 쓰기: 0 = 발바닥 늘 평평, 1 = 뒤꿈치 착지·발끝 밀기 크게
    sway_deg: float = 3.0          # 좌우 흔들기 (고관절 롤 ±, 두 다리 같은 쪽, 디딤 다리 쪽으로)
    duration_s: float = 30.0       # 걷는 시간. 0이면 멈출 때까지
    scale: float = 1.0             # 진폭 배율: 보폭·발 높이·좌우·발목에 곱함 (걷는 중에 바꿀 수 있음)
    ramp_in_cycles: float = 2.0
    ramp_out_cycles: float = 1.0
    approach_dps: float = 20.0     # 준비 자세로 갈 때·돌아올 때 최고 속도
    end_relax: bool = True         # 끝나면 처음 자세로 돌아와 모두 힘 풀기 (끄면 처음 자세에서 토크 유지)

    @property
    def period_s(self):
        return 120.0 / max(self.steps_per_min, 1.0)


@dataclass
class LegGeom:
    """사지 치수 (0° 자세, 시상면). 각도는 아래 수직에서 앞쪽이 +."""
    L1: float          # 고관절 피치 축 → 무릎 축 [m]
    L2: float          # 무릎 축 → 발목 피치 축 [m]
    a1_0: float        # 0° 자세 허벅지 방향 [rad]
    a2_0: float        # 0° 자세 정강이 방향 [rad]

    def reach(self, knee_flex):
        """무릎 굽힘 knee_flex [rad]일 때 고관절 축–발목 축 거리."""
        g = knee_flex + (self.a1_0 - self.a2_0)
        return math.sqrt(self.L1 ** 2 + self.L2 ** 2 + 2.0 * self.L1 * self.L2 * math.cos(g))


def leg_geometry(model, infos):
    """MJCF 0° 자세에서 다리 치수를 읽는다 (왼다리 기준, 좌우 대칭). 실패하면 None."""
    try:
        d = mujoco.MjData(model)
        mujoco.mj_kinematics(model, d)
        a = {k: d.xanchor[infos[f"leg_left_{k}_joint"].jid].copy() for k in ("hip_pitch", "knee_pitch", "ankle_pitch")}
    except (KeyError, AttributeError, TypeError):
        return None
    th, sh = a["knee_pitch"] - a["hip_pitch"], a["ankle_pitch"] - a["knee_pitch"]
    return LegGeom(L1=float(math.hypot(th[0], th[2])), L2=float(math.hypot(sh[0], sh[2])),
                   a1_0=float(math.atan2(th[0], -th[2])), a2_0=float(math.atan2(sh[0], -sh[2])))


def leg_ik(g, x, z):
    """발목 위치 (고관절 피치 축 기준, x 앞 +, z 위 + [m]) -> (고관절 굽힘, 무릎 굽힘 [rad], 닿지 않아 자름 여부)."""
    D = math.hypot(x, z)
    d_max, d_min = g.reach(math.radians(KNEE_MIN_DEG)), g.reach(math.radians(KNEE_MAX_DEG))
    limited = D > d_max or D < d_min
    D = min(max(D, d_min), d_max)
    c = (D * D - g.L1 ** 2 - g.L2 ** 2) / (2.0 * g.L1 * g.L2)
    gam = math.acos(min(max(c, -1.0), 1.0))          # 허벅지와 정강이 방향 사이 각
    beta = math.atan2(g.L2 * math.sin(gam), g.L1 + g.L2 * math.cos(gam))
    a1 = math.atan2(x, -z) + beta                      # 정강이가 뒤로 접힌 만큼 허벅지는 앞으로
    return a1 - g.a1_0, gam - (g.a1_0 - g.a2_0), limited


def leg_fk(g, fh, fk):
    """leg_ik의 반대 (시험용): -> 발목 (x, z)."""
    a1 = g.a1_0 + fh
    a2 = g.a2_0 + fh - fk
    return g.L1 * math.sin(a1) + g.L2 * math.sin(a2), -g.L1 * math.cos(a1) - g.L2 * math.cos(a2)


def _foot_pitch(s, duty, amp_ankle):
    """한 다리 위상 s (0 = 착지)에서 발바닥 기울기 [deg, 발끝 들기 +]. 뒤꿈치 착지 → 평평 → 뒤꿈치 들기 → 발끝 밀기."""
    dd, pp = HEEL_STRIKE_DEG * amp_ankle, PUSH_OFF_DEG * amp_ankle
    if s < duty:
        u = s / duty
        if u < 0.15:
            return dd * (1.0 - _minjerk(u / 0.15))
        if u > 0.7:
            return -pp * _minjerk((u - 0.7) / 0.3)
        return 0.0
    u = (s - duty) / (1.0 - duty)
    return -pp + (pp + dd) * _minjerk(u / 0.5)


def foot_path(p, s, amp, period, h0):
    """한 다리 위상 s에서 발목 목표 (x, z) [m]. 디딤: 일정 높이에서 앞→뒤로 등속, 흔듦: 들어서 앞으로 (속도 이어짐)."""
    S, h, bob = amp * p.stride_cm / 100.0, amp * p.lift_cm / 100.0, amp * BOB_M
    duty = min(max(p.duty, 0.5), 0.85)
    if s < duty:
        u = s / duty
        return S / 2.0 - S * u, -h0 - bob * math.sin(math.pi * u) ** 2
    u = (s - duty) / (1.0 - duty)
    t_sw, v = (1.0 - duty) * period, -S / max(duty * period, 1e-6)
    h00, h10 = 2 * u ** 3 - 3 * u ** 2 + 1, u ** 3 - 2 * u ** 2 + u      # 끝 속도를 디딤 속도에 맞춘 3차 곡선
    h01, h11 = -2 * u ** 3 + 3 * u ** 2, u ** 3 - u ** 2
    x = h00 * (-S / 2.0) + h10 * t_sw * v + h01 * (S / 2.0) + h11 * t_sw * v
    return x, -h0 + h * math.sin(math.pi * u) ** 2


def leg_pose(p, g, s, amp, period, h0):
    """-> (고관절 굽힘, 무릎 굽힘, 발목 발끝 들기) [deg, 해부학 방향], 닿지 않아 자름 여부."""
    duty = min(max(p.duty, 0.5), 0.85)
    x, z = foot_path(p, s, amp, period, h0)
    fh, fk, limited = leg_ik(g, x, z)
    fh, fk = math.degrees(fh), math.degrees(fk)
    fa = _foot_pitch(s, duty, amp * p.ankle_use) - (fh - fk)    # 발바닥 기울기 = 고관절 − 무릎 + 발목
    if s >= duty:      # 흔듦: 무릎을 많이 굽힐 때 발바닥을 평평하게 두려면 발목 한계를 넘으므로 발끝이 처지게
        u = (s - duty) / (1.0 - duty)
        fa -= math.sin(math.pi * u) ** 2 * _softplus(fa - SWING_DORSI_DEG, 3.0)
    return fh, fk, fa, limited


def _softplus(x, k):
    """부드러운 max(x, 0)."""
    return x if x / k > 30.0 else k * math.log1p(math.exp(x / k))


def sway_deg(p, s_left, amp):
    """고관절 롤 [deg, MJCF: 두 다리 모두 +면 발이 +y]. 왼다리 디딤 한가운데에서 발이 −y (= 몸이 왼발 위로)."""
    duty = min(max(p.duty, 0.5), 0.85)
    return -amp * p.sway_deg * math.cos(2.0 * math.pi * (s_left - duty / 2.0))


def soft_clip(v, lo, hi, k=SOFT_DEG):
    """범위 [lo, hi] 끝 k 안쪽부터 부드럽게 눌러 범위를 넘지 않게 (기울기 연속)."""
    if hi - lo < 2 * k:
        return min(max(v, lo), hi)
    if v > hi - k:
        return hi - k + k * math.tanh((v - (hi - k)) / k)
    if v < lo + k:
        return lo + k - k * math.tanh(((lo + k) - v) / k)
    return v


def joint_role(name):
    """'leg_left_hip_pitch_joint' -> ('left', 'hip_pitch')."""
    parts = name.split("_")
    return parts[1], "_".join(parts[2:-1])


def _minjerk(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * s * (10.0 + s * (-15.0 + 6.0 * s))


def _smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def pose_all(p, g, h0, s_left, amp, period):
    """모든 관절 목표 [deg, MJCF 부호, 범위 자르기 전] {(쪽, 역할): 값}, 닿지 않아 자른 다리가 있는지.
    s_left: 왼다리 위상 (0 = 왼발 착지), 오른다리는 반 주기 뒤."""
    out = {}
    limited = False
    sw = sway_deg(p, s_left, amp)
    for side, s in (("left", s_left % 1.0), ("right", (s_left + 0.5) % 1.0)):
        fh, fk, fa, lim = leg_pose(p, g, s, amp, period, h0)
        limited = limited or lim
        out[(side, "hip_pitch")] = ROLE_SIGN[(side, "hip_pitch")] * fh
        out[(side, "knee_pitch")] = ROLE_SIGN[(side, "knee_pitch")] * fk
        out[(side, "ankle_pitch")] = ROLE_SIGN[(side, "ankle_pitch")] * fa
        out[(side, "hip_roll")] = sw
        out[(side, "ankle_roll")] = ANKLE_ROLL_PER_HIP_ROLL * sw
        out[(side, "hip_yaw")] = 0.0
    return out, limited


def preview(p, g, lim, roles, n=360):
    """설정 p(진폭 배율 포함)로 한 주기를 돌려 본 결과: 관절별 (최소, 최대, 최고 속도), 범위에 눌린 관절 등."""
    period = p.period_s
    h0 = g.reach(math.radians(p.crouch_deg))
    vals = {name: [] for name in roles}
    limited, clipped = False, set()
    for i in range(n + 1):
        pose, lim_flag = pose_all(p, g, h0, i / n, p.scale, period)
        limited = limited or lim_flag
        for name, role in roles.items():
            raw = pose[role]
            v = soft_clip(raw, *lim[name])
            if abs(v - raw) > 0.5:
                clipped.add(name)
            vals[name].append(v)
    dt = period / n
    joints = {name: (min(v), max(v), max(abs(v[i + 1] - v[i]) for i in range(n)) / dt) for name, v in vals.items()}
    peak = max((r[2], name) for name, r in joints.items())
    duty = min(max(p.duty, 0.5), 0.85)
    return {"joints": joints, "peak_vel": peak, "reach_limited": limited, "clipped": sorted(clipped),
            "period": period, "speed_mps": p.scale * p.stride_cm / 100.0 / (duty * period), "h0": h0}


class GaitPlan:
    """시간 t의 관절 목표. 바꿀 때는 새 Plan을 만들어 통째로 갈아 끼운다 (스레드 사이 잠금 없음)."""

    kind = "gait"

    def __init__(self, p, roles, lim, q_start, q_ready, g, h0, *, t_gait0, period, a0, a1, TA, phi0=0.0,
                 t_phi0=None, t_app0=None, T_app=0.0, t_ret0=None, T_ret=2.0, t_walk0=None):
        self.p, self.roles, self.lim = p, roles, lim
        self.g, self.h0 = g, h0
        self._cache = {}
        self.q_start, self.q_ready = q_start, q_ready
        self.t_gait0, self.period = t_gait0, period
        self.a0, self.a1, self.tA0, self.TA = a0, a1, t_gait0, TA
        self.phi0, self.t_phi0 = phi0, t_gait0 if t_phi0 is None else t_phi0
        self.t_app0, self.T_app = t_app0, T_app
        self.t_ret0, self.T_ret = t_ret0, T_ret
        self.t_walk0 = t_gait0 if t_walk0 is None else t_walk0     # 걷기 시작 시각 (걸은 시간·주기 수 계산)

    @property
    def t_end(self):
        return math.inf if self.t_ret0 is None else self.t_ret0 + self.T_ret

    def phase(self, t):
        return self.phi0 + 2.0 * math.pi * (t - self.t_phi0) / self.period

    def amp(self, t):
        if self.t_app0 is not None and t < self.t_gait0:
            return 0.0
        if self.TA <= 0.0:
            return self.a1
        return self.a0 + (self.a1 - self.a0) * _smooth((t - self.tA0) / self.TA)

    def stage(self, t):
        if self.t_app0 is not None and t < self.t_gait0:
            return "approach"
        if self.t_ret0 is not None:
            if t >= self.t_end:
                return "done"
            return "return" if t >= self.t_ret0 else "stopping"
        if self.a1 > self.a0 and t < self.tA0 + self.TA:
            return "ramp"
        return "walk"

    def _pose(self, t):
        """시각 t의 걷기 자세 (같은 t를 여러 관절이 읽으므로 몇 개만 기억)."""
        c = self._cache
        hit = c.get(t)
        if hit is not None:
            return hit
        s_left = (self.phase(t) / (2.0 * math.pi)) % 1.0
        pose, _ = pose_all(self.p, self.g, self.h0, s_left, self.amp(t), self.period)
        if len(c) > 16:
            c.clear()
        c[t] = pose
        return pose

    def q_deg(self, name, t):
        qs, qr = self.q_start[name], self.q_ready[name]
        # 준비 자세로 가는 길·돌아오는 길은 (실제 자세)와 (범위 안 준비 자세) 사이 보간이라 자르지 않는다
        if self.t_app0 is not None and t < self.t_gait0:
            return qs + (qr - qs) * _minjerk((t - self.t_app0) / max(self.T_app, 1e-6))
        if self.t_ret0 is not None and t >= self.t_ret0:
            return qr + (qs - qr) * _minjerk((t - self.t_ret0) / max(self.T_ret, 1e-6))
        lo, hi = self.lim[name]
        return soft_clip(self._pose(t)[self.roles[name]], lo, hi)

    def q(self, name, t):
        return math.radians(self.q_deg(name, t))

    def derive(self, t, *, period=None, scale=None, stop=False, ramp_s=1.0):
        """t부터 이어지는 새 Plan (위상·진폭 연속). stop이면 진폭을 ramp_s에 걸쳐 0으로 줄이고 처음 자세로."""
        a_now = self.amp(t)
        new_period = self.period if period is None else period
        a1 = 0.0 if stop else (self.a1 if scale is None else scale)
        return GaitPlan(self.p, self.roles, self.lim, self.q_start, self.q_ready, self.g, self.h0, t_gait0=t,
                        period=new_period, a0=a_now, a1=a1, TA=ramp_s, phi0=self.phase(t), t_phi0=t,
                        t_ret0=(t + ramp_s) if stop else None, T_ret=self.T_ret, t_walk0=self.t_walk0)


class MovePlan:
    """모든 관절을 q_from에서 q_to로 최소 저크 이동 (준비 자세로 가는 중에 멈출 때)."""

    kind = "move"

    def __init__(self, q_from, q_to, t0, T, lim):
        self.q_from, self.q_to, self.t0, self.T, self.lim = q_from, q_to, t0, T, lim
        self.t_walk0 = None

    @property
    def t_end(self):
        return self.t0 + self.T

    def phase(self, t):
        return None

    def amp(self, t):
        return 0.0

    def stage(self, t):
        return "return" if t < self.t_end else "done"

    def q_deg(self, name, t):
        a, b = self.q_from[name], self.q_to[name]
        return a + (b - a) * _minjerk((t - self.t0) / max(self.T, 1e-6))

    def q(self, name, t):
        return math.radians(self.q_deg(name, t))


class GaitTarget:
    """commander에 넣는 관절 목표. 실제 값은 WalkTest.plan에서 읽는다 (commander.Target과 같은 이름의 속성)."""

    __slots__ = ("walk", "name", "real", "t_created", "t0", "q0")

    def __init__(self, walk, name, real, t_created, q0):
        self.walk, self.name, self.real, self.t_created = walk, name, bool(real), t_created
        self.t0, self.q0 = t_created, q0

    @property
    def q1(self):
        return math.radians(self.walk.q_start.get(self.name, 0.0))

    @property
    def t_end(self):
        plan = self.walk.plan
        return math.inf if plan is None else plan.t_end

    @property
    def T(self):
        return self.t_end - self.t0

    def eval(self, t):
        plan = self.walk.plan
        return self.q0 if plan is None else plan.q(self.name, t)

    def vel(self, t, h=0.004):
        """목표 속도 [rad/s] (중앙 차분). CAN 링크가 속도 피드포워드로 함께 보낸다."""
        plan = self.walk.plan
        if plan is None or t >= plan.t_end:
            return 0.0
        return (plan.q(self.name, t + h) - plan.q(self.name, t - h)) / (2.0 * h)

    def moving(self, t):
        return t < self.t_end


class WalkTest:
    def __init__(self, app):
        self.app = app
        self.params = WalkParams()
        self.state = "idle"            # idle | enabling | running | done | aborted
        self.real = False
        self.plan = None
        self.targets = {}
        self.q_start = {}
        self.reason = ""
        self.logger = None
        self.last_dir = None
        self.t_begin = None
        self._enable_t = None
        self._installed_t = None
        self._log_seen = 0.0
        self._stop_asked = False
        self._t_walk_end = None        # 중단·끝난 시각 (걸은 시간 계산을 거기서 멈춤)
        self._cycles_done = 0.0
        self.geom = leg_geometry(app.model, app.infos)    # 다리 치수 (역기구학)

    # ------------------------------------------------------------------ 상태
    @property
    def active(self):
        return self.state in ("enabling", "running")

    def stage(self, t=None):
        if self.state != "running" or self.plan is None:
            return self.state
        return self.plan.stage(self.app.now() if t is None else t)

    def _walk_end_t(self, t):
        """걸은 시간을 셀 마지막 시각: 처음 자세로 돌아가기 시작하면 거기까지."""
        plan = self.plan
        t_ret0 = getattr(plan, "t_ret0", None)
        if self._t_walk_end is not None:
            t = min(t, self._t_walk_end)
        return t if t_ret0 is None else min(t, t_ret0)

    def walked_s(self, t=None):
        t = self._walk_end_t(self.app.now() if t is None else t)
        if self.plan is None or self.plan.t_walk0 is None or t < self.plan.t_walk0:
            return 0.0
        return t - self.plan.t_walk0

    def cycles(self, t=None):
        plan = self.plan
        if plan is None or plan.kind != "gait":
            return self._cycles_done
        t = self._walk_end_t(self.app.now() if t is None else t)
        if t < plan.t_walk0:
            return 0.0
        return max(0.0, plan.phase(t) / (2.0 * math.pi))

    # ------------------------------------------------------------------ 시작 전 점검
    def preflight(self, real):
        app, now = self.app, self.app.now()
        out = []
        if self.active:
            out.append("이미 보행 테스트 중입니다")
        missing = [app.cfg.joint(n).short for n in app.cfg.names if app.infos.get(n) is None]
        if missing:
            out.append("시뮬 모델에 없는 관절: " + ", ".join(missing))
        elif self.geom is None:
            out.append("시뮬 모델에서 다리 치수를 읽지 못했습니다")
        if app.sweep_active:
            out.append("가동범위 기록 중입니다 (먼저 '끝' 또는 '취소')")
        if not real:
            return out
        if not app.link.connected:
            out.append("실물 로봇에 연결되어 있지 않습니다")
            return out
        off, err, uncal, nodir = [], [], [], []
        for j in app.cfg.joints:
            ns = app.link.nodes[j.node]
            if not ns.online(now) or ns.pos is None:
                off.append(f"{j.short}(node {j.node})")
            elif ns.axis_error:
                err.append(f"{j.short}: {P.decode_bits(ns.axis_error, P.AXIS_ERRORS)}")
            if not j.calibrated:
                uncal.append(j.short)
            if not j.dir_checked:
                nodir.append(j.short)
        if off:
            out.append("응답 없는 관절: " + ", ".join(off))
        if err:
            out.append("에러가 있는 관절 (에러 지우기 후 다시): " + "; ".join(err))
        if uncal:
            out.append("영점이 없는 관절 ('영점·방향 설정' 탭에서 먼저): " + ", ".join(uncal))
        if nodir:
            # 방향이 틀린 관절은 화면(실측 각도)으로는 명령을 잘 따르는 것처럼 보여서, 실물로 확인한 표시를 요구한다
            out.append("방향을 실물로 확인하지 않은 관절 ('영점·방향 설정' 탭에서 +5° 시험 후 '실물로 확인함'): "
                       + ", ".join(nodir))
        return out

    def notes(self, real):
        """시작은 막지 않지만 알려 줄 것."""
        out = []
        if real and self.app.dirty:
            out.append("영점·방향 변경이 아직 저장되지 않았습니다. 지금 값으로 걷지만, 프로그램을 다시 켜면 사라집니다 (설정 저장)")
        return out

    def directions_text(self):
        """확인 창에 보여 줄 관절별 방향 (설정 파일 값)."""
        cfg = self.app.cfg
        rows = []
        for side, label in (("left", "왼"), ("right", "오른")):
            js = [j for j in cfg.joints if joint_role(j.name)[0] == side]
            rows.append(f"{label}: " + "  ".join(f"{j.short.split(' ', 1)[1]} {j.direction:+d}" for j in js))
        return "\n".join(rows)

    def preview(self, params):
        """설정으로 한 주기를 미리 계산 (UI 표시·점검용). 모델 치수가 없으면 None."""
        if self.geom is None or any(self.app.infos.get(n) is None for n in self.app.cfg.names):
            return None
        roles = {n: joint_role(n) for n in self.app.cfg.names}
        return preview(params, self.geom, self._limits(), roles)

    def check_params(self, params):
        """설정 자체의 문제 (시작·보행 중 변경을 막음). 미리보기로 판단."""
        r = self.preview(params)
        if r is None:
            return []
        out = []
        if r["reach_limited"]:
            out.append("보폭·발 높이에 비해 다리가 다 펴집니다 (각도가 꺾여 급하게 움직임). "
                       "보폭을 줄이거나 무릎 기본 굽힘을 늘리세요")
        pv, pj = r["peak_vel"]
        if pv > MAX_DPS:
            out.append(f"관절 최고 속도 {pv:.0f}°/s ({self.app.cfg.joint(pj).short})가 {MAX_DPS:.0f}°/s를 넘습니다. "
                       "걸음 속도·발 높이·진폭 배율을 줄이세요")
        return out

    # ------------------------------------------------------------------ 시작·멈춤
    def start(self, params, real):
        """real=True면 실물 12개 관절 + 시뮬, False면 시뮬만. 문제가 있으면 이유 문자열."""
        probs = self.preflight(real) + self.check_params(params)
        if probs:
            return " / ".join(probs)
        app = self.app
        t = app.now()
        self.params = WalkParams(**asdict(params))
        self.real = bool(real)
        self.plan = None
        self.targets = {}
        self.reason = ""
        self._stop_asked = False
        self._t_walk_end = None
        self._cycles_done = 0.0
        self.t_begin = t
        self._log_seen = t
        cfg = app.cfg
        self.q_start = {}
        for j in cfg.joints:
            if real:
                ns = app.link.nodes[j.node]
                self.q_start[j.name] = math.degrees(j.q_from_turn(ns.pos, ns.k))
            else:
                q = app.sim.q(j.name)
                self.q_start[j.name] = 0.0 if q is None else math.degrees(q)
        self.logger = WalkLogger(app, self)
        self.last_dir = self.logger.dir
        self.logger.event(f"보행 테스트 시작 ({'실물 + 시뮬' if real else '시뮬만'})")
        self.logger.event("설정: " + json.dumps(asdict(self.params), ensure_ascii=False))
        self.logger.event("처음 자세 [deg]: " + ", ".join(f"{cfg.joint(n).short} {v:+.1f}"
                                                      for n, v in self.q_start.items()))
        if real:
            for j in cfg.joints:
                app.link.send_enable(j.node)
                app.cmdr.hold(j.name, math.radians(self.q_start[j.name]), t, real=True)
            self.state = "enabling"
            self._enable_t = t
            app.log("보행 테스트: 12개 관절 토크 켜는 중 (지금 자세 유지)")
        else:
            self.state = "running"
            self._install(t)
        return None

    def _limits(self):
        app, m = self.app, math.radians(self.app.cfg.limit_margin_deg)
        lim = {}
        for n in app.cfg.names:
            info = app.infos[n]
            lo = -math.inf if info.lo is None else math.degrees(info.lo + m)
            hi = math.inf if info.hi is None else math.degrees(info.hi - m)
            lim[n] = (lo, hi)
        return lim

    def _install(self, t):
        app, p = self.app, self.params
        names = app.cfg.names
        roles = {n: joint_role(n) for n in names}
        lim = self._limits()
        g = self.geom
        h0 = g.reach(math.radians(p.crouch_deg))          # 기본 자세에서 고관절 축–발목 축 높이
        pose0, _ = pose_all(p, g, h0, 0.0, 0.0, p.period_s)  # 진폭 0 = 준비 자세 (발이 엉덩이 바로 아래)
        q_ready = {n: soft_clip(pose0[roles[n]], *lim[n]) for n in names}
        sp = math.radians(p.approach_dps)
        T_app = max(2.0, max(move_duration(math.radians(q_ready[n] - self.q_start[n]), sp) for n in names))
        self.plan = GaitPlan(p, roles, lim, dict(self.q_start), q_ready, g, h0, t_gait0=t + T_app,
                             period=p.period_s, a0=0.0, a1=p.scale, TA=p.ramp_in_cycles * p.period_s, phi0=0.0,
                             t_app0=t, T_app=T_app, T_ret=T_app)
        self.targets = {}
        for n in names:
            tg = GaitTarget(self, n, self.real, t, math.radians(self.q_start[n]))
            self.targets[n] = tg
            app.cmdr.set(n, tg)
        self._installed_t = t
        self.logger.event(f"준비 자세로 이동 {T_app:.1f} s: " + ", ".join(
            f"{app.cfg.joint(n).short} {v:+.1f}" for n, v in q_ready.items() if abs(v) > 1e-9))

    def stop(self, why="천천히 멈추기"):
        """진폭을 줄이고 처음 자세로 돌아간다 (설정에 따라 그 뒤 모두 힘 풀기)."""
        if not self.active:
            return "보행 테스트 중이 아닙니다"
        t = self.app.now()
        if self.state == "enabling":
            self.abort(f"{why} (토크 켜는 중)")
            return None
        st = self.plan.stage(t)
        if st in ("stopping", "return", "done") or self._stop_asked:
            return None
        self._stop_asked = True
        p = self.params
        if st == "approach":
            self._cycles_done = 0.0
            q_now = {n: self.plan.q_deg(n, t) for n in self.app.cfg.names}
            T = max(1.0, max(move_duration(math.radians(self.q_start[n] - q_now[n]), math.radians(p.approach_dps))
                             for n in q_now))
            self.plan = MovePlan(q_now, dict(self.q_start), t, T, self.plan.lim)
        else:
            self.plan = self.plan.derive(t, stop=True, ramp_s=max(0.5, p.ramp_out_cycles * self.plan.period))
        self.logger.event(f"멈춤 요청: {why} (단계 '{STAGE_KO.get(st, st)}', 걸은 시간 {self.walked_s(t):.1f} s)")
        self.app.log(f"보행 테스트: {why} → 진폭 줄이고 처음 자세로")
        return None

    def set_live(self, steps_per_min=None, scale=None):
        """걷는 중에 걸음 속도·진폭 배율 바꾸기 (위상·진폭이 이어지게 1 s에 걸쳐). 너무 빠르면 거부하고 이유를 돌려줌."""
        new = WalkParams(**asdict(self.params))
        if steps_per_min is not None:
            new.steps_per_min = float(steps_per_min)
        if scale is not None:
            new.scale = float(scale)
        probs = self.check_params(new)
        if probs:
            return "바꾸지 않음: " + " / ".join(probs)
        self.params.steps_per_min, self.params.scale = new.steps_per_min, new.scale
        if self.state != "running" or self.plan is None or self.plan.kind != "gait" or self._stop_asked:
            return None
        t = self.app.now()
        if self.plan.stage(t) not in ("ramp", "walk"):
            return None
        self.plan = self.plan.derive(t, period=self.params.period_s, scale=self.params.scale, ramp_s=1.0)
        self.logger.event(f"보행 중 변경: 걸음 속도 {self.params.steps_per_min:.0f} 걸음/분 "
                          f"(주기 {self.params.period_s:.2f} s), 진폭 배율 {self.params.scale:.2f}")
        return None

    def abort(self, reason):
        """보행을 바로 끝내고 모든 관절 힘 풀기 (시뮬만이면 목표 지움)."""
        if not self.active:
            return
        self._mark_end()
        self.state = "aborted"
        self.reason = reason
        app = self.app
        text = f"⚠ 보행 테스트 중단: {reason}"
        app.log(text)
        app.link.alerts.append(text)
        if self.logger:
            self.logger.event(text)
        if self.real:
            app.idle_all(f"보행 테스트 중단: {reason}")
        else:
            self._clear_targets()
        self._close_log(reason)

    def on_idle_all(self, reason):
        """App.idle_all()이 불릴 때 (Esc, 빨간 버튼, 연결 끊기). 힘 풀기·목표 지우기는 App이 한다."""
        if not self.active:
            return
        st = self.stage()
        self._mark_end()
        self.state = "aborted"
        self.reason = reason
        if self.logger:
            self.logger.event(f"모두 힘 풀기: {reason} (단계 '{STAGE_KO.get(st, st)}', "
                              f"걸은 시간 {self.walked_s():.1f} s)")
        self._close_log(reason)

    def _mark_end(self):
        t = self.app.now()
        self._cycles_done = self.cycles(t)
        self._t_walk_end = t

    def _finish(self, t):
        self._mark_end()
        self.state = "done"
        self.reason = "끝까지 실행"
        app = self.app
        if self.real and self.params.end_relax:
            self.logger.event("처음 자세로 돌아옴 → 모두 힘 풀기")
            app.idle_all("보행 테스트 끝")
        elif self.real:
            for n in app.cfg.names:
                app.cmdr.hold(n, math.radians(self.q_start[n]), t, real=True)
            self.logger.event("처음 자세로 돌아옴 → 토크 유지 (힘 풀기는 직접)")
        else:
            if self.params.end_relax:
                self._clear_targets()
            else:
                for n in app.cfg.names:
                    app.cmdr.hold(n, math.radians(self.q_start[n]), t, real=False)
        app.log(f"보행 테스트 끝 (걸은 시간 {self.walked_s(t):.1f} s)")
        self._close_log(self.reason)

    def _clear_targets(self):
        for n, tg in self.targets.items():
            if self.app.cmdr.get(n) is tg:
                self.app.cmdr.clear(n)

    def _close_log(self, reason):
        if self.logger is not None:
            self.logger.finish(reason)

    # ------------------------------------------------------------------ 매 프레임 (GUI 스레드)
    def update(self):
        if self.logger is not None:
            self._copy_logs()
        if not self.active:
            return
        app = self.app
        t = app.now()
        if self.state == "enabling":
            self._check_nodes(t, need_closed=False)
            if not self.active:
                return
            on = [j for j in app.cfg.joints if app.link.nodes[j.node].state == P.STATE_CLOSED_LOOP]
            if len(on) == len(app.cfg.joints):
                self.logger.event(f"12개 관절 위치 제어 진입 ({t - self._enable_t:.2f} s)")
                self.state = "running"
                self._install(t)
            elif t - self._enable_t > ENABLE_TIMEOUT_S:
                off = [j.short for j in app.cfg.joints if j not in on]
                self.abort("토크가 켜지지 않은 관절: " + ", ".join(off))
            return
        # running
        if self.real:
            self._check_nodes(t, need_closed=True)
            if not self.active:
                return
        for n, tg in self.targets.items():
            if app.cmdr.get(n) is not tg:
                self.abort(f"{app.cfg.joint(n).short}: 목표가 지워짐 (보호 정지, 힘 풀기 또는 응답 끊김 — 기록 탭 참고)")
                return
        st = self.plan.stage(t)
        p = self.params
        if (p.duration_s > 0 and st in ("ramp", "walk") and not self._stop_asked
                and self.walked_s(t) >= p.duration_s):
            self.stop(f"정한 시간 {p.duration_s:g} s 끝")
        elif t >= self.plan.t_end:
            self._finish(t)

    def _check_nodes(self, t, need_closed):
        app = self.app
        for j in app.cfg.joints:
            ns = app.link.nodes[j.node]
            if not ns.online(t):
                self.abort(f"{j.short} (node {j.node}) 응답 끊김")
                return
            if ns.axis_error:
                self.abort(f"{j.short} (node {j.node}) 에러: {P.decode_bits(ns.axis_error, P.AXIS_ERRORS)}")
                return
            if need_closed and ns.state != P.STATE_CLOSED_LOOP and t - self._installed_t > 0.5:
                self.abort(f"{j.short} (node {j.node}) 위치 제어가 풀림 ({P.state_text(ns.state)})")
                return

    def _copy_logs(self):
        """GUI·CAN 기록 중 이번 실행에 해당하는 줄을 events.txt에도 남긴다."""
        lg = self.logger
        if lg is None or lg.closed:
            return
        rows = [(t, "GUI", s) for t, s in list(self.app.logs)] + [(t, "CAN", s) for t, s in list(self.app.link.logs)]
        new = sorted(r for r in rows if r[0] > self._log_seen)
        for t, src, s in new:
            lg.event(f"[{src}] {s}", t=t)
        if new:
            self._log_seen = new[-1][0]


class WalkLogger:
    """한 번의 보행 테스트 기록. 표본은 별도 스레드가 50 Hz로 남긴다."""

    def __init__(self, walk_app, walk):
        self.app, self.walk = walk_app, walk
        self.t0 = walk_app.now()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir = LOG_DIR / f"walk_{stamp}"
        n = 1
        while self.dir.exists():
            n += 1
            self.dir = LOG_DIR / f"walk_{stamp}_{n}"
        self.dir.mkdir(parents=True)
        self.closed = False
        self.reason = ""
        self.live = {"max_err": (0.0, ""), "max_iq": (0.0, ""), "n": 0}
        self._lock = threading.Lock()
        self._ev = open(self.dir / "events.txt", "w", encoding="utf-8", newline="\n")
        self._ev.write(f"MotorTestGUI 보행 테스트 기록 · 시작 {datetime.now().isoformat(timespec='seconds')}\n")
        self._finish_t = None
        self._write_meta()
        self.rate = min(max(walk_app.cfg.poll_hz, 10.0), 100.0)
        self._thread = threading.Thread(target=self._run, name="walk-log", daemon=True)
        self._thread.start()

    def event(self, text, t=None):
        t = self.app.now() if t is None else t
        with self._lock:
            if self._ev.closed:
                return
            self._ev.write(f"[{t - self.t0:+9.3f} s] {text}\n")
            self._ev.flush()

    def finish(self, reason):
        if self._finish_t is None:
            self.reason = reason
            self._finish_t = time.perf_counter()
            self.event(f"끝난 이유: {reason} (이후 {TAIL_S:g} s 더 기록)")

    def _write_meta(self):
        app, walk = self.app, self.walk
        joints = []
        for j in app.cfg.joints:
            info = app.infos.get(j.name)
            ns = app.link.nodes[j.node]
            kp_d, kd_d = driver_joint_gains(j.motor)
            joints.append({
                "name": j.name, "short": j.short, "node": j.node, "motor": j.motor, "direction": j.direction,
                "zero_turn": j.zero_turn, "torque_sign": j.torque_sign, "k": ns.k, "bus": app.link.bus_of(j.node),
                "range_deg": None if info is None or info.lo is None else [math.degrees(info.lo), math.degrees(info.hi)],
                "sim_kp": None if info is None else info.kp, "sim_kd": None if info is None else info.kd,
                "driver_kp": kp_d, "driver_kd": kd_d})
        meta = {"started": datetime.now().isoformat(timespec="seconds"), "mode": "real" if walk.real else "sim",
                "params": dict(asdict(walk.params), period_s=walk.params.period_s),
                "leg_geometry_m_rad": None if walk.geom is None else asdict(walk.geom),
                "connection": app.link.descr if app.link.connected else "연결 안 됨",
                "channels": app.link.channels, "poll_hz": app.cfg.poll_hz,
                "guard": {"track_err_deg": app.cfg.guard_err_deg, "track_err_s": app.cfg.guard_err_s,
                          "current_a": app.cfg.guard_current_a, "current_s": app.cfg.guard_current_s},
                "limit_margin_deg": app.cfg.limit_margin_deg, "model": str(app.mjcf_path),
                "config": str(app.cfg.path), "joints": joints}
        (self.dir / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    def _run(self):
        app, walk = self.app, self.walk
        cfg = app.cfg
        shorts = [j.short.replace(" ", "_") for j in cfg.joints]
        cols = ["t_s", "stage", "phase_deg", "amp", "vbus_V"]
        for s in shorts:
            cols += [f"{s}_tgt_deg", f"{s}_act_deg", f"{s}_err_deg", f"{s}_sim_deg", f"{s}_vel_dps", f"{s}_iq_A",
                     f"{s}_iqset_A", f"{s}_state", f"{s}_axis_err", f"{s}_age_ms"]
        acc = {j.name: {"n": 0, "e2": 0.0, "emax": 0.0, "i2": 0.0, "imax": 0.0, "ni": 0} for j in cfg.joints}
        vb = [math.inf, -math.inf]
        n_rows = 0
        f = open(self.dir / "samples.csv", "w", encoding="utf-8", newline="")
        w = csv.writer(f)
        w.writerow(cols)
        dt = 1.0 / self.rate
        nxt = time.perf_counter()
        last_flush = nxt
        try:
            while True:
                if self._finish_t is not None and time.perf_counter() - self._finish_t > TAIL_S:
                    break
                t = app.now()
                plan = walk.plan
                stage = walk.stage(t)
                ph = None if plan is None or walk.state != "running" else plan.phase(t)
                row = [f"{t - self.t0:.4f}", stage, "" if ph is None else f"{math.degrees(ph) % 360.0:.1f}",
                       "" if plan is None else f"{plan.amp(t):.3f}"]
                vs = [ns.vbus for ns in app.link.nodes.values() if ns.vbus is not None and ns.online(t)]
                if vs:
                    row.append(f"{sum(vs) / len(vs):.2f}")
                    vb[0], vb[1] = min(vb[0], min(vs)), max(vb[1], max(vs))
                else:
                    row.append("")
                measuring = stage in ("ramp", "walk", "stopping")
                for j in cfg.joints:
                    ns = app.link.nodes[j.node]
                    tg = app.cmdr.target_q(j.name, t)
                    on = ns.online(t) and ns.pos is not None
                    act = j.q_from_turn(ns.pos, ns.k) if on else None
                    # 추종 오차는 엔코더 값을 받은 시각의 목표와 비교 (지금 목표와 비교하면 통신 지연만큼 커 보임)
                    tg_enc = app.cmdr.target_q(j.name, ns.t_enc) if act is not None else None
                    err = None if tg_enc is None else math.degrees(tg_enc - act)
                    sim = app.sim.q(j.name)
                    vel = j.direction * ns.vel * 2 * math.pi / 15.0 if on and ns.vel is not None else None
                    iq = ns.iq if on else None
                    row += ["" if tg is None else f"{math.degrees(tg):.3f}",
                            "" if act is None else f"{math.degrees(act):.3f}",
                            "" if err is None else f"{err:.3f}",
                            "" if sim is None else f"{math.degrees(sim):.3f}",
                            "" if vel is None else f"{math.degrees(vel):.2f}",
                            "" if iq is None else f"{iq:.3f}",
                            "" if not on or ns.iq_set is None else f"{ns.iq_set:.3f}",
                            "" if ns.state is None else ns.state,
                            ns.axis_error,
                            "" if not on else f"{(t - ns.t_enc) * 1000:.0f}"]
                    if measuring:
                        a = acc[j.name]
                        # 묵은 엔코더 값(GUI가 잠깐 멈춘 동안 등)은 오차 통계에서 뺌. CSV에는 age_ms와 함께 남음
                        fresh = act is not None and t - ns.t_enc < STALE_S
                        if err is not None and fresh:
                            e = abs(err)
                            a["n"] += 1
                            a["e2"] += e * e
                            if e > a["emax"]:
                                a["emax"] = e
                                if e > self.live["max_err"][0]:
                                    self.live["max_err"] = (e, j.short)
                        if iq is not None:
                            a["ni"] += 1
                            a["i2"] += iq * iq
                            if abs(iq) > a["imax"]:
                                a["imax"] = abs(iq)
                                if abs(iq) > self.live["max_iq"][0]:
                                    self.live["max_iq"] = (abs(iq), j.short)
                w.writerow(row)
                n_rows += 1
                self.live["n"] = n_rows
                now = time.perf_counter()
                if now - last_flush > 1.0:
                    f.flush()
                    last_flush = now
                nxt += dt
                delay = nxt - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    nxt = time.perf_counter()
        finally:
            f.close()
            self._write_summary(acc, vb, n_rows)

    def _write_summary(self, acc, vb, n_rows):
        app, walk = self.app, self.walk
        per = {}
        for j in app.cfg.joints:
            a = acc[j.name]
            per[j.short] = {
                "node": j.node,
                "err_max_deg": round(a["emax"], 3) if a["n"] else None,
                "err_rms_deg": round(math.sqrt(a["e2"] / a["n"]), 3) if a["n"] else None,
                "iq_max_A": round(a["imax"], 3) if a["ni"] else None,
                "iq_rms_A": round(math.sqrt(a["i2"] / a["ni"]), 3) if a["ni"] else None}
        summary = {"mode": "real" if walk.real else "sim", "result": walk.state, "reason": self.reason,
                   "walked_s": round(walk.walked_s(), 2), "cycles": round(walk.cycles(), 2), "samples": n_rows,
                   "vbus_min_V": None if not math.isfinite(vb[0]) else round(vb[0], 2),
                   "vbus_max_V": None if not math.isfinite(vb[1]) else round(vb[1], 2),
                   "params": asdict(walk.params), "joints": per}
        (self.dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        lines = [f"요약: {summary['result']} · {self.reason} · 걸은 시간 {summary['walked_s']} s · "
                 f"{summary['cycles']} 주기 · 표본 {n_rows}개 · Vbus {summary['vbus_min_V']}~{summary['vbus_max_V']} V"]
        for s, r in per.items():
            lines.append(f"  {s:16s} 오차 최대 {r['err_max_deg']}° RMS {r['err_rms_deg']}° · |Iq| 최대 {r['iq_max_A']} A "
                         f"RMS {r['iq_rms_A']} A")
        with self._lock:
            for ln in lines:
                self._ev.write(ln + "\n")
            self._ev.close()
        self.closed = True


def open_folder(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    if hasattr(os, "startfile"):
        os.startfile(str(path))      # noqa: S606 - Windows 탐색기로 폴더 열기
