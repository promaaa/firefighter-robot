"""학습 기준값과 단위 변환.

출처: 수업 문서 Robot Model (v1), 코스 펌웨어 xdrive_config.txt, 인수 점검 artifact.
모터축 단위는 ODrive 그대로(turn, turn/s, Nm), 관절(출력)축은 rad / deg / Nm.
"""
import math

GEAR_RATIO = 15.0
TURN_TO_OUT_DEG = 360.0 / GEAR_RATIO      # 모터 1 turn = 출력 24°
CAN_BITRATE = 500000                       # 수업 문서 USB-to-CAN Adapter 기준
CURRENT_LIM = 18.0                         # A, 드라이버 전류 한계 (수업 문서 Robot Model v1). 팀 결정 2026-11-06: 그대로 둠
TEAM_TORQUE_LIMIT = 6.0                    # Nm 관절축, 강화학습 토크 한계 (팀 결정 2026-11-06, BHL처럼). 드라이버는 CURRENT_LIM 유지
VEL_LIMIT = 50.0                           # turn/s (모터축) = 관절 20.9 rad/s
POLICY_DT = 0.04                           # 25 Hz
G = 9.81

MOTORS = {
    "6512": {
        "name": "MAD M6C12 150KV",
        "joints": "고관절 roll/yaw/pitch, 무릎",
        "kv": 150,
        "kt": 0.067,               # Nm/A, 벤치 실측 (모터축)
        "kp": 90.01,               # 관절 Nm/rad
        "kd": 5.730,               # 관절 Nm*s/rad
        "zeta": 2.0,
        "armature": 0.0228,        # kg*m^2 (관절축)
        "coulomb": 0.377,          # Nm (관절축)
        "viscous": 0.090,          # Nm*s/rad (관절축)
        "torque_limit": 18.09,     # Nm (18 A 기준)
        "train_gains": (15.71, 0.160),     # ODrive pos_gain, vel_gain
        "course_gains": (12.6, 0.223),     # xdrive_config.txt
        "phase_R_ref": 0.0684,     # Ohm, 모터 보정 실측 (ID01 67.9, ID02 68.9 mOhm, 납땜 배선. 두 보드는 처음에 ID03·ID04로 매김)
        "phase_L_ref": 38.8e-6,    # H, 같은 보정 (38.5, 39.1 uH)
    },
    "5010": {
        "name": "MAD 5010 110KV",
        "joints": "발목 pitch/roll",
        "kv": 110,
        "kt": 0.0836,
        "kp": 21.95,
        "kd": 0.489,
        "zeta": 0.7,
        "armature": 0.00556,
        "coulomb": 0.317,
        "viscous": 0.030,
        "torque_limit": 22.57,
        "train_gains": (44.88, 0.01366),
        "course_gains": (20.0, 0.1),
        "phase_R_ref": 0.233,      # Ohm, 모터 보정 실측 (ID12 한 대, 두 번 보정 232.2 / 233.4 mOhm, 2026-11-09)
        "phase_L_ref": 103e-6,     # H, 같은 보정 (103.4, 102.5 uH)
    },
}

# ODrive fw 0.5.1 기본값 (erase_configuration 직후)
ODRIVE_DEFAULTS = {
    "pos_gain": 20.0,
    "vel_gain": 1.0 / 6.0,
    "vel_integrator_gain": 1.0 / 3.0,
    "vel_limit": 2.0,
    "current_lim": 10.0,
    "torque_constant": 0.04,
    "baud_rate": 250000,
}


def out_deg_to_turn(deg):
    return deg / TURN_TO_OUT_DEG


def turn_to_out_deg(turn):
    return turn * TURN_TO_OUT_DEG


def turn_to_out_rad(turn):
    return turn * 2.0 * math.pi / GEAR_RATIO


def out_rad_to_turn(rad):
    return rad * GEAR_RATIO / (2.0 * math.pi)


def joint_gains(pos_gain, vel_gain, torque_constant=None, kt=None):
    """ODrive 게인 -> 관절축 (Kp [Nm/rad], Kd [Nm*s/rad]).

    torque_constant(드라이버 설정)가 실제 kt와 다르면 실제 토크가 kt/torque_constant 배가 되므로
    둘 다 주면 그 비율을 곱한다.
    """
    scale = kt / torque_constant if (kt and torque_constant) else 1.0
    kd = vel_gain * GEAR_RATIO ** 2 / (2.0 * math.pi) * scale
    return pos_gain * kd, kd


def odrive_gains(kp, kd):
    """관절축 Kp, Kd -> ODrive (pos_gain, vel_gain)."""
    vel_gain = kd * 2.0 * math.pi / GEAR_RATIO ** 2
    return kp / kd, vel_gain


def damping_ratio(kp, kd, inertia):
    return kd / (2.0 * math.sqrt(kp * inertia)) if kp > 0 and inertia > 0 else float("nan")


def gains_for(motor, which):
    """which: 'train' | 'course' | 'default' -> (pos_gain, vel_gain, vel_integrator_gain)."""
    m = MOTORS[motor]
    if which == "train":
        return (*m["train_gains"], 0.0)
    if which == "course":
        return (*m["course_gains"], 0.0)
    if which == "default":
        return ODRIVE_DEFAULTS["pos_gain"], ODRIVE_DEFAULTS["vel_gain"], ODRIVE_DEFAULTS["vel_integrator_gain"]
    raise ValueError(which)
