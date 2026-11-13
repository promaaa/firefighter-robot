"""관절 설정(node, 모터, 방향, 영점)과 엔코더 <-> 관절 각도 변환.

관절 각도 q [rad] (시뮬레이션 MJCF 기준):
    q = direction * (p + k - zero_turn) * 2*pi / 15
    p          : 드라이버 pos_estimate [motor turn]
    zero_turn  : q = 0일 때의 모터 위치 [turn] (영점 보정으로 정함, 파일에 저장)
    direction  : +1 / -1 (모터 + 방향이 시뮬 관절 + 방향과 같은지)
    k          : 정수 turn 보정. 코스 펌웨어는 부팅 때 pos_estimate를 절대 엔코더 값(0~1 turn)으로
                 시작하므로 관절 기준으로는 24도마다 같은 값이 된다. 노드가 처음 보일 때(전원을 켠 뒤)
                 관절이 boot_ref_deg 근처(+-12도 안)에 있다고 보고 k를 고른다.

토크 [Nm, 관절, 시뮬 + 방향] = direction * torque_sign * Iq * kt * 15
    torque_sign은 드라이버 motor.config.direction (펌웨어가 전류 명령에 direction을 곱해서
    direction = -1인 보드는 Iq 부호가 실제 토크와 반대, bench/odrv_usb.torque_sign 참고).
    BasicTest/data/IDxx의 setup 백업에서 기본값을 읽는다.
"""
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

GUI_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = GUI_ROOT.parent
BASIC_DIR = REPO_ROOT / "BasicTest"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from bench import joints as bench_joints  # noqa: E402
from bench import params  # noqa: E402

DEFAULT_MJCF = GUI_ROOT / "assets" / "asmr2026-humanoid-v1" / "mjcf" / "asmr2026_humanoid.xml"
CONFIG_PATH = GUI_ROOT / "config" / "robot_config.json"
TURN_TO_JOINT_DEG = 360.0 / params.GEAR_RATIO          # 모터 1 turn = 관절 24도
RAD_PER_TURN = 2.0 * math.pi / params.GEAR_RATIO

# 기구 끝(하드 스톱) 관절 각도 [deg], None = 그쪽은 막히는 곳 없음.
# 2026-11-11 수업 모델 v1 메쉬 충돌 검사(시각용 STL, 다른 관절 0°, 반대쪽 다리 제외):
#   knee·ankle은 MJCF 범위 끝과 정확히 같고, hip pitch는 MJCF보다 19~24° 바깥(허벅지가 hip yaw 링크·몸통에 닿음),
#   hip yaw는 한쪽만 몸통 프레임에 닿음, hip roll은 양쪽 다 막히는 곳 없음.
# 실물에서 다르게 나오면 config/robot_config.json의 stop_lo_deg / stop_hi_deg를 고친다.
MECH_STOPS_DEG = {
    "leg_left_hip_roll_joint": (None, None),
    "leg_left_hip_yaw_joint": (None, 53.38),
    "leg_left_hip_pitch_joint": (-75.55, 133.09),
    "leg_left_knee_pitch_joint": (-140.0, 0.0),
    "leg_left_ankle_pitch_joint": (-60.0, 30.0),
    "leg_left_ankle_roll_joint": (-15.0, 15.0),
    "leg_right_hip_roll_joint": (None, None),
    "leg_right_hip_yaw_joint": (-54.85, None),
    "leg_right_hip_pitch_joint": (-133.11, 75.55),
    "leg_right_knee_pitch_joint": (0.0, 140.0),
    "leg_right_ankle_pitch_joint": (-30.0, 60.0),
    "leg_right_ankle_roll_joint": (-15.0, 15.0),
}
MECH_STOPS_SRC = "수업 모델 v1 메쉬 충돌 검사 (2026-11-11)"


def short_name(name):
    """'leg_left_hip_roll_joint' -> 'L hip roll'."""
    s = name
    if s.startswith("leg_"):
        s = s[4:]
    if s.endswith("_joint"):
        s = s[:-6]
    side, _, part = s.partition("_")
    side = {"left": "L", "right": "R"}.get(side, side)
    return f"{side} {part.replace('_', ' ')}"


def driver_joint_gains(motor):
    """드라이버에 저장된 코스 xdrive_config 게인의 관절축 값 (Kp [Nm/rad], Kd [Nm*s/rad]).

    CAN Simple(fw 0.5.1)로는 게인을 읽을 수 없어서 setup_driver.py가 저장하는 값을 보여 준다.
    """
    pg, vg = params.MOTORS[motor]["course_gains"]
    return params.joint_gains(pg, vg)


def _backup_motor_direction(node):
    """BasicTest/data/IDxx의 가장 최근 setup 후 백업에서 axis0.motor.config.direction."""
    folder = BASIC_DIR / "data" / f"ID{node:02d}"
    files = sorted(folder.glob(f"xdrive_ID{node:02d}_after_setup_*.json"))
    for path in reversed(files):
        try:
            d = json.loads(path.read_text(encoding="utf-8"))
            v = d["axis0"]["motor"]["config"]["direction"]
            if v in (1, -1):
                return int(v), path.name
        except (OSError, KeyError, TypeError, ValueError):
            continue
    return None, None


@dataclass
class JointCfg:
    name: str
    node: int
    motor: str
    direction: int = 1
    zero_turn: float | None = None
    boot_ref_deg: float = 0.0
    torque_sign: int = 1
    torque_sign_src: str = "기본값"
    stop_lo_deg: float | None = None     # 기구 끝 (가동범위 훑기 영점에 씀)
    stop_hi_deg: float | None = None
    stop_src: str = ""

    @property
    def short(self):
        return short_name(self.name)

    @property
    def calibrated(self):
        return self.zero_turn is not None

    @property
    def kt(self):
        return params.MOTORS[self.motor]["kt"]

    def q_from_turn(self, p, k=0):
        z = self.zero_turn if self.zero_turn is not None else 0.0
        return self.direction * (p + k - z) * RAD_PER_TURN

    def turn_from_q(self, q, k=0):
        z = self.zero_turn if self.zero_turn is not None else 0.0
        return z + self.direction * q / RAD_PER_TURN - k

    def unwrap_k(self, p):
        """지금 관절이 boot_ref_deg 근처라고 보고 정수 turn k를 고른다. 미보정이면 0."""
        if self.zero_turn is None:
            return 0
        return int(round(self.direction * self.boot_ref_deg / TURN_TO_JOINT_DEG + self.zero_turn - p))

    def torque_from_iq(self, iq):
        return self.direction * self.torque_sign * iq * self.kt * params.GEAR_RATIO

    def to_json(self):
        return {"node": self.node, "motor": self.motor, "direction": self.direction,
                "zero_turn": self.zero_turn, "boot_ref_deg": self.boot_ref_deg,
                "torque_sign": self.torque_sign, "torque_sign_src": self.torque_sign_src,
                "stop_lo_deg": self.stop_lo_deg, "stop_hi_deg": self.stop_hi_deg, "stop_src": self.stop_src}


@dataclass
class RobotConfig:
    joints: list
    can_interface: str = "slcan"
    can_channel: str = ""             # 포트 1 (CANable 하나)
    can_channel2: str = ""            # 포트 2 (두 번째 CANable, 비우면 버스 1개)
    can_bitrate: int = params.CAN_BITRATE
    poll_hz: float = 50.0
    mjcf: str = ""
    max_speed_dps: float = 30.0
    max_step_deg: float = 30.0
    limit_margin_deg: float = 2.0
    guard_err_deg: float = 10.0       # 목표-실측 오차가 이보다 크게
    guard_err_s: float = 0.5          #   이 시간 이어지면 그 관절 IDLE
    guard_current_a: float = 8.0      # |Iq|가 이보다 크게
    guard_current_s: float = 1.0      #   이 시간 이어지면 그 관절 IDLE
    path: Path = field(default=CONFIG_PATH, repr=False)

    # ---- 조회 ----
    def joint(self, name):
        return next(j for j in self.joints if j.name == name)

    def by_node(self, node):
        return next((j for j in self.joints if j.node == node), None)

    @property
    def names(self):
        return [j.name for j in self.joints]

    def mjcf_path(self):
        if not self.mjcf:
            return DEFAULT_MJCF
        p = Path(self.mjcf)
        return p if p.is_absolute() else (GUI_ROOT / p).resolve()

    # ---- 파일 ----
    @classmethod
    def default(cls):
        joints = []
        for name, node, motor in bench_joints.JOINTS:
            j = JointCfg(name=name, node=node, motor=motor)
            sign, src = _backup_motor_direction(node)
            if sign is not None:
                j.torque_sign, j.torque_sign_src = sign, f"백업 {src}"
            if name in MECH_STOPS_DEG:
                j.stop_lo_deg, j.stop_hi_deg = MECH_STOPS_DEG[name]
                j.stop_src = MECH_STOPS_SRC
            joints.append(j)
        return cls(joints=joints)

    @classmethod
    def load(cls, path=CONFIG_PATH):
        cfg = cls.default()
        cfg.path = Path(path)
        if not cfg.path.exists():
            return cfg
        d = json.loads(cfg.path.read_text(encoding="utf-8"))
        can = d.get("can", {})
        cfg.can_interface = can.get("interface", cfg.can_interface)
        cfg.can_channel = can.get("channel", cfg.can_channel)
        cfg.can_channel2 = can.get("channel2", cfg.can_channel2)
        cfg.can_bitrate = int(can.get("bitrate", cfg.can_bitrate))
        cfg.poll_hz = float(d.get("poll_hz", cfg.poll_hz))
        cfg.mjcf = d.get("mjcf", cfg.mjcf)
        mv = d.get("move", {})
        cfg.max_speed_dps = float(mv.get("max_speed_dps", cfg.max_speed_dps))
        cfg.max_step_deg = float(mv.get("max_step_deg", cfg.max_step_deg))
        cfg.limit_margin_deg = float(mv.get("limit_margin_deg", cfg.limit_margin_deg))
        gd = d.get("guard", {})
        cfg.guard_err_deg = float(gd.get("track_err_deg", cfg.guard_err_deg))
        cfg.guard_err_s = float(gd.get("track_err_s", cfg.guard_err_s))
        cfg.guard_current_a = float(gd.get("current_a", cfg.guard_current_a))
        cfg.guard_current_s = float(gd.get("current_s", cfg.guard_current_s))
        for name, jd in d.get("joints", {}).items():
            try:
                j = cfg.joint(name)
            except StopIteration:
                continue
            j.node = int(jd.get("node", j.node))
            j.motor = str(jd.get("motor", j.motor))
            j.direction = -1 if int(jd.get("direction", j.direction)) < 0 else 1
            z = jd.get("zero_turn", None)
            j.zero_turn = None if z is None else float(z)
            j.boot_ref_deg = float(jd.get("boot_ref_deg", j.boot_ref_deg))
            if "torque_sign" in jd:
                j.torque_sign = -1 if int(jd["torque_sign"]) < 0 else 1
                j.torque_sign_src = jd.get("torque_sign_src", "설정 파일")
            if "stop_lo_deg" in jd or "stop_hi_deg" in jd:
                lo, hi = jd.get("stop_lo_deg"), jd.get("stop_hi_deg")
                j.stop_lo_deg = None if lo is None else float(lo)
                j.stop_hi_deg = None if hi is None else float(hi)
                j.stop_src = jd.get("stop_src", "설정 파일")
        return cfg

    def save(self, path=None):
        path = Path(path or self.path)
        path.parent.mkdir(parents=True, exist_ok=True)
        d = {
            "_설명": "MotorTestGUI 관절 설정. q[rad] = direction*(pos_estimate + k - zero_turn)*2pi/15, "
                    "k는 노드가 처음 보일 때 관절이 boot_ref_deg +-12도 안에 있다고 보고 정함.",
            "can": {"interface": self.can_interface, "channel": self.can_channel, "channel2": self.can_channel2,
                    "bitrate": self.can_bitrate},
            "poll_hz": self.poll_hz,
            "mjcf": self.mjcf,
            "move": {"max_speed_dps": self.max_speed_dps, "max_step_deg": self.max_step_deg,
                     "limit_margin_deg": self.limit_margin_deg},
            "guard": {"track_err_deg": self.guard_err_deg, "track_err_s": self.guard_err_s,
                      "current_a": self.guard_current_a, "current_s": self.guard_current_s},
            "joints": {j.name: j.to_json() for j in self.joints},
        }
        path.write_text(json.dumps(d, indent=2, ensure_ascii=False), encoding="utf-8")
        self.path = path
        return path
