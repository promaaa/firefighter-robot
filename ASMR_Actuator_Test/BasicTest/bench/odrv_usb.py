"""USB(odrivetool 0.5.1의 fibre)로 드라이버에 접속하고 RAM에서만 설정을 바꾸는 도우미.

save_configuration()은 어디에서도 부르지 않는다. RamSession으로 바꾼 값은 끝날 때 되돌리고,
되돌리지 못해도 재부팅하면 저장된 값으로 돌아간다.
"""
import math
import statistics
import sys
import time

from . import params, util

STATE_IDLE = 1
STATE_CLOSED_LOOP = 8
CONTROL_TORQUE = 1
CONTROL_POSITION = 3
INPUT_PASSTHROUGH = 1

USB_HINT = """
ODrive를 찾지 못했습니다. 확인할 것:
  1) DC 전원이 켜져 있는지 (USB 전원만으로는 켜지지 않음)
  2) odrivetool 창이나 다른 스크립트가 이미 연결 중이면 닫기 (한 번에 한 프로그램만 연결됨)
  3) Zadig: Options > List All Devices > 'ODrive 3.x Native Interface (Interface 2)'
     (USB ID 1209 0D32) 선택 > libusb-win32 > Install Driver
  4) python 00_env_check.py 로 장치와 드라이버 상태 확인
"""


ERROR_HINTS = {
    "PHASE_RESISTANCE_OUT_OF_RANGE": "calibration_current를 흘리는 데 resistance_calib_max_voltage(2 V)보다 큰 전압이 "
                                     "필요했음 (motor.cpp). 모터 3상(A/B/C) 연결·납땜, 전원 끄고 멀티미터로 A-B, B-C, "
                                     "C-A 저항이 모두 비슷하고 작은지 확인. 악어 케이블·바나나 집게는 이 에러를 낸 적 있음",
    "PHASE_INDUCTANCE_OUT_OF_RANGE": "모터 3상 연결과 모터 종류 확인",
    "CPR_POLEPAIRS_MISMATCH": "pole_pairs(14)와 encoder cpr(16384), 엔코더 자석 위치 확인",
    "NO_RESPONSE": "보정 중 엔코더 값이 변하지 않음: 모터가 실제로 돌았는지, 엔코더 자석·모드(257) 확인",
    "ABS_SPI": "보드 엔코더(AS5047P) 통신 문제: abs_spi_cs_gpio_pin(7), 자석 거리 확인",
    "DC_BUS_UNDER_VOLTAGE": "Vbus가 저전압 차단값 아래로 떨어짐: 전원선 굵기·길이, 커넥터·단자 조임, SMPS 전압 확인",
}


def error_hints(errs):
    """axis_errors() 결과에 맞는 힌트 목록."""
    return [h for key, h in ERROR_HINTS.items() if any(key in e for e in errs)]


def add_args(parser, motor_required=True):
    parser.add_argument("--id", default="00", help="액추에이터 번호 (파일 이름에 씀), 예: 01")
    parser.add_argument("--motor", choices=sorted(params.MOTORS), required=motor_required,
                        help="6512 (고관절·무릎) 또는 5010 (발목)")
    parser.add_argument("--sim", action="store_true", help="장치 없이 시뮬레이션 드라이버로 실행 (리허설용)")
    parser.add_argument("--timeout", type=float, default=15.0, help="USB 장치 검색 시간 [s]")


def connect(args):
    if args.sim:
        from .sim import FakeODrive
        print("[SIM] 시뮬레이션 드라이버를 씁니다. 실제 장치에는 연결하지 않습니다.")
        return FakeODrive(getattr(args, "motor", None) or "6512")
    try:
        import odrive
        import usb.backend.libusb0
        import usb.backend.libusb1
    except ImportError:
        sys.exit("odrive 패키지가 없습니다. 먼저 %USERPROFILE%\\odrv051\\Scripts\\activate 를 실행하세요.")
    if usb.backend.libusb0.get_backend() is None and usb.backend.libusb1.get_backend() is None:
        sys.exit("USB 백엔드(libusb0.dll)가 없습니다. 장치를 꽂고 Zadig로 ODrive Interface 2에 libusb-win32를 "
                 "먼저 설치하세요 (설치하면 libusb0.dll도 같이 들어옵니다)." + USB_HINT)
    print(f"USB로 ODrive를 찾는 중... (최대 {args.timeout:.0f}초)")
    odrv = odrive.find_any(timeout=args.timeout)
    if odrv is None:
        sys.exit(USB_HINT)
    print(f"연결됨: serial {read(odrv, 'serial_number', 0):012X}, fw {fw_string(odrv)}, "
          f"Vbus {read(odrv, 'vbus_voltage', float('nan')):.2f} V")
    return odrv


def resolve(root, path):
    obj = root
    parts = path.split(".")
    for p in parts[:-1]:
        obj = getattr(obj, p)
    return obj, parts[-1]


def read(root, path, default=None):
    try:
        obj, attr = resolve(root, path)
        return getattr(obj, attr)
    except Exception:
        return default


def fw_string(odrv):
    v = [read(odrv, f"fw_version_{k}") for k in ("major", "minor", "revision")]
    return ".".join(str(x) for x in v) if None not in v else "?"


def state_name(state):
    try:
        import odrive.enums as E
        for k, v in vars(E).items():
            if k.startswith("AXIS_STATE_") and v == state:
                return k[len("AXIS_STATE_"):]
    except ImportError:
        pass
    return str(state)


def axis_errors(axis):
    """에러가 있는 모듈만 ['motor: 0x.. NAME | NAME', ...] 형태로."""
    try:
        import odrive.enums as E
        table = vars(E)
    except ImportError:
        table = {}
    out = []
    for name, path, prefix in (("axis", "error", "AXIS_ERROR_"), ("motor", "motor.error", "MOTOR_ERROR_"),
                               ("encoder", "encoder.error", "ENCODER_ERROR_"),
                               ("controller", "controller.error", "CONTROLLER_ERROR_")):
        code = read(axis, path, 0) or 0
        if code:
            names = [k[len(prefix):] for k, v in table.items()
                     if k.startswith(prefix) and isinstance(v, int) and v and code & v]
            out.append(f"{name}: 0x{code:X} {' | '.join(names) or '?'}")
    return out


def clear_errors(axis):
    for path in ("error", "motor.error", "encoder.error", "controller.error"):
        try:
            obj, attr = resolve(axis, path)
            setattr(obj, attr, 0)
        except Exception:
            pass


class RamSession:
    """with 블록 안에서만 설정을 바꾼다.

    끝나면 (예외·Ctrl+C 포함) 토크 명령 0, 축 IDLE, 바꾼 설정을 원래 값으로 되돌린다.
    """

    def __init__(self, odrv, axis="axis0", restore=True):
        self.odrv = odrv
        self.axis = getattr(odrv, axis)
        self.restore = restore
        self.orig = {}

    def set(self, path, value):
        obj, attr = resolve(self.axis, path)
        old = getattr(obj, attr)
        if path not in self.orig:
            self.orig[path] = old
        if old != value:
            setattr(obj, attr, value)
            print(f"  RAM: {path} {old:g} -> {value:g}" if isinstance(value, (int, float)) else
                  f"  RAM: {path} {old} -> {value}")

    def remember(self, path):
        """펌웨어가 바꿀 값(예: 모터 보정 결과)을 지금 값으로 기록해 두었다가 끝날 때 되돌린다."""
        if path not in self.orig:
            obj, attr = resolve(self.axis, path)
            self.orig[path] = getattr(obj, attr)

    def set_gains(self, motor, which):
        """which: asis | train | course | default. asis면 바꾸지 않음."""
        if which == "asis":
            return
        pg, vg, ig = params.gains_for(motor, which)
        self.set("controller.config.vel_integrator_gain", ig)
        self.set("controller.config.pos_gain", pg)
        self.set("controller.config.vel_gain", vg)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.axis.controller.input_torque = 0.0
            self.axis.controller.input_vel = 0.0  # 남으면 다음 위치 제어에서 속도 피드포워드로 작용
            self.axis.requested_state = STATE_IDLE
            time.sleep(0.1)
            print("축 IDLE (힘 풀림).")
        except Exception as e:
            print(f"!! IDLE 전환 실패: {e}  -> 전원을 차단하세요.")
        if self.restore and self.orig:
            for path, value in reversed(list(self.orig.items())):
                try:
                    obj, attr = resolve(self.axis, path)
                    setattr(obj, attr, value)
                except Exception as e:
                    print(f"!! {path} 원복 실패: {e}")
            print(f"RAM 설정 {len(self.orig)}개를 원래 값으로 되돌렸습니다 (저장은 하지 않음).")
        if exc_type is KeyboardInterrupt:
            print("중단됨.")
            return True
        return False


def enter_closed_loop(axis, timeout=3.0):
    errs = axis_errors(axis)
    if errs:
        print("현재 에러:\n  " + "\n  ".join(errs))
        util.confirm("에러를 지우고 closed loop를 시도할까요?")
        clear_errors(axis)
    axis.requested_state = STATE_CLOSED_LOOP
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.05)
        if axis.current_state == STATE_CLOSED_LOOP:
            time.sleep(0.2)
            if axis.current_state == STATE_CLOSED_LOOP:
                return
        if read(axis, "error", 0):
            break
    raise RuntimeError(f"closed loop 진입 실패 (state {state_name(axis.current_state)}): {axis_errors(axis)}\n"
                       "보정이 안 되어 있다는 에러면 전체 보정(state 3)이 필요합니다. 먼저 HW팀에 확인하세요.")


def check_alive(axis):
    if axis.current_state != STATE_CLOSED_LOOP:
        raise RuntimeError(f"축이 closed loop에서 빠졌습니다 (state {state_name(axis.current_state)}): "
                           f"{axis_errors(axis)}")


def wait_settled(axis, timeout=3.0, vel_tol=0.02, hold=0.25):
    """|vel| < vel_tol [turn/s]가 hold초 이어지면 True."""
    t0 = time.time()
    calm = None
    while time.time() - t0 < timeout:
        check_alive(axis)
        now = time.time()
        if abs(axis.encoder.vel_estimate) < vel_tol:
            calm = calm or now
            if now - calm >= hold:
                return True
        else:
            calm = None
        time.sleep(0.01)
    return False


def average(axis, duration, with_iq=True):
    """duration초 동안 pos_estimate [turn]와 Iq_measured [A] 평균."""
    ps, iqs = [], []
    t_end = time.time() + duration
    while time.time() < t_end:
        ps.append(axis.encoder.pos_estimate)
        if with_iq:
            iqs.append(axis.motor.current_control.Iq_measured)
        time.sleep(0.001)
    return {
        "pos": statistics.fmean(ps),
        "pos_min": min(ps),
        "pos_max": max(ps),
        "iq": statistics.fmean(iqs) if iqs else float("nan"),
        "n": len(ps),
    }


def max_speed(vbus, torque_constant):
    """역기전력이 쓸 수 있는 전압(0.8 x sqrt3/2 x 2/3 x Vbus)의 75%가 되는 모터 속도 [turn/s]. 12 V면 약 14.8."""
    v_avail = 0.8 * math.sqrt(3) / 2 * 2 / 3 * vbus
    emf_per_turn_s = torque_constant / 1.5 * 2 * math.pi  # 상 역기전력 진폭 [V] / [turn/s]
    return 0.75 * v_avail / emf_per_turn_s


def fet_temperature(axis):
    """보드 FET 온도 [degC]. 읽을 수 없으면 None."""
    v = read(axis, "fet_thermistor.temperature")
    return v if isinstance(v, (int, float)) and math.isfinite(v) else None


def torque_sign(axis):
    """Iq_measured에 곱하면 pos_estimate(+) 방향 토크 부호가 되는 값.

    펌웨어가 전류 명령에 motor.config.direction을 곱하므로(motor.cpp `current_setpoint *= config_.direction`)
    엔코더 보정에서 direction = -1이 나온 보드는 Iq 부호가 실제 토크와 반대다 (ID01·ID02 모두 -1).
    """
    return -1.0 if read(axis, "motor.config.direction", 1) == -1 else 1.0


def firmware_from_boot_pos(pos_estimate):
    """부팅 직후(움직이기 전) pos_estimate로 펌웨어 추정.

    코스 펌웨어는 절대 엔코더 값으로 위치를 시작해 0~1 turn, MKS 출고 펌웨어는 0에서 시작한 PLL이
    가까운 쪽으로 따라가 -0.5~0.5 turn이 된다 (encoder.cpp abs_spi_cb / update). 0~0.5는 둘 다 가능.
    """
    if 0.5 < pos_estimate < 1.0:
        return "course", "코스 펌웨어 (부팅 위치가 0.5~1 turn)"
    if -0.5 <= pos_estimate < 0.0:
        return "stock", "MKS 출고 펌웨어로 보임 (부팅 위치가 음수)"
    return "unknown", "이 자세에서는 구분 불가 (0~0.5 turn). 출력축을 약 12도 돌려 두고 전원을 껐다 켜서 다시 확인"


def drive_snapshot(axis):
    """모델 계산에 필요한 드라이버 설정."""
    c, m = axis.controller.config, axis.motor.config
    return {
        "pos_gain": c.pos_gain,
        "vel_gain": c.vel_gain,
        "vel_integrator_gain": c.vel_integrator_gain,
        "vel_limit": c.vel_limit,
        "current_lim": m.current_lim,
        "torque_constant": m.torque_constant,
        "torque_lim": read(m, "torque_lim", float("inf")),
        "control_mode": c.control_mode,
        "input_mode": c.input_mode,
    }
