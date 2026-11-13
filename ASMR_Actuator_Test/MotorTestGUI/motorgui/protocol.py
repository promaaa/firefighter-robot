"""ODrive fw 0.5.1 CAN Simple: 메시지 해석과 상태·에러 이름.

명령 ID는 BasicTest/bench/can_simple.py와 같다. 에러 비트 이름은 odrive 0.5.1.post0의 enums.py에서 옮겼다
(odrive 패키지를 import하면 USB 백엔드까지 올라와서 CAN 전용인 이 프로그램에서는 쓰지 않는다).
"""
import struct

from bench import can_simple as cs  # noqa: F401  (명령 ID 재사용)

STATE_IDLE = 1
STATE_CLOSED_LOOP = 8
CONTROL_POSITION = 3
INPUT_PASSTHROUGH = 1

STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    5: "SENSORLESS_CONTROL", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND", 11: "HOMING",
}
STATE_KO = {1: "힘 풀림 (IDLE)", 8: "위치 제어 (CLOSED_LOOP)", 3: "전체 보정 중", 4: "모터 보정 중",
            7: "엔코더 보정 중", 2: "시작 중"}

AXIS_ERRORS = {
    0x1: "INVALID_STATE", 0x2: "DC_BUS_UNDER_VOLTAGE", 0x4: "DC_BUS_OVER_VOLTAGE",
    0x8: "CURRENT_MEASUREMENT_TIMEOUT", 0x10: "BRAKE_RESISTOR_DISARMED", 0x20: "MOTOR_DISARMED",
    0x40: "MOTOR_FAILED", 0x80: "SENSORLESS_ESTIMATOR_FAILED", 0x100: "ENCODER_FAILED",
    0x200: "CONTROLLER_FAILED", 0x400: "POS_CTRL_DURING_SENSORLESS", 0x800: "WATCHDOG_TIMER_EXPIRED",
    0x1000: "MIN_ENDSTOP_PRESSED", 0x2000: "MAX_ENDSTOP_PRESSED", 0x4000: "ESTOP_REQUESTED",
    0x20000: "HOMING_WITHOUT_ENDSTOP", 0x40000: "OVER_TEMP",
}
MOTOR_ERRORS = {
    0x1: "PHASE_RESISTANCE_OUT_OF_RANGE", 0x2: "PHASE_INDUCTANCE_OUT_OF_RANGE", 0x4: "ADC_FAILED",
    0x8: "DRV_FAULT", 0x10: "CONTROL_DEADLINE_MISSED", 0x20: "NOT_IMPLEMENTED_MOTOR_TYPE",
    0x40: "BRAKE_CURRENT_OUT_OF_RANGE", 0x80: "MODULATION_MAGNITUDE", 0x100: "BRAKE_DEADTIME_VIOLATION",
    0x200: "UNEXPECTED_TIMER_CALLBACK", 0x400: "CURRENT_SENSE_SATURATION", 0x1000: "CURRENT_LIMIT_VIOLATION",
    0x2000: "BRAKE_DUTY_CYCLE_NAN", 0x4000: "DC_BUS_OVER_REGEN_CURRENT", 0x8000: "DC_BUS_OVER_CURRENT",
}
ENCODER_ERRORS = {
    0x1: "UNSTABLE_GAIN", 0x2: "CPR_POLEPAIRS_MISMATCH", 0x4: "NO_RESPONSE", 0x8: "UNSUPPORTED_ENCODER_MODE",
    0x10: "ILLEGAL_HALL_STATE", 0x20: "INDEX_NOT_FOUND_YET", 0x40: "ABS_SPI_TIMEOUT", 0x80: "ABS_SPI_COM_FAIL",
    0x100: "ABS_SPI_NOT_READY",
}


def state_text(state):
    if state is None:
        return "?"
    return STATE_KO.get(state, STATE_NAMES.get(state, str(state)))


def decode_bits(code, table):
    if not code:
        return ""
    names = [n for bit, n in table.items() if code & bit]
    rest = code & ~sum(b for b in table if code & b)
    if rest:
        names.append(f"0x{rest:X}")
    return " | ".join(names)


def parse_heartbeat(data):
    """(axis_error, axis_state). 0.5.1은 error u32 + state, 이후 버전은 state u8 + 플래그라 state는 byte 4만 본다."""
    err = struct.unpack_from("<I", data, 0)[0]
    state = data[4] if len(data) > 4 else 0
    return err, state


def parse_two_floats(data):
    return struct.unpack_from("<ff", data, 0)


def parse_float(data):
    return struct.unpack_from("<f", data, 0)[0]


def parse_motor_error(data):
    """0.5.1 motor.error는 64비트일 수 있어 8바이트를 다 읽는다."""
    return struct.unpack_from("<Q", data, 0)[0] if len(data) >= 8 else struct.unpack_from("<I", data, 0)[0]


def parse_u32(data):
    return struct.unpack_from("<I", data, 0)[0]


def pack_input_pos(pos, vel_ff=0.0, torque_ff=0.0):
    v = max(-32768, min(32767, int(round(vel_ff * 1000))))
    t = max(-32768, min(32767, int(round(torque_ff * 1000))))
    return struct.pack("<fhh", pos, v, t)


def frame_bits(dlc, rtr=False):
    """표준 프레임 대략 비트 수 (스터핑 약 15% 포함). 버스 점유율 추정용."""
    payload = 0 if rtr else dlc * 8
    return int((47 + payload) * 1.15)
