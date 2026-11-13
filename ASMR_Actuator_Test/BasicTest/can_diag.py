"""CAN 진단: 한 node의 에러를 CAN으로 읽고, 원하면 엔코더 오프셋 보정(AXIS_STATE 7)을 CAN으로 돌린다.

    python can_diag.py --channel COM16 --node 7                     # 읽기만 (상태, 에러, 위치, Vbus)
    python can_diag.py --channel COM16 --node 7 --calibrate-offset  # 엔코더 오프셋 보정 (출력축이 앞뒤로 약 14도 돎)

CAN 보정값은 저장되지 않는다 (fw 0.5.1 CAN Simple에는 저장 명령이 없음). 드라이버 전원을 끄면 원래 값으로 돌아간다.
"""
import argparse
import time

from bench import can_simple

try:
    from odrive import enums
except Exception:  # noqa: BLE001
    enums = None

STATE = {0: "UNDEFINED", 1: "IDLE", 2: "STARTUP", 3: "FULL_CAL", 4: "MOTOR_CAL", 6: "ENC_INDEX",
         7: "ENC_OFFSET_CAL", 8: "CLOSED_LOOP", 9: "LOCKIN", 10: "ENC_DIR_FIND", 11: "HOMING"}


def decode(prefix, value):
    if not value:
        return "없음"
    if enums is None:
        return hex(value)
    names = [n[len(prefix):] for n in dir(enums)
             if n.startswith(prefix) and n != prefix + "NONE"
             and isinstance(getattr(enums, n), int) and getattr(enums, n) and value & getattr(enums, n)]
    return f"0x{value:X} ({', '.join(names) or '?'})"


def report(node, tag):
    hb = node.wait_heartbeat(0.5)
    if hb is None:
        print(f"[{tag}] heartbeat 없음")
        return None
    err, st = hb
    pos, vel = node.get_encoder_estimates()
    me, ee = node.get_motor_error(), node.get_encoder_error()
    iq_set, iq = node.get_iq()
    print(f"[{tag}] 상태 {STATE.get(st, st)} | axis {decode('AXIS_ERROR_', err)} | motor {decode('MOTOR_ERROR_', me)}"
          f" | encoder {decode('ENCODER_ERROR_', ee)}")
    print(f"[{tag}] pos {pos:.4f} turn (출력 {pos * 24:.2f} deg), vel {vel:.3f} turn/s, Iq {iq:.2f} A,"
          f" Vbus {node.get_vbus():.2f} V")
    return hb


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", required=True)
    ap.add_argument("--node", type=int, required=True)
    ap.add_argument("--interface", default="slcan")
    ap.add_argument("--bitrate", type=int, default=500000)
    ap.add_argument("--calibrate-offset", action="store_true")
    ap.add_argument("--timeout", type=float, default=30.0)
    a = ap.parse_args()
    import can
    bus = can.Bus(interface=a.interface, channel=a.channel, bitrate=a.bitrate)
    node = can_simple.Node(bus, a.node, timeout=0.1)
    try:
        hb = report(node, "전")
        if hb is None or not a.calibrate_offset:
            return
        if hb[1] != 1:
            print("IDLE이 아니라서 보정하지 않습니다.")
            return
        node.clear_errors()
        time.sleep(0.1)
        print("엔코더 오프셋 보정 시작 (AXIS_STATE 7)")
        node.set_axis_state(7)
        t0, left_idle, last = time.perf_counter(), False, None
        while time.perf_counter() - t0 < a.timeout:
            hb = node.wait_heartbeat(0.3)
            if hb is None:
                continue
            if hb[1] != last:
                print(f"  {time.perf_counter() - t0:5.1f} s: {STATE.get(hb[1], hb[1])}, axis {decode('AXIS_ERROR_', hb[0])}")
                last = hb[1]
            if hb[1] != 1:
                left_idle = True
            elif left_idle or time.perf_counter() - t0 > 2.0:
                break
        else:
            print("시간 초과 -> IDLE 요청")
            node.set_axis_state(1)
        time.sleep(0.2)
        hb = report(node, "후")
        if hb and not hb[0] and not node.get_motor_error() and not node.get_encoder_error():
            print("결과: 보정 성공 (에러 없음). 전원을 끄기 전까지 이 값으로 동작합니다.")
        else:
            print("결과: 보정 중 에러 -> 위 에러 이름 확인")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
