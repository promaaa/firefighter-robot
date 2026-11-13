"""센서 모드: 현재 각도를 한 줄에 덮어쓰며 보여 준다. Ctrl+C로 끝낸다.

    python sensor_mode.py                          # 힘 뺀(IDLE) 상태, 손으로 돌려 보기
    python sensor_mode.py --rate 50                # 화면 갱신을 더 빠르게
    python sensor_mode.py --hold                   # 위치 제어로 지금 각도를 붙잡고 부하 걸어 보기
    python sensor_mode.py --hold --current-lim 12  # 더 센 힘까지 (1 A = 출력 약 1 Nm)
    python sensor_mode.py --hold --gains train --motor 6512   # 학습 등가 게인으로

--hold: 지금 각도를 목표로 위치 제어(PASSTHROUGH)에 들어가고, 밀린 각도·전류·출력 토크를 보여 준다.
        전류 한계와 게인은 RAM에서만 바꾸고 끝나면(Ctrl+C 포함) IDLE로 풀고 원래 값으로 되돌린다.
출력 각도 = pos_estimate [turn] x 24 deg (감속비 15). 코스 펌웨어는 부팅 직후 0~24 deg에서 시작한다.
"""
import argparse
import time

from bench import odrv_usb, params, util


def show(line):
    print("\r" + line, end="", flush=True)


def watch(odrv, dt):
    axis = odrv.axis0
    if axis.current_state != odrv_usb.STATE_IDLE:
        axis.requested_state = odrv_usb.STATE_IDLE
        print("축을 IDLE로 바꿨습니다 (힘 풀림).")
    p0 = axis.encoder.pos_estimate
    print("손으로 출력축을 천천히 돌려 보세요. 끝내려면 Ctrl+C\n")
    try:
        while True:
            p = axis.encoder.pos_estimate
            show(f"출력 {params.turn_to_out_deg(p):+9.2f} deg   "
                 f"시작 기준 {params.turn_to_out_deg(p - p0):+9.2f} deg   "
                 f"모터 {p:+9.4f} turn")
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n종료.")


def hold(odrv, a, dt):
    axis = odrv.axis0
    kt = axis.motor.config.torque_constant
    print(f"\n지금 각도를 위치 제어로 붙잡습니다. 전류 한계 {a.current_lim:g} A "
          f"(출력 최대 약 {a.current_lim * kt * params.GEAR_RATIO:.1f} Nm). 손을 놓으면 목표 각도로 돌아갑니다.")
    kt *= odrv_usb.torque_sign(axis)  # 토크 부호를 각도(+) 방향 기준으로
    util.confirm("시작할까요?")
    with odrv_usb.RamSession(odrv) as s:
        s.set("motor.config.current_lim", a.current_lim)
        s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
        s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
        s.set_gains(a.motor, a.gains)
        c = axis.controller.config
        kp, kd = params.joint_gains(c.pos_gain, c.vel_gain)
        print(f"  게인 pos_gain {c.pos_gain:g}, vel_gain {c.vel_gain:g} (관절 Kp {kp:.1f} Nm/rad, Kd {kd:.2f} Nm*s/rad)")
        target = axis.encoder.pos_estimate
        axis.controller.input_pos = target
        odrv_usb.enter_closed_loop(axis)
        print("위치 유지 중. 출력축을 밀거나 추를 걸어 보세요. 끝내려면 Ctrl+C\n")
        try:
            while True:
                odrv_usb.check_alive(axis)
                p = axis.encoder.pos_estimate
                iq = axis.motor.current_control.Iq_measured
                vbus = odrv.vbus_voltage
                show(f"목표 {params.turn_to_out_deg(target):+8.2f} deg   "
                     f"실제 {params.turn_to_out_deg(p):+8.2f} deg   "
                     f"밀린 각 {params.turn_to_out_deg(p - target):+7.2f} deg   "
                     f"전류 {iq:+6.2f} A   토크 약 {iq * kt * params.GEAR_RATIO:+6.2f} Nm   "
                     f"Vbus {vbus:5.2f} V")
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            print()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap, motor_required=False)
    ap.add_argument("--rate", type=float, default=20.0, help="화면 갱신 횟수 [Hz] (기본 20)")
    ap.add_argument("--hold", action="store_true", help="위치 제어로 지금 각도를 붙잡고 밀린 각도·전류·토크 표시")
    ap.add_argument("--current-lim", type=float, default=6.0,
                    help="--hold 전류 한계 [A], RAM만 (기본 6 = 출력 약 6 Nm, 최대 18)")
    ap.add_argument("--gains", choices=["asis", "course", "train"], default="asis",
                    help="--hold 게인: asis(보드에 저장된 값, 기본) | course | train (train·course는 --motor 필요)")
    a = ap.parse_args()
    if not 0 < a.current_lim <= params.CURRENT_LIM:
        ap.error(f"--current-lim은 0~{params.CURRENT_LIM:g} A")
    if a.gains != "asis" and not a.motor:
        ap.error("--gains train/course 는 --motor 6512 또는 5010 이 필요합니다")

    odrv = odrv_usb.connect(a)
    dt = 1.0 / a.rate
    if a.hold:
        hold(odrv, a, dt)
    else:
        watch(odrv, dt)


if __name__ == "__main__":
    util.run(main)
