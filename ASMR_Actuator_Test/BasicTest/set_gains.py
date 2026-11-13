"""드라이버 게인을 RAM에서만 바꾸고 그대로 둔다 (CAN 시험 전에 씀). 저장하지 않으므로 재부팅하면 원래대로.

    python set_gains.py --motor 6512 --gains train      # 학습 등가: pos_gain 15.71, vel_gain 0.160, 적분 0
    python set_gains.py --motor 5010 --gains train      # 발목: 44.88 / 0.01366
    python set_gains.py --motor 6512 --show             # 바꾸지 않고 현재 값과 관절축 환산만

--gains train/course는 input_mode 1(PASSTHROUGH), vel_limit 50도 같이 맞춘다. current_lim은 --current-lim 줄 때만.
"""
import argparse

from bench import odrv_usb, params, util


def show(axis, motor):
    s = odrv_usb.drive_snapshot(axis)
    m = params.MOTORS[motor]
    kp, kd = params.joint_gains(s["pos_gain"], s["vel_gain"], s["torque_constant"], m["kt"])
    print(f"  pos_gain {s['pos_gain']:g}, vel_gain {s['vel_gain']:g}, 적분 {s['vel_integrator_gain']:g}, "
          f"vel_limit {s['vel_limit']:g}, current_lim {s['current_lim']:g}, torque_constant {s['torque_constant']:g}, "
          f"control_mode {s['control_mode']}, input_mode {s['input_mode']}")
    print(f"  관절축 Kp {kp:.2f} (학습 {m['kp']}), Kd {kd:.3f} (학습 {m['kd']})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--gains", choices=["train", "course", "default"])
    ap.add_argument("--current-lim", type=float, default=None)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    odrv = odrv_usb.connect(args)
    axis = odrv.axis0
    print("현재:")
    show(axis, args.motor)
    if args.show or not args.gains:
        return
    if axis.current_state == odrv_usb.STATE_CLOSED_LOOP:
        util.confirm("축이 closed loop 상태입니다. 게인을 바로 바꿀까요?")
    s = odrv_usb.RamSession(odrv, restore=False)
    s.set_gains(args.motor, args.gains)
    s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
    s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
    s.set("controller.config.vel_limit", params.VEL_LIMIT)
    if args.current_lim:
        s.set("motor.config.current_lim", args.current_lim)
    print("바꾼 뒤 (RAM, 저장 안 함):")
    show(axis, args.motor)


if __name__ == "__main__":
    util.run(main)
