"""7단계: 토크-전류, 마찰, 백래시.

    python 04_torque_friction.py torque   --id 01 --motor 6512   # 토크 모드 0.05/0.1/0.2 Nm, 저울 값 입력
    python 04_torque_friction.py friction --id 01 --motor 6512   # IDLE에서 움직이기 시작하는 힘 기록
    python 04_torque_friction.py backlash --id 01 --motor 6512   # 위치 유지 중 손으로 흔들며 엔코더 범위 기록

torque: 출력 암을 저울에 대고 움직이지 않게 고정한 뒤 시작. 토크 모드는 막는 게 없으면 계속 돈다.
        스크립트는 vel_limit을 2 turn/s로 낮추고, 출력이 --max-move 이상 움직이면 바로 멈춘다.
"""
import argparse
import math
import time

from bench import odrv_usb, params, util


def torque_test(odrv, args, out_dir):
    m, axis = params.MOTORS[args.motor], odrv.axis0
    cmds = [float(x) for x in args.torques.split(",")]
    print("\n!! 출력 암을 먼저 고정하세요. 토크 모드에서는 막는 것이 없으면 속도 제한까지 계속 돕니다.")
    print(f"   +방향 토크를 겁니다 (부호 {args.sign:+d}). 암이 저울을 누르는(당기는) 방향이 맞는지 확인.")
    print(f"   명령(모터축) {cmds} Nm. 예상 출력 = 명령 x 15 x (kt/torque_constant) x 기어효율")
    util.confirm("암을 고정했으면")

    rows = []
    with odrv_usb.RamSession(odrv) as s:
        axis.controller.input_torque = 0.0
        s.set("controller.config.vel_limit", args.vel_limit)
        s.set("controller.config.enable_current_mode_vel_limit", True)
        s.set("controller.config.control_mode", odrv_usb.CONTROL_TORQUE)
        tc = axis.motor.config.torque_constant
        odrv_usb.enter_closed_loop(axis)
        p0 = axis.encoder.pos_estimate
        max_move = params.out_deg_to_turn(args.max_move)

        for cmd in cmds:
            util.confirm(f"\n토크 {cmd:g} Nm (모터축) 걸기")
            if args.sim:
                odrv.ext_torque = -args.sign * cmd * params.GEAR_RATIO * m["kt"] / tc  # 저울이 받쳐 주는 셈
            axis.controller.input_torque = args.sign * cmd
            t_end = time.time() + 1.0
            while time.time() < t_end:
                odrv_usb.check_alive(axis)
                if abs(axis.encoder.pos_estimate - p0) > max_move:
                    axis.controller.input_torque = 0.0
                    raise RuntimeError(f"출력이 {args.max_move:g} deg 이상 움직였습니다. 암 고정을 확인하세요.")
                time.sleep(0.005)
            avg = odrv_usb.average(axis, 0.5)
            kg = util.ask_float(f"  Iq {avg['iq']:+.2f} A. 저울 값 [kg] (비우면 생략): ")
            axis.controller.input_torque = 0.0
            if args.sim:
                odrv.ext_torque = 0.0
            expected = cmd * params.GEAR_RATIO * m["kt"] / tc
            row = {"cmd_Nm": cmd, "iq_A": round(avg["iq"], 4), "expected_out_Nm": round(expected, 4)}
            if kg is not None:
                out = kg * params.G * args.arm
                row.update(scale_kg=kg, out_Nm=round(out, 4), ratio=round(out / expected, 4),
                           kt_x_eff=round(out / (abs(avg["iq"]) * params.GEAR_RATIO), 5))
                print(f"  출력 {out:.3f} Nm / 예상 {expected:.3f} Nm = {out / expected:.3f}  "
                      f"(출력 / (Iq x 15) = {row['kt_x_eff']:.4f} Nm/A, 실측 kt {m['kt']} x 효율)")
            rows.append(row)
            time.sleep(0.3)

    for r in rows:
        util.append_csv(out_dir / "torque.csv", {"time": util.stamp(), "arm_m": args.arm, **r})
    print(f"학습 토크 한계: {m['torque_limit']} Nm (18 A x kt x 15, 효율 100% 가정). 큰 토크는 기어 허용치를 들은 뒤에만.")


def friction_test(odrv, args, out_dir):
    m, axis = params.MOTORS[args.motor], odrv.axis0
    axis.requested_state = odrv_usb.STATE_IDLE
    model_g = m["coulomb"] / args.arm / params.G * 1000
    print(f"\nIDLE(힘 풀림). 암 끝({args.arm:g} m)을 저울로 천천히 당겨 움직이기 시작하는 값을 읽으세요.")
    print(f"학습 모델 쿨롱 마찰 {m['coulomb']} Nm = {args.arm:g} m에서 약 {model_g:.0f} g")
    print("한 바퀴 돌며 여러 위치에서 잽니다. 손으로 빠르게 돌리면 과전압 에러가 날 수 있습니다. 끝내려면 q")
    vals = []
    while True:
        s = util.ask("\nEnter=현재 위치 기록 후 측정, q=끝: ")
        if s.lower() == "q":
            break
        pos = axis.encoder.pos_estimate
        g = util.ask_float(f"  위치 {params.turn_to_out_deg(pos):.1f} deg. 움직이기 시작한 값 [g]: ")
        if g is None:
            continue
        tau = g / 1000 * params.G * args.arm
        vals.append(tau)
        print(f"  {tau:.3f} Nm (모델 {m['coulomb']} Nm의 x{tau / m['coulomb']:.2f})")
        util.append_csv(out_dir / "friction.csv", {"time": util.stamp(), "pos_turn": round(pos, 4),
                                                   "arm_m": args.arm, "grams": g, "tau_Nm": round(tau, 4)})
    if vals:
        print(f"\n평균 {sum(vals) / len(vals):.3f} Nm, 범위 {min(vals):.3f}-{max(vals):.3f} Nm (n={len(vals)})")


def backlash_test(odrv, args, out_dir):
    axis = odrv.axis0
    with odrv_usb.RamSession(odrv) as s:
        s.set_gains(args.motor, args.gains)
        s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
        s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
        util.confirm("위치 유지 시작")
        odrv_usb.enter_closed_loop(axis)
        util.confirm(f"Enter 후 {args.duration:g}초 동안 암을 양방향으로 살살 흔드세요")
        ps = []
        t_end = time.time() + args.duration
        while time.time() < t_end:
            odrv_usb.check_alive(axis)
            if args.sim:
                odrv.ext_torque = 1.0 * math.sin(2 * math.pi * 1.0 * time.time())
            ps.append(axis.encoder.pos_estimate)
            time.sleep(0.002)
        if args.sim:
            odrv.ext_torque = 0.0
    enc = params.turn_to_out_deg(max(ps) - min(ps))
    print(f"\n엔코더 범위 {enc:.3f} deg (출력 환산, 모터축 엔코더라 백래시는 안 보임)")
    cm = util.ask_float(f"레이저가 벽({args.wall:g} m)에서 움직인 전체 폭 [cm] (비우면 생략): ")
    row = {"time": util.stamp(), "gains": args.gains, "enc_p2p_deg": round(enc, 4)}
    if cm:
        laser = math.degrees(math.atan(cm / 100.0 / args.wall))
        row.update(laser_p2p_deg=round(laser, 4), backlash_deg=round(laser - enc, 4))
        print(f"레이저 {laser:.3f} deg - 엔코더 {enc:.3f} deg = 백래시+기어 변형 {laser - enc:.3f} deg")
    util.append_csv(out_dir / "backlash.csv", row)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=["torque", "friction", "backlash"])
    odrv_usb.add_args(ap)
    ap.add_argument("--arm", type=float, default=0.2, help="회전축에서 저울/힘 작용점까지 [m]")
    ap.add_argument("--torques", default="0.05,0.1,0.2", help="모터축 토크 명령 목록 [Nm]")
    ap.add_argument("--sign", type=int, default=1, choices=[1, -1], help="토크 방향")
    ap.add_argument("--vel-limit", type=float, default=2.0, help="토크 모드 속도 제한 [turn/s]")
    ap.add_argument("--max-move", type=float, default=15.0, help="이만큼 [출력 deg] 움직이면 중단")
    ap.add_argument("--gains", default="asis", choices=["asis", "train", "course", "default"], help="backlash용")
    ap.add_argument("--duration", type=float, default=10.0, help="backlash 기록 시간 [s]")
    ap.add_argument("--wall", type=float, default=2.0, help="레이저-벽 거리 [m]")
    args = ap.parse_args()

    out_dir = util.data_dir(args.id)
    odrv = odrv_usb.connect(args)
    {"torque": torque_test, "friction": friction_test, "backlash": backlash_test}[args.test](odrv, args, out_dir)


if __name__ == "__main__":
    util.run(main)
