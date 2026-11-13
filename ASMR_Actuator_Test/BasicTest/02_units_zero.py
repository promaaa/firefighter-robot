"""5단계: 단위·방향·영점.

    python 02_units_zero.py --id 01 --motor 6512 --boot   # 부팅 직후 위치만 기록 (모터 안 움직임)
    python 02_units_zero.py --id 01 --motor 6512          # 출력 +10°, +90° 왕복으로 감속비·방향·연속성 확인

이동 시험은 RAM에서 current_lim 5 A, vel_limit 5 turn/s로 낮추고 시작하며, 끝나면 IDLE + 원래 값 복원.
--boot는 전원을 껐다 켤 때마다, 축을 움직이기 전에 한 번씩 실행. 코스 펌웨어는 부팅 직후 pos_estimate가
0~1 turn(출력 0~24°), MKS 출고 펌웨어는 -0.5~0.5 turn에서 시작하므로 펌웨어 구분에도 쓴다.
출력축 절대 위치는 전원 켤 때 자세로 정해진다.
"""
import argparse
import time

from bench import odrv_usb, params, util


def boot_record(odrv, args, out_dir):
    axis = odrv.axis0
    pos = axis.encoder.pos_estimate
    pos_abs = odrv_usb.read(axis, "encoder.pos_abs")
    fw, verdict = odrv_usb.firmware_from_boot_pos(pos)
    print(f"pos_estimate {pos:.4f} turn = 출력 {params.turn_to_out_deg(pos):.2f} deg, pos_abs {pos_abs}, "
          f"state {odrv_usb.state_name(axis.current_state)}")
    print(f"펌웨어 추정: {verdict}  (부팅 후 축을 움직이지 않았을 때만 유효)")
    note = util.ask("출력축 자세 메모 (예: 테이프 영점, +30deg 등. 비워도 됨): ")
    util.append_csv(out_dir / "boot_pos.csv", {
        "time": time.strftime("%Y-%m-%d %H:%M:%S"), "pos_turn": f"{pos:.5f}",
        "out_deg": f"{params.turn_to_out_deg(pos):.3f}", "pos_abs": pos_abs, "fw_guess": fw, "note": note})


def move_and_measure(axis, target, timeout=8.0):
    """목표로 보내면서 pos 연속성(샘플 간 최대 점프)을 본다. 반환 (정착 위치, 최대 점프 turn)."""
    axis.controller.input_pos = target
    prev = axis.encoder.pos_estimate
    jump = 0.0
    t0 = time.time()
    while time.time() - t0 < timeout:
        odrv_usb.check_alive(axis)
        p = axis.encoder.pos_estimate
        jump = max(jump, abs(p - prev))
        prev = p
        if abs(p - target) < 0.01 and abs(axis.encoder.vel_estimate) < 0.02:
            break
        time.sleep(0.001)
    odrv_usb.wait_settled(axis, timeout=2.0)
    return odrv_usb.average(axis, 0.3, with_iq=False)["pos"], jump


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--boot", action="store_true", help="부팅 직후 위치만 기록")
    ap.add_argument("--small", type=float, default=10.0, help="첫 이동 [출력 deg]")
    ap.add_argument("--big", type=float, default=90.0, help="두 번째 이동 [출력 deg]")
    ap.add_argument("--current-lim", type=float, default=5.0)
    ap.add_argument("--vel-limit", type=float, default=5.0, help="turn/s 모터축 (5 = 출력 120 deg/s)")
    args = ap.parse_args()

    out_dir = util.data_dir(args.id)
    odrv = odrv_usb.connect(args)
    if args.boot:
        boot_record(odrv, args, out_dir)
        return

    axis = odrv.axis0
    print("\n정지 방법: Ctrl+C (스크립트가 IDLE로 전환) 또는 전원 차단.")
    print(f"출력축이 +{args.small:g} deg, +{args.big:g} deg 까지 돕니다. 암이 걸리지 않게 주변을 비우세요.")
    util.confirm("준비되면")

    results, p0 = [], None
    with odrv_usb.RamSession(odrv) as s:
        s.set("motor.config.current_lim", min(args.current_lim, axis.motor.config.current_lim))
        s.set("controller.config.vel_limit", args.vel_limit)
        s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
        s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
        odrv_usb.enter_closed_loop(axis)
        p0 = odrv_usb.average(axis, 0.3, with_iq=False)["pos"]
        print(f"시작 위치 p0 = {p0:.4f} turn. 출력축 현재 자세를 테이프로 표시해 두세요.")

        for deg in (args.small, 0.0, args.big, 0.0):
            target = p0 + params.out_deg_to_turn(deg)
            util.confirm(f"\n출력 {deg:+g} deg (모터 {target - p0:+.4f} turn)로 이동")
            pos, jump = move_and_measure(axis, target)
            enc_deg = params.turn_to_out_deg(pos - p0)
            print(f"  명령 {deg:+.2f} deg -> 엔코더 {enc_deg:+.3f} deg (오차 {enc_deg - deg:+.3f}), "
                  f"샘플 간 최대 점프 {jump:.4f} turn" + ("  !! 위치 값이 끊김" if jump > 0.3 else ""))
            row = {"cmd_deg": deg, "enc_deg": round(enc_deg, 4), "max_jump_turn": round(jump, 5)}
            if deg != 0.0:
                row["measured_deg"] = util.ask_float("  각도기/레이저로 잰 실제 각도 [deg] (비우면 생략): ")
                if deg == args.small:
                    row["direction"] = util.ask("  출력면에서 볼 때 + 방향은? (cw/ccw): ")
            results.append(row)

    meas = [r for r in results if r.get("measured_deg")]
    for r in meas:
        print(f"  {r['cmd_deg']:+g} deg 명령: 실측/명령 = {r['measured_deg'] / r['cmd_deg']:.3f} "
              f"(1.000이면 감속비 15와 단위 변환이 맞음)")
    util.save_json(out_dir / f"units_{util.stamp()}.json",
                   {"id": args.id, "motor": args.motor, "p0_turn": p0, "results": results})


if __name__ == "__main__":
    util.run(main)
