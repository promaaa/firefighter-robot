"""3·4단계: 아무것도 바꾸기 전에 설정 전체를 백업하고, 핵심 값을 학습 기준과 대조한다.

    python 01_backup_and_check.py --id 01 --motor 6512

- 백업: data/ID01/xdrive_ID01.json  (odrivetool restore-config 파일명 으로 되돌릴 수 있는 형식)
- 대조 결과: data/ID01/check_<시각>.json
- 장치에서 읽기만 한다. 아무것도 쓰지 않는다.
"""
import argparse
import json
import math
import time

from bench import odrv_usb, params, util


def check_rows(motor):
    m = params.MOTORS[motor]
    pg, vg = m["train_gains"]
    cpg, cvg = m["course_gains"]
    a, c, mc, e = "axis0", "axis0.controller.config", "axis0.motor.config", "axis0.encoder.config"
    return [
        # (이름, 경로, 학습과 맞추려면, 설명)
        ("vbus_voltage", "vbus_voltage", None, "V"),
        ("control_mode", f"{c}.control_mode", 3, "3 위치 / 2 속도 / 1 토크"),
        ("input_mode", f"{c}.input_mode", 1, "1 PASSTHROUGH. 3 POS_FILTER, 5 TRAP_TRAJ면 목표가 걸러져 늦게 따라감"),
        ("pos_gain", f"{c}.pos_gain", pg, f"코스 설정 {cpg}, 기본 20"),
        ("vel_gain", f"{c}.vel_gain", vg, f"코스 설정 {cvg}, 기본 0.1667"),
        ("vel_integrator_gain", f"{c}.vel_integrator_gain", 0.0, "학습은 순수 PD. 기본 0.333"),
        ("vel_limit", f"{c}.vel_limit", params.VEL_LIMIT, "turn/s 모터축. 기본 2"),
        ("enable_overspeed_error", f"{c}.enable_overspeed_error", None, ""),
        ("vel_limit_tolerance", f"{c}.vel_limit_tolerance", None, "vel_limit x 이 값을 넘으면 overspeed 에러"),
        ("current_lim", f"{mc}.current_lim", params.CURRENT_LIM, "A. 기본 10"),
        ("torque_constant", f"{mc}.torque_constant", m["kt"], f"실측 kt. 8.27/KV = {8.27 / m['kv']:.4f}, 기본 0.04"),
        ("torque_lim", f"{mc}.torque_lim", None, "Nm 모터축"),
        ("motor.pre_calibrated", f"{mc}.pre_calibrated", True, ""),
        ("pole_pairs", f"{mc}.pole_pairs", 14, ""),
        ("encoder.mode", f"{e}.mode", 257, "257 = SPI 절대 엔코더(AS5047P)"),
        ("encoder.cpr", f"{e}.cpr", 16384, ""),
        ("encoder.bandwidth", f"{e}.bandwidth", 3000.0, "코스 설정 3000, 기본 1000"),
        ("encoder.pre_calibrated", f"{e}.pre_calibrated", True, ""),
        ("can_node_id", f"{a}.config.can_node_id", None, "bench/joints.py 표"),
        ("can.baud_rate", "can.config.baud_rate", params.CAN_BITRATE, "수업 문서 500k, 기본 250k"),
        ("can_heartbeat_rate_ms", f"{a}.config.can_heartbeat_rate_ms", None, ""),
        ("axis1.can_node_id", "axis1.config.can_node_id", 63, "안 쓰는 축"),
        ("axis1.heartbeat_ms", "axis1.config.can_heartbeat_rate_ms", 0, "0 = 끔 (보드마다 같은 ID 63 방지)"),
        ("startup_encoder_cal", f"{a}.config.startup_encoder_offset_calibration", False, "코스 펌웨어는 부팅 보정 불필요"),
        ("startup_closed_loop", f"{a}.config.startup_closed_loop_control", None, ""),
        ("enable_watchdog", f"{a}.config.enable_watchdog", None, "결정 필요"),
        ("watchdog_timeout", f"{a}.config.watchdog_timeout", None, "s"),
        ("brake_resistance", "config.brake_resistance", None, "0이면 브레이크 저항 없음"),
        ("dc_max_negative_current", "config.dc_max_negative_current", None, "A. 회생 전류 한계"),
        ("dc_bus_overvoltage_trip", "config.dc_bus_overvoltage_trip_level", 26.0, "V"),
        ("dc_bus_undervoltage_trip", "config.dc_bus_undervoltage_trip_level", 10.0,
         "V. 12 V SMPS용 (수업 21 V, 모델링 끝나면 올림)"),
    ]


def status(value, expected):
    if value is None:
        return "읽기 실패"
    if expected is None:
        return ""
    if isinstance(expected, bool) or isinstance(value, bool):
        return "OK" if bool(value) == expected else "다름"
    if isinstance(expected, float):
        return "OK" if math.isclose(value, expected, rel_tol=0.02, abs_tol=1e-6) else "다름"
    return "OK" if value == expected else "다름"


def fmt(v):
    if isinstance(v, float):
        return f"{v:.5g}"
    return str(v)


def backup(odrv, act_id, out_dir):
    from odrive.configuration import get_dict
    path = out_dir / f"xdrive_ID{act_id}.json"
    if path.exists():
        path = out_dir / f"xdrive_ID{act_id}_{util.stamp()}.json"
    print("설정 전체 백업 중 (몇 초 걸림)...")
    data = get_dict(odrv, False)  # odrivetool backup-config 와 같은 내용
    with open(path, "w") as f:
        json.dump(data, f, indent=1)
    print(f"백업 저장: {path}")
    print(f"  되돌리기: odrivetool restore-config \"{path}\"  (RAM에만 씀. 저장은 별도)")
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--no-backup", action="store_true", help="백업 생략 (이미 했을 때)")
    args = ap.parse_args()

    out_dir = util.data_dir(args.id)
    odrv = odrv_usb.connect(args)
    axis = odrv.axis0
    m = params.MOTORS[args.motor]

    backup_path = None
    if not args.no_backup:
        if args.sim:
            print("[SIM] 백업 생략")
        else:
            backup_path = backup(odrv, args.id, out_dir)

    fw = odrv_usb.fw_string(odrv)
    print(f"\n== 설정 대조: ID{args.id}, {args.motor} ({m['name']}, {m['joints']}) ==")
    if fw == "0.0.0":
        print("펌웨어 0.0.0: MKS 출고 펌웨어로 보임 (코스 펌웨어 v0.5.1-instant-start.1은 0.5.1로 표시됨, "
              "2026-11-05 확인). 코스 펌웨어를 올리세요")
    else:
        print(f"펌웨어 {fw}  (0.5.1 예상)" + ("" if fw.startswith("0.5.1") else "  !! 다름: odrivetool 버전과 맞는지 확인"))
    print(f"{'항목':24s} {'현재':>12s} {'학습 기준':>12s} {'':6s} 설명")
    rows = []
    for name, path, expected, note in check_rows(args.motor):
        value = odrv_usb.read(odrv, path)
        st = status(value, expected)
        rows.append({"name": name, "path": path, "value": value, "expected": expected, "status": st})
        exp = "" if expected is None else fmt(expected)
        print(f"{name:24s} {fmt(value):>12s} {exp:>12s} {st:6s} {note}")

    snap = odrv_usb.drive_snapshot(axis)
    kp, kd = params.joint_gains(snap["pos_gain"], snap["vel_gain"], snap["torque_constant"], m["kt"])
    zeta = params.damping_ratio(kp, kd, m["armature"])
    t_lim = min(snap["current_lim"] * snap["torque_constant"], snap["torque_lim"]) * m["kt"] / snap[
        "torque_constant"] * params.GEAR_RATIO
    print(f"\n== 지금 드라이버 게인을 관절축으로 환산 (실제 kt {m['kt']} 가정) ==")
    print(f"  Kp {kp:7.2f} Nm/rad     학습 {m['kp']:6.2f}  -> x{kp / m['kp']:.2f}")
    print(f"  Kd {kd:7.3f} Nm*s/rad   학습 {m['kd']:6.3f}  -> x{kd / m['kd']:.2f}")
    print(f"  감쇠비 {zeta:.2f} (학습 {m['zeta']}), 적분 게인 {snap['vel_integrator_gain']:g}")
    print(f"  1.96 Nm 처짐 예상 {math.degrees(1.96 / kp):.2f} deg  (학습 {math.degrees(1.96 / m['kp']):.2f} deg)")
    print(f"  출력 토크 한계 {t_lim:.1f} Nm (수업 문서 {m['torque_limit']} Nm, 팀 학습 {params.TEAM_TORQUE_LIMIT:g} Nm)")
    if snap["vel_integrator_gain"]:
        print("  !! 적분 게인이 켜져 있음: 정적 처짐이 시간이 지나며 0으로 돌아갑니다.")

    errs = odrv_usb.axis_errors(axis)
    print("\n== 에러 ==")
    print("  " + ("\n  ".join(errs) if errs else "없음"))
    print(f"  state {odrv_usb.state_name(axis.current_state)}, pos_estimate {axis.encoder.pos_estimate:.4f} turn "
          f"(출력 {params.turn_to_out_deg(axis.encoder.pos_estimate):.2f} deg)")

    util.save_json(out_dir / f"check_{util.stamp()}.json", {
        "id": args.id, "motor": args.motor, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "serial": f"{odrv_usb.read(odrv, 'serial_number', 0):012X}", "fw": fw, "backup": backup_path,
        "rows": rows, "drive": snap, "joint_kp": kp, "joint_kd": kd, "zeta": zeta, "errors": errs,
    })


if __name__ == "__main__":
    util.run(main)
