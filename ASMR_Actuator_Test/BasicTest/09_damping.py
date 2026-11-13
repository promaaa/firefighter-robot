"""Kd(감쇠) 측정: 목표 위치를 일정한 속도로 움직이며, 실제 위치가 얼마나 뒤처지는지로 Kd를 구한다. 추·암 없이.

    python 09_damping.py --id 03 --motor 6512 --gains train
    python 09_damping.py --id 03 --motor 6512 --gains asis --speeds 0.15,0.3,0.45,0.6

드라이버 위치 제어 토크 = Kp x (목표 - 실제) + Kd x (목표 속도 - 실제 속도).
로봇에서는 정책이 위치 목표만 보내므로(input_vel = 0) 감쇠 항은 -Kd x 실제 속도다 (시뮬레이션 PD와 같은 꼴).
목표를 일정한 관절 속도 w로 움직이면, 다 따라붙은 뒤에는
    속도 피드포워드 없음 (배포와 같음):  Kp x 지연1 = 마찰 + Kd x w
    속도 피드포워드 있음 (input_vel = w): Kp x 지연2 = 마찰
    -> Kd = Kp x (지연1 - 지연2) / w
마찰, 그리고 PC가 목표를 계단식으로 보내서 생기는 지연 치우침은 두 경우에 똑같아서 빠진다.
정·역방향을 모두 돌려 평균한다. Kp는 게인으로 계산한 값을 쓴다 (03에서 실측과 2% 안으로 확인).
Iq로 본 토크(= 마찰)도 기록해 두 경우가 같은지 확인한다.

출력축이 구간마다 한쪽으로 수십 도씩 돈다 (정·역 번갈아서 제자리 근처로 돌아옴). 암·추를 떼고 몸체를 고정한다.
전류 한계는 팀 토크 한계(기본 6 Nm)에 맞춰 RAM에서 바꾼다. 게인·모드도 RAM에서만 바꾸고 끝나면 되돌린다.
결과: data/ID03/damping.csv (한 줄 요약), damping_<시각>.csv (구간별 값)
"""
import argparse
import math
import statistics
import time

from bench import odrv_usb, params, util

SETTLE = 0.6  # 목표가 움직이기 시작한 뒤 버리는 시간 [s] (따라붙는 시상수 약 60~80 ms의 8배 이상)
REST = 0.5    # 구간 사이 정지 [s]
FRIC_GUESS = 0.5  # 속도 상한 계산에 쓰는 마찰 어림값 [Nm]


def segment(axis, w, ff, duration, max_err, pause):
    """지금 목표 위치에서 관절 속도 w [rad/s]로 목표를 움직인다. ff면 input_vel도 준다. -> dict (관절 단위)"""
    v = params.out_rad_to_turn(w)  # 모터 turn/s
    rec_time = max(duration, 1.0 / abs(v))  # 모터 한 바퀴 이상 평균 (코깅 상쇄)
    p0 = axis.controller.input_pos
    es, vs, iqs, dts = [], [], [], []
    t0 = time.perf_counter()
    t_prev, k = None, 0
    sp = p0
    while (t := time.perf_counter() - t0) < SETTLE + rec_time:
        sp = p0 + v * t
        axis.controller.input_vel = v if ff else 0.0
        axis.controller.input_pos = sp
        pos = axis.encoder.pos_estimate
        vel = axis.encoder.vel_estimate
        iq = axis.motor.current_control.Iq_measured
        if abs(sp - pos) > max_err:
            raise RuntimeError(f"목표와 실제가 {params.turn_to_out_deg(abs(sp - pos)):.1f} deg 벌어짐: "
                               "출력축이 어디 걸렸거나 전류 한계가 낮음")
        if t >= SETTLE:
            es.append(sp - pos)
            vs.append(vel)
            iqs.append(iq)
            if t_prev is not None:
                dts.append(t - t_prev)
        t_prev = t
        k += 1
        if k % 25 == 0:
            odrv_usb.check_alive(axis)
        if pause:
            time.sleep(pause)
    axis.controller.input_vel = 0.0
    axis.controller.input_pos = sp
    time.sleep(REST)
    return {"w_cmd": w, "ff": ff,
            "w": params.turn_to_out_rad(statistics.fmean(vs)),
            "e": params.turn_to_out_rad(statistics.fmean(es)),
            "e_sd": params.turn_to_out_rad(statistics.pstdev(es)),
            "iq": statistics.fmean(iqs), "n": len(es),
            "dt_ms": 1000 * statistics.fmean(dts) if dts else math.nan}


def fit_origin(x, y):
    """y = k x (원점 통과) 최소제곱 -> k"""
    return sum(a * b for a, b in zip(x, y)) / sum(a * a for a in x)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--gains", default="asis", choices=["asis", "train", "course", "default"],
                    help="asis: 드라이버 현재 값 / train: 학습 등가 (수업 문서) / course: xdrive_config.txt")
    ap.add_argument("--speeds", default="0.15,0.3,0.45,0.6", help="목표 이동 속도 목록 [관절 rad/s]")
    ap.add_argument("--duration", type=float, default=1.5, help="속도마다 기록 시간 [s] (최소 모터 한 바퀴)")
    ap.add_argument("--torque-limit", type=float, default=params.TEAM_TORQUE_LIMIT,
                    help="관절 토크 한계 [Nm]. 전류 한계 = 이 값 / (kt x 15)로 RAM에 설정")
    args = ap.parse_args()

    m = params.MOTORS[args.motor]
    kt = m["kt"]
    current_lim = args.torque_limit / (kt * params.GEAR_RATIO)
    speeds = [float(s) for s in args.speeds.split(",")]

    odrv = odrv_usb.connect(args)
    axis = odrv.axis0
    sign = odrv_usb.torque_sign(axis)
    segs = []
    with odrv_usb.RamSession(odrv) as s:
        s.set_gains(args.motor, args.gains)
        s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
        s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
        s.set("motor.config.current_lim", round(current_lim, 3))
        snap = odrv_usb.drive_snapshot(axis)
        kp, kd_exp = params.joint_gains(snap["pos_gain"], snap["vel_gain"], snap["torque_constant"], kt)
        if snap["vel_integrator_gain"]:
            raise RuntimeError(f"적분 게인 {snap['vel_integrator_gain']:g}이 켜져 있으면 지연이 0으로 가서 잴 수 없습니다. "
                               "--gains train 또는 course를 주세요")
        w_max = 0.8 * (args.torque_limit - FRIC_GUESS) / kd_exp
        skip = [w for w in speeds if w > w_max]
        speeds = [w for w in speeds if w <= w_max]
        if not speeds:
            raise RuntimeError(f"모든 속도가 상한 {w_max:.2f} rad/s보다 큽니다 (토크 한계 {args.torque_limit:g} Nm)")
        max_err = params.out_rad_to_turn(max(math.radians(5.0), 2 * (FRIC_GUESS + kd_exp * max(speeds)) / kp))
        print(f"\n게인 [{args.gains}]: pos_gain {snap['pos_gain']:g}, vel_gain {snap['vel_gain']:g} "
              f"-> 관절 Kp {kp:.1f} Nm/rad, Kd {kd_exp:.3f} Nm*s/rad (학습 Kp {m['kp']}, Kd {m['kd']})")
        print(f"토크 한계 {args.torque_limit:g} Nm -> 전류 한계 {current_lim:.2f} A (RAM)")
        if skip:
            print(f"건너뜀: {', '.join(f'{w:g}' for w in skip)} rad/s (Kd x 속도 + 마찰이 토크 한계의 80%를 넘음)")
        print(f"속도 {', '.join(f'{w:g}' for w in speeds)} rad/s, 속도마다 정·역 x 피드포워드 없음/있음 4구간")
        util.confirm("암·추를 떼고 몸체를 고정했으면 (Enter를 누르면 지금 위치에서 시작)")
        try:
            axis.controller.input_vel = 0.0
            axis.controller.input_pos = axis.encoder.pos_estimate
            odrv_usb.enter_closed_loop(axis)
            time.sleep(0.3)
            pause = 0.001 if args.sim else 0.0
            for w in speeds:
                for ff in (False, True):
                    for d in (1, -1):
                        sg = segment(axis, d * w, ff, args.duration, max_err, pause)
                        segs.append(sg)
                        print(f"  {d * w:+5.2f} rad/s  피드포워드 {'있음' if ff else '없음'}: "
                              f"지연 {math.degrees(sg['e']):+7.3f} deg, 토크(Iq) {kt * sign * sg['iq'] * 15:+6.3f} Nm, "
                              f"실제 {sg['w']:+6.3f} rad/s")
        finally:
            axis.controller.input_vel = 0.0
        vbus = odrv_usb.read(odrv, "vbus_voltage")

    def pair(w, ff):
        p = next(g for g in segs if g["w_cmd"] == w and g["ff"] == ff)
        n = next(g for g in segs if g["w_cmd"] == -w and g["ff"] == ff)
        return p, n

    rows = []
    for w in speeds:
        p1, n1 = pair(w, False)
        p2, n2 = pair(w, True)
        e1, e2 = (p1["e"] - n1["e"]) / 2, (p2["e"] - n2["e"]) / 2
        wm = (abs(p1["w"]) + abs(n1["w"])) / 2
        f1 = kt * sign * (p1["iq"] - n1["iq"]) / 2 * params.GEAR_RATIO
        f2 = kt * sign * (p2["iq"] - n2["iq"]) / 2 * params.GEAR_RATIO
        rows.append({"w": wm, "e1": e1, "e2": e2, "kd": kp * (e1 - e2) / wm, "f1": f1, "f2": f2,
                     "kd_iq": (kp * e1 - f1) / wm})
    kd_fit = fit_origin([r["w"] for r in rows], [kp * (r["e1"] - r["e2"]) for r in rows])

    print(f"\n== 결과 ID{args.id} [{args.gains}] ==")
    print(f"{'속도':>6} {'지연(FF없음)':>11} {'지연(FF있음)':>11} {'Kd':>7} {'Kd(Iq)':>7} {'마찰(FF없음)':>11} {'마찰(FF있음)':>11}")
    for r in rows:
        print(f"{r['w']:6.3f} {math.degrees(r['e1']):9.3f}deg {math.degrees(r['e2']):9.3f}deg {r['kd']:7.3f} "
              f"{r['kd_iq']:7.3f} {r['f1']:9.3f}Nm {r['f2']:9.3f}Nm")
    print(f"\nKd (모든 속도 맞춤) = {kd_fit:.3f} Nm*s/rad   게인으로 예상 {kd_exp:.3f} -> x{kd_fit / kd_exp:.3f}, "
          f"학습 {m['kd']}")
    print("Kd(Iq) = (Kp x 지연 - Iq 토크) / 속도: 피드포워드 없이 한 번만 재는 방법 (참고용, 계단 치우침이 남음)")
    print("마찰 두 열이 비슷하면 피드포워드 유무로 조건이 같았다는 뜻 (07의 같은 속도 값과도 비슷해야 함)")
    dt = statistics.fmean(g["dt_ms"] for g in segs)
    print(f"PC 목표 갱신 간격 평균 {dt:.2f} ms")

    st = util.stamp()
    out_dir = util.data_dir(args.id)
    util.write_log(out_dir / f"damping_{st}.csv",
                   {"id": args.id, "motor": args.motor, "gains": args.gains, "kp": kp, "kd_expected": kd_exp,
                    "torque_limit": args.torque_limit, "current_lim": current_lim, "torque_sign": sign,
                    "vbus": vbus, "drive": snap},
                   ["w_cmd_rad_s", "ff", "w_rad_s", "lag_rad", "lag_sd_rad", "iq_A", "n", "dt_ms"],
                   [[g["w_cmd"], int(g["ff"]), round(g["w"], 5), round(g["e"], 6), round(g["e_sd"], 6),
                     round(g["iq"], 4), g["n"], round(g["dt_ms"], 3)] for g in segs])
    util.append_csv(out_dir / "damping.csv", {
        "time": st, "motor": args.motor, "gains": args.gains, "pos_gain": snap["pos_gain"],
        "vel_gain": snap["vel_gain"], "kp": round(kp, 2), "kd_expected": round(kd_exp, 4), "kd_fit": round(kd_fit, 4),
        "ratio": round(kd_fit / kd_exp, 4), "kd_by_speed": " ".join(f"{r['kd']:.3f}" for r in rows),
        "speeds": " ".join(f"{r['w']:.3f}" for r in rows), "torque_limit": args.torque_limit,
        "vbus": round(vbus, 2) if vbus else vbus})


if __name__ == "__main__":
    util.run(main)
