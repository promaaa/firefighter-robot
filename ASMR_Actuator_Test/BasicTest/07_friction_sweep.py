"""무부하 마찰: 여러 속도로 일정하게 돌리며 필요한 전류를 재서 쿨롱·점성 마찰을 구한다. 추·암 필요 없음.

    python 07_friction_sweep.py --id 04 --motor 6512 --plot
    python 07_friction_sweep.py --id 04 --motor 6512 --speeds 0.5,1,2,4,7,10,14 --plot

출력축이 계속 돈다 (속도마다 정방향 -> 역방향, 최고 속도에서 한쪽으로 약 3바퀴). 출력축에 아무것도 달지 말고,
액추에이터 몸체는 클램프로 고정하거나 손으로 잡는다.
속도 제어(VEL_RAMP)로 각 속도를 유지하고, 정속 구간의 Iq 평균으로 관절 토크 = kt x Iq x 15를 구한다.
무부하라서 이 토크는 전부 모터+감속기 마찰이다. 관절 속도 w [rad/s]에 대해
    tau = Fc x sign(w) + B x w      (학습 모델 6512: Fc 0.377 Nm, B 0.090 Nm*s/rad)
로 정방향·역방향을 따로 맞춘다. Iq 영점 오차는 두 방향 평균에서 상쇄된다.
전류 한계(기본 5 A)와 속도 제어 설정은 RAM에서만 바꾸고, 끝나면 IDLE + 원래 값으로 되돌린다.
역기전력 때문에 Vbus에 비해 너무 빠른 속도는 건너뛴다 (12 V면 약 14 turn/s까지).
결과: data/ID04/friction_sweep.csv (요약 한 줄), friction_sweep_<시각>.csv (속도별 값, --plot이면 .png)
"""
import argparse
import math
import statistics
import time

from bench import odrv_usb, params, util

CONTROL_VELOCITY = 2
INPUT_VEL_RAMP = 2
SETTLE = 0.5  # 목표 속도 도달 뒤 기다리는 시간 [s]


def linfit(x, y):
    """y = a + b x 최소제곱. -> (a, b, rms 잔차)"""
    mx, my = statistics.fmean(x), statistics.fmean(y)
    sxx = sum((xi - mx) ** 2 for xi in x)
    b = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y)) / sxx
    a = my - b * mx
    rms = math.sqrt(statistics.fmean([(yi - a - b * xi) ** 2 for xi, yi in zip(x, y)]))
    return a, b, rms


def run_segment(odrv, axis, v_cmd, prev, ramp, duration):
    """input_vel = v_cmd로 바꾸고 정속이 되면 duration 동안 기록. -> dict"""
    axis.controller.input_vel = v_cmd
    t_end = time.time() + abs(v_cmd - prev) / ramp + SETTLE
    while time.time() < t_end:
        odrv_usb.check_alive(axis)
        time.sleep(0.02)
    vs, iqs = [], []
    t_end = time.time() + max(duration, 1.0 / abs(v_cmd))  # 모터 한 바퀴 이상 평균 (코깅 상쇄)
    while time.time() < t_end:
        vs.append(axis.encoder.vel_estimate)
        iqs.append(axis.motor.current_control.Iq_measured)
        time.sleep(0.001)
    odrv_usb.check_alive(axis)
    v_mean = statistics.fmean(vs)
    if abs(v_mean) < 0.5 * abs(v_cmd):
        raise RuntimeError(f"명령 {v_cmd:+g} turn/s인데 {v_mean:+.2f} turn/s밖에 안 돎: 출력축이 걸렸거나 전류 한계가 낮음")
    return {"v_cmd": v_cmd, "v": v_mean, "iq": statistics.fmean(iqs), "iq_sd": statistics.pstdev(iqs),
            "n": len(vs), "vbus": odrv.vbus_voltage}


def plot(path, rows, fits, m):
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot([r["w"] for r in rows], [r["tau"] for r in rows], "o", label="측정 (정속 평균)")
    w_max = max(abs(r["w"]) for r in rows)
    for sign, (a, b, _) in fits.items():
        xs = [0.0, sign * w_max]
        ax.plot(xs, [a + b * x for x in xs], "-", color="C0", alpha=0.6)
    xs = [i / 50 * w_max for i in range(-50, 51) if i]
    ax.plot(xs, [math.copysign(m["coulomb"], x) + m["viscous"] * x for x in xs], "--", color="C1",
            label=f"학습 모델 (Fc {m['coulomb']}, B {m['viscous']})")
    ax.axhline(0, color="0.8", lw=0.8)
    ax.axvline(0, color="0.8", lw=0.8)
    ax.set_xlabel("관절 속도 [rad/s]")
    ax.set_ylabel("관절 토크 = kt x Iq x 15 [Nm]")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"그림: {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--speeds", default="0.5,1,2,4,7,10,14", help="모터 속도 목록 [turn/s] (출력은 1/15)")
    ap.add_argument("--duration", type=float, default=2.0, help="속도마다 기록 시간 [s] (최소 모터 한 바퀴)")
    ap.add_argument("--ramp", type=float, default=10.0, help="가감속 [turn/s^2]")
    ap.add_argument("--current-lim", type=float, default=5.0, help="RAM 전류 한계 [A] (막혔을 때 출력 약 5 Nm까지)")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()

    m = params.MOTORS[a.motor]
    odrv = odrv_usb.connect(a)
    axis = odrv.axis0
    kt = m["kt"] * odrv_usb.torque_sign(axis)  # direction = -1 보드는 Iq 부호가 토크와 반대
    vbus = odrv.vbus_voltage
    v_top = odrv_usb.max_speed(vbus, axis.motor.config.torque_constant)
    speeds = sorted(float(x) for x in a.speeds.split(","))
    skipped = [v for v in speeds if v > v_top]
    speeds = [v for v in speeds if v <= v_top]
    if skipped:
        print(f"Vbus {vbus:.1f} V에서는 {v_top:.1f} turn/s까지만 안전하게 낼 수 있어 {skipped} turn/s는 건너뜁니다.")
    if not speeds:
        raise RuntimeError("잴 수 있는 속도가 없습니다")
    rev = max(speeds) * (max(speeds) / a.ramp + SETTLE + a.duration) / params.GEAR_RATIO
    print(f"\n속도 {speeds} turn/s (관절 {params.turn_to_out_rad(speeds[0]):.2f}~"
          f"{params.turn_to_out_rad(speeds[-1]):.2f} rad/s), 각각 정방향 -> 역방향.")
    print(f"!! 출력축이 계속 돕니다 (한쪽으로 최대 약 {rev:.0f}바퀴). 출력축에 아무것도 달지 말고 몸체를 고정하세요.")
    print(f"   전류 한계 {a.current_lim:g} A (RAM). 약 {len(speeds) * 2 * (a.duration + 2):.0f}초 걸립니다.")
    util.confirm("시작할까요?")

    segs = []
    with odrv_usb.RamSession(odrv) as s:
        s.set("motor.config.current_lim", a.current_lim)
        s.set("controller.config.vel_limit", max(speeds) * 1.3 + 2.0)
        s.set("controller.config.vel_integrator_gain", 0.0)
        s.set("controller.config.vel_ramp_rate", a.ramp)
        s.set("controller.config.control_mode", CONTROL_VELOCITY)
        s.set("controller.config.input_mode", INPUT_VEL_RAMP)
        axis.controller.input_vel = 0.0
        odrv_usb.enter_closed_loop(axis)
        prev = 0.0
        for v in speeds:
            for sign in (1, -1):
                seg = run_segment(odrv, axis, sign * v, prev, a.ramp, a.duration)
                prev = sign * v
                segs.append(seg)
                w = params.turn_to_out_rad(seg["v"])
                print(f"  {seg['v_cmd']:+6.1f} turn/s: 실제 {seg['v']:+6.2f} turn/s (관절 {w:+5.2f} rad/s), "
                      f"Iq {seg['iq']:+6.3f} A -> 관절 토크 {kt * seg['iq'] * params.GEAR_RATIO:+6.3f} Nm")
        axis.controller.input_vel = 0.0
        time.sleep(abs(prev) / a.ramp + 0.3)
    if len(segs) < 2 * len(speeds):
        return

    rows = [{**sg, "w": params.turn_to_out_rad(sg["v"]), "tau": kt * sg["iq"] * params.GEAR_RATIO} for sg in segs]
    fits = {}
    for sign in (1, -1):
        side = [r for r in rows if r["v_cmd"] * sign > 0]
        if len(side) >= 2:
            fits[sign] = linfit([r["w"] for r in side], [r["tau"] for r in side])
    if len(fits) < 2:
        print("속도가 2개 이상이어야 직선을 맞출 수 있습니다.")
        return
    (ap_, bp, rp), (an, bn, rn) = fits[1], fits[-1]
    fc_p, fc_n = ap_, -an
    fc, b = (fc_p + fc_n) / 2, (bp + bn) / 2

    print(f"\n== 결과 ID{a.id} 무부하 마찰 (Vbus {vbus:.1f} V) ==")
    print(f"{'관절 rad/s':>10} {'측정 +방향':>11} {'측정 -방향':>11} {'모델':>8}   [Nm]")
    for v in speeds:
        rp_ = next(r for r in rows if r["v_cmd"] == v)
        rn_ = next(r for r in rows if r["v_cmd"] == -v)
        w = (rp_["w"] - rn_["w"]) / 2
        print(f"{w:10.2f} {rp_['tau']:+11.3f} {rn_['tau']:+11.3f} {m['coulomb'] + m['viscous'] * w:8.3f}")
    print(f"쿨롱 Fc = {fc:.3f} Nm  (+방향 {fc_p:.3f}, -방향 {fc_n:.3f})   학습 모델 {m['coulomb']}")
    print(f"점성 B  = {b:.4f} Nm*s/rad  (+방향 {bp:.4f}, -방향 {bn:.4f})   학습 모델 {m['viscous']}")
    print(f"직선에서 벗어난 정도 (rms) +{rp:.3f} / -{rn:.3f} Nm  (크면 직선이 아닌 마찰, 예: 저속에서 커짐)")
    print("주의: 무부하 값이다. 03에서 본 것처럼 부하가 걸리면 마찰이 더 커질 수 있다.")

    st = util.stamp()
    out_dir = util.data_dir(a.id)
    log = out_dir / f"friction_sweep_{st}.csv"
    util.write_log(log, {"id": a.id, "motor": a.motor, "vbus": vbus, "kt_signed": kt, "current_lim": a.current_lim,
                         "ramp": a.ramp, "duration": a.duration},
                   ["v_cmd_turn_s", "v_turn_s", "w_rad_s", "iq_A", "iq_sd_A", "tau_Nm", "n", "vbus_V"],
                   [[r["v_cmd"], round(r["v"], 4), round(r["w"], 4), round(r["iq"], 4), round(r["iq_sd"], 4),
                     round(r["tau"], 4), r["n"], round(r["vbus"], 3)] for r in rows])
    util.append_csv(out_dir / "friction_sweep.csv", {
        "time": st, "motor": a.motor, "vbus_V": round(vbus, 2), "speeds": a.speeds,
        "Fc_Nm": round(fc, 4), "Fc_pos": round(fc_p, 4), "Fc_neg": round(fc_n, 4),
        "B_Nms": round(b, 5), "B_pos": round(bp, 5), "B_neg": round(bn, 5),
        "rms_pos": round(rp, 4), "rms_neg": round(rn, 4),
        "Fc_model": m["coulomb"], "B_model": m["viscous"]})
    if a.plot:
        plot(log.with_suffix(".png"), rows, fits, m)


if __name__ == "__main__":
    util.run(main)
