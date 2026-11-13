"""6단계: 강성(게인)과 부하 마찰을 자동으로 잰다. 추를 건 상태에서 모터가 암을 천천히 여러 번 올렸다 내린다.

    python 03_stiffness.py --id 04 --motor 6512 --gains asis --mass 2.486 --arm 0.1
    python 03_stiffness.py --id 04 --motor 6512 --gains train --mass 2.486 --arm 0.1 --cycles 8

순서: (1) 암만 단 상태로 왕복 (기준)  (2) 추를 걸고 왕복.
둘 다 시작 각도(암 수평) 기준 +-amp(기본 6도)를 일정한 속도(기본 출력 6 deg/s)로 cycles(기본 5)번 왕복한다.
이 범위에서 중력 토크는 거의 일정하다 (cos 6도 = 0.995). 일정 속도 구간(왕복 범위의 가운데 절반)에서
    +방향으로 움직일 때 모터 토크 = 중력 + 마찰,   -방향 = 중력 - 마찰
    -> 마찰 = (+방향 - -방향) / 2,   중력 = (+방향 + -방향) / 2
마찰은 실제 모터 전류(Iq)로만 계산한다 (위치 오차는 PC가 목표를 계단식으로 보내 움직이는 방향으로 약간 치우침).
위치 오차(목표 - 실제)는 +/- 방향 평균만 쓰므로 그 치우침이 상쇄된다. 추를 건 것과 안 건 것의 차이로
    외력 기준 Kp = 추 토크 / 위치 오차 차이,   Iq 기준 Kp = 모터 토크 차이 / 위치 오차 차이,
    부하 / Iq 토크 = 추 토크 / 모터 토크 차이 (1이면 kt 0.067이 맞음)
를 구한다. 왕복마다 마찰을 따로 계산해 평균 +- 표준편차와 반복에 따른 변화(길들이기)도 본다.
손으로 누르는 단계가 없어 매번 같은 조건이다. 게인·전류 한계·제어 모드는 RAM에서만 바꾸고 끝나면 되돌린다.
결과: data/ID04/stiffness_auto.csv (한 줄 요약), stiffness_auto_<시각>.csv (시간 기록)
"""
import argparse
import math
import statistics
import time

from bench import odrv_usb, params, util

ACCEL = 1.0  # 왕복 끝에서의 가감속 [turn/s^2] (매달린 추가 덜 흔들리게 작게)


def profile(dist, v, a):
    """거리 dist(>0)를 사다리꼴 속도로 갈 때 (총 시간, f(t) -> (이동량, 속도))."""
    vp = min(v, math.sqrt(dist * a))
    ta = vp / a
    tc = (dist - vp * ta) / vp
    total = 2 * ta + tc

    def f(t):
        if t < ta:
            return 0.5 * a * t * t, a * t
        if t < ta + tc:
            return 0.5 * a * ta * ta + vp * (t - ta), vp
        td = max(total - t, 0.0)
        return dist - 0.5 * a * td * td, a * td

    return total, f


class Mover:
    """PASSTHROUGH 위치 제어에서 목표 위치·속도를 PC가 직접 흘려보낸다 (input_pos + input_vel 피드포워드)."""

    def __init__(self, axis, v, max_err, pause=0.0):
        self.axis, self.v, self.max_err = axis, v, max_err
        self.pause = pause  # 시뮬레이션에서 물리 스레드가 밀리지 않게 (실기는 USB 통신이 쉬는 시간)

    def move(self, p_from, p_to, rec=None, cycle=0):
        ax = self.axis
        d = p_to - p_from
        s = 1 if d > 0 else -1
        total, f = profile(abs(d), self.v, ACCEL)
        t0 = time.perf_counter()
        k = 0
        while True:
            t = time.perf_counter() - t0
            if t >= total:
                break
            x, vel = f(t)
            sp = p_from + s * x
            ax.controller.input_vel = s * vel
            ax.controller.input_pos = sp
            pos = ax.encoder.pos_estimate
            iq = ax.motor.current_control.Iq_measured
            if abs(sp - pos) > self.max_err:
                raise RuntimeError(f"목표와 실제가 {params.turn_to_out_deg(abs(sp - pos)):.1f} deg 벌어짐: "
                                   "암이 어디 걸렸거나 부하가 너무 큼")
            if rec is not None:
                rec.append((cycle, s, round(t, 4), sp, pos, iq))
            k += 1
            if k % 25 == 0:
                odrv_usb.check_alive(ax)
            if self.pause:
                time.sleep(self.pause)
        ax.controller.input_vel = 0.0
        ax.controller.input_pos = p_to

    def sweep(self, center, amp, cycles, label):
        """center -> +amp, 그다음 (-amp, +amp)를 cycles번, 마지막에 center로. 기록 반환."""
        rec = []
        self.move(center, center + amp)
        for c in range(1, cycles + 1):
            self.move(center + amp, center - amp, rec, c)
            self.move(center - amp, center + amp, rec, c)
            print(f"  [{label}] 왕복 {c}/{cycles}")
        self.move(center + amp, center)
        time.sleep(0.5)
        return rec


def per_cycle(rec, center, amp, kt):
    """왕복마다 가운데 절반 구간의 평균 -> [{'f','G','e','de'}] (토크 Nm, 위치 오차 관절 rad)."""
    groups = {}
    for cyc, s, _, sp, pos, iq in rec:
        if abs(sp - center) <= amp / 2:
            groups.setdefault((cyc, s), []).append((sp - pos, iq))
    out = []
    for cyc in sorted({c for c, _ in groups}):
        p, n = groups.get((cyc, 1)), groups.get((cyc, -1))
        if not p or not n:
            continue
        tp = kt * statistics.fmean(i for _, i in p) * params.GEAR_RATIO
        tn = kt * statistics.fmean(i for _, i in n) * params.GEAR_RATIO
        ep = params.turn_to_out_rad(statistics.fmean(e for e, _ in p))
        en = params.turn_to_out_rad(statistics.fmean(e for e, _ in n))
        out.append({"cycle": cyc, "f": (tp - tn) / 2, "G": (tp + tn) / 2, "e": (ep + en) / 2, "de": (ep - en) / 2,
                    "n": len(p) + len(n)})
    return out


def mean_sd(xs):
    return statistics.fmean(xs), (statistics.stdev(xs) if len(xs) > 1 else 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--gains", default="asis", choices=["asis", "train", "course", "default"],
                    help="asis: 드라이버 현재 값 / train: 학습 등가 / course: xdrive_config.txt / default: 0.5.1 기본")
    ap.add_argument("--mass", type=float, default=1.0, help="추 질량 [kg] (고리·끈 포함)")
    ap.add_argument("--arm", type=float, default=0.2, help="회전축 중심에서 추를 거는 지점까지 수평 거리 [m]")
    ap.add_argument("--cycles", type=int, default=5, help="왕복 횟수 (암만 / 추 각각)")
    ap.add_argument("--amp", type=float, default=6.0, help="왕복 진폭 [출력 deg] (시작 각도 기준 +-)")
    ap.add_argument("--speed", type=float, default=6.0, help="왕복 속도 [출력 deg/s]")
    ap.add_argument("--current-lim", type=float, default=10.0, help="RAM 전류 한계 [A] (걸렸을 때 출력 약 10 Nm까지)")
    args = ap.parse_args()

    m = params.MOTORS[args.motor]
    kt = m["kt"]
    tau = args.mass * params.G * args.arm
    amp = params.out_deg_to_turn(args.amp)
    v = params.out_deg_to_turn(args.speed)
    odrv = odrv_usb.connect(args)
    axis = odrv.axis0
    sign = odrv_usb.torque_sign(axis)

    rec0 = rec1 = snap = None
    with odrv_usb.RamSession(odrv) as s:
        s.set_gains(args.motor, args.gains)
        s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
        s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
        s.set("motor.config.current_lim", args.current_lim)
        snap = odrv_usb.drive_snapshot(axis)
        kp_exp, _ = params.joint_gains(snap["pos_gain"], snap["vel_gain"], snap["torque_constant"], kt)
        max_err = params.out_rad_to_turn(max(math.radians(5.0), 2 * (tau + 1.0) / kp_exp))
        print(f"\n게인 [{args.gains}]: pos_gain {snap['pos_gain']:g}, vel_gain {snap['vel_gain']:g}, "
              f"적분 {snap['vel_integrator_gain']:g} -> 관절 Kp {kp_exp:.1f} Nm/rad (학습 {m['kp']})")
        print(f"추 토크 {tau:.3f} Nm ({args.mass:g} kg x {args.arm:g} m) -> 예상 처짐 {math.degrees(tau / kp_exp):.2f} deg")
        print(f"왕복: +-{args.amp:g} deg, {args.speed:g} deg/s, {args.cycles}회 (암만 1번 + 추 1번, 각각 약 "
              f"{args.cycles * 2 * (2 * args.amp / args.speed + 0.5):.0f}초)")
        if snap["vel_integrator_gain"]:
            print(f"!! 적분 게인 {snap['vel_integrator_gain']:g}이 켜져 있어 위치 오차가 0으로 가므로 Kp 값은 의미 없음")

        mover = Mover(axis, v, max_err, pause=0.001 if args.sim else 0.0)
        try:
            util.confirm("\n[1] 추는 빼고, 암을 손으로 수평으로 받친 채로 (Enter를 누르면 지금 각도로 위치 유지 시작)")
            axis.controller.input_vel = 0.0
            odrv_usb.enter_closed_loop(axis)
            center = axis.encoder.pos_estimate
            axis.controller.input_pos = center
            util.confirm("    위치 유지 중. 암에서 손을 떼고 주변을 비운 뒤 (암만 단 상태로 왕복 시작)")
            rec0 = mover.sweep(center, amp, args.cycles, "암만")

            util.confirm(f"\n[2] 추({args.mass:g} kg)를 {args.arm:g} m 지점에 천천히 걸고 손을 뗀 뒤 "
                         "(추가 아무 데도 닿지 않게, 흔들림이 멈추면)")
            if args.sim:
                odrv.ext_torque = -tau
                time.sleep(0.5)
            rec1 = mover.sweep(center, amp, args.cycles, "추")
            util.confirm("\n[3] 추를 빼고 암을 손으로 받친 뒤 (Enter를 누르면 힘이 풀림)")
            if args.sim:
                odrv.ext_torque = 0.0
        finally:
            axis.controller.input_vel = 0.0
    if rec1 is None:
        return

    kts = kt * sign  # direction = -1 보드는 Iq 부호가 토크와 반대
    c0 = per_cycle(rec0, center, amp, kts)
    c1 = per_cycle(rec1, center, amp, kts)
    if not c0 or not c1:
        raise RuntimeError("일정 속도 구간의 기록이 없습니다 (--amp가 너무 작음?)")
    g0, e0 = statistics.fmean(c["G"] for c in c0), statistics.fmean(c["e"] for c in c0)
    g1, e1 = statistics.fmean(c["G"] for c in c1), statistics.fmean(c["e"] for c in c1)
    dg, de = abs(g1 - g0), abs(e1 - e0)
    kp_ext = tau / de if de else math.inf
    kp_iq = dg / de if de else math.inf
    kt_ratio = tau / dg if dg else math.inf
    f0, f0_sd = mean_sd([c["f"] for c in c0])
    f1, f1_sd = mean_sd([c["f"] for c in c1])
    slope = (f1 - f0) / tau

    print(f"\n왕복별 마찰 [Nm, Iq 기준]  (속도 {args.speed:g} deg/s = 관절 {math.radians(args.speed):.2f} rad/s)")
    print(f"{'왕복':>4} {'암만':>8} {'추':>8}")
    for i in range(max(len(c0), len(c1))):
        a = f"{c0[i]['f']:8.3f}" if i < len(c0) else " " * 8
        b = f"{c1[i]['f']:8.3f}" if i < len(c1) else " " * 8
        print(f"{i + 1:4d} {a} {b}")

    print(f"\n== 결과 ID{args.id} [{args.gains}] 추 토크 {tau:.3f} Nm, 왕복 {len(c1)}회 ==")
    if snap["vel_integrator_gain"]:
        print("(적분 게인이 켜져 있어 아래 Kp 값은 의미 없음)")
    print(f"외력 기준 Kp = 추 토크 / 처짐 = {kp_ext:.1f} Nm/rad   (게인으로 예상 {kp_exp:.1f}, 학습 {m['kp']})")
    print(f"Iq 기준 Kp  = 모터 토크 / 처짐 = {kp_iq:.1f} Nm/rad   (드라이버가 게인대로 토크를 내는지)")
    print(f"부하 / Iq 토크 = {kt_ratio:.3f}   (1이면 kt {kt}가 맞음)")
    print(f"처짐 {math.degrees(de):.3f} deg (추로 생긴 위치 오차), 암 자체 중력 토크 약 {abs(g0):.3f} Nm")
    print(f"마찰 (암만)  = {f0:.3f} +- {f0_sd:.3f} Nm   (왕복 {len(c0)}회 평균 +- 표준편차)")
    print(f"마찰 (추)    = {f1:.3f} +- {f1_sd:.3f} Nm   학습 모델 {m['coulomb']} Nm 고정")
    print(f"-> 마찰 약 {f0:.3f} + {slope:.3f} x 부하 [Nm]  (두 점으로 본 근사, 암 자체 무게는 무시)")
    if len(c1) >= 3:
        first, last = c1[0]["f"], c1[-1]["f"]
        th = max(2 * f1_sd, 0.05 * abs(f1), 0.01)
        trend = "줄어듦" if last < first - th else ("커짐" if last > first + th else "뚜렷한 변화 없음")
        print(f"반복에 따른 변화 (추): 1회 {first:.3f} -> {len(c1)}회 {last:.3f} Nm, {trend}")

    st = util.stamp()
    out_dir = util.data_dir(args.id)
    rows = [("arm", *r) for r in rec0] + [("weight", *r) for r in rec1]
    util.write_log(out_dir / f"stiffness_auto_{st}.csv",
                   {"id": args.id, "motor": args.motor, "gains": args.gains, "tau": tau, "mass": args.mass,
                    "arm": args.arm, "amp_deg": args.amp, "speed_deg_s": args.speed, "center": center,
                    "torque_sign": sign, "drive": snap},
                   ["sweep", "cycle", "dir", "t", "sp_turn", "pos_turn", "iq"], rows)
    util.append_csv(out_dir / "stiffness_auto.csv", {
        "time": st, "gains": args.gains, "pos_gain": snap["pos_gain"], "vel_gain": snap["vel_gain"],
        "integrator": snap["vel_integrator_gain"], "mass_kg": args.mass, "arm_m": args.arm, "tau_Nm": round(tau, 4),
        "cycles": len(c1), "amp_deg": args.amp, "speed_deg_s": args.speed,
        "sag_deg": round(math.degrees(de), 4), "kp_ext": round(kp_ext, 2), "kp_iq": round(kp_iq, 2),
        "kp_expected": round(kp_exp, 2), "kt_ratio": round(kt_ratio, 4),
        "fric_arm_Nm": round(f0, 4), "fric_arm_sd": round(f0_sd, 4),
        "fric_load_Nm": round(f1, 4), "fric_load_sd": round(f1_sd, 4), "fric_slope": round(slope, 4)})


if __name__ == "__main__":
    util.run(main)
