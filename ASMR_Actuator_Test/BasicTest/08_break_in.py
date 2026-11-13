"""길들이기: 출력축을 일정 속도로 정방향·역방향 번갈아 돌리면서 마찰이 어떻게 변하는지 기록한다.

    python 08_break_in.py --id 03 --motor 6512 --minutes 20 --plot
    python 08_break_in.py --id 03 --motor 6512 --minutes 20 --speed 10 --reverse 30 --check-every 5 --plot

- 출력축에 아무것도 달지 않는다 (계속 돈다). 액추에이터 몸체는 클램프로 고정한다.
- 속도 제어(VEL_RAMP)로 --speed [모터 turn/s] (기본 10 = 관절 4.2 rad/s, 출력 40 rpm)로 돌리고,
  --reverse초(기본 30)마다 방향을 바꾼다. 구간마다 정속 평균 전류로 마찰 토크(kt x Iq x 15)를 기록한다.
- --check-every분(기본 5)마다 저속(모터 1 turn/s = 관절 0.42 rad/s) 마찰도 잰다. 시작과 끝에도 잰다.
- 안전 정지: 속도가 명령의 절반 미만(걸림), 마찰이 처음보다 --max-rise배(기본 1.5) 넘게 커짐
  (3D 프린팅 기어 손상·이물 의심), 보드 FET 온도가 --max-fet(기본 70 도) 넘음. Ctrl+C로 언제든 멈춘다.
  전류 한계는 RAM에서만 5 A로 낮추고, 끝나면 IDLE + 원래 설정으로 되돌린다. 멈춰도 그때까지 기록은 저장한다.
- 돌리는 동안 그리스가 데워져 마찰이 줄어드는 효과가 섞인다. 길들이기 효과는 '식은 상태'끼리 비교한다:
  세션 시작 때 저속 마찰끼리, 또는 길들이기 전후에 충분히 식힌 뒤 07_friction_sweep.py·03_stiffness.py로.
결과: data/ID03/break_in.csv (세션 요약 + 누적 시간), break_in_<시각>.csv (구간별 기록, --plot이면 .png)
"""
import argparse
import csv
import statistics
import time

from bench import odrv_usb, params, util

CONTROL_VELOCITY = 2
INPUT_VEL_RAMP = 2
SETTLE = 0.5      # 목표 속도 도달 뒤 기다리는 시간 [s]
SLOW = 1.0        # 저속 확인 속도 [모터 turn/s]
SLOW_HOLD = 2.0   # 저속 확인 방향당 기록 시간 [s]


class Stop(Exception):
    """안전 조건으로 멈춤 (기록은 저장)."""


class Spinner:
    def __init__(self, odrv, axis, kts, ramp, min_ratio=0.5):
        self.odrv, self.axis, self.kts, self.ramp = odrv, axis, kts, ramp
        self.min_ratio = min_ratio
        self.prev = 0.0

    def spin(self, v_cmd, hold):
        """input_vel = v_cmd로 바꾸고, 정속이 되면 hold초 동안 평균. -> (평균 속도, 관절 토크)"""
        ax = self.axis
        ax.controller.input_vel = v_cmd
        t_end = time.time() + abs(v_cmd - self.prev) / self.ramp + SETTLE
        self.prev = v_cmd
        while time.time() < t_end:
            odrv_usb.check_alive(ax)
            time.sleep(0.02)
        sv = si = 0.0
        n = 0
        t_end = time.time() + hold
        while time.time() < t_end:
            sv += ax.encoder.vel_estimate
            si += ax.motor.current_control.Iq_measured
            n += 1
            if n % 50 == 0:
                odrv_usb.check_alive(ax)
            time.sleep(0.001)
        v = sv / n
        if abs(v) < self.min_ratio * abs(v_cmd):
            raise Stop(f"명령 {v_cmd:+g} turn/s인데 {v:+.2f} turn/s밖에 안 돎: 출력축이 걸렸거나 마찰이 크게 늘었음")
        return v, self.kts * (si / n) * params.GEAR_RATIO

    def check(self):
        """저속 정·역방향 -> 마찰 (두 방향 평균) [Nm]"""
        _, tp = self.spin(SLOW, SLOW_HOLD)
        _, tn = self.spin(-SLOW, SLOW_HOLD)
        return (tp - tn) / 2


def past_minutes(path):
    if not path.exists():
        return 0.0, 0
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return sum(float(r.get("minutes") or 0) for r in rows), len(rows)


def plot(path, rows, title):
    import matplotlib
    matplotlib.use("Agg")
    matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.5))
    run = [r for r in rows if r["kind"] == "run"]
    chk = [r for r in rows if r["kind"] == "check"]
    ax.plot([r["t_min"] for r in run], [abs(r["tau"]) for r in run], ".", ms=4, alpha=0.6,
            label="길들이기 속도 (구간별)")
    ax.plot([r["t_min"] for r in chk], [r["tau"] for r in chk], "o-", label=f"저속 {SLOW:g} turn/s (정·역 평균)")
    ax.set_xlabel("경과 시간 [분]")
    ax.set_ylabel("마찰 토크 [Nm] (관절)")
    ax.grid(alpha=0.3)
    temps = [(r["t_min"], r["fet"]) for r in rows if r["fet"] is not None]
    if temps:
        ax2 = ax.twinx()
        ax2.plot([t for t, _ in temps], [c for _, c in temps], color="C3", alpha=0.4, lw=1, label="FET 온도")
        ax2.set_ylabel("보드 FET 온도 [도]", color="C3")
    ax.legend(loc="upper right")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"그림: {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--minutes", type=float, default=20.0, help="이번 세션 길이 [분]")
    ap.add_argument("--speed", type=float, default=10.0, help="길들이기 속도 [모터 turn/s] (출력은 1/15)")
    ap.add_argument("--reverse", type=float, default=30.0, help="방향을 바꾸는 간격 [s]")
    ap.add_argument("--check-every", type=float, default=5.0, help="저속 마찰 확인 간격 [분]")
    ap.add_argument("--ramp", type=float, default=10.0, help="가감속 [turn/s^2]")
    ap.add_argument("--current-lim", type=float, default=5.0, help="RAM 전류 한계 [A]")
    ap.add_argument("--max-rise", type=float, default=1.5, help="마찰이 처음의 이 배수를 넘으면 정지")
    ap.add_argument("--max-fet", type=float, default=70.0, help="보드 FET 온도 한계 [도]")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()

    hold = a.reverse - 2 * a.speed / a.ramp - SETTLE
    if hold < 1.0:
        ap.error(f"--reverse가 너무 짧습니다 (가감속에 {2 * a.speed / a.ramp + SETTLE:.1f}초 필요). 더 길게 주세요.")

    m = params.MOTORS[a.motor]
    out_dir = util.data_dir(a.id)
    prev_min, prev_n = past_minutes(out_dir / "break_in.csv")
    odrv = odrv_usb.connect(a)
    axis = odrv.axis0
    kts = m["kt"] * odrv_usb.torque_sign(axis)
    v_top = odrv_usb.max_speed(odrv.vbus_voltage, axis.motor.config.torque_constant)
    if a.speed > v_top:
        raise RuntimeError(f"Vbus {odrv.vbus_voltage:.1f} V에서는 {v_top:.1f} turn/s까지만 낼 수 있습니다. --speed를 낮추세요.")
    fet0 = odrv_usb.fet_temperature(axis)
    revs = a.speed * a.minutes * 60 / params.GEAR_RATIO
    print(f"\n이번 세션: {a.minutes:g}분, 모터 {a.speed:g} turn/s (관절 {params.turn_to_out_rad(a.speed):.1f} rad/s, "
          f"출력 {a.speed / params.GEAR_RATIO * 60:.0f} rpm), {a.reverse:g}초마다 방향 전환, 출력 약 {revs:.0f}바퀴")
    print(f"지금까지 길들이기: {prev_n}세션 {prev_min:.0f}분 (data/ID{a.id}/break_in.csv 기준)")
    print(f"보드 FET 온도: {'읽을 수 없음' if fet0 is None else f'{fet0:.0f} 도'}")
    print("!! 출력축이 계속 돕니다. 출력축에 아무것도 달지 말고 몸체를 고정하세요.")
    print("   가끔 모터·감속기 몸체를 만져 보고, 손대기 어려울 만큼 뜨거우면 Ctrl+C로 멈추세요 (3D 프린팅 부품).")
    util.confirm("시작할까요?")

    rows = []
    stop_reason = None
    t0 = None
    base_run = None
    with odrv_usb.RamSession(odrv) as s:
        s.set("motor.config.current_lim", a.current_lim)
        s.set("controller.config.vel_limit", a.speed * 1.3 + 2.0)
        s.set("controller.config.vel_integrator_gain", 0.0)
        s.set("controller.config.vel_ramp_rate", a.ramp)
        s.set("controller.config.control_mode", CONTROL_VELOCITY)
        s.set("controller.config.input_mode", INPUT_VEL_RAMP)
        axis.controller.input_vel = 0.0
        odrv_usb.enter_closed_loop(axis)
        sp = Spinner(odrv, axis, kts, a.ramp)
        t0 = time.time()

        def log(kind, v_cmd, v, tau):
            fet = odrv_usb.fet_temperature(axis)
            row = {"kind": kind, "t_min": (time.time() - t0) / 60, "v_cmd": v_cmd, "v": v, "tau": tau,
                   "fet": fet, "vbus": odrv.vbus_voltage}
            rows.append(row)
            if fet is not None and fet > a.max_fet:
                raise Stop(f"보드 FET 온도 {fet:.0f} 도 > {a.max_fet:g} 도")
            return row

        try:
            f = sp.check()
            log("check", SLOW, SLOW, f)
            print(f"\n[시작] 저속 마찰 {f:.3f} Nm (식은 상태 기준값)")
            next_check = a.check_every
            sign, pair = 1, []
            while (time.time() - t0) / 60 < a.minutes:
                v, tau = sp.spin(sign * a.speed, hold)
                row = log("run", sign * a.speed, v, tau)
                pair.append(abs(tau))
                if len(pair) == 2:
                    fr = sum(pair) / 2
                    base_run = base_run or fr
                    fet = row["fet"]
                    print(f"  {row['t_min']:5.1f}분  마찰 {fr:.3f} Nm (처음 {base_run:.3f}, {100 * (fr / base_run - 1):+.0f}%)"
                          f"  FET {'-' if fet is None else f'{fet:.0f}도'}  Vbus {row['vbus']:.1f} V")
                    if fr > a.max_rise * base_run:
                        raise Stop(f"마찰이 처음의 {fr / base_run:.2f}배로 커짐: 기어 손상·이물 확인")
                    pair = []
                sign = -sign
                if row["t_min"] >= next_check:
                    f = sp.check()
                    log("check", SLOW, SLOW, f)
                    f0 = rows[0]["tau"]
                    print(f"  [확인] 저속 마찰 {f:.3f} Nm (시작 {f0:.3f}, {100 * (f / f0 - 1):+.0f}%)")
                    if f > a.max_rise * f0:
                        raise Stop(f"저속 마찰이 시작의 {f / f0:.2f}배로 커짐: 기어 손상·이물 확인")
                    next_check += a.check_every
            if rows[-1]["kind"] != "check":  # 마지막이 방금 한 정기 확인이면 다시 재지 않음
                f = sp.check()
                log("check", SLOW, SLOW, f)
            print(f"[끝] 저속 마찰 {rows[-1]['tau']:.3f} Nm")
        except (Stop, RuntimeError) as e:
            stop_reason = str(e)
            print(f"\n!! 안전 정지: {stop_reason}")
        except KeyboardInterrupt:
            stop_reason = "Ctrl+C로 중단"
            print(f"\n{stop_reason}")
        finally:
            axis.controller.input_vel = 0.0
            time.sleep(abs(sp.prev) / a.ramp + 0.3)

    if not rows:
        return
    minutes = rows[-1]["t_min"]
    run = [abs(r["tau"]) for r in rows if r["kind"] == "run"]
    chk = [r["tau"] for r in rows if r["kind"] == "check"]
    n_avg = min(4, len(run) // 2) or 1
    run_first = statistics.fmean(run[:n_avg]) if run else float("nan")
    run_last = statistics.fmean(run[-n_avg:]) if run else float("nan")
    print(f"\n== 길들이기 ID{a.id}: {minutes:.1f}분 (누적 {prev_min + minutes:.0f}분) ==")
    if stop_reason:
        print(f"안전 정지로 끝남: {stop_reason}")
    print(f"길들이기 속도 마찰: 처음 {run_first:.3f} -> 끝 {run_last:.3f} Nm ({100 * (run_last / run_first - 1):+.0f}%)"
          "  (데워진 효과 포함)")
    if len(chk) >= 2:
        print(f"저속 마찰: 시작 {chk[0]:.3f} -> 끝 {chk[-1]:.3f} Nm ({100 * (chk[-1] / chk[0] - 1):+.0f}%)"
              "  (시작 값이 식은 상태 기준. 세션끼리는 '시작' 값을 비교)")
    temps = [r["fet"] for r in rows if r["fet"] is not None]
    if temps:
        print(f"보드 FET 온도: {temps[0]:.0f} -> 최고 {max(temps):.0f} 도")

    st = util.stamp()
    log_path = out_dir / f"break_in_{st}.csv"
    util.write_log(log_path, {"id": a.id, "motor": a.motor, "speed": a.speed, "reverse": a.reverse,
                              "minutes_planned": a.minutes, "prev_minutes": prev_min, "stop": stop_reason},
                   ["kind", "t_min", "v_cmd_turn_s", "v_turn_s", "tau_Nm", "fet_C", "vbus_V"],
                   [[r["kind"], round(r["t_min"], 3), r["v_cmd"], round(r["v"], 3), round(r["tau"], 4),
                     "" if r["fet"] is None else round(r["fet"], 1), round(r["vbus"], 2)] for r in rows])
    util.append_csv(out_dir / "break_in.csv", {
        "time": st, "motor": a.motor, "minutes": round(minutes, 2), "cum_minutes": round(prev_min + minutes, 2),
        "speed_turn_s": a.speed, "reverse_s": a.reverse,
        "slow_start_Nm": round(chk[0], 4) if chk else "", "slow_end_Nm": round(chk[-1], 4) if len(chk) > 1 else "",
        "run_first_Nm": round(run_first, 4), "run_last_Nm": round(run_last, 4),
        "fet_max_C": round(max(temps), 1) if temps else "", "stop": stop_reason or ""})
    if a.plot:
        plot(log_path.with_suffix(".png"), rows,
             f"ID{a.id} 길들이기 {minutes:.0f}분 (누적 {prev_min + minutes:.0f}분), {a.speed:g} turn/s")


if __name__ == "__main__":
    util.run(main)
