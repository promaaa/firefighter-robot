"""배선(모터 3상, DC 전원)을 바꾼 뒤 접촉 저항이 커지지 않았는지 확인한다. 보드에 저장하지 않는다.

    python check_wiring.py --id 03 --motor 6512 --note "XT30 커넥터"
    python check_wiring.py --id 03 --motor 6512 --repeat 8     # 재는 동안 선을 살살 흔들어 접촉 불량 찾기
    python check_wiring.py --id 03 --motor 6512 --skip-dc      # A(모터 3상)만

A. 모터 3상 (드라이버 A/B/C - 모터): 모터 보정(AXIS_STATE_MOTOR_CALIBRATION, calibration_current 5 A)으로
   상저항·인덕턴스를 다시 재서 기준과 비교한다. 기준은 보드에 저장된 값(setup_driver.py 6단계)이다.
   악어 케이블로 연결했을 때 0.513 Ohm, 같은 액추에이터를 납땜한 뒤 0.069 Ohm이었다.
B. DC 전원 (SMPS - 드라이버): 같은 측정을 큰 전류(기본 15 A)로 3초 동안 하면서 Vbus가 얼마나 내려가는지 본다.
   전원 경로 저항 = 전압 강하 / DC 전류. DC 전류는 모터에 들어간 전력(1.5 x I^2 x R)을 Vbus로 나눠 추정한다.
   이 저항으로 최대 DC 전류(dc_max_positive_current)일 때 Vbus가 저전압 차단에 닿는지 계산한다.

출력축은 거의 돌지 않는다 (전류 방향으로 정렬하며 1도 미만 움찔, 삑 소리). 끝나면 RAM에서 바뀐 값
(상저항·인덕턴스·calibration_current)을 원래 값으로 되돌린다. 결과는 data/ID{id}/wiring_check.csv에 쌓인다.
"""
import argparse
import math
import statistics
import time

from bench import odrv_usb, params, util

STATE_MOTOR_CAL = 4
LOAD_WINDOW = (0.6, 2.6)   # 보정 요청 뒤 저항 측정(3 s, motor.cpp) 중 평균 낼 구간 [s]
R_WARN, R_BAD = 1.15, 1.5  # 기준 대비 상저항 배율 (모터가 따뜻하면 구리 저항이 +0.4 %/도)
R_LOW = 0.85               # 기준보다 이만큼 작으면 기준 쪽을 의심
L_TOL = 0.2                # 인덕턴스 허용 변화 (비율)
SPREAD_WARN = 0.05         # 반복 측정 (최대-최소)/평균
PATH_OK, PATH_BAD = 0.10, 0.30  # DC 전원 경로 저항 [Ohm]
UV_MARGIN = 0.5            # 최대 DC 전류일 때 저전압 차단까지 남길 여유 [V]
SYS_V = 0.15               # 큰 전류 중 보드의 Vbus 읽기 치우침 [V]. ID02(옛 ID04) 정상 배선에서 부하 중 +150 mV로 읽힘 (2026-11-05)
LEVELS = ("정상", "주의", "문제")


def worse(a, b):
    return a if LEVELS.index(a) >= LEVELS.index(b) else b


def wait_idle(axis, timeout=3.0):
    if axis.current_state == odrv_usb.STATE_IDLE:
        return
    axis.requested_state = odrv_usb.STATE_IDLE
    t0 = time.time()
    while axis.current_state != odrv_usb.STATE_IDLE:
        if time.time() - t0 > timeout:
            raise RuntimeError("축이 IDLE이 되지 않습니다. 전원을 끄세요.")
        time.sleep(0.05)


def calibrate(axis, current, sample=None, timeout=20.0):
    """모터 보정 1회 (RAM). sample(t)가 있으면 도는 동안 계속 부른다. -> (R, L, errors)"""
    odrv_usb.clear_errors(axis)
    axis.motor.config.calibration_current = current
    axis.requested_state = STATE_MOTOR_CAL
    t0 = time.perf_counter()
    next_check, seen = t0 + 0.3, False
    while True:
        now = time.perf_counter()
        if sample:
            sample(now - t0)
        else:
            time.sleep(0.05)
        if now >= next_check:
            state = axis.current_state
            seen = seen or state == STATE_MOTOR_CAL
            if now - t0 > 1.0 and state == odrv_usb.STATE_IDLE:
                break
            next_check = now + 0.1
        if now - t0 > timeout:
            axis.requested_state = odrv_usb.STATE_IDLE
            raise RuntimeError(f"모터 보정이 {timeout:.0f}초 안에 끝나지 않았습니다")
    errs = odrv_usb.axis_errors(axis)
    if not seen and not errs:
        raise RuntimeError("모터 보정 상태로 들어가지 않았습니다 (측정이 안 됨). odrivetool에서 dump_errors(odrv0) 확인")
    mc = axis.motor.config
    return mc.phase_resistance, mc.phase_inductance, errs


def mean_vbus(odrv, duration):
    vs = []
    t_end = time.perf_counter() + duration
    while time.perf_counter() < t_end:
        vs.append(odrv.vbus_voltage)
        time.sleep(0.001)
    return statistics.fmean(vs), (statistics.stdev(vs) / math.sqrt(len(vs)) if len(vs) > 1 else 0.0)


def baseline(mc, motor):
    m = params.MOTORS[motor]
    if mc.pre_calibrated:
        return mc.phase_resistance, mc.phase_inductance, "보드에 저장된 값 (setup_driver.py 6단계에서 잰 값)"
    if m.get("phase_R_ref"):
        return m["phase_R_ref"], m["phase_L_ref"], f"{m['name']} 실측 평균 (bench/params.py, 이 보드는 보정 저장 안 됨)"
    return None, None, "기준 없음 (이 보드는 보정 저장 안 됨, 이 모터 실측값도 없음)"


def part_a(axis, motor, repeat):
    mc = axis.motor.config
    ref_r, ref_l, src = baseline(mc, motor)
    cur = mc.calibration_current
    print(f"\n[A. 모터 3상] {cur:g} A로 상저항·인덕턴스 {repeat}회 측정 (1회 약 4초, 삑 소리)")
    if ref_r:
        print(f"  기준: {ref_r * 1e3:.1f} mOhm, {ref_l * 1e6:.1f} uH  <- {src}")
    else:
        print(f"  {src}")
    if motor == "6512":
        print("  참고: M6C12 납땜 배선 두 대 67.9 / 68.9 mOhm, 악어 케이블일 때 513 mOhm")

    rs, ls = [], []
    for k in range(repeat):
        r, l, errs = calibrate(axis, cur)
        if errs:
            print(f"  {k + 1}/{repeat}  실패:\n    " + "\n    ".join(errs))
            for h in odrv_usb.error_hints(errs):
                print(f"    확인: {h}")
            lim = mc.resistance_calib_max_voltage / cur
            note = (f"상저항이 {lim * 1e3:.0f} mOhm보다 큼 (측정 한계 초과). 악어 케이블 때와 같은 증상"
                    if any("PHASE_RESISTANCE" in e for e in errs) else "보정 실패")
            return {"verdict": "문제", "notes": [note], "R": None, "ref_R": ref_r}
        tail = f"  (기준의 {r / ref_r:.2f}배)" if ref_r else ""
        print(f"  {k + 1}/{repeat}  R {r * 1e3:6.1f} mOhm   L {l * 1e6:5.1f} uH{tail}")
        rs.append(r)
        ls.append(l)
        time.sleep(0.3)

    r, l = statistics.fmean(rs), statistics.fmean(ls)
    spread = (max(rs) - min(rs)) / r
    verdict, notes = "정상", []
    if ref_r:
        ratio = r / ref_r
        if ratio > R_BAD:
            verdict = "문제"
            notes.append(f"상저항이 기준의 {ratio:.1f}배: 커넥터·납땜·단자에 접촉 저항이 있음")
        elif ratio > R_WARN:
            verdict = "주의"
            notes.append(f"상저항이 기준보다 {100 * (ratio - 1):.0f}% 큼: 방금까지 돌려서 모터가 따뜻하면 식힌 뒤 "
                         "다시 재고, 그래도 크면 연결부 확인")
        elif ratio < R_LOW:
            verdict = "주의"
            notes.append(f"상저항이 기준보다 {100 * (1 - ratio):.0f}% 작음: 기준을 잴 때 배선이 나빴을 수 있음. "
                         "지금 배선이 확실하면 setup_driver.py --from-step 6 으로 다시 보정·저장")
        if ref_l and abs(l / ref_l - 1) > L_TOL:
            verdict = worse(verdict, "주의")
            notes.append(f"인덕턴스가 기준의 {l / ref_l:.2f}배: 상 연결 상태 확인")
    if repeat > 1 and spread > SPREAD_WARN:
        verdict = worse(verdict, "주의")
        notes.append(f"측정마다 상저항이 {100 * spread:.0f}% 흔들림: 접촉이 불안정할 수 있음 (커넥터 체결, 냉납)")
    print(f"  평균 R {r * 1e3:.1f} mOhm, L {l * 1e6:.1f} uH, 반복 편차 {100 * spread:.1f}%")
    return {"verdict": verdict, "notes": notes, "R": r, "L": l, "ref_R": ref_r, "ref_L": ref_l,
            "spread": spread}


def part_b(odrv, axis, r_motor, want_current):
    mc = axis.motor.config
    cur = min(want_current, 0.6 * mc.resistance_calib_max_voltage / r_motor, mc.current_lim - 2.0)
    i_dc_guess = 1.5 * cur ** 2 * r_motor / max(odrv.vbus_voltage, 1.0)
    print(f"\n[B. DC 전원] {cur:.1f} A로 3초 (DC 약 {i_dc_guess:.1f} A) 흘리며 Vbus 강하 측정")
    v0, e0 = mean_vbus(odrv, 1.0)
    samples = []

    def sample(t):
        samples.append((t, odrv.vbus_voltage))
        time.sleep(0.001)

    r, _, errs = calibrate(axis, cur, sample)
    v1, e1 = mean_vbus(odrv, 1.0)
    if errs:
        print("  실패:\n    " + "\n    ".join(errs))
        for h in odrv_usb.error_hints(errs):
            print(f"    확인: {h}")
        return {"verdict": "문제", "notes": ["큰 전류에서 보정 실패: " + "; ".join(errs)], "I": cur}

    load = [v for t, v in samples if LOAD_WINDOW[0] <= t <= LOAD_WINDOW[1]]
    if len(load) < 20:
        return {"verdict": "주의", "notes": [f"부하 구간 Vbus 샘플이 {len(load)}개뿐이라 판단 못 함"], "I": cur}
    v_idle = (v0 + v1) / 2
    v_load = statistics.fmean(load)
    drop = v_idle - v_load
    i_dc = 1.5 * cur ** 2 * r / v_load
    # 불확실성: 평균의 표준오차 + 측정 전후 Vbus 변화의 절반 + 큰 전류 중 Vbus 읽기 치우침
    unc_v = (math.hypot(statistics.stdev(load) / math.sqrt(len(load)), math.hypot(e0, e1) / 2)
             + abs(v0 - v1) / 2 + SYS_V)
    unc_r = unc_v / i_dc
    measurable = drop > unc_v
    r_path = drop / i_dc if measurable else unc_r  # 못 재면 상한값으로 판단
    imax = odrv.config.dc_max_positive_current
    uv = odrv.config.dc_bus_undervoltage_trip_level
    v_min = v_idle - imax * r_path

    print(f"  Vbus 대기 {v_idle:.3f} V (전 {v0:.3f}, 후 {v1:.3f}) -> 부하 {v_load:.3f} V, 강하 {drop * 1e3:.0f} mV "
          f"(샘플 {len(load)}개)")
    if measurable:
        print(f"  DC 전류 약 {i_dc:.2f} A -> 전원 경로 저항 약 {r_path * 1e3:.0f} +- {unc_r * 1e3:.0f} mOhm "
              "(SMPS 출력 + 전원선 + 커넥터 + 보드 입력)")
    else:
        print(f"  DC 전류 약 {i_dc:.2f} A -> 강하가 측정 한계(+-{unc_v * 1e3:.0f} mV) 안: "
              f"전원 경로 저항 {unc_r * 1e3:.0f} mOhm 미만")
    print(f"  최대 DC 전류 {imax:g} A일 때 예상 Vbus {v_min:.2f} V{'' if measurable else ' 이상'} "
          f"(저전압 차단 {uv:g} V)")

    verdict, notes = "정상", []
    if v_min < uv + UV_MARGIN:
        verdict = "문제"
        notes.append(f"DC {imax:g} A에서 Vbus가 {v_min:.1f} V까지 내려가 저전압 차단({uv:g} V)에 걸릴 수 있음")
    if measurable and r_path > PATH_BAD:
        verdict = "문제"
        notes.append(f"전원 경로 저항 {r_path * 1e3:.0f} mOhm: 전원 커넥터·단자 조임, 선 굵기 확인")
    elif measurable and r_path > PATH_OK:
        verdict = worse(verdict, "주의")
        notes.append(f"전원 경로 저항 {r_path * 1e3:.0f} mOhm (정상이면 수십 mOhm): 커넥터·단자, 선 길이 확인")
    if i_dc < 0.8:
        notes.append(f"DC 전류가 {i_dc:.1f} A로 작아 해상도가 낮음 (+-{unc_r * 1e3:.0f} mOhm)")
    return {"verdict": verdict, "notes": notes, "I": cur, "i_dc": i_dc, "drop": drop, "r_path": r_path,
            "measurable": measurable, "unc": unc_r, "v_idle": v_idle, "v_min": v_min}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    ap.add_argument("--repeat", type=int, default=3, help="A 측정 횟수 (선을 흔들며 볼 때 8 정도)")
    ap.add_argument("--load-current", type=float, default=15.0,
                    help="B에서 흘릴 전류 [A]. 상저항에 맞춰 자동으로 줄임 (기본 15)")
    ap.add_argument("--skip-dc", action="store_true", help="B(DC 전원) 생략")
    ap.add_argument("--note", default="", help='이번 배선 설명 (CSV에 기록), 예: "XT30 커넥터"')
    a = ap.parse_args()

    odrv = odrv_usb.connect(a)
    axis = odrv.axis0
    print(f"축 상태 {odrv_usb.state_name(axis.current_state)}, 저장된 보정: motor {axis.motor.config.pre_calibrated}")
    errs = odrv_usb.axis_errors(axis)
    if errs:
        print("현재 에러 (측정 전에 지웁니다):\n  " + "\n  ".join(errs))
    print("\n!! 모터에 전류를 흘립니다. A: 5 A x 회당 4초, B: 최대 15 A x 3초.")
    print("   출력축은 1도 미만 움찔합니다. 손을 떼고, 모터·커넥터를 만지지 마세요.")
    if a.repeat > 3:
        print(f"   {a.repeat}회 재는 동안 전선의 피복 부분을 잡고 커넥터 근처를 살살 흔들어 보세요.")
    util.confirm("시작할까요?")

    res = {}
    with odrv_usb.RamSession(odrv) as s:
        for p in ("motor.config.phase_resistance", "motor.config.phase_inductance",
                  "motor.config.calibration_current"):
            s.remember(p)
        wait_idle(axis)
        res["A"] = part_a(axis, a.motor, max(1, a.repeat))
        if a.skip_dc:
            print("\n[B. DC 전원] --skip-dc 로 생략")
        elif res["A"]["verdict"] == "문제":
            print("\n[B. DC 전원] A가 '문제'라서 큰 전류는 흘리지 않음. 3상 연결부터 고친 뒤 다시 실행")
        else:
            res["B"] = part_b(odrv, axis, res["A"]["R"], a.load_current)
    if "A" not in res:
        return

    print("\n================ 결과 ================")
    for key, title in (("A", "A. 모터 3상"), ("B", "B. DC 전원")):
        if key not in res:
            continue
        r = res[key]
        print(f"{title}: {r['verdict']}")
        for n in r["notes"]:
            print(f"   - {n}")
    print("보드에 저장한 것은 없습니다 (RAM 값 원복).")

    A, B = res["A"], res.get("B", {})
    row = {
        "time": util.stamp(),
        "note": a.note,
        "motor": a.motor,
        "A": A["verdict"],
        "R_mOhm": round(A["R"] * 1e3, 2) if A.get("R") else "",
        "R_ref_mOhm": round(A["ref_R"] * 1e3, 2) if A.get("ref_R") else "",
        "R_spread_pct": round(A["spread"] * 100, 2) if "spread" in A else "",
        "L_uH": round(A["L"] * 1e6, 2) if "L" in A else "",
        "B": B.get("verdict", ""),
        "load_A": round(B["I"], 2) if "I" in B else "",
        "Idc_A": round(B["i_dc"], 3) if "i_dc" in B else "",
        "Vbus_V": round(B["v_idle"], 3) if "v_idle" in B else "",
        "drop_mV": round(B["drop"] * 1e3, 1) if "drop" in B else "",
        "Rpath_mOhm": ((f"{B['r_path'] * 1e3:.1f}" if B["measurable"] else f"<{B['r_path'] * 1e3:.0f}")
                       if "r_path" in B else ""),
        "Vmin_pred_V": round(B["v_min"], 2) if "v_min" in B else "",
    }
    util.append_csv(util.data_dir(a.id) / "wiring_check.csv", row)


if __name__ == "__main__":
    util.run(main)
