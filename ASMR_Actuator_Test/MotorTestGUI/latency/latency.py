"""지연(latency)·응답 실험: Radxa에서 실제 로봇의 CAN 통신과 관절 응답을 잰다 (현실 기준, 시뮬과 비교하지 않음).

재는 것:
  rtt   읽기만  엔코더 요청→응답 왕복, 12개를 한꺼번에 읽는 데 걸리는 시간, 다 받았을 때 값의 나이
  loop  읽기만  제어 주기를 25~400 Hz로 훑으며 '12개 읽기 + 목표 12개 보내기'가 몇 Hz까지 되는지
  step  움직임  관절 하나에 계단 목표: 명령 → 드라이버가 목표를 적용(Iq 명령값 변화) → 전류 → 움직임 시작,
                상승 시간·오버슈트·정착 시간·정상 오차, '지연 + 2차계' 맞춤
  sine  움직임  관절 하나에 사인 목표를 정한 주기(기본 50 Hz)로: 주파수별 진폭비·위상(보드 선도), 같은 시간 지연

실행 (PowerShell, GUI의 Radxa 경유 연결은 끊어 둠):
  python latency\\latency.py run rtt
  python latency\\latency.py run loop
  python latency\\latency.py run step --joint left_knee_pitch [--yes]
  python latency\\latency.py run sine --joint left_knee_pitch [--yes] [--hz 50]
  python latency\\latency.py analyze logs\\latency\\<폴더>
  python latency\\latency.py report

run은 GUI 설정(config/robot_config.json)으로 joints.json(방향·영점·관절 범위)을 만들어 Radxa ~/motor_latency/에
can_bridge.py, latency_test.py와 함께 복사하고, 끝나면 기록을 MotorTestGUI/logs/latency/<날짜_시각_시험>/으로 가져와 분석한다.
"""
import argparse
import csv
import json
import math
import subprocess
import sys
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
GUI = HERE.parent
sys.path.insert(0, str(GUI))
sys.path.insert(0, str(GUI.parent / "BasicTest"))

import numpy as np  # noqa: E402

LOG_ROOT = GUI / "logs" / "latency"
REMOTE = "~/motor_latency"
SSH = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=8"]
NOWIN = getattr(subprocess, "CREATE_NO_WINDOW", 0)
TURN_DEG = 24.0


# ---------------------------------------------------------------------- 준비·실행
def build_joints():
    from motorgui.config import CONFIG_PATH, RobotConfig, driver_joint_gains
    from motorgui.sim import joint_infos, load_model
    cfg = RobotConfig.load(CONFIG_PATH)
    model, _ = load_model(cfg.mjcf_path(), visuals=False)
    infos = joint_infos(model, cfg)
    out = []
    for j in cfg.joints:
        if j.zero_turn is None:
            raise SystemExit(f"{j.short}: 영점이 없습니다. GUI에서 영점을 잡고 설정 저장 후 다시")
        info = infos[j.name]
        m = math.radians(cfg.limit_margin_deg)
        kp, kd = driver_joint_gains(j.motor)
        out.append({"name": j.name, "short": j.short, "node": j.node, "motor": j.motor, "direction": j.direction,
                    "zero_turn": j.zero_turn, "boot_ref_deg": j.boot_ref_deg, "torque_sign": j.torque_sign,
                    "kt": j.kt, "driver_kp": kp, "driver_kd": kd,
                    "range_deg": [math.degrees(info.lo + m), math.degrees(info.hi - m)]})
    return {"config": str(CONFIG_PATH), "made": datetime.now().isoformat(timespec="seconds"), "joints": out}


def ssh(host, cmd, **kw):
    return subprocess.run(["ssh", *SSH, host, cmd], capture_output=True, creationflags=NOWIN, **kw)


def deploy(host):
    files = {"can_bridge.py": (GUI / "radxa" / "can_bridge.py").read_bytes(),
             "latency_test.py": (GUI / "radxa" / "latency_test.py").read_bytes(),
             "joints.json": json.dumps(build_joints(), ensure_ascii=False, indent=1).encode("utf-8")}
    for name, data in files.items():
        r = ssh(host, f"mkdir -p {REMOTE} && cat > {REMOTE}/{name}", input=data, timeout=30)
        if r.returncode != 0:
            raise SystemExit(f"Radxa({host})에 복사 실패: {r.stderr.decode(errors='replace').strip()}\n"
                             "확인: ssh radxa hostname, Radxa 전원·랜선")
    print(f"Radxa {REMOTE}/에 복사: {', '.join(files)}")


def run(args, extra):
    test = args.test
    moving = test in ("step", "sine")
    if moving and not args.joint:
        raise SystemExit("step/sine에는 --joint가 필요합니다 (예: --joint left_knee_pitch)")
    deploy(args.host)
    if moving and not args.yes:
        print(f"\n[{test}] {args.joint} 관절이 움직입니다 (그 관절만 위치 제어, 끝나면 IDLE).")
        print("  - 로봇이 하네스에 매달려 있고 발이 바닥에 닿지 않습니까?")
        print("  - 그 다리 주변에 사람·물건이 없습니까?  - GUI(Radxa 경유)는 끊었습니까?")
        if input("  계속하려면 y: ").strip().lower() != "y":
            print("취소")
            return None
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"{stamp}_{test}" + (f"_{args.joint}" if moving else "") + (f"_{args.tag}" if args.tag else "")
    remote_out = f"runs/{name}"
    cmd = (f"cd {REMOTE} && exec python3 -u latency_test.py {test} --out {remote_out}"
           + (f" --joint {args.joint} --yes" if moving else "") + (" " + " ".join(extra) if extra else ""))
    print(f"Radxa에서 실행: {cmd}\n(Ctrl+C = 중단, 움직이던 관절은 IDLE)\n", flush=True)
    p = subprocess.Popen(["ssh", *SSH, args.host, cmd], stdin=subprocess.PIPE)
    try:
        rc = p.wait()
    except KeyboardInterrupt:
        p.stdin.close()
        p.wait(timeout=5)
        rc = 130
    local = LOG_ROOT / name
    local.mkdir(parents=True, exist_ok=True)
    got = 0
    for f in ("meta.json", "samples.csv"):
        r = ssh(args.host, f"cat {REMOTE}/{remote_out}/{f}", timeout=60)
        if r.returncode == 0 and r.stdout:
            (local / f).write_bytes(r.stdout)
            got += 1
    if got < 2:
        print(f"기록을 가져오지 못했습니다 (종료 코드 {rc})")
        return None
    print(f"\n기록: {local}", flush=True)
    analyze(local)
    return local


# ---------------------------------------------------------------------- 읽기
def load(d):
    d = Path(d)
    meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
    rows = []
    with open(d / "samples.csv", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append((r["kind"], int(r["node"]), int(r["t_tx_ns"]), int(r["t_rx_ns"]), float(r["v1"]), float(r["v2"])))
    return meta, rows


def pct(xs, p):
    return float(np.percentile(xs, p)) if len(xs) else float("nan")


def ms_stats(xs):
    xs = np.asarray([x for x in xs if x is not None], float)
    if not len(xs):
        return None
    return {"n": int(len(xs)), "mean": round(float(xs.mean()), 3), "p50": round(pct(xs, 50), 3),
            "p95": round(pct(xs, 95), 3), "max": round(float(xs.max()), 3), "min": round(float(xs.min()), 3)}


def plt_module():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    return plt


# ---------------------------------------------------------------------- 분석: 통신
def analyze_rtt(d, meta, rows):
    enc = [(n, tx, rx) for k, n, tx, rx, *_ in rows if k == "enc"]
    rtt = np.array([(rx - tx) / 1e6 for _, tx, rx in enc])
    allr = sorted((tx, rx) for k, n, tx, rx, *_ in rows if k == "enc_all")
    batches, cur = [], []
    for tx, rx in allr:                       # 같은 묶음 = 보낸 시각이 3 ms 안
        if cur and tx - cur[0][0] > 3_000_000:
            batches.append(cur)
            cur = []
        cur.append((tx, rx))
    if cur:
        batches.append(cur)
    one_way = float(np.median(rtt)) / 2.0 if len(rtt) else 0.0
    ages, durs = [], []
    for b in batches:
        t_done = max(rx for _, rx in b)
        durs.append((t_done - min(tx for tx, _ in b)) / 1e6)
        ages += [(t_done - rx) / 1e6 + one_way for _, rx in b]
    per_node = {}
    for n, tx, rx in enc:
        per_node.setdefault(n, []).append((rx - tx) / 1e6)
    res = {"rtt_ms": ms_stats(rtt), "rtt_p50_ms_per_node": {str(n): round(float(np.median(v)), 3)
                                                            for n, v in sorted(per_node.items())},
           "one_way_ms_est": round(one_way, 3), "read_12_ms": ms_stats(durs), "value_age_ms": ms_stats(ages),
           "note": "read_12 = 12개 요청을 보낸 뒤 마지막 응답까지. value_age = 12개를 다 받은 시점에서 각 값의 나이 "
                   "(받은 뒤 지난 시간 + 한쪽 전송 RTT/2)"}
    plt = plt_module()
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    ax[0].hist(rtt, bins=60, color="#3b7dd8")
    ax[0].set_xlabel("엔코더 요청→응답 [ms]")
    ax[0].set_ylabel("횟수")
    ax[0].set_title(f"중앙값 {np.median(rtt):.2f} ms, p95 {pct(rtt, 95):.2f} ms")
    if durs:
        ax[1].hist(durs, bins=50, color="#2a9d8f")
        ax[1].set_xlabel("12개 한꺼번에 읽기 [ms]")
        ax[1].set_title(f"중앙값 {np.median(durs):.2f} ms, p95 {pct(durs, 95):.2f} ms")
    fig.tight_layout()
    fig.savefig(d / "rtt.png", dpi=110)
    plt.close(fig)
    return res


def analyze_loop(d, meta, rows):
    by = {}
    for k, n, tx, rx, v1, v2 in rows:
        if k == "tick":
            by.setdefault(n, []).append((tx, rx, v1, v2))
    table = []
    for hz, ticks in sorted(by.items()):
        ticks.sort()
        t = np.array([x[0] for x in ticks], float) / 1e6
        per = np.diff(t)
        achieved = (len(ticks) - 1) / ((t[-1] - t[0]) / 1000.0) if len(ticks) > 1 else 0.0
        reads = np.array([x[2] for x in ticks])
        table.append({"hz": hz, "achieved_hz": round(achieved, 2), "ok": bool(achieved >= 0.98 * hz),
                      "period_std_ms": round(float(per.std()), 3) if len(per) else None,
                      "period_max_ms": round(float(per.max()), 3) if len(per) else None,
                      "read_12_p50_ms": round(pct(reads, 50), 3), "read_12_p99_ms": round(pct(reads, 99), 3),
                      "cycle_p99_ms": round(pct([(x[1] - x[0]) / 1e6 for x in ticks], 99), 3),
                      "incomplete_frac": round(float(np.mean([x[3] < 12 for x in ticks])), 4)})
    ok = [r["hz"] for r in table if r["ok"] and r["incomplete_frac"] < 0.01]
    res = {"rates": table, "max_ok_hz": max(ok) if ok else None,
           "note": "ok = 실제 주기가 목표의 98% 이상. max_ok_hz = 그러면서 12개를 못 다 읽은 틱이 1% 미만인 가장 높은 주기"}
    plt = plt_module()
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    hz = [r["hz"] for r in table]
    ax[0].plot(hz, hz, "k--", lw=0.8, label="목표")
    ax[0].plot(hz, [r["achieved_hz"] for r in table], "o-", label="실제")
    ax[0].set_xlabel("목표 주기 [Hz]")
    ax[0].set_ylabel("실제 주기 [Hz]")
    ax[0].legend()
    ax[1].plot(hz, [r["read_12_p50_ms"] for r in table], "o-", label="12개 읽기 p50")
    ax[1].plot(hz, [r["read_12_p99_ms"] for r in table], "s--", label="p99")
    ax[1].plot(hz, [1000.0 / h for h in hz], "k:", lw=0.8, label="주기")
    ax[1].set_xlabel("목표 주기 [Hz]")
    ax[1].set_ylabel("[ms]")
    ax[1].set_yscale("log")
    ax[1].legend()
    fig.suptitle(f"제어 루프 (12개 읽기 + 목표 12개), 가능한 최고 {res['max_ok_hz']} Hz")
    fig.tight_layout()
    fig.savefig(d / "loop.png", dpi=110)
    plt.close(fig)
    return res


# ---------------------------------------------------------------------- 분석: 관절 응답
def joint_series(meta, rows):
    """움직인 관절의 (명령 시각·목표[deg]), (표본 시각·각도[deg]·RTT), (표본 시각·Iq명령·Iq측정·RTT).
    표본 시각 = 요청과 응답의 가운데 (드라이버가 답한 순간의 추정)."""
    j = meta["joint"]
    k = meta["k"]
    node = j["node"]

    def q_of(p):
        return j["direction"] * (p + k - j["zero_turn"]) * TURN_DEG

    cmds = [(tx, q_of(v1)) for kind, n, tx, rx, v1, v2 in rows if kind == "cmd" and n == node]
    enc = [((tx + rx) / 2.0, q_of(v1), (rx - tx) / 1e6) for kind, n, tx, rx, v1, v2 in rows
           if kind == "enc" and n == node]
    iq = [((tx + rx) / 2.0, v1, v2, (rx - tx) / 1e6) for kind, n, tx, rx, v1, v2 in rows if kind == "iq" and n == node]
    return cmds, enc, iq


def _onset(t, v, t_cmd, thr_min, sd_mult=6.0):
    """명령 뒤 v가 처음 크게 바뀐 시각 [ms]: 마지막 '안 바뀐' 표본과 첫 '바뀐' 표본의 가운데. 없으면 (None, None)."""
    before = (t > t_cmd - 60e6) & (t < t_cmd)
    after = (t >= t_cmd) & (t < t_cmd + 80e6)
    if before.sum() < 3 or after.sum() < 2:
        return None, None
    base, sd = float(v[before].mean()), float(v[before].std())
    thr = max(thr_min, sd_mult * sd)
    ta, va = t[after], v[after]
    hit = np.nonzero(np.abs(va - base) > thr)[0]
    if not len(hit):
        return None, None
    i = hit[0]
    t_hit = ta[i]
    t_prev = ta[i - 1] if i > 0 else max(t[before][-1], t_cmd)
    return float((0.5 * (t_hit + t_prev) - t_cmd) / 1e6), float((t_hit - t_prev) / 1e6)


def step2(t, wn, z):
    """2차계 단위 계단 응답 (t < 0이면 0)."""
    t = np.maximum(t, 0.0)
    if z < 0.999:
        wd = wn * math.sqrt(1 - z * z)
        return 1 - np.exp(-z * wn * t) * (np.cos(wd * t) + z / math.sqrt(1 - z * z) * np.sin(wd * t))
    if z <= 1.001:
        return 1 - np.exp(-wn * t) * (1 + wn * t)
    s1 = -wn * (z - math.sqrt(z * z - 1))
    s2 = -wn * (z + math.sqrt(z * z - 1))
    return 1 + (s2 * np.exp(s1 * t) - s1 * np.exp(s2 * t)) / (s1 - s2)


def fit_delay_2nd(tg, xg):
    """정규화 계단 응답 xg(tg [s])에 x = step2(t - L) 맞춤 (격자 탐색). -> (L ms, wn, zeta, rms)."""
    best = None
    zs = np.concatenate([np.linspace(0.3, 1.0, 15), np.geomspace(1.05, 6.0, 25)])
    wns = np.geomspace(4.0, 400.0, 70)
    for L in np.arange(0.0, 0.0305, 0.0005):
        for z in zs:
            for wn in wns:
                r = float(np.sqrt(np.mean((step2(tg - L, wn, z) - xg) ** 2)))
                if best is None or r < best[3]:
                    best = (L * 1000.0, float(wn), float(z), r)
    return best


def fopdt(t, L, tau):
    """순수 지연 L [s] + 1차계 시간 상수 tau [s]의 단위 계단 응답."""
    return np.where(t > L, 1.0 - np.exp(-(t - L) / tau), 0.0)


def fit_fopdt(tg, xg):
    """xg(tg [s])에 '지연 + 1차계' 맞춤 (격자 탐색). -> (L ms, tau ms, rms)."""
    best = None
    for L in np.arange(0.0, 0.0402, 0.00025):
        for tau in np.geomspace(0.005, 0.6, 240):
            r = float(np.sqrt(np.mean((fopdt(tg, L, tau) - xg) ** 2)))
            if best is None or r < best[2]:
                best = (L * 1000.0, tau * 1000.0, r)
    return best


def analyze_step(d, meta, rows):
    cmds, enc, iq = joint_series(meta, rows)
    if len(cmds) < 3 or len(enc) < 50:
        return {"error": "기록이 모자랍니다"}
    et = np.array([e[0] for e in enc])
    eq = np.array([e[1] for e in enc])
    it = np.array([e[0] for e in iq])
    iset = np.array([e[1] for e in iq])
    imeas = np.array([e[2] for e in iq])
    hold_s = float(meta["args"].get("hold", 1.0))
    steps = []
    for (t_cmd, q_tgt), (t_prev, q_prev) in zip(cmds[1:], cmds[:-1]):
        dq = q_tgt - q_prev
        if abs(dq) < 0.5:
            continue
        before = (et > t_cmd - 100e6) & (et < t_cmd)
        win = (et >= t_cmd) & (et < t_cmd + hold_s * 1e9 * 0.98)
        if before.sum() < 5 or win.sum() < 30:
            continue
        q_b = float(eq[before].mean())
        ta = (et[win] - t_cmd) / 1e6
        x_cmd = (eq[win] - q_b) / dq              # 명령한 크기 기준
        t_set, res_set = _onset(it, iset, t_cmd, 0.3)
        t_meas, _ = _onset(it, imeas, t_cmd, 0.5)
        t_move, _ = _onset(et, eq, t_cmd, 0.1)
        tail = ta > ta[-1] - 200
        x_end = float(x_cmd[tail].mean())         # 실제로 간 비율 (마찰 등으로 1보다 작음)
        x = x_cmd / x_end if x_end > 0.3 else x_cmd   # 상승·정착은 실제 최종값 기준

        def cross(level, x=x, ta=ta):
            idx = np.nonzero(x >= level)[0]
            return float(ta[idx[0]]) if len(idx) else None

        out_band = np.nonzero(np.abs(x - 1.0) > 0.05)[0]
        settle = float(ta[out_band[-1]]) if len(out_band) else 0.0
        ipk = None
        iw = (it >= t_cmd) & (it < t_cmd + 300e6)
        if iw.sum():
            ipk = float(imeas[iw][np.argmax(np.abs(imeas[iw]))])
        steps.append({"t_cmd": t_cmd, "dir": "+" if dq > 0 else "-", "dq": dq, "t": ta, "x": x,
                      "cmd_to_iqset_ms": t_set, "iqset_res_ms": res_set, "cmd_to_iq_ms": t_meas,
                      "cmd_to_move_ms": t_move, "t10": cross(0.1), "t50": cross(0.5), "t90": cross(0.9),
                      "overshoot": float(max(0.0, x.max() - 1.0)), "settle_5pct_ms": settle,
                      "reached_frac": x_end, "steady_err_deg": float((1.0 - x_end) * dq), "iq_peak_A": ipk,
                      "err_to_target_deg": float((eq[win][tail].mean() - q_tgt) * np.sign(dq))})
    if not steps:
        return {"error": "계단을 찾지 못했습니다"}

    def col(key, dirn=None):
        return [s[key] for s in steps if s[key] is not None and (dirn is None or s["dir"] == dirn)]

    grid = np.arange(0.0, 0.6, 0.001)
    fits = {}
    for dirn in ("+", "-"):
        ss = [s for s in steps if s["dir"] == dirn]
        if not ss:
            continue
        xm = np.mean([np.interp(grid * 1000.0, s["t"], s["x"]) for s in ss], axis=0)
        L, tau, r = fit_fopdt(grid, xm)
        fits[dirn] = {"delay_ms": round(L, 2), "tau_ms": round(tau, 1), "fit_rms": round(r, 4)}
    j = meta["joint"]
    pd_tau = 1000.0 * j["driver_kd"] / j["driver_kp"] if j.get("driver_kp") else None
    # 목표 대비 끝 오차 (음수 = 못 미침). 두 방향 평균 = 마찰분, 차이의 반 = 중력 등 한쪽 치우침
    e_p, e_m = col("err_to_target_deg", "+"), col("err_to_target_deg", "-")
    stick = None
    if e_p and e_m:
        f_deg = -(float(np.mean(e_p)) + float(np.mean(e_m))) / 2
        g_deg = (float(np.mean(e_m)) - float(np.mean(e_p))) / 2
        stick = {"friction_part_deg": round(f_deg, 3), "bias_part_deg": round(g_deg, 3),
                 "friction_torque_Nm": round(j["driver_kp"] * math.radians(f_deg), 3) if j.get("driver_kp") else None}
    res = {"joint": j["short"], "motor": j["motor"], "steps": len(steps), "step_deg": round(abs(steps[0]["dq"]), 2),
           "start_deg": round(meta.get("q0_deg", float("nan")), 2),
           "driver_gains_assumed": {"kp_nm_rad": j.get("driver_kp"), "kd_nm_s_rad": j.get("driver_kd")},
           "cmd_to_iqset_ms": ms_stats(col("cmd_to_iqset_ms")),
           "iqset_sample_gap_ms": ms_stats(col("iqset_res_ms")),
           "cmd_to_iq_meas_ms": ms_stats(col("cmd_to_iq_ms")),
           "cmd_to_motion_ms": ms_stats(col("cmd_to_move_ms")),
           "t10_ms": ms_stats(col("t10")), "t50_ms": ms_stats(col("t50")), "t90_ms": ms_stats(col("t90")),
           "rise_10_90_ms": ms_stats([s["t90"] - s["t10"] for s in steps if s["t90"] and s["t10"]]),
           "overshoot_frac": ms_stats(col("overshoot")), "settle_5pct_ms": ms_stats(col("settle_5pct_ms")),
           "reached_frac": {dn: ms_stats(col("reached_frac", dn)) for dn in ("+", "-")},
           "steady_err_deg": {dn: ms_stats(col("steady_err_deg", dn)) for dn in ("+", "-")},
           "err_to_target_deg": {dn: ms_stats(col("err_to_target_deg", dn)) for dn in ("+", "-")},
           "stiction_estimate": stick,
           "iq_peak_A": {dn: ms_stats(col("iq_peak_A", dn)) for dn in ("+", "-")},
           "fit_delay_1st_order": fits,
           "pd_time_constant_ms": round(pd_tau, 1) if pd_tau else None,
           "rtt_ms_during": ms_stats([e[2] for e in enc] + [i[3] for i in iq]),
           "note": ("cmd_to_iqset = 명령을 보낸 뒤 드라이버의 전류 명령값이 바뀐 시각 = 명령이 드라이버 PD에 닿은 시각. "
                    "첫 Iq 표본에서 이미 바뀌어 있으면 실제 값은 0~(표본 시각) 사이 (iqset_sample_gap). "
                    "t10·t50·t90·정착·오버슈트는 실제 최종값 기준 (reached_frac = 실제 출발점에서 명령한 크기 중 간 비율; 출발점도 앞 계단에서 "
                    "덜 온 자리라 steady_err_deg는 마찰 오차의 약 2배). err_to_target_deg = 목표 대비 끝 오차, stiction_estimate = "
                    "그 두 방향 평균(마찰분)과 Kp로 환산한 마찰 토크. "
                    "fit = 평균 응답에 '순수 지연 + 1차계' 맞춤. pd_time_constant = 설정 게인의 Kd/Kp (관성이 작을 때의 시간 상수)")}
    plt = plt_module()
    fig, ax = plt.subplots(1, 3, figsize=(15, 3.8))
    colors = {"+": "#3b7dd8", "-": "#e76f51"}
    for s in steps:
        ax[0].plot(s["t"], s["x"], color=colors[s["dir"]], alpha=0.3, lw=0.8)
    for dn, f in fits.items():
        ax[0].plot(grid * 1000, fopdt(grid, f["delay_ms"] / 1000, f["tau_ms"] / 1000), color="k",
                   lw=1.2, ls="--" if dn == "-" else "-",
                   label=f"맞춤 {dn}: 지연 {f['delay_ms']} ms, τ {f['tau_ms']} ms")
    if pd_tau:
        ax[0].plot(grid * 1000, fopdt(grid, 0.0, pd_tau / 1000), color="#999", lw=1, ls=":",
                   label=f"설정 게인 Kd/Kp = {pd_tau:.0f} ms")
    ax[0].set_xlim(-10, 500)
    ax[0].set_xlabel("명령 후 [ms]")
    ax[0].set_ylabel("정규화 변위 (최종값 기준)")
    ax[0].set_title(f"{res['joint']} 계단 ±{res['step_deg']}° (파랑 +, 주황 −)")
    ax[0].legend(fontsize=8)
    for s in steps[:8]:
        m = (it > s["t_cmd"] - 10e6) & (it < s["t_cmd"] + 40e6)
        ax[1].plot((it[m] - s["t_cmd"]) / 1e6, iset[m], ".-", ms=3, lw=0.7, alpha=0.8, color=colors[s["dir"]])
    ax[1].axvline(0, color="k", lw=0.8)
    ax[1].set_xlabel("명령 후 [ms]")
    ax[1].set_ylabel("Iq 명령값 [A]")
    p50 = res["cmd_to_iqset_ms"]["p50"] if res["cmd_to_iqset_ms"] else "-"
    ax[1].set_title(f"드라이버가 목표를 적용한 시각: 중앙값 {p50} ms")
    v = col("cmd_to_iqset_ms")
    if v:
        ax[2].hist(v, bins=15, color="#e9c46a", label="명령→Iq 명령값")
    v = col("cmd_to_move_ms")
    if v:
        ax[2].hist(v, bins=15, color="#2a9d8f", alpha=0.7, label="명령→움직임 0.1°")
    ax[2].set_xlabel("[ms]")
    ax[2].legend()
    fig.tight_layout()
    fig.savefig(d / "step.png", dpi=110)
    plt.close(fig)
    return res


def fit_sine(t, y, w):
    A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ c
    return math.hypot(c[0], c[1]), math.atan2(c[1], c[0]), float(np.sqrt(np.mean(resid ** 2)))


def analyze_sine(d, meta, rows):
    cmds, enc, iq = joint_series(meta, rows)
    ct = np.array([c[0] for c in cmds])
    cq = np.array([c[1] for c in cmds])
    et = np.array([e[0] for e in enc])
    eq = np.array([e[1] for e in enc])
    hz = float(meta["args"].get("hz") or 50)
    out = []
    for seg in meta.get("segments", []):
        f, t0, dur = seg["freq"], seg["t0_ns"], seg["dur_s"]
        w = 2 * math.pi * f
        lo, hi = t0 + 1e9 / f, t0 + (dur - 1.0 / f) * 1e9          # 진폭을 키우고 줄이는 첫·끝 주기는 뺌
        me = (et >= lo) & (et < hi)
        mc = (ct >= lo) & (ct < hi)
        if mc.sum() < 6 or me.sum() < 20:
            continue
        a_r, ph_r, res_r = fit_sine((et[me] - t0) / 1e9, eq[me], w)
        a_c, ph_c, _ = fit_sine((ct[mc] - t0) / 1e9, cq[mc], w)
        lag = (ph_c - ph_r) % (2 * math.pi)
        out.append({"freq_hz": f, "amp_cmd_deg": round(a_c, 3), "amp_ratio": round(a_r / a_c, 4),
                    "phase_lag_deg": round(math.degrees(lag), 2), "time_lag_ms": round(lag / w * 1000.0, 2),
                    "zoh_half_period_ms": round(500.0 / hz, 2), "fit_rms_deg": round(res_r, 3)})
    j = meta["joint"]
    res = {"joint": j["short"], "motor": j["motor"], "cmd_rate_hz": hz, "center_deg": meta.get("center_deg"),
           "per_freq": out,
           "note": ("amp_ratio·phase_lag = 목표 사인 대비 실측 (보드 선도). time_lag = 위상 지연을 시간으로. "
                    "목표는 cmd_rate_hz마다 바뀌는 계단이라 평균 반 주기(zoh_half_period)만큼은 명령 쪽에서 생기는 지연")}
    if out:
        plt = plt_module()
        fig, ax = plt.subplots(1, 3, figsize=(14, 3.4))
        fr = [o["freq_hz"] for o in out]
        ax[0].semilogx(fr, [20 * math.log10(o["amp_ratio"]) for o in out], "o-")
        ax[0].set_xlabel("주파수 [Hz]")
        ax[0].set_ylabel("진폭비 [dB]")
        ax[0].grid(True, which="both", alpha=0.3)
        ax[1].semilogx(fr, [-o["phase_lag_deg"] for o in out], "o-")
        ax[1].set_xlabel("주파수 [Hz]")
        ax[1].set_ylabel("위상 [deg]")
        ax[1].grid(True, which="both", alpha=0.3)
        ax[2].plot(fr, [o["time_lag_ms"] for o in out], "o-")
        ax[2].set_xlabel("주파수 [Hz]")
        ax[2].set_ylabel("시간 지연 [ms]")
        fig.suptitle(f"{res['joint']} 사인 응답 (목표를 {hz:g} Hz로 보냄)")
        fig.tight_layout()
        fig.savefig(d / "sine.png", dpi=110)
        plt.close(fig)
    return res


def analyze(d):
    d = Path(d)
    meta, rows = load(d)
    test = meta["test"]
    fn = {"rtt": analyze_rtt, "loop": analyze_loop, "step": analyze_step, "sine": analyze_sine}[test]
    res = fn(d, meta, rows)
    res = {"test": test, "dir": d.name, "radxa_result": meta.get("result"), **res}
    (d / "summary.json").write_text(json.dumps(res, ensure_ascii=False, indent=2, default=float), encoding="utf-8")
    show = {k: v for k, v in res.items() if k not in ("radxa_result", "note", "rtt_p50_ms_per_node")}
    print(json.dumps(show, ensure_ascii=False, indent=1, default=float))
    if res.get("note"):
        print("※", res["note"])
    return res


def report():
    for s in sorted(LOG_ROOT.glob("*/summary.json")):
        r = json.loads(s.read_text(encoding="utf-8"))
        t = r["test"]
        try:
            if t == "rtt":
                text = (f"왕복 p50 {r['rtt_ms']['p50']} / p95 {r['rtt_ms']['p95']} ms, 12개 읽기 p50 "
                        f"{r['read_12_ms']['p50']} / p95 {r['read_12_ms']['p95']} ms, 값 나이 p50 {r['value_age_ms']['p50']} ms")
            elif t == "loop":
                text = f"가능한 최고 주기 {r['max_ok_hz']} Hz | " + ", ".join(
                    f"{x['hz']}→{x['achieved_hz']:.0f}" + ("" if x["ok"] else "✗") for x in r["rates"])
            elif t == "step":
                fit = r.get("fit_delay_1st_order", {})
                text = (f"{r['joint']}: 명령→Iq명령 p50 {(r['cmd_to_iqset_ms'] or {}).get('p50')} ms, 명령→움직임 p50 "
                        f"{(r['cmd_to_motion_ms'] or {}).get('p50')} ms, t50 p50 {(r['t50_ms'] or {}).get('p50')} ms | "
                        + ", ".join(f"맞춤{dn} L {f['delay_ms']} ms τ {f['tau_ms']} ms" for dn, f in fit.items())
                        + f" (Kd/Kp {r.get('pd_time_constant_ms')} ms)")
            elif t == "sine":
                text = f"{r['joint']} @{r['cmd_rate_hz']:g} Hz: " + ", ".join(
                    f"{o['freq_hz']:g} Hz ×{o['amp_ratio']:.2f} {o['time_lag_ms']:.0f} ms" for o in r["per_freq"])
            else:
                continue
        except (KeyError, TypeError):
            text = "(요약 불완전)"
        print(f"{r['dir']:48s} {text}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("test", choices=["rtt", "loop", "step", "sine"])
    r.add_argument("--joint")
    r.add_argument("--yes", action="store_true")
    r.add_argument("--host", default="radxa")
    r.add_argument("--tag", default="", help="기록 폴더 이름 끝에 붙일 말 (예: 100hz)")
    a = sub.add_parser("analyze")
    a.add_argument("dir")
    sub.add_parser("report")
    sub.add_parser("deploy").add_argument("--host", default="radxa")
    args, extra = ap.parse_known_args()
    if args.cmd == "run":
        run(args, extra)
    elif args.cmd == "analyze":
        analyze(args.dir)
    elif args.cmd == "report":
        report()
    else:
        deploy(args.host)


if __name__ == "__main__":
    main()
