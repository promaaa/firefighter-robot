"""Sim-to-real risks of the course motor drives, for the shipped BHL policy on the final design F2 (bhl_F2).

The robot uses XDrive MINI drives (ODrive v3.6 clone, firmware v0.5.1). The drive does the PD at 8 kHz, in motor turns:
vel_sp = pos_gain (pos_set - pos) + vel_ff, torque = vel_gain (vel_sp - vel). With the gear ratio N = 15 this gives
joint K = N^2/(2 pi) pos_gain vel_gain and joint D = N^2/(2 pi) vel_gain. This script runs the run.py policy loop and
adds: per-joint K, D and torque limit, a joint zero error, command and observation latency, and the TRAP_TRAJ input mode.

    uv run python sim/sim2sim/hw_risks.py            all cases -> results/hw_risks.md, hw_risks.json, hw_risks*.png
    uv run python sim/sim2sim/hw_risks.py --report   tables and plots again from results/hw_risks.json
    uv run python sim/sim2sim/hw_risks.py --regen    regen analysis only, added to results/hw_risks.json, about 1 min
    uv run python sim/sim2sim/hw_risks.py --check    self-check, about 20 s
    MUJOCO_GL=egl uv run python sim/sim2sim/hw_risks.py --video   the two videos (needs ffmpeg and hw_risks.json)
"""
import json
import math
import os
import sys
import time
from collections import deque
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "payload")]
import run as s2s  # noqa: E402  sim/sim2sim/run.py
import report  # noqa: E402  sim/payload/report.py: palette and plot style

plt = report.plt
OUT = s2s.OUT
MODEL = "bhl_F2"  # final design F2: two full R2 tanks outside the torso (sim/payload/README.md)
N = 15  # BHL actuator gear ratio
# Leg joint order of run.py JOINTS: hip roll, hip yaw, hip pitch, knee (6512), ankle pitch, ankle roll (5010).
# Source: BHL robot_configuration.backup.json torque_constant 0.0919 (150KV 6512) and 0.1176 (110KV 5010).
IS_6512 = np.tile([True] * 4 + [False] * 2, 2)


def joint_gains(pos_gain, vel_gain):
    """ODrive pos_gain ((turn/s)/turn) and vel_gain (Nm/(turn/s)), motor side -> joint K (Nm/rad), D (Nm s/rad)."""
    return N * N / (2 * math.pi) * pos_gain * vel_gain, N * N / (2 * math.pi) * vel_gain


# Course config: omletkang/xdrive-mini-fw-instant-start xdrive_config.txt (read 2026-10-15). 6512 / 5010 values.
COURSE_K = np.where(IS_6512, joint_gains(12.6, 0.223)[0], joint_gains(20.0, 0.1)[0])
COURSE_D = np.where(IS_6512, joint_gains(12.6, 0.223)[1], joint_gains(20.0, 0.1)[1])
COURSE_LIM = 18.0 * N * np.where(IS_6512, 0.067, 0.0836)  # current_lim 18 A x torque_constant x N
TRAP_V, TRAP_A = 10 * 2 * math.pi / N, 5 * 2 * math.pi / N  # trap_traj vel_limit 10 turn/s, accel 5 turn/s^2 -> joint
OVERSPEED = 1.2 * 50 * 2 * math.pi / N  # rad/s at the joint: vel_limit 50 turn/s x vel_limit_tolerance 1.2 (default)
REGEN_W, BUS_W = 3.0 * 24.0, 12.0 * 24.0  # W: dc_max_negative_current 3 A, dc_max_positive_current 12 A, at 24 V

BASE = dict(K=s2s.KP, D=s2s.KD, lim=6.0, zero_deg=0.0, cmd_ms=0, obs_ms=0, trap=False)
COURSE = dict(K=COURSE_K, D=COURSE_D)
CASES = {  # id: (label, settings). The first character of the id is the case group (GROUPS).
    "1": ("baseline: K 20, D 2, 6 Nm (as trained)", BASE),
    "2": ("course gains, 18 A, PASSTHROUGH (course file run to the end)", {**BASE, **COURSE, "lim": COURSE_LIM}),
    "2b": ("course gains, 18 A, TRAP_TRAJ (course file section 4)", {**BASE, **COURSE, "lim": COURSE_LIM, "trap": True}),
    "3": ("course gains, 6 Nm", {**BASE, **COURSE}),
    "4a": ("zero error ±1°", {**BASE, "zero_deg": 1.0}),
    "4b": ("zero error ±2°", {**BASE, "zero_deg": 2.0}),
    "4c": ("zero error ±4°", {**BASE, "zero_deg": 4.0}),
    "5a": ("command latency 10 ms", {**BASE, "cmd_ms": 10}),
    "5b": ("command latency 20 ms", {**BASE, "cmd_ms": 20}),
    "5c": ("command latency 40 ms", {**BASE, "cmd_ms": 40}),
    "5d": ("command latency 60 ms (edge test)", {**BASE, "cmd_ms": 60}),
    "5e": ("observation latency 10 ms", {**BASE, "obs_ms": 10}),
    "6a": ("torque limit 4 Nm", {**BASE, "lim": 4.0}),
    "6b": ("torque limit 9 Nm", {**BASE, "lim": 9.0}),
    "7": ("realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency", {**BASE, "zero_deg": 2.0, "cmd_ms": 10}),
    "7b": ("realistic + 10 ms observation latency", {**BASE, "zero_deg": 2.0, "cmd_ms": 10, "obs_ms": 10}),
    "7c": ("course file (case 2) + ±2°, 10 ms command latency",
           {**BASE, **COURSE, "lim": COURSE_LIM, "zero_deg": 2.0, "cmd_ms": 10}),
    "7d": ("7b with a 9 Nm limit", {**BASE, "lim": 9.0, "zero_deg": 2.0, "cmd_ms": 10, "obs_ms": 10}),
}
SEEDS, PUSH_SEEDS = 10, 5  # noise sequence and zero errors per seed; the push time also changes with the seed
PUSH_DIRS = {"back": math.pi, "left": math.pi / 2}  # walking at 0.3 m/s. Sideways pushes are the weakest (payload push test)
PUSH_HI, PUSH_ITERS = 12.0, 5  # N s, bisection resolution 0.375 N s
VIDEO_WORST, VIDEO_REC = "7c", "7b"
COURSE_IDS = ("2", "2b", "3", "7c")


@lru_cache(maxsize=1)
def load():
    """Model and policy, once per worker process."""
    m = mujoco.MjModel.from_xml_path(str(s2s.MODELS / f"{MODEL}.xml"))
    m.opt.timestep = s2s.PHYSICS_DT
    m.actuator_forcerange[:] = [-30.0, 30.0]  # the model stops at 20 Nm; the drive limit below does the clipping
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1  # one thread per worker process
    return m, ort.InferenceSession(str(s2s.POLICY), so, providers=["CPUExecutionProvider"])


def trap_step(p, v, goal, dt):
    """One step of a trapezoidal profile toward goal: brake so that the setpoint stops at goal, |v| <= TRAP_V.
    Limit: replans every step from (p, v). The ODrive replans at each new input_pos; the shapes are close."""
    e = goal - p
    v_des = np.sign(e) * np.minimum(np.sqrt(2 * TRAP_A * np.abs(e)), TRAP_V)
    v = v + np.clip(v_des - v, -TRAP_A * dt, TRAP_A * dt)
    return p + v * dt, v


def simulate(cfg, command, seed, seconds=20.0, push=None, frames=None, log=None):
    """One run of the shipped policy on MODEL with the drive model cfg, with the training observation noise.
    push = (t0 s, impulse N s, direction rad in the heading frame): a horizontal force on the torso for 0.1 s.
    Returns the fall time and the metrics. The torque metrics start at 1 s, after the BHL start drop.
    log: optional list that gets [12 joint torques, 12 joint speeds] per physics step from 1 s."""
    m, policy = load()
    d = mujoco.MjData(m)
    d.qpos[7:19] = s2s.DEFAULT  # BHL reset, as run.py start="bhl"
    mujoco.mj_forward(m, d)
    rng = np.random.default_rng(seed)
    off = np.deg2rad(np.random.default_rng([seed, 1]).uniform(-1, 1, 12) * cfg["zero_deg"])  # measured - true
    K, D, lim = (np.broadcast_to(np.asarray(cfg[k], dtype=float), 12) for k in ("K", "D", "lim"))
    dt, sub = s2s.PHYSICS_DT, round(s2s.POLICY_DT / s2s.PHYSICS_DT)
    cmd_steps, obs_steps = round(cfg["cmd_ms"] / 1000 / dt), round(cfg["obs_ms"] / 1000 / dt)
    base, z0 = m.body("base").id, d.site("imu").xpos[2]
    c = np.array(command, dtype=np.float32)

    def reading():
        return (d.sensor("imu_quat").data.copy(), d.sensor("imu_gyro").data.copy(), d.qpos[7:19] + off, d.qvel[6:18].copy())
    hist = deque([reading()], maxlen=obs_steps + 1)  # hist[0] is the reading of obs_steps physics steps ago
    queue = deque()  # (physics step when the drive gets it, joint target)
    goal = s2s.DEFAULT.astype(float)  # the drive holds the stand pose until the first command arrives
    sp_p, sp_v = s2s.DEFAULT + off, np.zeros(12)  # TRAP_TRAJ setpoint, measured coordinates
    action = np.zeros(12, dtype=np.float32)
    n, fall, vx, wz = 0, None, [], []
    tau_sum, tau_n, peak, sat, qd_peak, p_min, p_max, rate_sq, rate_n = 0.0, 0, 0.0, 0, 0.0, 0.0, 0.0, 0.0, 0
    for _ in range(round(seconds / s2s.POLICY_DT)):
        obs = s2s.observation(c, *hist[0], action)
        obs += (rng.uniform(-1, 1, 45) * s2s.NOISE).astype(np.float32)
        prev, action = action, policy.run(None, {"obs": obs})[0][0]
        if d.time >= 1.0:
            rate_sq, rate_n = rate_sq + float(np.sum((action - prev) ** 2)), rate_n + 12
        queue.append((n + cmd_steps, s2s.DEFAULT + s2s.ACTION_SCALE * action))
        for _ in range(sub):
            while queue and queue[0][0] <= n:
                goal = queue.popleft()[1]
            qm, qd = d.qpos[7:19] + off, d.qvel[6:18]
            if cfg["trap"]:
                sp_p, sp_v = trap_step(sp_p, sp_v, goal, dt)
                tau = K * (sp_p - qm) + D * (sp_v - qd)
            else:
                tau = K * (goal - qm) - D * qd
            d.ctrl[:] = np.clip(tau, -lim, lim)
            if d.time >= 1.0:
                a = np.abs(d.ctrl)
                tau_sum, tau_n, peak = tau_sum + a.sum(), tau_n + 12, max(peak, float(a.max()))
                sat += bool(np.any(np.abs(tau) > lim))
                pw = d.ctrl * qd  # mechanical power at each joint, W
                qd_peak, p_min, p_max = max(qd_peak, float(np.abs(qd).max())), min(p_min, pw.min()), max(p_max, pw.max())
                if log is not None:
                    log.append(np.r_[d.ctrl, qd])
            d.xfrc_applied[base, :3] = 0
            if push and push[0] <= d.time < push[0] + 0.1:
                R = d.xmat[base].reshape(3, 3)
                ang = math.atan2(R[1, 0], R[0, 0]) + push[2]
                d.xfrc_applied[base, :2] = push[1] / 0.1 * np.array([math.cos(ang), math.sin(ang)])
            mujoco.mj_step(m, d)
            n += 1
            hist.append(reading())
            if frames and d.time * 30 >= frames.n:
                frames(m, d)
        t = d.time
        tilt = np.arccos(np.clip(-s2s.quat_rotate_inverse(d.sensor("imu_quat").data, np.array([0, 0, -1.0]))[2], -1, 1))
        if fall is None and (tilt > s2s.TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0):
            fall = t
        if fall is not None and (not frames or t > fall + 2.0):  # a video shows 2 s after the fall
            break
        if t >= 5.0 and fall is None:  # as run.py: speeds from 5 s, in the heading frame
            R = d.xmat[base].reshape(3, 3)
            yaw = np.arctan2(R[1, 0], R[0, 0])
            v = d.sensor("frame_vel").data
            vx.append(np.cos(yaw) * v[0] + np.sin(yaw) * v[1])
            wz.append((R @ d.qvel[3:6])[2])
    ok = vx and fall is None
    return dict(model=MODEL, command=list(command), seed=seed, limit=float(lim.max()), fall=fall,
                vx=float(np.mean(vx)) if ok else None, wz=float(np.mean(wz)) if ok else None,
                tau_mean=tau_sum / max(tau_n, 1), peak=peak, sat=100.0 * sat / max(tau_n // 12, 1),
                rate=s2s.ACTION_SCALE * math.sqrt(rate_sq / max(rate_n, 1)), qd_peak=qd_peak,
                p_regen=float(-p_min), p_drive=float(p_max))


def walk_job(cid, cmd_name, seed):
    return {"case": cid, "cmd": cmd_name, **simulate(CASES[cid][1], s2s.COMMANDS[cmd_name], seed)}


def push_job(cid, dir_name, seed):
    """Largest push (N s) with no fall in the 3 s after it, by bisection, walking at 0.3 m/s. 0 if it falls anyway."""
    cfg, cmd, t0 = CASES[cid][1], s2s.COMMANDS["vx 0.3"], 3.0 + 0.1 * seed  # 5 seeds span one step (0.5 s)
    alive = lambda j: simulate(cfg, cmd, seed, t0 + 3.0, push=(t0, j, PUSH_DIRS[dir_name]))["fall"] is None
    lo, hi = 0.0, PUSH_HI
    if not alive(0.0):
        return {"case": cid, "dir": dir_name, "seed": seed, "impulse": 0.0}
    for _ in range(PUSH_ITERS):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if alive(mid) else (lo, mid)
    return {"case": cid, "dir": dir_name, "seed": seed, "impulse": lo}


# Regen with the bus diode: the course BOM has a Schottky diode (MBRT400100) in the battery line, so regen current
# cannot go back into the batteries. All 12 drives share one bus. Their capacitors take the regen energy.
R_PHASE = 0.1  # ohm, GUESS for both motors: no datasheet value found for the MAD M6C12 150KV or the MAD 5010 110KV
KT = np.where(IS_6512, 0.067, 0.0836)  # Nm/A, course config torque_constant
C_BUS = 12 * 5 * 100e-6  # F: 5 x 100 uF per XDrive MINI (from photos), 12 drives
V_FULL, V_TRIP = 25.2, 26.0  # V: full 6S battery (diode drop ignored), course dc_bus_overvoltage_trip_level
REGEN_CASES = ("1", "7")


def bus_metrics(tau, qd, dt):
    """tau, qd: (steps, 12) joint torque and speed. Drive input power = mechanical + copper loss (1.5 R I^2, I = Iq)."""
    p = tau * qd + 1.5 * R_PHASE * (tau / N / KT) ** 2
    bus = p.sum(axis=1)
    edges = np.flatnonzero(np.diff(np.r_[0, (bus < 0).astype(int), 0]))  # start, end, start, end ... of negative stretches
    e = e_max = 0.0
    for b in bus:  # energy above the battery level in the capacitors: the diode blocks the way back
        e = max(0.0, e - b * dt)
        e_max = max(e_max, e)
    return dict(p_drive=float(p.min()), p_bus=float(bus.min()), e=e_max,
                t_neg=float((edges[1::2] - edges[::2]).max(initial=0) * dt))


def regen_job(cid, cmd_name, seed):
    log = []
    r = simulate(CASES[cid][1], s2s.COMMANDS[cmd_name], seed, log=log)
    a = np.array(log)
    return {"case": cid, "cmd": cmd_name, "seed": seed, "fall": r["fall"], **bus_metrics(a[:, :12], a[:, 12:], s2s.PHYSICS_DT)}


def add_regen(res):
    with Pool(os.cpu_count()) as pool:
        res["regen"] = pool.starmap(regen_job, [(c, k, s) for c in REGEN_CASES for k in s2s.COMMANDS for s in range(SEEDS)],
                                    chunksize=1)


def regen_table(res):
    L = ["| Case | Lowest drive input power (W) | Drive current at 24 V (A) | Runs with a drive below −3 A | Lowest bus power, "
         "12 drives (W) | Longest negative bus power (ms) | Largest energy into the capacitors (J) | Peak bus voltage from "
         f"{V_FULL} V (V) |", "|---|---|---|---|---|---|---|---|"]
    for cid in REGEN_CASES:
        rs = [r for r in res["regen"] if r["case"] == cid]
        pd, e = min(r["p_drive"] for r in rs), max(r["e"] for r in rs)
        L.append(f"| {cid} | {pd:.0f} | {pd / 24:.1f} | {sum(r['p_drive'] < -REGEN_W for r in rs)} of {len(rs)} "
                 f"| {min(r['p_bus'] for r in rs):.0f} | {1000 * max(r['t_neg'] for r in rs):.0f} | {e:.2f} "
                 f"| {math.sqrt(V_FULL ** 2 + 2 * e / C_BUS):.2f} |")
    return "\n".join(L) + "\n"


def summary(res, cid):
    w = [r for r in res["walk"] if r["case"] == cid]
    falls = [r["fall"] for r in w if r["fall"] is not None]
    mean = lambda rs, k: float(np.mean([r[k] for r in rs if r[k] is not None])) if any(r[k] is not None for r in rs) else None
    by = lambda c: [r for r in w if r["cmd"] == c]
    push = {k: [p["impulse"] for p in res["push"] if p["case"] == cid and p["dir"] == k] for k in PUSH_DIRS}
    return dict(runs=len(w), falls=len(falls), t_fall=float(np.mean(falls)) if falls else None,
                vx03=mean(by("vx 0.3"), "vx"), vx02=mean(by("vx 0.2 + yaw 0.5"), "vx"), wz02=mean(by("vx 0.2 + yaw 0.5"), "wz"),
                tau_mean=mean(w, "tau_mean"), peak=max(r["peak"] for r in w), sat=mean(w, "sat"), rate=mean(w, "rate"),
                qd_peak=max(r["qd_peak"] for r in w), p_regen=max(r["p_regen"] for r in w), p_drive=max(r["p_drive"] for r in w),
                push={k: (min(v), float(np.mean(v))) for k, v in push.items()})


HEAD = ("| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) "
        "| Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |")


def table_row(cid, s):
    f = lambda x, p=2: "-" if x is None else f"{x:.{p}f}"
    pb, pl = s["push"]["back"], s["push"]["left"]
    return (f"| {cid}: {CASES[cid][0]} | {s['falls']} of {s['runs']} | {f(s['t_fall'], 1)} | {f(s['vx03'])} "
            f"| {f(s['vx02'])}, {f(s['wz02'])} | {f(s['tau_mean'])} | {f(s['peak'], 1)} | {f(s['sat'], 0)} | {f(s['rate'], 3)} "
            f"| {pb[0]:.1f} / {pb[1]:.1f} | {pl[0]:.1f} / {pl[1]:.1f} |")


GROUPS = {"1": "Case 1: baseline", "2": "Case 2: course default gains, course current limit",
          "3": "Case 3: course default gains, 6 Nm", "4": "Case 4: joint zero error (baseline gains)",
          "5": "Case 5: latency (baseline gains)", "6": "Case 6: torque limit (baseline gains)",
          "7": "Case 7: combined realistic cases"}


def tables(res):
    S = {cid: summary(res, cid) for cid in CASES}
    L = []
    for g, title in GROUPS.items():
        L += [f"### {title}", "", HEAD, "|" + "---|" * 11]
        L += [table_row(cid, S[cid]) for cid in CASES if cid == "1" or (cid[0] == g and g != "1")]
        L.append("")
    L += ["### Drive limits that the simulation does not enforce", "",
          f"Above these values the drive stops with an error, and the joint goes limp. Overspeed: {OVERSPEED:.0f} rad/s at the "
          f"joint. Regen: about {REGEN_W:.0f} W per drive. Bus current: about {BUS_W:.0f} W per drive. "
          "Power is the mechanical power at one joint, without motor losses. The peaks include the runs that fall, up to the fall.",
          "",
          "| Case | Peak joint speed (rad/s) | Peak power into one drive, regen (W) | Peak power from one drive (W) |",
          "|---|---|---|---|"]
    L += [f"| {c} | {S[c]['qd_peak']:.1f} | {S[c]['p_regen']:.0f} | {S[c]['p_drive']:.0f} |" for c in CASES]
    L += ["", "### All runs per command (falls, mean vx, mean yaw rate of the runs without a fall)", "",
          "| Case | " + " | ".join(s2s.COMMANDS) + " |", "|---|" + "---|" * len(s2s.COMMANDS)]
    for cid in CASES:
        cells = []
        for c in s2s.COMMANDS:
            rs = [r for r in res["walk"] if r["case"] == cid and r["cmd"] == c]
            ok = [r for r in rs if r["fall"] is None]
            fl = len(rs) - len(ok)
            cells.append(f"{fl} falls" + (f", vx {np.mean([r['vx'] for r in ok]):+.2f}, yaw {np.mean([r['wz'] for r in ok]):+.2f}"
                                          if ok else ""))
        L.append(f"| {cid} | " + " | ".join(cells) + " |")
    return "\n".join(L) + "\n", S


def plots(S):
    ids = list(CASES)
    y = np.arange(len(ids))[::-1]
    fig, axs = plt.subplots(1, 4, figsize=(15, 5.5), sharey=True)
    runs = S["1"]["runs"]
    panels = [("Falls (of %d runs)" % runs, [S[c]["falls"] for c in ids]),
              ("Mean vx at a 0.3 m/s command (m/s)", [S[c]["vx03"] or 0 for c in ids]),
              ("Peak joint torque (Nm)", [S[c]["peak"] for c in ids]),
              ("Largest backward push, walking, mean of 5 (N s)", [S[c]["push"]["back"][1] for c in ids])]
    for ax, (title, v) in zip(axs, panels):
        ax.barh(y, v, color=[report.S2 if c in COURSE_IDS else report.S1 for c in ids], height=0.65)
        ax.axvline(v[0], color=report.INK2, lw=1, ls="--")  # baseline
        ax.set_title(title, fontsize=10, loc="left")
        ax.grid(axis="x", color=report.GRID)
        ax.set_axisbelow(True)
    axs[1].axvline(0.3, color=report.INK, lw=1)
    axs[0].set_yticks(y, [f"{c}: {CASES[c][0]}" for c in ids], fontsize=8)
    fig.suptitle(f"Drive risks, {MODEL}, shipped BHL policy, {SEEDS} seeds x 4 commands. Dashed line: baseline. "
                 "Orange: course drive gains.", fontsize=10, x=0.01, ha="left")
    fig.savefig(OUT / "hw_risks.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


REPORT_HEAD = """# Sim-to-real risks of the course motor drives

**Status:** tested in simulation (MuJoCo, CPU only). Nothing here is measured on board.

**Not verified:** The MuJoCo actuator model has no motor dynamics, no CAN jitter and no gearbox backlash.
The model has only a constant joint friction of 0.1 Nm and an armature of 0.005 kg m², from the BHL model.
We did not check the drive trip limits (last table) on an XDrive MINI. They come from the course config and the ODrive v0.5.1 defaults.

**Assumed:** Hip roll, hip yaw, hip pitch and knee use the 6512 motor. Ankle pitch and ankle roll use the 5010 motor.
Source: the BHL low-level config `robot_configuration.backup.json` (torque constant 0.0919 for the 150KV 6512, 0.1176 for the 110KV 5010).

## Setup

- Model: `bhl_F2` (`sim/payload/models/bhl_F2.xml`). This is the final design F2 with two full R2 tanks, 15.99 kg.
- Policy: the shipped BHL policy at 25 Hz, physics at 2 kHz, the training observation noise, and the BHL start with a 4 cm drop.
- `hw_risks.py` imports the `run.py` functions. With the baseline settings, it gives the same result as `run.py` `simulate()` (self-check).
- Commands: the 4 commands of `run.py`: stand, vx 0.3, yaw 0.5, and vx 0.2 + yaw 0.5. Each run lasts 20 s.
- Seeds: {seeds} seeds per command, so each case has {runs} runs. The seed sets the sensor noise and the zero errors.
- Push test: the robot walks at 0.3 m/s. A 0.1 s push hits the torso, backward or to the left.
  A bisection finds the largest push without a fall in the next 3 s (resolution 0.375 N s).
- The {push_seeds} push seeds put the push at 3.0 to 3.4 s, so it hits 5 phases of one step. "Worst" is the smallest of the 5 values.
  A worst value of 0.0 means that the robot fell without a push before 6 s.
- Drive model: the drive does the PD. Torque = K × (target − measured position) − D × velocity, clipped to the joint limit.
- Course gains: pos_gain 12.6 and vel_gain 0.223 (6512) give K = 100.6 Nm/rad and D = 8.0 Nm s/rad.
  pos_gain 20 and vel_gain 0.1 (5010) give K = 71.6 Nm/rad and D = 3.6 Nm s/rad.
  current_lim 18 A gives 18.1 Nm (6512) and 22.6 Nm (5010) at the joint.
- TRAP_TRAJ: the drive moves its setpoint to each new target at 4.2 rad/s and 2.1 rad/s² at most (joint side).
  These are trap_traj vel_limit 10 turn/s and accel 5 turn/s², divided by the gear ratio 15.
- Zero error: each joint gets a random offset, uniform in ±1°, ±2° or ±4°. The policy and the drive both see the offset position.
- Command latency: the drive gets the new target L ms after the policy computes it.
  Observation latency: the policy sees the sensor values of 10 ms before.
- Extra cases: 2b tests TRAP_TRAJ, because section 4 of the course config sets it. 5d finds the latency limit.
  7b to 7d support the recommendation.
- 8 kHz check: the real drive runs at 8 kHz. A one-off run of cases 2 and 3 at 8 kHz physics agreed with 2 kHz.
  That run used an earlier payload model and is not in the script.
- Run time: {minutes:.1f} min on 8 CPU cores.

Metrics:

- Falls: runs with a torso tilt above 0.78 rad or a large height drop, as in `run.py`. Mean time to fall uses only the runs that fall.
- vx: mean forward speed from 5 s to 20 s. Only runs without a fall count.
- Mean abs torque and peak torque: all 12 joints, from 1 s (after the start drop) to the end or the fall.
- Clipped steps: the share of physics steps where at least one joint asks for more than its limit.
- Action rate: the RMS change of the joint target between two policy steps, in rad. A higher value means a less smooth gait.

![Falls, speed, torque and push per case](hw_risks.png)

## Results

"""

REPORT_TAIL = """
## Findings

1. **The course gains do not suit this policy.** With K ≈ 101 / 72 Nm/rad and 18 A, 20 of 40 runs fall.
   vx reaches only 0.10 of 0.3 m/s. The stiff joints and the policy oscillate: the action rate almost doubles (0.214 against 0.127 rad).
   The peak joint speed reaches 45 rad/s. The drive trips at about 25 rad/s (overspeed), so the real robot would lose joints.
   With a 6 Nm limit (case 3), the course gains still give 21 falls in 40 runs and clip a torque in 87 % of the steps.
2. **TRAP_TRAJ makes the robot fall at once.** All 40 runs fall, after 1.6 s on average.
   The course trajectory limits (2.1 rad/s² at the joint) are far too slow for new targets every 40 ms.
   The last section of the course file sets PASSTHROUGH and saves it. So the input mode depends on how far the course file was run.
3. **Keep the joint zero error within ±2°.** ±1° changes nothing (1 fall in 40 runs, as the baseline). ±2° gives 4 falls. ±4° gives 10 falls.
   ±4° also cuts the worst backward push from 6.0 to 0.0 N s.
   The BHL training randomizes the default joint position by ±0.05 rad (±2.9°). This probably explains why ±2° works and ±4° does not.
4. **Keep the latency at 20 ms or less.** A command latency of 10 ms changes nothing.
   20 ms gives 2 falls in 40 runs, 40 ms gives 13, and 60 ms gives 28.
   An observation latency of 10 ms alone gives 1 fall. The shipped policy saw no delay in training.
5. **4 Nm is too low. 6 Nm works. 9 Nm adds margin.** With 4 Nm, 27 of 40 runs fall.
   With 9 Nm, no run falls, only 11 % of the steps clip, and the mean backward push rises from 8.6 to 11.6 N s.
6. **Small errors add up.** The realistic case 7 (matched gains, ±2°, 10 ms command latency, 6 Nm) falls in 1 of 40 runs.
   With 10 ms observation latency as well (case 7b), 7 of 40 runs fall, and the worst backward push drops from 4.1 to 0.8 N s.
   The course file with the same errors (case 7c) falls in 30 of 40 runs.

## Recommendation

Status: tested in simulation. Set these values on all 12 leg drives:

| Drive setting (ODrive v0.5.1) | Value | Joint value |
|---|---|---|
| `controller.config.control_mode` | `CONTROL_MODE_POSITION_CONTROL` | - |
| `controller.config.input_mode` | `INPUT_MODE_PASSTHROUGH` (1). Never `TRAP_TRAJ` (5) or `POS_FILTER` (3) | the target goes straight to the PD |
| `controller.config.pos_gain` | 10.0 (turn/s)/turn | K = 20 Nm/rad, as trained |
| `controller.config.vel_gain` | 0.0559 Nm/(turn/s) | D = 2 Nm s/rad, as trained |
| `controller.config.vel_integrator_gain` | 0 | no integral term |
| `motor.config.current_lim` | 6.0 A (6512), 4.8 A (5010) | 6 Nm, as trained |

- The 6 Nm limit uses the course torque constants (0.067 and 0.0836 Nm/A). `motor.config.torque_lim` = 0.4 Nm (motor side) gives the same limit, if v0.5.1 has it.
- Start with 6 Nm. 9 Nm (9.0 A and 7.2 A) also worked in simulation and gave more push margin.
  With all realistic errors, 9 Nm gave 1 fall in 40 runs against 7 for 6 Nm (case 7d against 7b). Check the printed gearboxes before you use 9 Nm.
- Zero tolerance: aim for ±1° on every joint. Accept ±2° at most.
  A one-turn error of the motor encoder is 24° at the joint. The start-up check must detect it and stop.
- Latency budget: at most 20 ms from the encoder sample to the new target in the drive, and aim for 10 ms.
  Read the encoders right before each policy step. Send the targets right after it.
- One CAN frame takes about 0.25 ms at 500 kbps. 12 frames per bus per policy step (6 targets, 6 encoder replies) take about 3 ms.
  So the CAN bus alone fits the budget. The USB-CAN adapters and Linux can add more. Log the loop times on the robot.
- For more margin, retrain the policy with a random action delay of 0 to 20 ms. The shipped policy saw no delay in training.

**Not verified:** bench checks before the first walk:

- Regen: the bus will probably trip on overvoltage in normal walking. See the section "Regen and the bus diode" below.
- Run `dump_errors(odrv0)` on each drive after the first walks. Look for overspeed and regen errors.
"""


REGEN_TEXT = """
## Regen and the bus diode

**Status:** tested in simulation.
**Not verified:** the diode layout, the bus capacitance and the motor phase resistance.
**Assumed:** the phase resistance is 0.1 Ω for both motors. We found no datasheet value for the MAD M6C12 150KV or the MAD 5010 110KV.

- The course BOM has a Schottky diode module (MBRT400100, dual, common cathode) in the battery line. It probably joins the two 6S batteries.
- So regen current cannot flow back into the batteries. All 12 drives share one 24 V bus, and only their capacitors can take the energy.
- Capacitance: 5 × 100 µF per XDrive MINI (from photos), so 6 mF for 12 drives.
- Input power of one drive = τ × ω + 1.5 × R × I², with I = τ / (15 × Kt). Kt is 0.067 Nm/A (6512) and 0.0836 Nm/A (5010).
- The bus power is the sum over the 12 drives. When it is negative, the capacitors charge above the battery voltage. The script tracks this energy at each step.
- Start voltage: 25.2 V, a full 6S battery. We ignore the diode drop. The course config trips at 26 V (overvoltage).
- Runs: cases 1 and 7, 4 commands × 10 seeds, 20 s each. Runs that fall count up to the fall.

{table}
- One drive alone is at its limit. The worst drive reaches −3.3 A in case 7, against the −3 A trip (1 of 40 runs).
- The shared bus is the real problem. The 12 drives together return up to 2.1 J in one stretch of up to 153 ms.
- Only 0.12 J lifts 6 mF from 25.2 V to 26 V. So the drives would trip on overvoltage in normal walking.
- The guesses do not change this. A one-off check used R = 0.05 to 0.2 Ω, with and without 1.5 W standby power per drive.
  The peak stayed at 26.9 to 38.7 V. That check used an earlier payload model and is not in the script.
- The mean regen power is small: about 1.4 W at most in one run (`docs/hardware/power_calc.md`). So a small resistor can take it.
- The course config sets `brake_resistance` = 0, so the drives themselves do not burn regen energy.

**Recommendation:** add a brake chopper on the shared 24 V bus before the first walk: about 5 Ω, 10 to 25 W, switched on above about 25.5 V.
Raise `dc_max_negative_current` to about −5 A only after the brake chopper works.
"""


def write(res):
    text, S = tables(res)
    (OUT / "hw_risks.md").write_text(REPORT_HEAD.format(minutes=res["minutes"], seeds=SEEDS, runs=4 * SEEDS, push_seeds=PUSH_SEEDS)
                                     + text + REPORT_TAIL
                                     + (REGEN_TEXT.format(table=regen_table(res)) if "regen" in res else ""))
    plots(S)
    print(text)


def main():
    t0 = time.time()
    walk = [(cid, c, s) for cid in CASES for c in s2s.COMMANDS for s in range(SEEDS)]
    push = [(cid, k, s) for cid in CASES for k in PUSH_DIRS for s in range(PUSH_SEEDS)]
    with Pool(os.cpu_count()) as pool:
        res = {"push": pool.starmap(push_job, push, chunksize=1), "walk": pool.starmap(walk_job, walk, chunksize=1)}
    res["minutes"] = (time.time() - t0) / 60
    res["cases"] = {cid: {"label": lab, **{k: np.asarray(v).tolist() if isinstance(v, np.ndarray) else v
                                            for k, v in cfg.items()}} for cid, (lab, cfg) in CASES.items()}
    add_regen(res)
    (OUT / "hw_risks.json").write_text(json.dumps(res))
    write(res)
    print(f"run time {res['minutes']:.1f} min, without the regen runs")


def video(cid, cmd_name, seed, out, seconds):
    """run.py video() with this drive model: run.py looks up simulate() at call time, so point it here."""
    s2s.simulate = lambda _m, command, _l, secs, frames=None, noise=False: simulate(CASES[cid][1], command, seed, secs, frames=frames)
    s2s.video(MODEL, cmd_name, 0, str(OUT / out), seconds)


def videos():
    res = json.load(open(OUT / "hw_risks.json"))
    w = sorted((r for r in res["walk"] if r["case"] == VIDEO_WORST and r["fall"] is not None), key=lambda r: r["fall"])
    r = w[len(w) // 2]  # the median fall, so the video shows a typical run
    video(VIDEO_WORST, r["cmd"], r["seed"], "hw_risks_worst.mp4", r["fall"] + 2.0)
    r = next(r for r in res["walk"] if r["case"] == VIDEO_REC and r["cmd"] == "vx 0.2 + yaw 0.5" and r["fall"] is None)
    video(VIDEO_REC, r["cmd"], r["seed"], "hw_risks_recommended.mp4", 10.0)


def check():
    K, D = joint_gains(12.6, 0.223)
    assert abs(K - 100.6) < 0.5 and abs(D - 7.99) < 0.05, (K, D)  # course config gains at the joint
    assert abs(COURSE_LIM[0] - 18.1) < 0.1 and abs(COURSE_LIM[4] - 22.6) < 0.1
    p, v = np.zeros(1), np.zeros(1)
    for _ in range(20000):  # 10 s: a 1 rad step takes 2 sqrt(1 / a) at a = 2.09 rad/s^2, below the speed limit
        p, v = trap_step(p, v, np.ones(1), s2s.PHYSICS_DT)
    assert abs(p[0] - 1) < 1e-3 and abs(v[0]) < 1e-2, (p, v)
    a = simulate(BASE, (0.3, 0.0, 0.0), 0, 6.0)
    b = s2s.simulate(MODEL, (0.3, 0.0, 0.0), 6.0, 6.0, noise=True)  # same noise seed, same loop -> same result
    assert a["fall"] is None and b["fall"] is None and abs(a["vx"] - b["vx"]) < 1e-6, (a, b)
    lat = simulate({**BASE, "cmd_ms": 40, "obs_ms": 10}, (0.0, 0.0, 0.0), 0, 2.0)
    assert lat["fall"] is None, lat
    print("check passed: gains, current limit, trap profile, baseline matches run.py, latency runs")


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--video" in sys.argv:
        videos()
    elif "--regen" in sys.argv:
        res = json.load(open(OUT / "hw_risks.json"))
        add_regen(res)
        (OUT / "hw_risks.json").write_text(json.dumps(res))
        print(regen_table(res))
    elif "--report" in sys.argv:
        write(json.load(open(OUT / "hw_risks.json")))
    else:
        main()
