"""Payload check of the final design F2: statics, walking, push test and the drive model, with the shipped BHL policy.

    uv run python sim/payload/final_check.py           # about 10 min on 8 cores -> results/final_f2.md, final_f2.json
    uv run python sim/payload/final_check.py check     # self-check: F2 stands 3 s, statics of the bare biped (about 10 s)
    MUJOCO_GL=egl uv run python sim/payload/final_check.py video   # results/final_f2_walk.mp4, final_f2_push.mp4 (ffmpeg)

Needs models/bhl_biped.xml and models/bhl_F2*.xml: run models.py first. Statics, walking and the push test use
torso_tank.py. The drive cases use sim/sim2sim/hw_risks.py (K 20 / D 2 in the drive, zero errors, CAN latency).
"""
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "sim2sim")]
import torso_tank as tk  # noqa: E402
import hw_risks as hw  # noqa: E402

hw.MODEL = "bhl_F2"  # set before the pool forks, so every worker loads the design
OUT = HERE / "results"
LIMITS = (6.0, 9.0)
HW = ("1", "7", "7b")      # hw_risks cases: as trained, realistic drive, realistic + 10 ms observation latency
WH_MOTOR = 2 * 111.0       # Wh, two Zeee 6S 5000 mAh (cad/final/parts.json)
USABLE = 0.8               # ASSUMED: do not run a LiPo below 20 %. The drives also trip at 21 V under load.


def hw_job(cid, cmd_name, seed):
    log = []
    r = hw.simulate(hw.CASES[cid][1], hw.s2s.COMMANDS[cmd_name], seed, log=log)
    a = np.array(log)
    tau, qd = a[:, :12], a[:, 12:]
    p = (tau * qd + 1.5 * hw.R_PHASE * (tau / hw.N / hw.KT) ** 2).sum(axis=1)
    return {"case": cid, "cmd": cmd_name, "seed": seed, "fall": r["fall"], "vx": r["vx"], "peak": r["peak"],
            "p_mean": float(p.mean()), **hw.bus_metrics(tau, qd, hw.s2s.PHYSICS_DT)}


def tables(res):
    L = ["# Final design F2: payload check (tested in simulation)", "",
         "Model: the BHL biped with the parts of `cad/final/out_f2/layout.json`. The BHL battery (603 g) and PC (280 g) "
         "are taken out of the BHL torso mass. Policy: the shipped BHL policy `policy_biped_25hz_a.onnx`.", "",
         "## Statics", "", "| Model | Mass (kg) | Added (kg) | CoM x / y (mm) | CoM height (m) | Peak torque, stand (Nm) | "
         "Peak pitch torque, one leg (Nm) |", "|---|---|---|---|---|---|---|"]
    for v, r in res["static"].items():
        L.append(f"| {r['label']} | {r['mass']:.2f} | {r['add']:+.2f} | {r['com_dx'] * 1000:+.0f} / {r['com_dy'] * 1000:+.0f} | "
                 f"{r['com_h']:.3f} | {r['stand']:.1f} | {r['single_pitch']:.1f} |")
    L += ["", "## Walking, 20 s with observation noise, PD in software as trained", "",
          "| Model | Limit (Nm) | stand | vx 0.3 | vx 0.2 + yaw 0.5 |", "|---|---|---|---|---|"]
    for label, v in tk.ROWS:
        for lim in LIMITS:
            cells = []
            for c in tk.COMMANDS:
                w = next(w for w in res["walk"] if w["model"] == f"bhl_{v}" and w["cmd"] == c and w["limit"] == lim)
                cells.append(f"falls at {w['fall']:.1f} s" if w["fall"] else f"vx {w['vx']:+.2f}, wz {w['wz']:+.2f}")
            L.append(f"| {label} | {lim:.0f} | " + " | ".join(cells) + " |")
    L += ["", "## Push test: largest push with no fall (N s, worst of 3 push times)", "",
          "| Model | " + " | ".join(f"{c}, {d}" for c in tk.PUSH_CMDS for d in tk.PUSH_DIRS) + " |",
          "|---|" + "---|" * (len(tk.PUSH_CMDS) * len(tk.PUSH_DIRS))]
    for label, v in tk.ROWS:
        L.append(f"| {label} | " + " | ".join(f"{tk.push_worst(res['push'], v, c, d):.1f}" for c in tk.PUSH_CMDS
                                               for d in tk.PUSH_DIRS) + " |")
    L += ["", "## F2 with the drive model of `sim/sim2sim/hw_risks.py`", "",
          "Each case: 4 commands x 10 seeds of 20 s. Drive power = mechanical + copper loss (R = 0.1 ohm, ASSUMED).", "",
          "| Case | Falls | Mean vx at 0.3 m/s | Mean motor bus power, walking (W) | Lowest bus power (W) | "
          "Energy into the bus capacitors (J) |", "|---|---|---|---|---|---|"]
    for cid in HW:
        rs = [r for r in res["hw"] if r["case"] == cid]
        vx = [r["vx"] for r in rs if r["cmd"] == "vx 0.3" and r["vx"] is not None]
        pw = [r["p_mean"] for r in rs if r["cmd"] == "vx 0.3" and not r["fall"]]
        L.append(f"| {cid}: {hw.CASES[cid][0]} | {sum(r['fall'] is not None for r in rs)} of {len(rs)} | "
                 f"{np.mean(vx):.2f} | {np.mean(pw):.0f} | {min(r['p_bus'] for r in rs):.0f} | "
                 f"{max(r['e'] for r in rs):.2f} |")
    pw = np.mean([r["p_mean"] for r in res["hw"] if r["case"] == "7" and r["cmd"] == "vx 0.3" and not r["fall"]])
    res["runtime_h"] = WH_MOTOR * USABLE / pw
    L += ["", f"Motor battery time while walking (two 6S, {USABLE:.0%} usable, case 7 power): about "
          f"{res['runtime_h']:.1f} h. **Not verified:** drive standby power, motor iron losses and the real phase "
          "resistance are not in this number. Measure the bus current on the robot.", ""]
    return "\n".join(L)


def main():
    res = {"static": tk.static_rows()}
    print("statics done", flush=True)
    models_all = [v for _, v in tk.ROWS]
    with Pool(8) as pool:
        res["walk"] = pool.starmap(tk.walk_job, [(f"bhl_{v}", c, lim) for v in models_all for c in tk.COMMANDS
                                                 for lim in LIMITS])
        print("walking done", flush=True)
        res["push"] = pool.starmap(tk.push_job, [(v, c, d, t0) for v in models_all for c in tk.PUSH_CMDS
                                                 for d in tk.PUSH_DIRS for t0 in tk.PUSH_T], chunksize=1)
        print("push done", flush=True)
        res["hw"] = pool.starmap(hw_job, [(cid, c, s) for cid in HW for c in hw.s2s.COMMANDS for s in range(hw.SEEDS)],
                                 chunksize=1)
    md = tables(res)
    (OUT / "final_f2.json").write_text(json.dumps(res))
    (OUT / "final_f2.md").write_text(md)
    print(md)


def videos():
    tk.video("F2", tk.COMMANDS["vx 0.2 + yaw 0.5"], "final_f2_walk.mp4", xray=True)
    tk.video("F2", tk.PUSH_CMDS["walking 0.3 m/s"], "final_f2_push.mp4", seconds=8.0, push=(3.0, 6.0, tk.PUSH_DIRS["left"]))


def check():
    assert tk.policy_run("F2", tk.COMMANDS["stand"], seconds=3.0) is None, "F2 falls while it stands"
    r = tk.static_rows()["biped"]
    assert abs(r["add"]) < 1e-3 and r["stand"] < 6.0, r
    print("final_check.py check: OK (F2 stands 3 s, bare biped statics)")


if __name__ == "__main__":
    {"video": videos, "check": check}.get(sys.argv[1] if len(sys.argv) > 1 else "", main)()
