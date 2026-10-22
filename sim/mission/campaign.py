"""Monte Carlo campaign: the same random missions for each configuration, 3 processes in parallel.

    MUJOCO_GL=egl uv run python sim/mission/campaign.py run f2_kit_real 50             50 missions of configuration "f2_kit_real"
    MUJOCO_GL=egl uv run python sim/mission/campaign.py run f2_kit_real 50 3 reverse   a second runner (3 processes), from the last seed
    uv run python sim/mission/campaign.py report                                    tables and figures from all finished missions

Each mission writes ~/runs/firefighter/mission/campaign/<config>/<seed>.json, so a stopped campaign resumes.
The report writes results/campaign.md, results/campaign.json and results/fig_*.png.
"""
import json
import math
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNS = Path.home() / "runs" / "firefighter" / "mission" / "campaign"
OUT = HERE / "results"
SEED0 = 1000  # mission seeds SEED0, SEED0 + 1, ...: every configuration gets the same missions
SHIPPED = ROOT / "sim" / "sim2sim" / "policy_biped_25hz_a.onnx"
BASELINE = ROOT / "sim" / "sim2sim" / "baseline_detector.py"
YOLO = ROOT / "vision" / "detector.py"  # YOLO11n fire detector
CONFIGS = {  # name: (detector, policy, leg torque limit Nm, design, label[, extras]). All: hw_risks case 7 drive errors, tank full.
    # extras: flames "procedural" (default) or "real" (sprites from real fire videos, sprites/SOURCES.md), aim: ControllerConfig overrides
    "f2_kit_real": (BASELINE, SHIPPED, 6.0, "f2", "baseline colour detector, shipped BHL policy, 6 Nm, real-fire flames",
                    dict(flames="real")),
    "f2_real_conf03": (YOLO, SHIPPED, 6.0, "f2", "YOLO11n detector (vision/detector.py) with aim/ min_conf 0.3, shipped BHL policy, "
                       "6 Nm, real-fire flames", dict(flames="real", aim=dict(min_conf=0.3))),
}


def job(args):
    cfg, seed = args
    out = RUNS / cfg / f"{seed}.json"
    if out.exists():
        return
    sys.path.insert(0, str(HERE))
    import mission
    import world
    det, pol, lim, design = CONFIGS[cfg][:4]
    try:
        r = mission.run(world.random_spec(seed, design=design, flames=flames(cfg)), det, pol, keep_log=False, limit=lim, aim=extras(cfg).get("aim"))
    except Exception as e:  # keep the campaign going, and count the crash
        r = dict(seed=seed, crash=repr(e))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(r, default=str))
    print(f"{cfg} {seed}: {'crash' if 'crash' in r else ('OK' if r['success'] else 'fail')} "
          f"{r.get('reason', '')} sim {r.get('sim_time')} s wall {r.get('wall_s')} s", flush=True)


def extras(cfg):
    return CONFIGS[cfg][5] if len(CONFIGS[cfg]) > 5 else {}


def flames(cfg):
    return extras(cfg).get("flames", "procedural")


def run(cfg, n, procs=3, reverse=False):
    """reverse: last seed first, so a second runner of the same configuration meets the first one in the middle."""
    t0 = time.time()
    seeds = [SEED0 + k for k in range(n)][::-1 if reverse else 1]
    with mp.get_context("spawn").Pool(procs) as pool:
        list(pool.imap_unordered(job, [(cfg, s) for s in seeds], chunksize=1))
    print(f"{cfg}: {n} missions in {(time.time() - t0) / 60:.1f} min")


# ---------------- report ----------------
def load(cfg):
    rs = [json.loads(p.read_text()) for p in sorted((RUNS / cfg).glob("*.json"))]
    return [r for r in rs if "crash" not in r], [r for r in rs if "crash" in r]


def wilson(k, n, z=1.96):
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return c - h, c + h


def pct(x):
    return f"{100 * x:.0f} %"


def summary(rs):
    n = len(rs)
    ok = [r for r in rs if r["success"]]
    lo, hi = wilson(len(ok), n)
    t = [r["t_all_out"] for r in ok]
    w = [r["water_l"] for r in ok]
    minutes = sum(r["sim_time"] for r in rs) / 60
    reasons = {}
    for r in rs:
        if not r["success"]:
            k = "fall" if r["fall"] else "abort" if r["abort"] else (r["reason"] or r["end_state"] or "?")
            reasons[k] = reasons.get(k, 0) + 1
    return dict(n=n, success=len(ok), ci=(lo, hi), t_med=float(np.median(t)) if t else None,
                t_p90=float(np.percentile(t, 90)) if t else None, water_med=float(np.median(w)) if w else None,
                water_max=float(max(w)) if w else None, falls=sum(1 for r in rs if r["fall"]), aborts=sum(1 for r in rs if r["abort"]),
                false_spray_missions=sum(1 for r in rs if r["false_spray_episodes"] > 0),
                false_spray_s=sum(r["false_spray_s"] for r in rs), fp_per_min=sum(r["fp"] for r in rs) / max(minutes, 1e-9),
                fires=sum(r["n_fires"] for r in rs), fires_out=sum(sum(t is not None for t in r["t_out"]) for r in rs),
                all_out=sum(1 for r in rs if r["all_out"]), all_out_fall=sum(1 for r in rs if r["all_out"] and (r["fall"] or r["abort"])),
                ignored_fire=sum(1 for r in rs for q in r.get("ignored", []) if q < 0.5),
                ignored_other=sum(1 for r in rs for q in r.get("ignored", []) if q >= 0.5),
                tank_empty=sum(1 for r in rs if r["tank_empty"]), reasons=reasons,
                wall_med=float(np.median([r["wall_s"] for r in rs])), sim_med=float(np.median([r["sim_time"] for r in rs])),
                on_fire=sum(r["water_on_fire_l"] for r in rs) / max(sum(r["water_l"] for r in rs), 1e-9))


BINS = [0, 1, 1.5, 2, 2.5, 3, 4, 5, 6, 8]


def binned(samples, bins=BINS):
    """samples: (distance, value). Rows: (bin label, n, median, 90th percentile)."""
    a = np.array([s[:2] for s in samples], float).reshape(-1, 2)
    rows = []
    for lo, hi in zip(bins, bins[1:]):
        v = a[(a[:, 0] >= lo) & (a[:, 0] < hi), 1]
        if len(v):
            rows.append((f"{lo}-{hi}", len(v), float(np.median(v)), float(np.percentile(v, 90)), float(np.mean(v))))
    return rows


def report():
    sys.path.insert(0, str(ROOT / "sim" / "payload"))
    import report as style  # sim/payload/report.py: palette and plot style
    plt = style.plt
    cfgs = [c for c in CONFIGS if (RUNS / c).exists() and any((RUNS / c).glob("*.json"))]
    data = {c: load(c) for c in cfgs}
    S = {c: summary(data[c][0]) for c in cfgs}
    f = lambda x, p=1: "-" if x is None else f"{x:.{p}f}"
    L = ["Comparison (the same mission seeds in every configuration). Robot: final design F2 (bhl_F2.xml, R2 tanks full).", "",
         "| Configuration | Jet and fire model | Leg torque limit | Detector | Success | All fires out | Falls "
         "| Missions with a false spray | Median time to put out all fires (s) | Median water used (L) | Water that hit a fire |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cfgs:
        s_, (det, pol, lim, design) = S[c], CONFIGS[c][:4]
        sp = data[c][0][0]["spec"]  # the jet and fire model that the runs used
        jet = (f"v0 {sp['v0']:g} m/s at 11.1 V, {sp['flow']:g} L/min, {sp['tank']:g} L, trays "
               f"{100 * min(f_['d'] for r in data[c][0] for f_ in r['spec']['fires']):.0f}-"
               f"{100 * max(f_['d'] for r in data[c][0] for f_ in r['spec']['fires']):.0f} cm")
        dn = "perfect (true boxes)" if str(det) == "oracle" else Path(str(det)).name
        L.append(f"| {c} | {jet} | {lim:.0f} Nm | {dn}{', real flames' if flames(c) == 'real' else ''}{''.join(f', {k} {v}' for k, v in extras(c).get('aim', {}).items())} | {s_['success']} of {s_['n']} "
                 f"| {s_['all_out']} | {s_['falls']} | {s_['false_spray_missions']} | {f(s_['t_med'])} | {f(s_['water_med'], 3)} "
                 f"| {pct(s_['on_fire'])} |")
    L += ["", "Success: every fire out, and no fall or ABORT before the end of the mission. Times and water: successful missions only.", "",
         "| Configuration | Missions | Success | 95 % interval | All fires out | Fires out | Median time to put out all fires (s) "
         "| 90th percentile (s) | Median water used (L) | Falls | Missions with a false spray | False detections per minute "
         "| Targets given up: not a fire / a fire |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cfgs:
        s = S[c]
        L.append(f"| {c}: {CONFIGS[c][4]} | {s['n']} | {s['success']} ({pct(s['success'] / s['n'])}) | {pct(s['ci'][0])} to {pct(s['ci'][1])} "
                 f"| {s['all_out']} | {s['fires_out']} of {s['fires']} | {f(s['t_med'])} | {f(s['t_p90'])} | {f(s['water_med'], 3)} "
                 f"| {s['falls']} | {s['false_spray_missions']} | {s['fp_per_min']:.1f} | {s['ignored_other']} / {s['ignored_fire']} |")
    L += ["", "Why the missions failed:", "", "| Configuration | " + " | ".join(["reason", "missions"]) + " |", "|---|---|---|"]
    for c in cfgs:
        for k, v in sorted(S[c]["reasons"].items(), key=lambda kv: -kv[1]):
            L.append(f"| {c} | {k} | {v} |")
    L += ["", "Success by number of fires and by scene:", "", "| Configuration | 1 fire | 2 fires | 3 fires | indoor | outdoor |", "|---|---|---|---|---|---|"]
    for c in cfgs:
        rs = data[c][0]
        cell = lambda sel: (lambda g: f"{sum(r['success'] for r in g)} of {len(g)}")([r for r in rs if sel(r)])
        L.append(f"| {c} | {cell(lambda r: r['n_fires'] == 1)} | {cell(lambda r: r['n_fires'] == 2)} | {cell(lambda r: r['n_fires'] == 3)} "
                 f"| {cell(lambda r: not r['outdoor'])} | {cell(lambda r: r['outdoor'])} |")
    L += ["", "Fires put out, by tray diameter (all fires of all missions, also missions that ended in a fall):", "",
          "| Configuration | 10-12.5 cm | 12.5-15 cm | 15-17.5 cm | 17.5-20 cm | 20-25 cm | 25-30 cm |", "|---|---|---|---|---|---|---|"]
    edges = [0.10, 0.125, 0.15, 0.175, 0.20, 0.25, 0.30001]
    for c in cfgs:
        cells = []
        for lo, hi in zip(edges, edges[1:]):
            g = [r["t_out"][i] is not None for r in data[c][0] for i, d_ in enumerate(r["fire_d"]) if lo <= d_ < hi]
            cells.append(f"{sum(g)} of {len(g)}" if g else "-")
        L.append(f"| {c} | " + " | ".join(cells) + " |")
    L += ["", "Localisation error of single detections (floor-plane ray, aim/ geometry, true robot pose), by distance from the robot:", "",
          "| Configuration | Distance (m) | Detections | Median error (m) | 90th percentile (m) |", "|---|---|---|---|---|"]
    for c in cfgs:
        for lab, n, med, p90, _ in binned([s for r in data[c][0] for s in r["loc_samples"]]):
            L.append(f"| {c} | {lab} | {n} | {med:.3f} | {p90:.3f} |")
    L += ["", "Detection rate: share of camera images with a visible fire (10 flame pixels or more) in which the detector found it:", "",
          "| Configuration | Distance (m) | Images | Detected |", "|---|---|---|---|"]
    for c in cfgs:
        for lab, n, med, p90, mean in binned([(s[0], s[2]) for r in data[c][0] for s in r["det_samples"]]):
            L.append(f"| {c} | {lab} | {n} | {pct(mean)} |")
    L += ["", "Largest distance at which each fire was detected, over all fires:", "",
          "| Configuration | Fires | Never detected | Median largest detection distance (m) | Fires detected from 4 m or more |", "|---|---|---|---|---|"]
    for c in cfgs:
        far = []
        for r in data[c][0]:
            for i in range(r["n_fires"]):
                d = [s_[0] for s_ in r["det_samples"] if s_[4] == i and s_[2]]
                far.append(max(d) if d else None)
        got = [x for x in far if x is not None]
        L.append(f"| {c} | {len(far)} | {sum(x is None for x in far)} | {np.median(got):.2f} | {sum(x >= 4 for x in got)} |" if got else f"| {c} | {len(far)} | {len(far)} | - | 0 |")
    L += ["", "False detections by nearest object (all missions):", "", "| Configuration | " + " | ".join(["object", "detections"]) + " |", "|---|---|---|"]
    for c in cfgs:
        tot = {}
        for r in data[c][0]:
            for k, v in r["false_dets"].items():
                tot[k] = tot.get(k, 0) + v
        for k, v in sorted(tot.items(), key=lambda kv: -kv[1]):
            L.append(f"| {c} | {k} | {v} |")
    crashes = {c: len(data[c][1]) for c in cfgs}
    L += ["", "Run time: " + ", ".join(f"{c}: median {S[c]['wall_med']:.0f} s wall for {S[c]['sim_med']:.0f} s simulated" for c in cfgs) + ".",
          "Crashed missions (not in the tables): " + ", ".join(f"{c}: {v}" for c, v in crashes.items()) + "."]
    text = "\n".join(L) + "\n"
    OUT.mkdir(exist_ok=True)
    (OUT / "campaign.md").write_text(text)
    (OUT / "campaign.json").write_text(json.dumps({c: {**S[c], "label": CONFIGS[c][4]} for c in cfgs}, indent=1))
    print(text)

    curves(data, cfgs, OUT / "fig_campaign.png", style)
    maps(data[cfgs[0]][0][:16], OUT / "fig_paths.png", style)


def curves(data, cfgs, path, style):
    """Fire position error and detection rate against distance, and the share of missions with all fires out against time."""
    plt = style.plt
    colors = [style.S1, style.S2, style.S3, style.INK2, "#8a5cc2", "#b8860b", "#2f855a"]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.2))
    for c, col in zip(cfgs, colors):
        rows = binned([s for r in data[c][0] for s in r["loc_samples"]])
        x = [float(lab.split("-")[0]) / 2 + float(lab.split("-")[1]) / 2 for lab, *_ in rows]
        axs[0].plot(x, [r[2] for r in rows], "o-", color=col, label=f"{c}, median")
        axs[0].plot(x, [r[3] for r in rows], "o--", color=col, alpha=0.6, label=f"{c}, 90th percentile")
        rows = binned([(s[0], s[2]) for r in data[c][0] for s in r["det_samples"]])
        x = [float(lab.split("-")[0]) / 2 + float(lab.split("-")[1]) / 2 for lab, *_ in rows]
        axs[1].plot(x, [100 * r[4] for r in rows], "o-", color=col, label=c)
        t = sorted(r["t_all_out"] for r in data[c][0] if r["success"])
        axs[2].step([0] + t, np.arange(len(t) + 1) / max(len(data[c][0]), 1) * 100, where="post", color=col, label=c)
    axs[0].set(xlabel="distance from the robot (m)", ylabel="localisation error (m)", title="Fire position error, single detections")
    axs[1].set(xlabel="distance from the robot (m)", ylabel="images with the fire detected (%)", title="Detection rate, visible fires", ylim=(0, 105))
    axs[2].set(xlabel="mission time (s)", ylabel="missions with all fires out (%)", title="Time to put out all fires", ylim=(0, 100))
    for ax in axs:
        ax.grid(color=style.GRID)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


STATE_COL = {"SEARCH": "#7a7a7a", "APPROACH": "#2b6cb0", "AIM": "#d69e2e", "SPRAY": "#c53030", "VERIFY": "#2f855a", "DONE": "#000000",
             "ABORT": "#9b2c2c", "START": "#7a7a7a"}


def maps(rs, path, style):
    """Top-down view of each mission: room, furniture, fires, distractors, and the robot path coloured by state."""
    plt = style.plt
    cols = 4
    rows = math.ceil(len(rs) / cols)
    fig, axs = plt.subplots(rows, cols, figsize=(4 * cols, 3.6 * rows))
    for ax, r in zip(np.ravel(axs), rs):
        sp = r["spec"]
        lx, ly = sp["room"]
        if not sp["outdoor"]:
            ax.add_patch(plt.Rectangle((-lx / 2, -ly / 2), lx, ly, fill=False, color="#555", lw=1.5))
        for fu in sp["furniture"]:
            ax.add_patch(plt.Rectangle((fu["x"] - fu["ex"], fu["y"] - fu["ey"]), 2 * fu["ex"], 2 * fu["ey"], color="#cbd5e0"))
        for dd in sp["distractors"]:
            ax.plot(dd["x"], dd["y"], "s", color="#dd6b20", ms=5)
        if sp["lamp"]:
            ax.plot(sp["lamp"]["x"], sp["lamp"]["y"], "*", color="#d69e2e", ms=9)
        p = r["path"]
        for (x0, y0, s0), (x1, y1, _) in zip(p, p[1:]):
            ax.plot([x0, x1], [y0, y1], color=STATE_COL.get(s0, "#999"), lw=1.6)
        x, y, yaw = sp["start"]
        ax.annotate("", (x + 0.6 * math.cos(yaw), y + 0.6 * math.sin(yaw)), (x, y), arrowprops=dict(arrowstyle="->", lw=1.5))
        for i, fi in enumerate(sp["fires"]):
            out = r["t_out"][i] is not None
            ax.plot(fi["x"], fi["y"], "o", ms=5 + 30 * fi["d"], color="#c53030" if not out else "#2f855a")
        t_end = r["fall"] or r["abort"]
        if t_end:
            q = p[min(int(t_end * 2), len(p) - 1)]  # the path has one point per 0.5 s
            ax.plot(q[0], q[1], "x", color="#000", ms=10, mew=2.5)
        why = "fall" if t_end else (r["reason"] or "")
        ax.set_title(f"seed {r['seed']}: all fires out at {r['t_all_out']:.0f} s" if r["success"] else f"seed {r['seed']}: FAIL, {why}", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(min(-lx / 2, *[q[0] for q in p]) - 0.3 if not sp["outdoor"] else min(q[0] for q in p) - 4,
                    max(lx / 2, *[q[0] for q in p]) + 0.3 if not sp["outdoor"] else max(q[0] for q in p) + 4)
        ax.set_ylim(min(-ly / 2, *[q[1] for q in p]) - 0.3 if not sp["outdoor"] else min(q[1] for q in p) - 4,
                    max(ly / 2, *[q[1] for q in p]) + 0.3 if not sp["outdoor"] else max(q[1] for q in p) + 4)
        ax.tick_params(labelsize=7)
    for ax in np.ravel(axs)[len(rs):]:
        ax.axis("off")
    handles = [plt.Line2D([], [], color=c, lw=2, label=s) for s, c in STATE_COL.items() if s not in ("START", "DONE")]
    handles += [plt.Line2D([], [], marker="o", ls="", color="#c53030", label="fire, still burning"),
                plt.Line2D([], [], marker="o", ls="", color="#2f855a", label="fire, out"),
                plt.Line2D([], [], marker="s", ls="", color="#dd6b20", label="red or orange object"),
                plt.Line2D([], [], marker="*", ls="", color="#d69e2e", label="warm lamp"),
                plt.Line2D([], [], marker="x", ls="", color="#000", mew=2, label="fall")]
    fig.legend(handles=handles, loc="lower center", ncol=len(handles), fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    os.environ.setdefault("MUJOCO_GL", "egl")
    if sys.argv[1] == "run":
        run(sys.argv[2], int(sys.argv[3]), int(sys.argv[4]) if len(sys.argv) > 4 else 3, len(sys.argv) > 5 and sys.argv[5] == "reverse")
    else:
        report()
