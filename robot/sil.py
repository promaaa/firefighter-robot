"""Software-in-the-loop test: the real robot/launch.py --sim, end to end, with a log, a plot and checks.

    uv run python robot/sil.py                      all scenarios (about 3 min)
    uv run python robot/sil.py mission              one scenario

Each scenario runs robot/launch.py --sim as on the robot: legs/run_policy.py on the fake CAN bus (MuJoCo biped),
actuators/actuator_node.py with mock servos and a mock pump pin, robot/camera_node.py with the synthetic camera
and the baseline detector, and aim/run.py. This script only watches the logs and injects the faults.

mission         one fire at config.toml [sim] fires: the robot finds it, turns and walks to it, sprays, the fire goes
                out, the robot checks it and searches for more fires, DONE, then the safe stop (--stop-when-done).
hung_actuator   SIGSTOP the actuator node 1 s after the pump switched on: the worst case, a frozen node with the
                pump on. launch.py must see the missing heartbeat, kill the node and force the pump pin low.
camera_dies     SIGKILL the camera node while the robot walks (APPROACH): launch.py must stop everything safely.

Outputs: robot/results/sil_<scenario>.png, robot/results/sil_results.md. Logs: robot/logs/sil-<time>-<scenario>/.
Ports: ROBOT_PORT_OFFSET (default here 10000), so the test does not disturb a running pipeline.
"""
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
RESULTS = HERE / "results"
SCENARIOS = ("mission", "hung_actuator", "camera_dies")
STATES = ["SEARCH", "APPROACH", "AIM", "SPRAY", "CHECK", "VERIFY", "DONE", "ABORT"]


def read_log(run_dir):
    p = run_dir / "launch.log"
    return p.read_text() if p.exists() else ""


def pid_of(run_dir, node):
    m = re.search(rf"started {node}, pid (\d+)", read_log(run_dir))
    return int(m.group(1)) if m else None


def telemetry(run_dir):
    rows = []
    p = run_dir / "telemetry.jsonl"
    if p.exists():
        for line in p.read_text().splitlines():
            try:
                rows.append(json.loads(line))
            except ValueError:
                pass  # the last line can be half written while the run goes on
    return rows


def run(name, fires, timeout=150.0):
    """Run launch.py --sim for one scenario, inject its fault. Returns (run dir, launch exit code, fault info)."""
    run_dir = ROOT / "robot" / "logs" / f"sil-{datetime.now():%Y%m%d-%H%M%S}-{name}"
    env = dict(os.environ, ROBOT_PORT_OFFSET=os.environ.get("ROBOT_PORT_OFFSET", "10000"))
    cmd = [sys.executable, "-u", "robot/launch.py", "--sim", "--log-dir", str(run_dir)]
    cmd += ["--stop-when-done"] if name == "mission" else []
    p = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    fault, t0 = {}, time.monotonic()
    while p.poll() is None and time.monotonic() - t0 < timeout:
        time.sleep(0.1)
        if fault or name == "mission":
            continue
        rows = telemetry(run_dir)
        if name == "hung_actuator":
            on = [r["rt"] for r in rows if r["src"] == "pump_pin" and r["on"]]
            if on and rows[-1]["rt"] > on[0] + 1.0:
                fault = dict(node="actuator", signal="SIGSTOP", pid=pid_of(run_dir, "actuator"))
        elif name == "camera_dies":
            walking = [r["rt"] for r in rows if r["src"] == "aim" and r["state"] == "APPROACH"]
            if walking and rows[-1]["rt"] > walking[0] + 1.0:
                fault = dict(node="camera", signal="SIGKILL", pid=pid_of(run_dir, "camera"))
        if fault:
            fault["rt"] = telemetry(run_dir)[-1]["rt"]
            os.kill(fault["pid"], getattr(signal, fault["signal"]))
            print(f"  {name}: {fault['signal']} to the {fault['node']} node (pid {fault['pid']}) at {fault['rt']:.2f} s", flush=True)
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        p.wait(30)
    return run_dir, p.returncode, fault


def analyse(name, run_dir, code, fault, fires):
    rows, log = telemetry(run_dir), read_log(run_dir)
    pick = lambda src: [r for r in rows if r["src"] == src]  # noqa: E731
    truth, aim, cam, pin = pick("truth"), pick("aim"), pick("camera"), pick("pump_pin")
    states = [r["state"] for r in aim]
    pin_on = [r["rt"] for r in pin if r["on"]]
    pin_off = [r["rt"] for r in pin if not r["on"]]
    health = cam[-1].get("sim_fire_health", []) if cam else []
    stopped = re.search(r"all nodes stopped: exit codes (\{.*\})", log)
    t_stop = re.search(r"\s([\d.]+) s  (STOP|FAULT): (.*)\. Safe stop", log)
    t_end = re.search(r"\s([\d.]+) s  all nodes stopped", log)
    damped = "Damping for 3 s" in (run_dir / "legs.log").read_text() if (run_dir / "legs.log").exists() else False
    r = dict(name=name, exit_code=code, states=" ".join(dict.fromkeys(states)), fault=fault,
             stop_reason=t_stop.group(3) if t_stop else None, t_stop=float(t_stop.group(1)) if t_stop else None,
             t_end=float(t_end.group(1)) if t_end else None, pump_on_s=None, fire_health=health, legs_damped=damped)
    if pin_on:  # total time with the pump pin high, over all sprays
        on, total = None, 0.0
        for p in pin + [dict(rt=rows[-1]["rt"], on=False)]:
            if p["on"] and on is None:
                on = p["rt"]
            elif not p["on"] and on is not None:
                total, on = total + p["rt"] - on, None
        r["pump_on_s"] = round(total, 2)
    checks = {}
    if truth:
        f = np.array(fires[0])
        d0 = math.dist((truth[0]["x"], truth[0]["y"]), f)
        dmin = min(math.dist((t["x"], t["y"]), f) for t in truth if not pin_on or t["rt"] <= pin_on[0])
        turn = max(abs(t["yaw"]) for t in truth if not pin_on or t["rt"] <= pin_on[0])
        r.update(dist_start=round(d0, 2), dist_at_spray=round(dmin, 2), max_turn_deg=round(math.degrees(turn), 1))
    if name == "mission":
        checks["the robot walked toward the fire (0.4 m or more closer)"] = r.get("dist_start", 0) - r.get("dist_at_spray", 9) >= 0.4
        checks["the robot turned toward the fire (5 deg or more)"] = r.get("max_turn_deg", 0) >= 5
        checks["the pump switched on"] = bool(pin_on)
        checks["the fire went out"] = bool(health) and max(health) == 0
        checks["aim/run.py reached DONE"] = "DONE" in states
        checks["launch.py exit code 0"] = code == 0
    else:
        checks[f"launch.py saw the fault ({fault.get('node')})"] = bool(r["stop_reason"]) and fault.get("node", "?") in r["stop_reason"]
        checks["launch.py exit code 1 (fault)"] = code == 1
        if name == "hung_actuator":
            off = [t for t in pin_off if t > fault.get("rt", math.inf)]
            r["pump_off_after_fault_s"] = round(off[0] - fault["rt"], 2) if off else None
            checks["the pump pin went low within 1.0 s of the freeze"] = r["pump_off_after_fault_s"] is not None and r["pump_off_after_fault_s"] <= 1.0
        else:
            checks["the pump stayed off"] = not pin_on
    checks["the legs got damping, then motors off"] = damped
    checks["all nodes stopped"] = bool(stopped)
    checks["the pump pin is low at the end"] = not pin or not pin[-1]["on"]
    r["checks"] = checks
    r["passed"] = all(checks.values())
    plot(name, rows, r, fires)
    return r


def plot(name, rows, r, fires):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pick = lambda src: [x for x in rows if x["src"] == src]  # noqa: E731
    truth, est, aim, pin, act = pick("truth"), pick("legs"), pick("aim"), pick("pump_pin"), pick("actuator")
    fig, (top, tl) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw=dict(width_ratios=[1, 1.6]))
    if truth:
        top.plot([t["x"] for t in truth], [t["y"] for t in truth], color="black", label="robot, true (MuJoCo)")
    if est:
        top.plot([t["x"] for t in est], [t["y"] for t in est], color="tab:blue", ls="--", label="robot, estimate (IMU yaw + command)")
    for x, y in fires:
        top.add_patch(plt.Circle((x, y), 0.15, color="tab:orange", alpha=0.8))
    top.plot([], [], "o", color="tab:orange", label="fire")
    top.plot(0, 0, "k^", label="start")
    top.set_aspect("equal")
    top.set_xlabel("x (m)")
    top.set_ylabel("y (m)")
    top.set_title("Top view")
    top.legend(fontsize=8, loc="best")
    top.grid(alpha=0.3)
    t_end = rows[-1]["rt"] if rows else 1
    if aim:
        tl.step([a["rt"] for a in aim], [STATES.index(a["state"]) if a["state"] in STATES else -1 for a in aim], where="post", color="black", label="aim/run.py state")
        tl.plot([a["rt"] for a in aim], [3 * a["vx"] for a in aim], color="tab:blue", lw=0.8, label="vx command x 3 (m/s)")
    if act:
        tl.plot([a["rt"] for a in act], [3 * a["pan"] for a in act], color="tab:green", lw=0.8, label="measured pan x 3 (rad)")
    on = None
    for p in pin + [dict(rt=t_end, on=False)]:
        if p["on"] and on is None:
            on = p["rt"]
        elif not p["on"] and on is not None:
            tl.axvspan(on, p["rt"], color="tab:cyan", alpha=0.35, label="pump pin on" if on == next(q["rt"] for q in pin if q["on"]) else None)
            on = None
    if r["fault"]:
        tl.axvline(r["fault"]["rt"], color="red", label=f"fault: {r['fault']['signal']} {r['fault']['node']}")
    if r["t_stop"]:
        tl.axvline(r["t_stop"], color="red", ls="--", label="launch.py starts the safe stop")
    if r["t_end"]:
        tl.axvline(r["t_end"], color="grey", ls=":", label="all nodes stopped")
    tl.set_yticks(range(len(STATES)), STATES)
    tl.set_ylim(-1.5, len(STATES) - 0.5)
    tl.set_xlabel("time since launch (s)")
    tl.set_title("Timeline")
    tl.legend(fontsize=7, loc="upper left")
    tl.grid(alpha=0.3)
    fig.suptitle(f"SIL {name}: {'PASS' if r['passed'] else 'FAIL'}")
    fig.tight_layout()
    RESULTS.mkdir(exist_ok=True)
    fig.savefig(RESULTS / f"sil_{name}.png", dpi=90)
    plt.close(fig)


def report(results):
    lines = [f"# SIL results ({datetime.now():%Y-%m-%d %H:%M})", "",
             "Written by `uv run python robot/sil.py`. Status: tested in simulation.", "",
             "| Scenario | Result | States | Distance to the fire: start, at spray (m) | Largest turn (deg) | Pump on (s) | Fire health at the end | Stop reason | Exit code |",
             "|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        lines.append(f"| {r['name']} | {'PASS' if r['passed'] else 'FAIL'} | {r['states']} | {r.get('dist_start', '-')}, {r.get('dist_at_spray', '-')} | "
                     f"{r.get('max_turn_deg', '-')} | {r['pump_on_s'] if r['pump_on_s'] is not None else '-'} | {r['fire_health']} | {r['stop_reason']} | {r['exit_code']} |")
    for r in results:
        lines += ["", f"## {r['name']}", "", f"![{r['name']}](sil_{r['name']}.png)", ""]
        if r["fault"]:
            lines.append(f"Fault: {r['fault']['signal']} to the {r['fault']['node']} node at {r['fault']['rt']:.2f} s. "
                         f"launch.py started the safe stop at {r['t_stop']} s. All nodes stopped at {r['t_end']} s.")
            if "pump_off_after_fault_s" in r:
                lines.append(f"The pump pin went low {r['pump_off_after_fault_s']} s after the freeze.")
            lines.append("")
        lines += [f"- {'PASS' if ok else 'FAIL'}: {c}" for c, ok in r["checks"].items()]
    (RESULTS / "sil_results.md").write_text("\n".join(lines) + "\n")


def main(names):
    sys.path.insert(0, str(HERE))
    from robot_config import CONF
    fires = CONF["sim"]["fires"]
    results = []
    for name in names:
        print(f"SIL {name}: running", flush=True)
        run_dir, code, fault = run(name, fires)
        r = analyse(name, run_dir, code, fault, fires)
        results.append(r)
        print(f"SIL {name}: {'PASS' if r['passed'] else 'FAIL'}  " + ", ".join(f"{c}: {'ok' if v else 'NO'}" for c, v in r["checks"].items()), flush=True)
        print(f"  logs: {run_dir.relative_to(ROOT)}", flush=True)
    report(results)
    print(f"wrote {(RESULTS / 'sil_results.md').relative_to(ROOT)}")
    return 0 if all(r["passed"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or SCENARIOS))
