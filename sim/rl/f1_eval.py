"""Compare a trained policy with the shipped BHL policy on bhl_F2, in MuJoCo on the CPU (sim/sim2sim harness style).

    uv run python sim/rl/f1_eval.py sim/rl/policies/f1_<name>.onnx           full table -> sim/rl/results/f2_eval_<name>.*
    ... --limit=9                                                            9 Nm leg limit -> results/..._<name>_9nm.*
    uv run python sim/rl/f1_eval.py <policy.onnx> --quick [--limit=9]        short check of one policy (2 seeds)
    MUJOCO_GL=egl uv run python sim/rl/f1_eval.py <policy.onnx> --video      three videos -> sim/rl/results/

The loop is the one of sim/sim2sim/run.py: 25 Hz policy, 2 kHz physics, PD 20 / 2 in software, 6 Nm clip,
training observation noise, BHL start drop. It adds the joint target clip of legs/robot.py (knee <= 1.4 rad),
seeds, foot contact metrics, a moving pan-tilt and a recoil force at the nozzle.
The push test calls sim/payload/torso_tank.py, the drive cases call sim/sim2sim/hw_risks.py (MODEL = bhl_F2).
"""
import json
import math
import subprocess
import sys
import time
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "sim2sim"), str(HERE.parent / "payload")]
import run as s2s  # noqa: E402  sim/sim2sim/run.py
import hw_risks as hw  # noqa: E402  sim/sim2sim/hw_risks.py
import torso_tank as tk  # noqa: E402  sim/payload/torso_tank.py: walking and push helpers
import report  # noqa: E402  sim/payload/report.py: palette and plot style

DESIGN = next((a.split("=")[1] for a in sys.argv if a.startswith("--model=")), "F2")  # model file bhl_<name>.xml
MODEL = f"bhl_{DESIGN}"
hw.MODEL = MODEL  # before the pool forks
OUT = HERE / "results"
SHIPPED = s2s.POLICY
LIMIT = float(next((a.split("=")[1] for a in sys.argv if a.startswith("--limit=")), 6.0))  # Nm, --limit=9 for a check
SECONDS, SEEDS = 20.0, 5
LO = np.array([-0.174533, -0.981748, -1.89805, 0.0, -0.785398, -0.261799,
               -1.5708, -0.589049, -1.89805, 0.0, -0.785398, -0.261799])  # legs/robot.py JOINTS lo, hi
HI = np.array([1.5708, 0.589049, 0.981748, 1.4, 0.785398, 0.261799,
               0.174533, 0.981748, 0.981748, 1.4, 0.785398, 0.261799])  # knee hi = --knee-max 1.4
KNEES = [3, 9]
# Pan-tilt of bhl_F2 (sim/payload/models): the pan servo turns the yoke, the tilt servo and the hose clips about z at
# PAN_AXIS. The tilt servo turns the U bracket, the camera and the brass nozzle about y at TILT_AXIS (Assumed: the
# centre of the U bracket). The nozzle tip is 0.021 m in front of the nozzle body centre.
PAN_AXIS, TILT_AXIS = np.array([-0.038, 0.0, 0.8445]), np.array([-0.038, 0.0, 0.9087])
PAN_PARTS = ["payload_pan_yoke", "payload_ref_tilt_servo", "payload_hose_clips"]
TILT_PARTS = ["payload_tilt_u", "payload_camera_b0201", "payload_nozzle_brass"]
PAN = lambda t: 0.6 * math.sin(2 * math.pi * 0.2 * t)  # rad, +-34 deg sweep, 5 s period (Assumed aiming motion)
TILT = lambda t: -0.15 + 0.2 * math.sin(2 * math.pi * 0.3 * t)
RECOIL_ON = lambda t: t >= 3.0 and (t - 3.0) % 4.0 < 3.0  # spray 3 s, pause 1 s, from 3 s
TESTS = {  # name: (model, command, recoil N, pan-tilt)
    "stand": (MODEL, (0.0, 0.0, 0.0), 0.0, False),
    **{f"start from a stand, vx {v:.2f}{n}": (MODEL, (v, 0.0, 0.0), 0.0, False) for n in ("", ", no noise")
       for v in (0.1, 0.15, 0.2, 0.25)},
    "vx 0.1": (MODEL, (0.1, 0.0, 0.0), 0.0, False),
    "vx 0.2": (MODEL, (0.2, 0.0, 0.0), 0.0, False),
    "vx 0.3": (MODEL, (0.3, 0.0, 0.0), 0.0, False),
    "turn on the spot 0.3": (MODEL, (0.0, 0.0, 0.3), 0.0, False),
    "turn on the spot 0.5": (MODEL, (0.0, 0.0, 0.5), 0.0, False),
    "turn on the spot 0.8": (MODEL, (0.0, 0.0, 0.8), 0.0, False),
    "vx 0.2 + yaw 0.5": (MODEL, (0.2, 0.0, 0.5), 0.0, False),
    "stand, pan-tilt moving": (MODEL, (0.0, 0.0, 0.0), 0.0, True),
    "stand, pan-tilt + recoil 0.3 N": (MODEL, (0.0, 0.0, 0.0), 0.3, True),
    "stand, pan-tilt + recoil 1.3 N": (MODEL, (0.0, 0.0, 0.0), 1.3, True),
    "empty tank: vx 0.3": (MODEL + "_empty", (0.3, 0.0, 0.0), 0.0, False),
    "empty tank: turn on the spot 0.5": (MODEL + "_empty", (0.0, 0.0, 0.5), 0.0, False),
}
START_S, START_SEEDS = 9.5, 8  # stand 2-3.75 s (by seed), then the walk command for 9.5 s (as the fake-drive test of legs/)
QUICK = ["start from a stand, vx 0.25", "start from a stand, vx 0.25, no noise", "stand", "vx 0.1", "vx 0.3", "turn on the spot 0.5", "turn on the spot 0.8", "stand, pan-tilt + recoil 1.3 N"]
HW_CASES = ("1", "7", "7b")


@lru_cache(maxsize=4)
def load_model(name, pantilt):
    """The payload model, optionally with a pan and a tilt joint on the top module (2 extra servos, after the legs)."""
    s = mujoco.MjSpec.from_file(str(s2s.MODELS / f"{name}.xml"))
    if pantilt:
        for k in list(s.keys):  # the "stand" keyframe has no pan and tilt; the runs start from the BHL pose
            s.delete(k)
        pan = s.body("base").add_body(name="pan", pos=PAN_AXIS, mass=1e-3, inertia=[1e-6] * 3, explicitinertial=True)
        pan.add_joint(name="pan", axis=[0, 0, 1], armature=0.005, damping=0.02)
        tilt = pan.add_body(name="tilt", pos=TILT_AXIS - PAN_AXIS, mass=1e-3, inertia=[1e-6] * 3, explicitinertial=True)
        tilt.add_joint(name="tilt", axis=[0, 1, 0], armature=0.005, damping=0.02)
        for parts, parent, origin in ((PAN_PARTS, pan, PAN_AXIS), (TILT_PARTS, tilt, TILT_AXIS)):
            for n in parts:  # move each part under its servo, same world position, mass and inertia
                b = s.body(n)
                pos, mass, inertia, size = np.array(b.pos), b.mass, np.array(b.inertia), \
                    [g.size.copy() for g in s.geoms if g.parent.name == n][0]
                s.delete(b)
                nb = parent.add_body(name=n, pos=pos - origin, mass=mass, inertia=inertia, explicitinertial=True)
                nb.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=size, contype=0, conaffinity=0, mass=0,
                            rgba=[0.9, 0.5, 0.1, 1])
        nozzle = np.array([-0.0201, 0.0, 0.9315]) + [0.021, 0, 0]  # tip of payload_nozzle_brass
        tilt.add_site(name="nozzle", pos=nozzle - TILT_AXIS)
        for j in ("pan", "tilt"):  # ST3215 position servo, about 3 Nm stall
            s.add_actuator(name=j, target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT, gainprm=[8.0] + [0] * 9,
                           biastype=mujoco.mjtBias.mjBIAS_AFFINE, biasprm=[0, -8.0, -0.3] + [0] * 7,
                           forcerange=[-3, 3], forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
    m = s.compile()
    if pantilt:  # the moving parts keep the mass of the model (2 g of joint bodies added)
        assert abs(m.body_subtreemass[1] - mujoco.MjModel.from_xml_path(str(s2s.MODELS / f"{name}.xml")).body_subtreemass[1]) < 0.003
    m.opt.timestep = s2s.PHYSICS_DT
    assert [m.joint(j).qposadr[0] for j in s2s.JOINTS] == list(range(7, 19))
    assert [m.actuator(i).name for i in range(12)] == s2s.JOINTS
    feet = [i for i in range(m.ngeom) if m.geom_bodyid[i] in (m.body("leg_left_ankle_roll").id,
            m.body("leg_right_ankle_roll").id) and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX]
    return m, np.array(feet)


@lru_cache(maxsize=2)
def session(path):
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    return ort.InferenceSession(str(path), so, providers=["CPUExecutionProvider"])


def episode(policy, test, seed, seconds=SECONDS, frames=None):
    """One run. Returns the fall time and the tracking, drift and stepping metrics (from 5 s, no metrics after a fall)."""
    name, command, recoil, pantilt = TESTS[test]
    t_cmd = 2.0 + 0.25 * seed if test.startswith("start") else 0.0  # zero command until t_cmd
    seconds = t_cmd + START_S if t_cmd else seconds
    noise = "no noise" not in test
    m, feet = load_model(name, pantilt)
    d = mujoco.MjData(m)
    d.qpos[7:19] = s2s.DEFAULT  # BHL reset, as run.py start="bhl"
    mujoco.mj_forward(m, d)
    sess, rng = session(policy), np.random.default_rng(seed)
    base, z0, floor = m.body("base").id, d.site("imu").xpos[2], m.geom("floor").id
    tilt_id = m.body("tilt").id if pantilt else None
    cmd, action = np.array(command, np.float32), np.zeros(12, np.float32)
    sub = round(s2s.POLICY_DT / s2s.PHYSICS_DT)
    fall, n, sat, peak, knee = None, 0, 0, 0.0, 0.0
    log = []  # t, x, y, yaw, vx, vy, wz, contact L, contact R, slip L, slip R
    for _ in range(round(seconds / s2s.POLICY_DT)):
        c_now = cmd if d.time >= t_cmd - 1e-9 else np.zeros(3, np.float32)
        obs = s2s.observation(c_now, d.sensor("imu_quat").data.copy(), d.sensor("imu_gyro").data.copy(),
                              d.qpos[7:19], d.qvel[6:18], action)
        obs += (rng.uniform(-1, 1, 45) * s2s.NOISE * noise).astype(np.float32)
        action = sess.run(None, {"obs": obs})[0][0]
        target = np.clip(s2s.DEFAULT + s2s.ACTION_SCALE * action, LO, HI)  # legs/robot.py command() clip
        for _ in range(sub):
            tau = s2s.KP * (target - d.qpos[7:19]) - s2s.KD * d.qvel[6:18]
            sat += np.any(np.abs(tau) > LIMIT)
            d.ctrl[:12] = np.clip(tau, -LIMIT, LIMIT)
            if pantilt:
                d.ctrl[12:14] = PAN(d.time), TILT(d.time)
                d.qfrc_applied[:] = 0
                if recoil and RECOIL_ON(d.time):  # recoil: backward along the nozzle axis, at the nozzle
                    axis = d.xmat[tilt_id].reshape(3, 3)[:, 0]
                    mujoco.mj_applyFT(m, d, -recoil * axis, np.zeros(3), d.site("nozzle").xpos, tilt_id, d.qfrc_applied)
            mujoco.mj_step(m, d)
            n += 1
            if d.time > 1.0:
                peak, knee = max(peak, float(np.abs(d.ctrl[:12]).max())), max(knee, float(d.qpos[[10, 16]].max()))
            if frames and d.time * 30 >= frames.n:
                frames(m, d)
        t = d.time
        tilt = np.arccos(np.clip(-s2s.quat_rotate_inverse(d.sensor("imu_quat").data, np.array([0, 0, -1.0]))[2], -1, 1))
        if fall is None and (tilt > s2s.TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0):
            fall = t
        if fall is not None and (not frames or t > fall + 2.0):
            break
        R = d.xmat[base].reshape(3, 3)
        yaw = math.atan2(R[1, 0], R[0, 0])
        v = d.sensor("frame_vel").data
        c = [False, False]
        for g1, g2 in zip(d.contact.geom1[:d.ncon], d.contact.geom2[:d.ncon]):
            for i, f in enumerate(feet):
                c[i] |= {g1, g2} == {f, floor}
        slip = []
        for f in feet:
            vel = np.zeros(6)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[f], vel, 0)
            slip.append(float(np.linalg.norm(vel[3:5])))
        log.append([t, *d.xpos[base][:2], yaw, math.cos(yaw) * v[0] + math.sin(yaw) * v[1],
                    -math.sin(yaw) * v[0] + math.cos(yaw) * v[1], (R @ d.qvel[3:6])[2], *c, *slip])
    a = np.array(log)
    r = dict(policy=Path(policy).name, test=test, seed=seed, fall=fall, sat=100.0 * sat / n, peak=peak, knee=knee)
    if fall is None and t_cmd:  # forward distance along the heading at the command change
        w = a[a[:, 0] >= t_cmd]
        dist = float(np.dot(w[-1, 1:3] - w[0, 1:3], [math.cos(w[0, 3]), math.sin(w[0, 3])]))
        r.update(start_dist=dist, started=dist > 0.5 * command[0] * START_S)
    if fall is None:
        w = a[a[:, 0] >= 5.0]
        contact = w[:, 7:9] > 0.5
        down = np.sum(contact[1:] & ~contact[:-1])  # touchdowns
        slip = w[:, 9:11][contact]
        r.update(vx=float(w[:, 4].mean()), vy=float(w[:, 5].mean()), wz=float(w[:, 6].mean()),
                 drift=float(np.hypot(*(w[-1, 1:3] - w[0, 1:3]))),
                 yaw_drift=float(np.unwrap(w[:, 3])[-1] - w[0, 3]),
                 air=float(100 * np.mean(~contact.all(axis=1))), steps=float(down / (w[-1, 0] - w[0, 0])),
                 slip=float(slip.mean()) if slip.size else 0.0)
    return r


# ---------------- jobs for the pool ----------------
_current = [None]


def use(policy):
    """Policy for the imported harness functions (run.py POLICY, hw_risks.load cache)."""
    if _current[0] != policy:
        s2s.POLICY = Path(policy)
        hw.load.cache_clear()
        _current[0] = policy


def job(kind, policy, *args):
    use(policy)
    if kind == "episode":
        return episode(policy, *args)
    if kind == "hw_walk":
        return {"policy": Path(policy).name, **hw.walk_job(*args)}
    if kind == "hw_push":
        return {"policy": Path(policy).name, **hw.push_job(*args)}
    if kind == "push":
        return {"policy": Path(policy).name, **tk.push_job(DESIGN, *args)}
    raise ValueError(kind)


# ---------------- tables ----------------
def mean(rs, k):
    v = [r[k] for r in rs if r.get(k) is not None]
    return float(np.mean(v)) if v else None


def fmt(x, p=2, sign=False):
    return "-" if x is None else (f"{x:+.{p}f}" if sign else f"{x:.{p}f}")


def tables(res, pols):
    names = {Path(p).name: label for label, p in pols.items()}
    mf = mujoco.MjModel.from_xml_path(str(s2s.MODELS / f"{MODEL}.xml"))
    L = [f"Model `sim/payload/models/{MODEL}.xml`: {mf.body_subtreemass[1]:.2f} kg, file time "
         f"{time.strftime('%Y-%m-%d %H:%M', time.localtime((s2s.MODELS / f'{MODEL}.xml').stat().st_mtime))}.", "",
         f"### Commands on {MODEL} ({LIMIT:g} Nm, training noise, BHL start drop, 20 s)", "",
         f"{SEEDS} seeds per test. Speeds and stepping metrics are means from 5 s to 20 s of the runs without a fall.",
         "Air: share of time with a foot off the ground. Steps: touchdowns per second. Slip: mean speed of a foot "
         "on the ground. Drift: distance moved from 5 s to 20 s.", "",
         "| Test | Policy | Falls | vx (m/s) | vy (m/s) | yaw rate (rad/s) | Drift (m) | Yaw drift (rad) | Air (%) "
         "| Steps (1/s) | Slip (m/s) | Max knee (rad) | Peak torque (Nm) |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for test in TESTS:
        for fname, label in names.items():
            rs = [r for r in res["episode"] if r["test"] == test and r["policy"] == fname]
            if not rs or test.startswith("start"):
                continue
            ok = [r for r in rs if r["fall"] is None]
            L.append(f"| {test} | {label} | {len(rs) - len(ok)} of {len(rs)} | {fmt(mean(ok, 'vx'), sign=True)} "
                     f"| {fmt(mean(ok, 'vy'), sign=True)} | {fmt(mean(ok, 'wz'), sign=True)} | {fmt(mean(ok, 'drift'))} "
                     f"| {fmt(mean(ok, 'yaw_drift'), sign=True)} | {fmt(mean(ok, 'air'), 0)} | {fmt(mean(ok, 'steps'), 1)} "
                     f"| {fmt(mean(ok, 'slip'), 3)} | {max(r['knee'] for r in rs):.2f} | {max(r['peak'] for r in rs):.1f} |")
    starts = [t for t in TESTS if t.startswith("start")]
    if any(r["test"] in starts for r in res["episode"]):
        L += ["", f"### Start walking from a stand ({MODEL}, {LIMIT:g} Nm)", "",
              f"Zero command for 2.0 to 3.75 s (by seed), then the walk command for {START_S} s. Distance: forward distance in "
              f"these {START_S} s. Started: the distance is more than half of command x {START_S} s.", "",
              "| Command (m/s) | Observation noise | Policy | Falls | Started | Distance, mean (min to max) (m) "
              "| Distance at the command (m) |", "|---|---|---|---|---|---|---|"]
        for test in starts:
            v = TESTS[test][1][0]
            for fname, label in names.items():
                rs = [r for r in res["episode"] if r["test"] == test and r["policy"] == fname]
                if not rs:
                    continue
                ok = [r for r in rs if r["fall"] is None]
                ds = [r["start_dist"] for r in ok]
                L.append(f"| {v:.2f} | {'no' if 'no noise' in test else 'training'} | {label} "
                         f"| {len(rs) - len(ok)} of {len(rs)} | {sum(r['started'] for r in ok)} of "
                         f"{len(rs)} | {fmt(np.mean(ds) if ds else None)} ({fmt(min(ds) if ds else None)} to "
                         f"{fmt(max(ds) if ds else None)}) | {v * START_S:.2f} |")
    if res.get("push"):
        L += ["", f"### Push test (`sim/payload/torso_tank.py`, {MODEL}, 6 Nm)", "",
              "Largest push with no fall in the next 3 s (N s, 0.1 s on the torso, worst of 3 gait phases).", "",
              "| Policy | " + " | ".join(f"{c}, {d}" for c in tk.PUSH_CMDS for d in tk.PUSH_DIRS) + " |",
              "|---|" + "---|" * (len(tk.PUSH_CMDS) * len(tk.PUSH_DIRS))]
        for fname, label in names.items():
            ps = [dict(p, model=p["policy"]) for p in res["push"]]
            L.append(f"| {label} | " + " | ".join(f"{tk.push_worst(ps, fname, c, d):.1f}" for c in tk.PUSH_CMDS
                                                  for d in tk.PUSH_DIRS) + " |")
    if res.get("hw_walk"):
        L += ["", f"### Drive cases of `sim/sim2sim/hw_risks.py` on {MODEL}", "",
              "Each case: 4 commands (stand, vx 0.3, turn on the spot 0.5, vx 0.2 + yaw 0.5) x 10 seeds of 20 s. "
              "Push: walking at 0.3 m/s, 5 push times, worst / mean (N s).", "",
              "| Case | Policy | Falls | vx at 0.3 | yaw rate at turn on the spot 0.5 | vx, yaw at 0.2 + 0.5 "
              "| Peak torque (Nm) | Action rate (rad) | Push back (N s) | Push left (N s) |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for cid in HW_CASES:
            for fname, label in names.items():
                w = [r for r in res["hw_walk"] if r["case"] == cid and r["policy"] == fname]
                ok = lambda c: [r for r in w if r["cmd"] == c and r["fall"] is None]
                pb = [p["impulse"] for p in res["hw_push"] if p["case"] == cid and p["policy"] == fname and p["dir"] == "back"]
                pl = [p["impulse"] for p in res["hw_push"] if p["case"] == cid and p["policy"] == fname and p["dir"] == "left"]
                L.append(f"| {cid}: {hw.CASES[cid][0]} | {label} | {sum(r['fall'] is not None for r in w)} of {len(w)} "
                         f"| {fmt(mean(ok('vx 0.3'), 'vx'))} | {fmt(mean(ok('yaw 0.5'), 'wz'))} "
                         f"| {fmt(mean(ok('vx 0.2 + yaw 0.5'), 'vx'))}, {fmt(mean(ok('vx 0.2 + yaw 0.5'), 'wz'))} "
                         f"| {max(r['peak'] for r in w):.1f} | {mean(w, 'rate'):.3f} "
                         f"| {min(pb):.1f} / {np.mean(pb):.1f} | {min(pl):.1f} / {np.mean(pl):.1f} |")
    return "\n".join(L) + "\n"


def plot(res, pols, out):
    plt = report.plt
    names = {Path(p).name: label for label, p in pols.items()}
    cols = {"shipped": report.INK2, "ours": report.S1}
    fig, axs = plt.subplots(1, 3, figsize=(14, 4.2))
    for ax, tests, key, unit in ((axs[0], ["vx 0.1", "vx 0.2", "vx 0.3"], "vx", "m/s"),
                                 (axs[1], ["turn on the spot 0.3", "turn on the spot 0.5", "turn on the spot 0.8"], "wz",
                                  "rad/s")):
        cmd = [TESTS[t][1][0 if key == "vx" else 2] for t in tests]
        ax.plot(cmd, cmd, "--", color=report.GRID, label="command")
        for fname, label in names.items():
            v = [mean([r for r in res["episode"] if r["test"] == t and r["policy"] == fname and r["fall"] is None], key)
                 for t in tests]
            ax.plot(cmd, [x if x is not None else np.nan for x in v], "-o", color=cols[label], label=label)
        ax.set_xlabel(f"command ({unit})")
        ax.set_ylabel(f"measured, mean of 5 seeds ({unit})")
        ax.set_title("Forward speed" if key == "vx" else "Turn on the spot", loc="left", fontsize=10)
        ax.grid(color=report.GRID)
        ax.legend(frameon=False, fontsize=9)
    if res.get("push"):
        labels = [f"{'stand' if c == 'standing' else 'walk'} {d}" for c in tk.PUSH_CMDS for d in tk.PUSH_DIRS]
        y = np.arange(len(labels))[::-1]
        ps = [dict(p, model=p["policy"]) for p in res["push"]]
        for i, (fname, label) in enumerate(names.items()):
            axs[2].barh(y + (0.2 if i == 0 else -0.2), [tk.push_worst(ps, fname, c, d) for c in tk.PUSH_CMDS
                                                         for d in tk.PUSH_DIRS], 0.4, color=cols[label], label=label)
        axs[2].set_yticks(y, labels, fontsize=8)
        axs[2].set_xlabel("largest push survived, worst of 3 (N s)")
        axs[2].set_title(f"Push test, {MODEL}, 6 Nm", loc="left", fontsize=10)
        axs[2].grid(axis="x", color=report.GRID)
        axs[2].legend(frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def video(policy, test, out, seconds=12.0):
    width, height = 640, 480
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
                           f"{width}x{height}", "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", "-crf", "26", str(out)],
                          stdin=subprocess.PIPE)
    state = {}

    def grab(m, d):
        if not state:
            state["r"] = mujoco.Renderer(m, height, width)
            cam = mujoco.MjvCamera()
            cam.type, cam.trackbodyid = mujoco.mjtCamera.mjCAMERA_TRACKING, m.body("base").id
            cam.distance, cam.elevation, cam.azimuth = 2.2, -15.0, 135.0
            state["cam"] = cam
        state["r"].update_scene(d, state["cam"])
        ff.stdin.write(state["r"].render().tobytes())
        grab.n += 1
    grab.n = 0
    r = episode(policy, test, 0, seconds, frames=grab)
    ff.stdin.close()
    ff.wait()
    state["r"].close()
    print(out, {k: v for k, v in r.items() if k in ("fall", "vx", "wz", "drift", "air", "steps")})


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        return
    ours = str(Path(sys.argv[1]).resolve())
    quick = "--quick" in sys.argv
    if "--video" in sys.argv:
        stem = Path(ours).stem
        video(ours, "turn on the spot 0.5", OUT / f"{stem}_turn_on_spot.mp4")
        video(ours, "vx 0.3", OUT / f"{stem}_walk.mp4")
        video(ours, "stand, pan-tilt + recoil 1.3 N", OUT / f"{stem}_stand_recoil.mp4", seconds=15.0)
        return
    pols = {"ours": ours} if quick else {"shipped": str(SHIPPED), "ours": ours}
    seeds = 2 if quick else SEEDS
    jobs = [("episode", p, t, s) for p in pols.values() for t in (QUICK if quick else TESTS)
            for s in range(START_SEEDS if t.startswith("start") and not quick else seeds)]
    if not quick:
        jobs += [("push", p, c, d, t0) for p in pols.values() for c in tk.PUSH_CMDS for d in tk.PUSH_DIRS
                 for t0 in tk.PUSH_T]
        jobs += [("hw_walk", p, cid, c, s) for p in pols.values() for cid in HW_CASES for c in s2s.COMMANDS
                 for s in range(hw.SEEDS)]
        jobs += [("hw_push", p, cid, d, s) for p in pols.values() for cid in HW_CASES for d in hw.PUSH_DIRS
                 for s in range(hw.PUSH_SEEDS)]
    t0 = time.time()
    with Pool(3) as pool:  # at most 3 CPU processes (shared machine)
        out = pool.starmap(job, jobs, chunksize=1)
    res = {k: [r for j, r in zip(jobs, out) if j[0] == k] for k in ("episode", "push", "hw_walk", "hw_push")}
    md = tables(res, pols)
    print(md)
    print(f"{len(jobs)} runs in {(time.time() - t0) / 60:.1f} min")
    if not quick:
        OUT.mkdir(exist_ok=True)
        stem = f"{DESIGN.lower()}_eval_{Path(ours).stem}" + ("" if LIMIT == 6.0 else f"_{LIMIT:g}nm")
        (OUT / f"{stem}.json").write_text(json.dumps({"policies": pols, **res}))
        (OUT / f"{stem}.md").write_text(md)
        plot(res, pols, OUT / f"{stem}.png")


if __name__ == "__main__":
    main()
