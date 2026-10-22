"""Walking, push and statics helpers of the payload checks. final_check.py and sim/rl/f1_eval.py use them.

policy_run() copies the loop of sim/sim2sim/run.py (shipped BHL policy, 25 Hz policy, 2000 Hz physics, kp 20 / kd 2,
torque clipped, training observation noise) and adds a horizontal push on the torso. max_push() finds the largest
push without a fall by bisection. video() films one run (needs ffmpeg).
"""
import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "sim2sim")]
import analyze  # noqa: E402
import models  # noqa: E402
import report  # noqa: E402  palette and plot style
import run as s2s  # noqa: E402  sim/sim2sim/run.py

OUT = HERE / "results"
ROWS = [("bare biped", "biped"), ("F2: course BOM parts, R2 tanks full", "F2"), ("F2, R2 half", "F2_half"),
        ("F2, R2 empty", "F2_empty")]  # (table label, model variant)
COMMANDS = {"stand": (0.0, 0.0, 0.0), "vx 0.3": (0.3, 0.0, 0.0), "vx 0.2 + yaw 0.5": (0.2, 0.0, 0.5)}
PUSH_DIRS = {"forward": 0.0, "back": math.pi, "left": math.pi / 2, "right": -math.pi / 2}
PUSH_CMDS = {"standing": (0.0, 0.0, 0.0), "walking 0.3 m/s": (0.3, 0.0, 0.0)}
PUSH_T = (3.0, 3.2, 3.4)  # s, three push times = three gait phases; the table keeps the worst one
PUSH_HI, PUSH_ITERS = 24.0, 6  # N s upper bound, bisection steps (resolution 0.375 N s)


def policy_run(model, cmd, limit=6.0, seconds=10.0, push=None, seed=0, frames=None, log=None):
    """One run of the shipped BHL policy. push = (t0 s, impulse N s, direction rad in the heading frame, 0 = forward):
    a horizontal force on the torso for 0.1 s. Returns the fall time (s) or None. log gets one row per policy step:
    time, base x, base y, then the extra joint positions."""
    m = mujoco.MjModel.from_xml_path(str(models.path(model)))
    m.opt.timestep = s2s.PHYSICS_DT
    d = mujoco.MjData(m)
    d.qpos[7:19] = s2s.DEFAULT  # BHL reset, as run.py start="bhl"
    mujoco.mj_forward(m, d)
    policy = ort.InferenceSession(str(s2s.POLICY), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(seed)
    base, z0 = m.body("base").id, d.site("imu").xpos[2]
    action, c = np.zeros(12, dtype=np.float32), np.array(cmd, dtype=np.float32)
    sub = round(s2s.POLICY_DT / s2s.PHYSICS_DT)
    for _ in range(round(seconds / s2s.POLICY_DT)):
        obs = s2s.observation(c, d.sensor("imu_quat").data.copy(), d.sensor("imu_gyro").data.copy(),
                              d.qpos[7:19], d.qvel[6:18], action)
        obs += (rng.uniform(-1, 1, 45) * s2s.NOISE).astype(np.float32)
        action = policy.run(None, {"obs": obs})[0][0]
        target = s2s.DEFAULT + s2s.ACTION_SCALE * action
        for _ in range(sub):
            d.ctrl[:] = np.clip(s2s.KP * (target - d.qpos[7:19]) - s2s.KD * d.qvel[6:18], -limit, limit)
            d.xfrc_applied[base, :3] = 0
            if push and push[0] <= d.time < push[0] + 0.1:
                R = d.xmat[base].reshape(3, 3)
                a = math.atan2(R[1, 0], R[0, 0]) + push[2]
                d.xfrc_applied[base, :2] = push[1] / 0.1 * np.array([math.cos(a), math.sin(a)])
            mujoco.mj_step(m, d)
            if frames and d.time * 30 >= frames.n:
                frames(m, d)
        if log is not None:
            log.append([d.time, *d.xpos[base][:2], *d.qpos[19:]])
        tilt = np.arccos(np.clip(-s2s.quat_rotate_inverse(d.sensor("imu_quat").data, np.array([0, 0, -1.0]))[2], -1, 1))
        if tilt > s2s.TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0:
            return float(d.time)
    return None


def max_push(model, cmd, angle, t0):
    """Largest push (N s) with no fall in the 3 s after it, by bisection. 0 when the model falls without a push."""
    if policy_run(model, cmd, seconds=t0 + 3.0) is not None:
        return 0.0
    lo, hi = 0.0, PUSH_HI
    for _ in range(PUSH_ITERS):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if policy_run(model, cmd, seconds=t0 + 3.0, push=(t0, mid, angle)) is None else (lo, mid)
    return lo


def walk_job(model, cmd_name, limit):
    r = s2s.simulate(model, COMMANDS[cmd_name], limit, 20.0, noise=True)
    return {"model": model, "cmd": cmd_name, "limit": limit, "fall": r["fall"], "vx": r["vx"], "wz": r["wz"]}


def push_job(model, cmd_name, dir_name, t0):
    return {"model": model, "cmd": cmd_name, "dir": dir_name, "t0": t0,
            "impulse": max_push(model, PUSH_CMDS[cmd_name], PUSH_DIRS[dir_name], t0)}


def static_rows():
    res = {}
    for label, v in ROWS:
        r = analyze.static_report(v)
        pitch = max(max(abs(r[f"single_{s}"]["tau"][j]) for j in analyze.LEG if any(p in j for p in report.PITCH))
                    for s in ("left", "right"))
        res[v] = {"label": label, "mass": r["mass"], "add": r["mass"] - models.URDF_MASS["biped"],
                  "com_dx": r["com_dx"], "com_dy": r["com_dy"], "com_h": r["com_h"], "stand": r["stand"]["peak"],
                  "crouch": r["crouch"]["peak"], "single_pitch": pitch, "single_all": r["single"]["peak"]}
    return res


def push_worst(push, v, c, d):
    return min(p["impulse"] for p in push if p["model"] == v and p["cmd"] == c and p["dir"] == d)


def video(model, cmd, out, seconds=10.0, push=None, dist=2.2, azim=135.0, elev=-15.0, xray=False, look_dz=0.0):
    """30 fps, 640 x 480, the camera follows the torso (as sim2sim/run.py video()). xray: the torso and the dry
    payload parts are see-through, so the water boxes show."""
    w, h = 640, 480
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                           "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", str(OUT / out)], stdin=subprocess.PIPE)
    st = {}

    def grab(m, d):
        if not st:
            st["r"] = mujoco.Renderer(m, h, w)
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.distance, cam.elevation, cam.azimuth = dist, elev, azim
            st["cam"], st["base"] = cam, m.body("base").id
            for g in range(m.ngeom) if xray else []:
                b = m.body(m.geom_bodyid[g]).name
                if b == "base" or b.startswith("payload_"):
                    m.geom_matid[g] = -1
                    m.geom_rgba[g] = ([0.45, 0.45, 0.47, 0.18] if b == "base" else [0.25, 0.55, 0.9, 0.3]
                                      if b.startswith("payload_water") else [0.85, 0.2, 0.1, 0.06])
        st["cam"].lookat[:] = d.subtree_com[st["base"]] + [0, 0, look_dz]  # whole-robot CoM, as the tracking camera
        st["r"].update_scene(d, st["cam"])
        ff.stdin.write(st["r"].render().tobytes())
        grab.n += 1
    grab.n = 0
    fall = policy_run(model, cmd, seconds=seconds, push=push, frames=grab)
    ff.stdin.close()
    ff.wait()
    st["r"].close()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", "2", "-i", str(OUT / out), "-frames:v", "1",
                    str(OUT / out.replace(".mp4", "_poster.jpg"))], check=True)
    print(out, "fall" if fall else "no fall", fall)
