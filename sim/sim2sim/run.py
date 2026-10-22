"""Shared helpers for the shipped BHL walking policy in MuJoCo, on the CPU: policy file, joint order, default pose,
gains, observation and one simulation run. legs/, robot/, sim/mission, sim/rl, sim/payload and docs/hardware/power_calc.py
import them.

    uv run python sim/sim2sim/run.py --check      self-check: the bare biped stands 5 s with a zero command
    uv run python sim/sim2sim/run.py              RUN_MODELS x commands x torque limits -> results/results.md
    uv run python sim/sim2sim/run.py --noise      the same, with the training observation noise -> results/results_noise.md
    uv run python sim/sim2sim/run.py --start key  the same, from the flat-foot "stand" keyframe -> results/results_keyframe.md
    MUJOCO_GL=egl uv run python sim/sim2sim/run.py --video MODEL CMD LIMIT OUT.mp4 [--noise]   one 10 s run (needs ffmpeg)

The models come from sim/payload/models.py. The observation, action and PD rules copy the BHL runtime (README.md).
"""
import subprocess
import sys
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve().parent
MODELS = HERE.parent / "payload" / "models"
OUT = HERE / "results"
OUT.mkdir(exist_ok=True)
POLICY = HERE / "policy_biped_25hz_a.onnx"  # BHL checkpoints/policy_biped_25hz_a.onnx, commit 984741a

# BHL configs/policy_biped_25hz_a.yaml
JOINTS = [f"leg_{s}_{j}_joint" for s in ("left", "right")
          for j in ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")]
DEFAULT = np.array([0, 0, -0.2, 0.4, -0.3, 0] * 2, dtype=np.float32)
KP, KD, ACTION_SCALE = 20.0, 2.0, 0.25
POLICY_DT, PHYSICS_DT = 0.04, 0.0005  # 25 Hz policy, 2000 Hz physics as in BHL sim2sim
TILT_LIMIT = 0.78  # rad, BHL training termination

RUN_MODELS = ["bhl_biped", "bhl_F2", "bhl_F2_half", "bhl_F2_empty"]  # the models that sim/payload/models.py builds for F2
COMMANDS = {"stand": (0.0, 0.0, 0.0), "vx 0.3": (0.3, 0.0, 0.0), "yaw 0.5": (0.0, 0.0, 0.5),
            "vx 0.2 + yaw 0.5": (0.2, 0.0, 0.5)}  # the last one is like aim/ APPROACH
LIMITS = [6.0, 4.0]  # Nm: training limit, and the limit of the BHL Python runtime on the robot
# uniform observation noise of the training (BHL biped/env_cfg.py:55-72), used only with --noise
NOISE = np.r_[np.zeros(3), np.full(3, 0.3), np.full(3, 0.05), np.full(12, 0.05), np.full(12, 2.0), np.zeros(12)]


def quat_rotate_inverse(q, v):
    """Same formula as BHL RlController.quat_rotate_inverse. q = [w, x, y, z]."""
    w, u = q[0], q[1:4]
    return v * (2.0 * w * w - 1.0) - np.cross(u, v) * w * 2.0 + u * np.dot(u, v) * 2.0


def observation(cmd, quat, gyro, q, qd, prev_action):
    """45 values in the BHL order: command, angular velocity, projected gravity, joint pos - default, joint vel, last action."""
    grav = quat_rotate_inverse(quat, np.array([0.0, 0.0, -1.0]))
    return np.concatenate([cmd, gyro, grav, q - DEFAULT, qd, prev_action]).astype(np.float32)[None]


def simulate(model, command, limit, seconds=20.0, frames=None, start="bhl", noise=False):
    """One run. Returns a dict of results. frames: optional callback(m, d) at 30 Hz for video.

    start="bhl": BHL MujocoSimulator.reset(): base at (0, 0, 0), upright, default joints, then the robot drops a few cm.
    start="key": the flat-foot "stand" keyframe of sim/payload/models (feet on the floor, torso pitched 0.1 rad).
    noise=True: add the training observation noise (fixed seed). The BHL runtime adds none, but real sensors are noisy.
    """
    rng = np.random.default_rng(0)
    m = mujoco.MjModel.from_xml_path(str(MODELS / f"{model}.xml"))
    m.opt.timestep = PHYSICS_DT
    d = mujoco.MjData(m)
    if start == "key":
        mujoco.mj_resetDataKeyframe(m, d, m.key("stand").id)
    else:
        d.qpos[7:19] = DEFAULT  # qpos0 already has the base at (0, 0, 0) with quaternion (1, 0, 0, 0)
    mujoco.mj_forward(m, d)
    assert [m.joint(j).qposadr[0] for j in JOINTS] == list(range(7, 19)), "joint order differs from the policy"
    assert [m.actuator(i).name for i in range(m.nu)] == JOINTS, "actuator order differs from the policy"
    policy = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
    base = m.body("base").id
    cmd = np.array(command, dtype=np.float32)
    action = np.zeros(12, dtype=np.float32)
    substeps = round(POLICY_DT / PHYSICS_DT)
    z0 = d.site("imu").xpos[2]
    vx, wz, sat, peak, fall = [], [], 0, 0.0, None
    for k in range(round(seconds / POLICY_DT)):
        quat, gyro = d.sensor("imu_quat").data.copy(), d.sensor("imu_gyro").data.copy()
        obs = observation(cmd, quat, gyro, d.qpos[7:19], d.qvel[6:18], action)
        if noise:
            obs += (rng.uniform(-1, 1, 45) * NOISE).astype(np.float32)
        action = policy.run(None, {"obs": obs})[0][0]  # BHL action limits are +-10000, so no clip
        target = DEFAULT + ACTION_SCALE * action
        for _ in range(substeps):
            tau = KP * (target - d.qpos[7:19]) - KD * d.qvel[6:18]
            sat += np.any(np.abs(tau) > limit)
            d.ctrl[:] = np.clip(tau, -limit, limit)
            peak = max(peak, float(np.abs(d.ctrl).max()))
            mujoco.mj_step(m, d)
            if frames and d.time * 30 >= frames.n:
                frames(m, d)
        t = d.time
        tilt = np.arccos(np.clip(-quat_rotate_inverse(d.sensor("imu_quat").data, np.array([0, 0, -1.0]))[2], -1, 1))
        if fall is None and (tilt > TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0):
            fall = t
        if fall is not None and (not frames or t > fall + 2.0):  # a video shows 2 s after the fall
            break
        if t >= 5.0 and fall is None:  # skip the start transient
            R = d.xmat[base].reshape(3, 3)
            yaw = np.arctan2(R[1, 0], R[0, 0])
            v = d.sensor("frame_vel").data
            vx.append(np.cos(yaw) * v[0] + np.sin(yaw) * v[1])  # forward speed in the heading frame
            wz.append((R @ d.qvel[3:6])[2])  # yaw rate in the world frame
    n = k + 1
    ok = vx and fall is None  # no tracking numbers for a run that falls
    return dict(model=model, command=command, limit=limit, fall=fall,
                vx=float(np.mean(vx)) if ok else None, wz=float(np.mean(wz)) if ok else None,
                sat=100.0 * sat / (n * substeps), peak=peak)


def row(r, name):
    res = f"falls at {r['fall']:.1f} s" if r["fall"] is not None else "no fall"
    f = lambda x: "-" if x is None else f"{x:+.2f}"
    return f"| {r['model']} | {r['limit']:.0f} | {name} | {res} | {f(r['vx'])} | {f(r['wz'])} | {r['sat']:.1f} | {r['peak']:.1f} |"


def run_all(start, noise):
    jobs = [(mo, COMMANDS[c], lim, 20.0, None, start, noise) for mo in RUN_MODELS for lim in LIMITS for c in COMMANDS]
    names = {v: k for k, v in COMMANDS.items()}
    with Pool(4) as pool:  # CPU only, 4 processes
        results = pool.starmap(simulate, jobs)
    lines = ["| Model | Torque limit (Nm) | Command | Result in 20 s | Mean vx, 5-20 s (m/s) | Mean yaw rate, 5-20 s (rad/s) | Steps with a clipped torque (%) | Peak torque (Nm) |",
             "|---|---|---|---|---|---|---|---|"]
    lines += [row(r, names[r["command"]]) for r in results]
    text = "\n".join(lines) + "\n"
    (OUT / f"results{'_keyframe' if start == 'key' else ''}{'_noise' if noise else ''}.md").write_text(text)
    print(text)


def video(model, cmd_name, limit, out, seconds=10.0, noise=False):
    """Render one run at 30 fps, 640x480, camera follows the torso. Pipes raw frames to ffmpeg."""
    width, height = 640, 480
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                           "-s", f"{width}x{height}", "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", out],
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
    r = simulate(model, COMMANDS[cmd_name], float(limit), seconds, frames=grab, noise=noise)
    ff.stdin.close()
    ff.wait()
    state["r"].close()  # free the EGL context before Python exits
    print(row(r, cmd_name))


def check():
    """Self-check: projected gravity of an upright torso is (0, 0, -1), and the bare biped stands 5 s."""
    assert np.allclose(quat_rotate_inverse(np.array([1.0, 0, 0, 0]), np.array([0, 0, -1.0])), [0, 0, -1])
    r = simulate("bhl_biped", (0.0, 0.0, 0.0), 6.0, seconds=5.0)
    assert r["fall"] is None, f"bare biped fell at {r['fall']:.2f} s"
    print("check passed: bare biped stands 5 s with a zero command (6 Nm)")


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--video" in sys.argv:
        i = sys.argv.index("--video")
        video(*sys.argv[i + 1:i + 5], noise="--noise" in sys.argv)
    else:
        run_all("key" if "--start" in sys.argv and sys.argv[sys.argv.index("--start") + 1] == "key" else "bhl",
                "--noise" in sys.argv)
