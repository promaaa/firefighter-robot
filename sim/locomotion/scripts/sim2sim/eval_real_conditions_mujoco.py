"""Real-test conditions in MuJoCo for the real walking policies (policies/real_clamp*): falls, torso pitch after the
policy start, torque use and joint tracking, with model errors the first real tests pointed at.

The policy runs as its deployment config says: the real-robot effects of `sim_real_robot` (zero offsets, backlash,
delays, IMU tilt and bias, armature), the measured friction and masses, observation noise, and the Radxa torque
clamp of `torque_clamp_nm`. It starts from the standing pose with a constant forward command, as on the robot, and
runs `--seconds`; nothing stops it, and the results record when the IMU tilt first passed 20 deg.

Variants (`--variant`, comma-separated key=value, or `base`):
    imu_pitch  IMU mounted pitched forward by this many degrees (reads forward when the torso is upright)
    lean       the whole robot starts rotated forward onto the toes by this many degrees
    knee_zero  both knees stand this many degrees more extended than their encoders say
    com_x      torso centre of mass moved forward by this many cm
    friction   foot and floor friction coefficient
    harness    fraction of the weight on a 1 m rope from above the torso top (constant tension)

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_real_conditions_mujoco.py \
        --policy real_clamp12 --variant com_x=4,knee_zero=2.5,imu_pitch=3
Results: <out_dir>/<policy>/<variant>.jsonl (one line per trial) and a summary line per forward speed.
"""

import argparse
import importlib
import json
import os
import sys
import types
from collections import deque
from pathlib import Path

import numpy as np
import torch
import mujoco
import mujoco.viewer
from omegaconf import OmegaConf

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402

parser = argparse.ArgumentParser(description="Real-test conditions in MuJoCo.")
policy_select.add_argument(parser)
parser.add_argument("--variant", default="base", help="base, or key=value pairs (see the module docstring)")
parser.add_argument("--vx", type=float, nargs="+", default=[0.1, 0.2], help="forward commands (m/s)")
parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3, 4])
parser.add_argument("--seconds", type=float, default=10.0)
parser.add_argument("--out_dir", default="logs/real_conditions")
args = parser.parse_args()


class ScriptedCommands:
    """Stands in for Se2Gamepad; the evaluation writes the commands."""

    def __init__(self, *a, **k):
        self.commands = {"velocity_x": 0.0, "velocity_y": 0.0, "velocity_yaw": 0.0, "mode_switch": 0}

    def run(self):
        pass


# run headless and as fast as possible
mujoco.viewer.launch_passive = lambda model, data: types.SimpleNamespace(sync=lambda: None)
mujoco_env = importlib.import_module("berkeley_humanoid_lite.environments.mujoco")
mujoco_env.Se2Gamepad = ScriptedCommands
mujoco_env.time.sleep = lambda seconds: None

from berkeley_humanoid_lite.environments import MujocoSimulator  # noqa: E402
from berkeley_humanoid_lite_lowlevel.policy.rl_controller import RlController  # noqa: E402

WALK_FROM = 2.0  # s; the walking averages start here
HIP_KNEE_ANKLE_PITCH = [2, 3, 4, 8, 9, 10]


def parse_variant(text):
    out = {"imu_pitch": None, "lean": 0.0, "knee_zero": 0.0, "com_x": 0.0, "friction": None, "harness": 0.0}
    if text != "base":
        for item in text.split(","):
            key, value = item.split("=")
            if key not in out:
                raise ValueError(f"unknown variant key {key!r}; known: {', '.join(out)}")
            out[key] = float(value)
    return out


def quat_pitch(deg):
    half = np.radians(deg) / 2
    return np.array([np.cos(half), 0.0, np.sin(half), 0.0])


def pitch_and_tilt(quat):
    """Forward pitch and total tilt (deg) of an orientation (w, x, y, z)."""
    w, x, y, z = quat
    g = -np.array([2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z])
    return np.degrees(np.arctan2(g[0], -g[2])), np.degrees(np.arccos(np.clip(-g[2], -1, 1)))


class Harness:
    """Rope with a constant tension from an anchor 1 m above the start position of the torso top."""

    def __init__(self, robot, fraction):
        m, d = robot.mj_model, robot.mj_data
        self.d = d
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
        mujoco.mj_kinematics(m, d)
        top = np.array([d.xipos[self.base][0], d.xipos[self.base][1], d.xpos[self.base][2] + 0.86])
        self.attach_local = d.xmat[self.base].reshape(3, 3).T @ (top - d.xpos[self.base])
        self.anchor = top + np.array([0.0, 0.0, 1.0])
        self.tension = fraction * m.body_mass.sum() * 9.81

    def apply(self):
        d = self.d
        point = d.xpos[self.base] + d.xmat[self.base].reshape(3, 3) @ self.attach_local
        rope = self.anchor - point
        force = self.tension * rope / np.linalg.norm(rope)
        d.xfrc_applied[self.base, :3] = force
        d.xfrc_applied[self.base, 3:] = np.cross(point - d.xipos[self.base], force)


def place_leaning(robot, lean_deg):
    """Rotate the standing robot forward about y and lower it until the lowest foot corner touches the floor."""
    m, d = robot.mj_model, robot.mj_data
    d.qpos[3:7] = quat_pitch(lean_deg)
    feet = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX and m.geom_conaffinity[g]
            and "ankle_roll" in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "")]
    mujoco.mj_kinematics(m, d)
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    low = min((d.geom_xpos[g] + (corners * m.geom_aabb[g][3:] + m.geom_aabb[g][:3])
               @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].min() for g in feet)
    d.qpos[2] -= low - 0.0005
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def run_trial(cfg, var, vx, seed):
    torch.manual_seed(seed)
    robot = MujocoSimulator(cfg, obs_noise=True)
    if robot.real is None:
        raise ValueError("this evaluation needs a config with sim_real_robot (the real walking policies)")
    controller = RlController(cfg)
    controller.load_policy()
    m, d = robot.mj_model, robot.mj_data
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    if var["com_x"]:
        m.body_ipos[base][0] += var["com_x"] / 100.0
    if var["friction"] is not None:
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0 or (m.geom_conaffinity[g] and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX):
                m.geom_friction[g][0] = var["friction"]
    robot.reset()
    if var["lean"]:
        place_leaning(robot, var["lean"])
    if var["imu_pitch"] is not None:
        robot.imu_mount_quat = torch.tensor(quat_pitch(var["imu_pitch"]), dtype=torch.float32)
    if var["knee_zero"]:
        robot.zero_offset[3] = np.radians(var["knee_zero"])  # left knee: + extends
        robot.zero_offset[9] = -np.radians(var["knee_zero"])  # right knee: - extends
    robot.reading_history = deque([robot._real_robot_readings()], maxlen=robot.reading_history.maxlen)
    obs = robot._get_observations()
    harness = Harness(robot, var["harness"]) if var["harness"] else None

    torques = []
    apply_real = robot._apply_real_robot_actions

    def apply_and_record(command):
        if harness is not None:
            harness.apply()
        apply_real(command)
        torques.append(robot.motor_torque.copy())

    robot._apply_real_robot_actions = apply_and_record

    standing = np.array(cfg.default_joint_positions)
    pitch, tracking, first_target = [], [], None
    stop_time, fell = None, False
    for k in range(int(args.seconds / cfg.policy_dt)):
        robot.command_controller.commands.update(velocity_x=vx, velocity_y=0.0, velocity_yaw=0.0)
        observation = obs.numpy()
        observation[32:35] = [vx, 0.0, 0.0]
        target = controller.update(observation)
        if first_target is None:
            first_target = float(np.degrees(np.abs(target - standing).max()))
        q, _, _, _ = robot._delayed_readings()
        sent = robot._clamp_targets(torch.tensor(target, dtype=torch.float32)) if robot.torque_clamp else target
        tracking.append(np.degrees(np.abs(np.asarray(sent) - q.numpy())))
        obs = robot.step(torch.tensor(target, dtype=torch.float32))
        t = (k + 1) * cfg.policy_dt
        imu_pitch, imu_tilt = pitch_and_tilt(robot._real_robot_readings()[2].numpy())
        pitch.append(imu_pitch)
        if stop_time is None and imu_tilt > 20.0:
            stop_time = t
        if pitch_and_tilt(d.qpos[3:7])[1] > 60.0:
            fell = True
            break
    pitch, tracking = np.array(pitch), np.array(tracking)
    tau = np.abs(np.array(torques))
    t = cfg.policy_dt * np.arange(1, len(pitch) + 1)
    walk = t >= WALK_FROM

    def pitch_at(sec):
        i = int(round(sec / cfg.policy_dt)) - 1
        return float(pitch[i]) if i < len(pitch) else None

    return {
        "policy": args.policy, "variant": args.variant, "vx": vx, "seed": seed, "fell": fell,
        "tilt20_time": stop_time, "duration": float(t[-1]),
        "imu_pitch_0.4": pitch_at(0.4), "imu_pitch_0.6": pitch_at(0.6), "imu_pitch_0.8": pitch_at(0.8),
        "imu_pitch_max_1s": float(pitch[t <= 1.0].max()),
        "imu_pitch_walk_mean": float(pitch[walk].mean()) if walk.any() else None,
        "first_target_off_standing_deg": first_target,
        "torque_abs_mean": tau.mean(0).tolist(),
        "torque_abs_p99": np.percentile(tau, 99, axis=0).tolist(),
        "torque_over_6_fraction": (tau > 6.0).mean(0).tolist(),
        "tracking_error_p99_deg": np.percentile(tracking, 99, axis=0).tolist(),
        "tracking_error_max_deg": tracking.max(0).tolist(),
    }


def main():
    cfg = OmegaConf.load(policy_select.config(args.policy))
    var = parse_variant(args.variant)
    out_dir = Path(args.out_dir) / args.policy
    os.makedirs(out_dir, exist_ok=True)
    path = out_dir / f"{args.variant.replace(',', '_').replace('=', '')}.jsonl"
    rows = []
    with open(path, "w") as f:
        for vx in args.vx:
            for seed in args.seeds:
                row = run_trial(cfg, var, vx, seed)
                rows.append(row)
                f.write(json.dumps(row) + "\n")
                f.flush()
    print(f"{args.policy} {args.variant} -> {path}")
    for vx in args.vx:
        sel = [r for r in rows if r["vx"] == vx]
        p = [np.mean([r[k] for r in sel if r[k] is not None]) for k in ("imu_pitch_0.4", "imu_pitch_0.6", "imu_pitch_0.8")]
        walk = [r["imu_pitch_walk_mean"] for r in sel if r["imu_pitch_walk_mean"] is not None and not r["fell"]]
        torque = np.mean([np.mean(r["torque_abs_mean"]) for r in sel])
        over6 = 100 * np.mean([np.mean(np.array(r["torque_over_6_fraction"])[HIP_KNEE_ANKLE_PITCH]) for r in sel])
        print(f"  vx {vx}: fell {sum(r['fell'] for r in sel)}/{len(sel)}, IMU tilt > 20 deg "
              f"{sum(r['tilt20_time'] is not None for r in sel)}/{len(sel)}, IMU pitch at 0.4/0.6/0.8 s "
              f"{p[0]:.1f}/{p[1]:.1f}/{p[2]:.1f} deg, walking {np.mean(walk) if walk else float('nan'):.1f} deg, "
              f"first target {np.mean([r['first_target_off_standing_deg'] for r in sel]):.1f} deg off the stand, "
              f"mean |torque| {torque:.2f} N m, > 6 N m {over6:.0f} % (hip/knee/ankle pitch)")


if __name__ == "__main__":
    main()
