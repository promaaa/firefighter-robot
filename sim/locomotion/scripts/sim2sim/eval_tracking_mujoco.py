"""Command-tracking evaluation in MuJoCo (sim2sim), the counterpart of eval_tracking_isaaclab.py.

Runs every command of tracking_common.COMMANDS from the standing pose, once per seed, with the deployment stack of
play_mujoco.py (MujocoSimulator + RlController) and training-matched observation noise unless --no_obs_noise.
Results go to <out_dir>/<name>_mujoco_<noise|clean>.csv and a summary is printed.

Usage (from the repository root):
    uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_tracking_mujoco.py --policy baseline --name baseline
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))  # repository root: policies/policy_select.py
from policies import policy_select  # noqa: E402
import importlib
import os
import types

import numpy as np
import torch
import mujoco
import mujoco.viewer
from omegaconf import OmegaConf

from tracking_common import COMMANDS, command_at, num_steps, print_summary, tilt_of, trial_metrics, write_rows, yaw_of

parser = argparse.ArgumentParser(description="Command-tracking evaluation in MuJoCo.")
policy_select.add_argument(parser)
parser.add_argument("--name", type=str, required=True, help="Name of the result file.")
parser.add_argument("--out_dir", type=str, default="logs/sim2sim_tracking", help="Output directory.")
parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2], help="Seeds of the observation noise.")
parser.add_argument("--no_obs_noise", action="store_true", help="Feed noise-free observations to the policy.")
parser.add_argument("--no_real_robot", action="store_true",
                    help="Ignore the config's sim_real_robot block (ideal actuators and sensors).")
parser.add_argument("--kp_scale", type=float, default=1.0,
                    help="Factor on the simulated PD stiffness of every joint (actuator gap; the policy is unchanged; "
                         "put the factors in --name).")
parser.add_argument("--kd_scale", type=float, default=1.0, help="Factor on the simulated PD damping of every joint.")
args = parser.parse_args()


class ScriptedCommands:
    """Stands in for Se2Gamepad; the evaluation writes the commands."""

    def __init__(self, *args, **kwargs):
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


def run_trial(robot, controller, cmd):
    model, data = robot.mj_model, robot.mj_data
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    # the torso is every body welded to the base (IMU, payload); Isaac Lab merges the payload into the base
    torso = np.nonzero(model.body_weldid == base_id)[0]
    torso_mass = model.body_mass[torso] / model.body_mass[torso].sum()
    obs = robot.reset()
    controller.prev_actions[:] = 0.0
    pos, yaw, tilt, torque = [], [], [], []
    for k in range(num_steps(robot.cfg.policy_dt)):
        actions = controller.update(obs.numpy())
        vx, vy, wz = command_at((k + 1) * robot.cfg.policy_dt, cmd)
        # MujocoSimulator halves velocity_y (gamepad scaling) before it reaches the policy, so pre-scale it
        robot.command_controller.commands.update(velocity_x=vx, velocity_y=vy / 0.5, velocity_yaw=wz)
        obs = robot.step(torch.tensor(actions))
        pos.append(torso_mass @ data.xipos[torso])
        yaw.append(yaw_of(data.xquat[base_id]))
        tilt.append(tilt_of(data.xquat[base_id]))
        torque.append(data.actuator_force.copy())
        if tilt[-1] > np.pi / 2:
            break
    return trial_metrics(pos, yaw, tilt, torque, robot.cfg.policy_dt)


def main():
    cfg = OmegaConf.load(policy_select.config(args.policy))
    if args.no_real_robot:
        cfg.sim_real_robot = None
    obs_noise = not args.no_obs_noise
    robot = MujocoSimulator(cfg, obs_noise=obs_noise)
    robot.joint_kp *= args.kp_scale
    robot.joint_kd *= args.kd_scale
    controller = RlController(cfg)
    controller.load_policy()

    rows = []
    for seed in args.seeds:
        torch.manual_seed(seed)
        for name, cmd in COMMANDS:
            row = {"sim": "mujoco", "obs_noise": int(obs_noise), "seed": seed, "command": name,
                   "cmd_vx": cmd[0], "cmd_vy": cmd[1], "cmd_wz": cmd[2]}
            row.update(run_trial(robot, controller, cmd))
            rows.append(row)
            print(f"[INFO] seed {seed} {name}: " + ", ".join(
                f"{k} {row[k]:.3f}" for k in ("fell", "vx", "vy", "wz", "height")))

    os.makedirs(args.out_dir, exist_ok=True)
    path = os.path.join(args.out_dir, f"{args.name}_mujoco_{'noise' if obs_noise else 'clean'}.csv")
    write_rows(path, rows)
    print(f"\nMuJoCo, observation noise {'on' if obs_noise else 'off'}, {len(args.seeds)} seeds -> {path}")
    print_summary(rows)


if __name__ == "__main__":
    main()
