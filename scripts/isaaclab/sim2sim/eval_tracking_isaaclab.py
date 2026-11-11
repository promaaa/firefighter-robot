"""Command-tracking evaluation in Isaac Lab, the reference for eval_tracking_mujoco.py (sim2sim).

Runs every command of tracking_common.COMMANDS from the standing pose in parallel (--replicas environments per
command), with the training task at its nominal dynamics: no mass, gain, default-pose or friction randomization
(foot friction 0.6 as in the course MJCF), no reset push or perturbation, the task's payload kept. The observation
noise is the training one unless --no_obs_noise. The policy is the exported ONNX file of a deployment config, the
same network the MuJoCo evaluation runs. Results go to <out_dir>/<name>_isaaclab_<noise|clean>.csv.

Usage (from the repository root):
    uv run scripts/sim2sim/eval_tracking_isaaclab.py --headless --task Velocity-ASMR-Biped-v0 \
        --config configs/policy_asmr_biped.yaml --name baseline
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Command-tracking evaluation in Isaac Lab.")
parser.add_argument("--task", type=str, default="Velocity-ASMR-Biped-v0", help="Name of the task.")
parser.add_argument("--config", type=str, required=True, help="Deployment config of the policy (ONNX).")
parser.add_argument("--name", type=str, required=True, help="Name of the result file.")
parser.add_argument("--out_dir", type=str, default="logs/sim2sim_tracking", help="Output directory.")
parser.add_argument("--replicas", type=int, default=8, help="Environments per command.")
parser.add_argument("--friction", type=float, default=0.6, help="Foot friction (static and dynamic).")
parser.add_argument("--no_obs_noise", action="store_true", help="Feed noise-free observations to the policy.")
parser.add_argument("--seed", type=int, default=0, help="Seed of the environment.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
simulation_app = AppLauncher(args_cli).app

import os

import gymnasium as gym
import numpy as np
import onnxruntime as ort
import torch
from omegaconf import OmegaConf

from isaaclab_tasks.utils import parse_env_cfg

import berkeley_humanoid_lite.tasks  # noqa: F401
from tracking_common import COMMANDS, command_at, num_steps, print_summary, tilt_of, trial_metrics, write_rows, yaw_of


def check_deploy_config(deploy_cfg, env_cfg, robot, dt):
    """Compare the deployment config, which drives the MuJoCo side, with the task (at its nominal dynamics)."""
    ids = [robot.joint_names.index(name) for name in deploy_cfg.joints]
    data = robot.data
    payload = env_cfg.events.add_payload.params if hasattr(env_cfg.events, "add_payload") else None
    sim_payload = deploy_cfg.get("sim_payload", None)
    checks = {
        "policy_dt": abs(dt - deploy_cfg.policy_dt),
        "action joint order": float(list(env_cfg.actions.joint_pos.joint_names) != list(deploy_cfg.joints)),
        "action_scale": abs(env_cfg.actions.joint_pos.scale - deploy_cfg.action_scale),
        "default_joint_positions": (data.default_joint_pos[0, ids].cpu()
                                    - torch.tensor(deploy_cfg.default_joint_positions)).abs().max().item(),
        "default_base_position z": abs(data.default_root_state[0, 2].item() - deploy_cfg.default_base_position[2]),
        "joint_kp": (data.joint_stiffness[0, ids].cpu() - torch.tensor(deploy_cfg.joint_kp)).abs().max().item(),
        "joint_kd": (data.joint_damping[0, ids].cpu() - torch.tensor(deploy_cfg.joint_kd)).abs().max().item(),
        "effort_limits": (data.joint_effort_limits[0, ids].cpu()
                          - torch.tensor(deploy_cfg.effort_limits)).abs().max().item(),
        "payload": float((payload is None) != (sim_payload is None) or (
            payload is not None and any(np.abs(np.array(payload[key]) - np.array(sim_payload[key])).max() > 1e-6
                                        for key in ("mass", "position", "size")))),
    }
    for key, err in checks.items():
        print(f"[CHECK] deploy config vs task: {key:24s} max difference {err:.2e}")
    if max(checks.values()) > 1e-4:
        raise ValueError("The deployment config does not match the task.")


def main():
    deploy_cfg = OmegaConf.load(args_cli.config)
    session = ort.InferenceSession(deploy_cfg.policy_checkpoint_path)
    input_name = session.get_inputs()[0].name

    num_envs = len(COMMANDS) * args_cli.replicas
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=num_envs)
    env_cfg.seed = args_cli.seed
    env_cfg.episode_length_s = 1000.0
    env_cfg.terminations.base_orientation = None
    env_cfg.observations.policy.enable_corruption = not args_cli.no_obs_noise
    for term in ["add_base_mass", "add_all_joint_default_pos", "scale_all_actuator_torque_constant",
                 "base_external_force_torque"]:
        setattr(env_cfg.events, term, None)
    material = env_cfg.events.physics_material.params
    material["static_friction_range"] = material["dynamic_friction_range"] = (args_cli.friction, args_cli.friction)
    env_cfg.events.reset_base.params["pose_range"] = {}
    env_cfg.events.reset_base.params["velocity_range"] = {}
    env_cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    command_cfg = env_cfg.commands.base_velocity
    command_cfg.heading_command = False
    command_cfg.rel_standing_envs = 0.0
    command_cfg.resampling_time_range = (1.0e6, 1.0e6)
    command_cfg.debug_vis = False

    env = gym.make(args_cli.task, cfg=env_cfg)
    unwrapped = env.unwrapped
    robot = unwrapped.scene["robot"]
    command_term = unwrapped.command_manager.get_term("base_velocity")
    origins = unwrapped.scene.env_origins
    dt = unwrapped.step_dt
    check_deploy_config(deploy_cfg, env_cfg, robot, dt)
    trials = [(name, cmd) for name, cmd in COMMANDS for _ in range(args_cli.replicas)]

    pos, yaw, tilt, torque = [], [], [], []
    with torch.inference_mode():
        obs = env.reset()[0]["policy"]
        for k in range(num_steps(dt)):
            command = torch.tensor([command_at(k * dt, cmd) for _, cmd in trials], device=unwrapped.device)
            command_term.vel_command_b[:] = command
            obs[:, 0:3] = command
            obs_np = obs.cpu().numpy()
            actions = np.concatenate([session.run(None, {input_name: obs_np[i:i + 1]})[0] for i in range(num_envs)])
            obs = env.step(torch.tensor(actions, device=unwrapped.device))[0]["policy"]
            quat = robot.data.root_quat_w.cpu().numpy()
            pos.append((robot.data.root_com_pos_w - origins).cpu().numpy())
            yaw.append([yaw_of(q) for q in quat])
            tilt.append([tilt_of(q) for q in quat])
            torque.append(robot.data.applied_torque.cpu().numpy())
    pos, yaw, tilt, torque = map(np.array, (pos, yaw, tilt, torque))

    rows = []
    for i, (name, cmd) in enumerate(trials):
        row = {"sim": "isaaclab", "obs_noise": int(not args_cli.no_obs_noise), "seed": i % args_cli.replicas,
               "command": name, "cmd_vx": cmd[0], "cmd_vy": cmd[1], "cmd_wz": cmd[2]}
        row.update(trial_metrics(pos[:, i], yaw[:, i], tilt[:, i], torque[:, i], dt))
        rows.append(row)

    os.makedirs(args_cli.out_dir, exist_ok=True)
    path = os.path.join(args_cli.out_dir, f"{args_cli.name}_isaaclab_{'clean' if args_cli.no_obs_noise else 'noise'}.csv")
    write_rows(path, rows)
    print(f"\nIsaac Lab, observation noise {'off' if args_cli.no_obs_noise else 'on'}, {args_cli.replicas} replicas -> {path}")
    print_summary(rows)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
