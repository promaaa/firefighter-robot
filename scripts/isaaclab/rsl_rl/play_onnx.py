"""Script to play an exported ONNX policy in the Isaac Lab training environment.

The policy is described by a deployment config (the same YAML used by sim2sim), so pretrained
checkpoints that ship only as ONNX can be checked against the training environment.
"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Play an exported ONNX policy in Isaac Lab.")
parser.add_argument("--config", type=str, required=True, help="Path to the deployment config of the policy.")
parser.add_argument("--video", action="store_true", default=False, help="Record a video.")
parser.add_argument("--video_length", type=int, default=500, help="Length of the recorded video (in steps).")
parser.add_argument("--follow", action="store_true", default=False, help="Make the camera follow the first robot.")
parser.add_argument("--steps", type=int, default=None, help="Number of policy steps to run before exiting.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to simulate.")
parser.add_argument(
    "--task", type=str, default="Velocity-Berkeley-Humanoid-Lite-Biped-v0", help="Name of the task."
)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import numpy as np
import os
import torch

import onnxruntime as ort
from omegaconf import OmegaConf

from isaaclab.utils.dict import print_dict
from isaaclab_tasks.utils import parse_env_cfg

# Import extensions to set up environment tasks
import berkeley_humanoid_lite.tasks  # noqa: F401


class OnnxPolicy:
    """Runs an ONNX policy on a batch of observations, one row at a time if the batch size is fixed."""

    def __init__(self, checkpoint_path: str):
        self.session = ort.InferenceSession(checkpoint_path, providers=["CPUExecutionProvider"])
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.fixed_batch = isinstance(model_input.shape[0], int)
        self.num_observations = model_input.shape[-1]

    def __call__(self, observations: torch.Tensor) -> torch.Tensor:
        obs = observations.detach().cpu().numpy().astype(np.float32)
        if obs.shape[1] + 1 == self.num_observations:
            # a policy that also observes the push signal (last input): no push is planned
            obs = np.concatenate([obs, np.zeros((obs.shape[0], 1), dtype=np.float32)], axis=1)
        if self.fixed_batch:
            actions = np.concatenate([self.session.run(None, {self.input_name: row[None]})[0] for row in obs])
        else:
            actions = self.session.run(None, {self.input_name: obs})[0]
        return torch.from_numpy(actions).to(observations.device)


def main():
    """Play an ONNX policy in Isaac Lab."""
    deploy_cfg = OmegaConf.load(args_cli.config)
    print(f"[INFO] Loading deployment config from: {args_cli.config}")

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    # run the policy at the rate it was trained for
    decimation = round(deploy_cfg.policy_dt / env_cfg.sim.dt)
    if decimation != env_cfg.decimation:
        print(f"[INFO] Overriding decimation {env_cfg.decimation} -> {decimation} (policy_dt={deploy_cfg.policy_dt})")
        env_cfg.decimation = decimation
        env_cfg.sim.render_interval = decimation
    if args_cli.follow:
        env_cfg.viewer.origin_type = "asset_root"
        env_cfg.viewer.asset_name = "robot"
        env_cfg.viewer.env_index = 0
        env_cfg.viewer.eye = (1.6, 1.6, 0.9)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.4)

    # check that the policy matches the environment
    if abs(deploy_cfg.action_scale - env_cfg.actions.joint_pos.scale) > 1e-6:
        raise ValueError(
            f"Action scale mismatch: policy {deploy_cfg.action_scale}, env {env_cfg.actions.joint_pos.scale}"
        )
    policy_joints = [deploy_cfg.joints[i] for i in deploy_cfg.action_indices]
    if policy_joints != list(env_cfg.actions.joint_pos.joint_names):
        raise ValueError(
            f"Joint order mismatch:\n policy {policy_joints}\n env {env_cfg.actions.joint_pos.joint_names}"
        )

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join("videos", "isaaclab"),
            "name_prefix": os.path.splitext(os.path.basename(args_cli.config))[0]
            + ("_follow" if args_cli.follow else ""),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during play.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    policy = OnnxPolicy(deploy_cfg.policy_checkpoint_path)
    print(f"[INFO]: Loading ONNX policy from: {deploy_cfg.policy_checkpoint_path}")
    num_obs = env.observation_space["policy"].shape[-1]
    if policy.num_observations not in (num_obs, num_obs + 1):
        raise ValueError(
            f"Observation size mismatch: policy {policy.num_observations}, "
            f"env {env.observation_space['policy'].shape[-1]}"
        )

    unwrapped = env.unwrapped
    robot = unwrapped.scene["robot"]
    policy_dt = unwrapped.step_dt

    # metrics
    num_steps = 0
    num_falls = 0
    num_timeouts = 0
    abs_err_sum = torch.zeros(3, device=unwrapped.device)
    num_valid = 0

    def report():
        mean_err = (abs_err_sum / max(num_valid, 1)).tolist()
        print(
            f"[METRIC] t={num_steps * policy_dt:6.1f}s  falls={num_falls}  timeouts={num_timeouts}  "
            f"mean |err| vx={mean_err[0]:.3f} m/s  vy={mean_err[1]:.3f} m/s  wz={mean_err[2]:.3f} rad/s"
        )

    # reset environment
    obs, _ = env.reset()
    obs = obs["policy"]
    max_steps = args_cli.steps
    if args_cli.video and max_steps is None:
        max_steps = args_cli.video_length
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, terminated, truncated, _ = env.step(actions)
            obs = obs["policy"]

            command = unwrapped.command_manager.get_command("base_velocity")
            measured = torch.cat([robot.data.root_lin_vel_b[:, :2], robot.data.root_ang_vel_b[:, 2:3]], dim=-1)
            # steps that end an episode measure the reset state, so leave them out
            done = terminated | truncated
            abs_err_sum += ((command - measured).abs() * (~done).unsqueeze(-1)).sum(dim=0)
            num_valid += int((~done).sum())
            num_falls += int(terminated.sum())
            num_timeouts += int(truncated.sum())
        num_steps += 1
        if num_steps % round(5.0 / policy_dt) == 0:
            report()
        if max_steps is not None and num_steps >= max_steps:
            break
    if num_steps % round(5.0 / policy_dt) != 0:
        report()

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
