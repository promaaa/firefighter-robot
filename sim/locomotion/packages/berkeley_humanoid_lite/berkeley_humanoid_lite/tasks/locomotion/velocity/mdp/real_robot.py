"""Terms of the real-robot model (source.robot_models.robots.asmr2026_real): sensor readings as the policy
sees them on the robot (motor-side encoders with their zero offset, IMU with its mounting tilt and gyro bias, all
delayed per environment) and the armature randomization."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import SceneEntityCfg

from .actuators import _selected_actuator_joints

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def encoder_joint_pos_rel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Encoder joint positions relative to the default pose (motor side, zero offset, delay)."""
    asset = env.scene[asset_cfg.name]
    pos, _ = asset.delayed_joint_readings()
    return pos[:, asset_cfg.joint_ids] - asset.data.default_joint_pos[:, asset_cfg.joint_ids]


def encoder_joint_vel_rel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Encoder joint velocities (motor side, delay)."""
    asset = env.scene[asset_cfg.name]
    _, vel = asset.delayed_joint_readings()
    return vel[:, asset_cfg.joint_ids] - asset.data.default_joint_vel[:, asset_cfg.joint_ids]


def imu_ang_vel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Base angular velocity in the base frame as the IMU reports it (delay)."""
    ang_vel, _ = env.scene[asset_cfg.name].delayed_imu_readings()
    return ang_vel


def imu_projected_gravity(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Gravity direction in the base frame as the IMU orientation gives it (delay)."""
    _, gravity = env.scene[asset_cfg.name].delayed_imu_readings()
    return gravity


def randomize_actuator_armature(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    armature_range: tuple[float, float],
):
    """Draw each selected joint's armature (kg m^2) uniformly in the range, in the simulation and in the actuator
    model (the backlash model uses it as the motor inertia)."""
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)
    for actuator in asset.actuators.values():
        indices = _selected_actuator_joints(asset, actuator, asset_cfg)
        if indices is None:
            continue
        values = _randomize_prop_by_op(
            actuator.armature[env_ids].clone(), armature_range, dim_0_ids=None, dim_1_ids=indices, operation="abs",
            distribution="uniform",
        )
        actuator.armature[env_ids] = values
        asset.write_joint_armature_to_sim(values, joint_ids=actuator.joint_indices, env_ids=env_ids)
