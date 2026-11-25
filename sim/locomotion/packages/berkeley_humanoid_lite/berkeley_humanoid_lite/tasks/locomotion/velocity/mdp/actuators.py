"""Randomization of the actuator model beyond the PD gains: torque limits and joint friction."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING, Literal

from isaaclab.actuators import ActuatorBase
from isaaclab.assets import Articulation
from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


def _selected_actuator_joints(asset: Articulation, actuator: ActuatorBase, asset_cfg: SceneEntityCfg):
    """Indices into the actuator's joints that asset_cfg selects (as in isaaclab's randomize_actuator_gains)."""
    if isinstance(asset_cfg.joint_ids, slice):
        return slice(None)
    asset_joint_ids = torch.as_tensor(asset_cfg.joint_ids, device=asset.device)
    if isinstance(actuator.joint_indices, slice):
        return asset_joint_ids
    actuator_joint_ids = torch.as_tensor(actuator.joint_indices, device=asset.device)
    indices = torch.nonzero(torch.isin(actuator_joint_ids, asset_joint_ids)).view(-1)
    return indices if len(indices) > 0 else None


def randomize_actuator_effort_limit(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    effort_distribution_params: tuple[float, float],
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """Scale each joint's torque limit, from the actuator's nominal limit, by its own random factor.

    For implicit actuators the limit is the PhysX joint force limit, so it is also written into the simulation.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    for actuator in asset.actuators.values():
        indices = _selected_actuator_joints(asset, actuator, asset_cfg)
        if indices is None:
            continue
        if not isinstance(actuator.cfg.effort_limit_sim, float):
            raise ValueError("randomize_actuator_effort_limit needs a scalar effort_limit_sim on the actuator")
        limit = actuator.effort_limit_sim[env_ids].clone()
        limit[:, indices] = actuator.cfg.effort_limit_sim
        limit = _randomize_prop_by_op(
            limit, effort_distribution_params, dim_0_ids=None, dim_1_ids=indices, operation="scale",
            distribution=distribution,
        )
        actuator.effort_limit_sim[env_ids] = limit
        actuator.effort_limit[env_ids] = limit
        asset.write_joint_effort_limit_to_sim(limit, joint_ids=actuator.joint_indices, env_ids=env_ids)


def randomize_actuator_friction(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    coulomb_distribution_params: tuple[float, float] | None = None,
    viscous_distribution_params: tuple[float, float] | None = None,
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "log_uniform",
):
    """Scale the Coulomb and viscous joint friction of friction actuators (FrictionImplicitActuator), from their
    nominal values, by independent random factors per joint.

    The tanh smoothing velocity of the Coulomb term grows with its factor (it never shrinks), so the explicit
    friction torque stays as stable as with the nominal friction.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    for actuator in asset.actuators.values():
        if not hasattr(actuator, "coulomb_friction"):
            continue
        indices = _selected_actuator_joints(asset, actuator, asset_cfg)
        if indices is None:
            continue
        ones = torch.ones(len(env_ids), actuator.num_joints, device=asset.device)
        if coulomb_distribution_params is not None:
            factor = _randomize_prop_by_op(
                ones.clone(), coulomb_distribution_params, dim_0_ids=None, dim_1_ids=indices, operation="scale",
                distribution=distribution,
            )
            actuator.coulomb_friction[env_ids] = actuator.cfg.coulomb_friction * factor
            actuator.friction_smoothing_velocity[env_ids] = (
                actuator.cfg.friction_smoothing_velocity * factor.clamp(min=1.0)
            )
        if viscous_distribution_params is not None:
            factor = _randomize_prop_by_op(
                ones.clone(), viscous_distribution_params, dim_0_ids=None, dim_1_ids=indices, operation="scale",
                distribution=distribution,
            )
            actuator.viscous_friction[env_ids] = actuator.cfg.viscous_friction * factor


def randomize_actuator_friction_model(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    coulomb_scale_range: tuple[float, float] | None = None,
    viscous_range: tuple[float, float] | None = None,
    load_scale_range: tuple[float, float] | None = None,
    stribeck_ratio_range: tuple[float, float] | None = None,
):
    """Draw the friction model of friction actuators (FrictionImplicitActuator) per joint, uniformly in the given
    ranges: the Coulomb friction as a factor on the nominal value, the viscous friction (N m s / rad), the factor s
    on the load-dependent friction and the Stribeck ratio. The smoothing velocity is left alone: use these ranges
    with adaptive smoothing, which follows the friction magnitude.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    if env_ids is None:
        env_ids = torch.arange(env.scene.num_envs, device=asset.device)

    for actuator in asset.actuators.values():
        if not hasattr(actuator, "load_friction_scale"):
            continue
        indices = _selected_actuator_joints(asset, actuator, asset_cfg)
        if indices is None:
            continue
        nominal = {
            "coulomb_friction": actuator.cfg.coulomb_friction,
            "viscous_friction": actuator.cfg.viscous_friction,
            "load_friction_scale": 1.0,
            "stribeck_ratio": actuator.cfg.stribeck_ratio,
        }
        ranges = {
            "coulomb_friction": None if coulomb_scale_range is None else
            (coulomb_scale_range[0] * nominal["coulomb_friction"], coulomb_scale_range[1] * nominal["coulomb_friction"]),
            "viscous_friction": viscous_range,
            "load_friction_scale": load_scale_range,
            "stribeck_ratio": stribeck_ratio_range,
        }
        for name, value_range in ranges.items():
            if value_range is None:
                continue
            values = getattr(actuator, name)[env_ids].clone()
            values[:, indices] = nominal[name]
            values = _randomize_prop_by_op(
                values, value_range, dim_0_ids=None, dim_1_ids=indices, operation="abs", distribution="uniform")
            getattr(actuator, name)[env_ids] = values
