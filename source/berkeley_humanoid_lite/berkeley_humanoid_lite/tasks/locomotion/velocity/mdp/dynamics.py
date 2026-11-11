"""Changes of the robot's inertial properties beyond what Isaac Lab ships: CoM randomization and rigid payloads."""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv


class randomize_body_com_offset(ManagerTermBase):
    """Shift the center of mass of bodies by a uniform random offset (m) in the body frame.

    The offsets are drawn around the CoM the asset was loaded with, so repeated calls do not drift.
    """

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        asset: Articulation = env.scene[cfg.params["asset_cfg"].name]
        # (num_envs, num_bodies, 7): position and orientation of each body's CoM frame, on the CPU
        self.default_coms = asset.root_physx_view.get_coms().clone()

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor | None,
        asset_cfg: SceneEntityCfg,
        x_range: tuple[float, float] = (0.0, 0.0),
        y_range: tuple[float, float] = (0.0, 0.0),
        z_range: tuple[float, float] = (0.0, 0.0),
    ):
        asset: Articulation = env.scene[asset_cfg.name]
        env_ids = torch.arange(env.scene.num_envs) if env_ids is None else env_ids.cpu()
        body_ids = torch.arange(asset.num_bodies) if asset_cfg.body_ids == slice(None) else torch.tensor(
            asset_cfg.body_ids
        )
        low = torch.tensor([x_range[0], y_range[0], z_range[0]])
        high = torch.tensor([x_range[1], y_range[1], z_range[1]])
        offsets = torch.rand(len(env_ids), len(body_ids), 3) * (high - low) + low
        coms = asset.root_physx_view.get_coms().clone()
        coms[env_ids[:, None], body_ids, :3] = self.default_coms[env_ids[:, None], body_ids, :3] + offsets
        asset.root_physx_view.set_coms(coms, env_ids)


def box_inertia(mass: torch.Tensor, size: tuple[float, float, float]) -> torch.Tensor:
    """Inertia (..., 3, 3) of solid boxes of the given masses (...) and edge lengths, about their centers."""
    x, y, z = size
    diagonal = torch.tensor([y * y + z * z, x * x + z * z, x * x + y * y], dtype=mass.dtype) / 12.0
    return torch.diag_embed(mass[..., None] * diagonal)


def combine_with_payload(
    mass: torch.Tensor,
    com: torch.Tensor,
    inertia: torch.Tensor,
    payload_mass: torch.Tensor,
    payload_position: torch.Tensor,
    payload_inertia: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Mass properties of bodies with a rigidly attached payload.

    Args:
        mass: body masses (N,).
        com: body CoM positions in the body frame (N, 3).
        inertia: body inertias about their CoMs in body-frame axes (N, 3, 3), as PhysX reports them.
        payload_mass: payload masses (N,).
        payload_position: payload CoM positions in the body frame (N, 3).
        payload_inertia: payload inertias about their own CoMs in body-frame axes (N, 3, 3).

    Returns:
        Combined mass (N,), CoM (N, 3) and inertia about the combined CoM (N, 3, 3).
    """
    total = mass + payload_mass
    new_com = (mass[:, None] * com + payload_mass[:, None] * payload_position) / total[:, None]

    def parallel_axis(m: torch.Tensor, d: torch.Tensor) -> torch.Tensor:
        eye = torch.eye(3, dtype=d.dtype).expand(len(d), 3, 3)
        return m[:, None, None] * ((d * d).sum(-1)[:, None, None] * eye - d[:, :, None] * d[:, None, :])

    new_inertia = (inertia + parallel_axis(mass, com - new_com)
                   + payload_inertia + parallel_axis(payload_mass, payload_position - new_com))
    return total, new_com, new_inertia


def add_body_payload(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor | None,
    asset_cfg: SceneEntityCfg,
    mass: float,
    position: tuple[float, float, float],
    size: tuple[float, float, float] = (0.1, 0.1, 0.1),
):
    """Rigidly attach a solid box payload to one body (mass in kg, box center in the body frame, edge lengths in m).

    The body's mass, CoM and inertia become those of body + payload, starting from the current values. Use it as
    a startup event placed after the other mass and CoM randomization terms, so the payload sits on the
    randomized body and is not overwritten by terms that start from the loaded values.
    """
    asset: Articulation = env.scene[asset_cfg.name]
    view = asset.root_physx_view
    env_ids = torch.arange(env.scene.num_envs) if env_ids is None else env_ids.cpu()
    body_id = asset_cfg.body_ids[0] if isinstance(asset_cfg.body_ids, list) else 0
    n = len(env_ids)
    masses, coms, inertias = view.get_masses(), view.get_coms(), view.get_inertias()
    payload_mass = torch.full((n,), float(mass))
    new_mass, new_com, new_inertia = combine_with_payload(
        masses[env_ids, body_id],
        coms[env_ids, body_id, :3],
        inertias[env_ids, body_id].reshape(n, 3, 3),
        payload_mass,
        torch.tensor(position, dtype=torch.float32).expand(n, 3),
        box_inertia(payload_mass, size),
    )
    masses[env_ids, body_id] = new_mass
    coms[env_ids, body_id, :3] = new_com
    inertias[env_ids, body_id] = new_inertia.reshape(n, 9)
    view.set_masses(masses, env_ids)
    view.set_coms(coms, env_ids)
    view.set_inertias(inertias, env_ids)


__all__ = ["randomize_body_com_offset", "box_inertia", "combine_with_payload", "add_body_payload"]
