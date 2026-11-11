"""Sustained push on the torso, e.g. the reaction force of a water jet sprayed from a nozzle on the robot.

The push is a command term: it is an input planned outside the policy (the spraying schedule), so the
term keeps the schedule, applies the force to the robot every step, and exposes a push signal (the planned
force, scaled by PUSH_SIGNAL_SCALE) that a policy may observe through ``generated_commands``.
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from dataclasses import MISSING
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation
from isaaclab.managers import CommandTerm, CommandTermCfg, ManagerTermBase, RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# push signal = planned push force / PUSH_SIGNAL_SCALE
PUSH_SIGNAL_SCALE = 30.0


class SustainedPushCommand(CommandTerm):
    """Alternating off and on phases; during an on phase a constant horizontal force pushes the torso.

    The force direction is fixed in the robot's heading frame and the force acts at a point above the torso
    CoM, fixed to the torso (like a nozzle). The push signal turns on ``lead`` seconds before the force and
    carries the planned force with an estimation error, as a controller that knows the spraying schedule
    would see it.
    """

    cfg: SustainedPushCommandCfg

    def __init__(self, cfg: SustainedPushCommandCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self.robot: Articulation = env.scene[cfg.asset_name]
        self.body_id = self.robot.find_bodies(cfg.body_name)[0][0]

        n, device = self.num_envs, self.device
        self.force_magnitude = torch.zeros(n, device=device)
        self.signal_magnitude = torch.zeros(n, device=device)
        self.direction = torch.zeros(n, device=device)
        self.offset_z = torch.zeros(n, device=device)
        self.lead = torch.zeros(n, device=device)
        self.pushing = torch.zeros(n, dtype=torch.bool, device=device)
        self.phase_time_left = torch.zeros(n, device=device)
        self.ramp = torch.zeros(n, device=device)
        self.bias_force_b = torch.zeros(n, 3, device=device)
        self.bias_torque_b = torch.zeros(n, 3, device=device)
        self.signal = torch.zeros(n, 1, device=device)
        # upper end of the force range; a curriculum may raise it
        self.max_force = cfg.force_range[1]

        self.metrics["push_force"] = torch.zeros(n, device=device)

    def __str__(self) -> str:
        return (f"SustainedPushCommand: force {self.cfg.force_range} N, direction {self.cfg.direction_range} deg, "
                f"on {self.cfg.on_duration_range} s, off {self.cfg.off_duration_range} s")

    @property
    def command(self) -> torch.Tensor:
        """The push signal, shape (num_envs, 1)."""
        return self.signal

    def _update_metrics(self):
        self.metrics["push_force"] = self.force_magnitude * self.ramp

    def _resample_command(self, env_ids: Sequence[int]):
        # called on reset: start in an off phase with the next push planned
        self.pushing[env_ids] = False
        self.ramp[env_ids] = 0.0
        self.phase_time_left[env_ids] = self._uniform(len(env_ids), self.cfg.first_off_duration_range)
        self._plan_push(env_ids)
        # small constant disturbance, as the reset wrench of the original task
        n = len(env_ids)
        self.bias_force_b[env_ids] = self._uniform((n, 3), self.cfg.bias_force_range)
        self.bias_torque_b[env_ids] = self._uniform((n, 3), self.cfg.bias_torque_range)

    def _update_command(self):
        dt = self._env.step_dt
        self.phase_time_left -= dt
        # on -> off: plan the next push
        end_on = self.pushing & (self.phase_time_left <= 0.0)
        if end_on.any():
            ids = end_on.nonzero().flatten()
            self.pushing[ids] = False
            self.phase_time_left[ids] = self._uniform(len(ids), self.cfg.off_duration_range)
            self._plan_push(ids)
        # off -> on
        start_on = ~self.pushing & (self.phase_time_left <= 0.0)
        if start_on.any():
            ids = start_on.nonzero().flatten()
            self.pushing[ids] = True
            self.phase_time_left[ids] = self._uniform(len(ids), self.cfg.on_duration_range)

        # the signal leads the force
        signal_on = self.pushing | (self.phase_time_left <= self.lead)
        self.signal[:, 0] = self.signal_magnitude * signal_on / PUSH_SIGNAL_SCALE

        # ramp the force in and out (valve opening)
        step = dt / max(self.cfg.ramp_time, dt)
        self.ramp = torch.clamp(self.ramp + torch.where(self.pushing, step, -step), 0.0, 1.0)

        # horizontal force along the heading, applied above the torso CoM
        yaw = math_utils.yaw_quat(self.robot.data.root_quat_w)
        direction_h = torch.stack(
            [torch.cos(self.direction), torch.sin(self.direction), torch.zeros_like(self.direction)], dim=-1
        )
        force_w = math_utils.quat_apply(yaw, direction_h) * (self.force_magnitude * self.ramp).unsqueeze(-1)
        force_b = math_utils.quat_rotate_inverse(self.robot.data.root_quat_w, force_w)
        offset_b = torch.zeros_like(force_b)
        offset_b[:, 2] = self.offset_z
        torque_b = torch.cross(offset_b, force_b, dim=-1)
        self.robot.set_external_force_and_torque(
            (force_b + self.bias_force_b).unsqueeze(1),
            (torque_b + self.bias_torque_b).unsqueeze(1),
            body_ids=[self.body_id],
        )

    def _plan_push(self, env_ids: Sequence[int] | torch.Tensor):
        n = len(env_ids)
        magnitude = self._uniform(n, (self.cfg.force_range[0], self.max_force))
        magnitude *= (torch.rand(n, device=self.device) < self.cfg.push_probability).float()
        if self.cfg.walking_force_scale != 1.0:
            # weaker pushes while walking: spraying happens mostly while standing
            standing = self._env.command_manager.get_term(self.cfg.velocity_command_name).is_standing_env[env_ids]
            magnitude *= torch.where(standing, 1.0, self.cfg.walking_force_scale)
        self.force_magnitude[env_ids] = magnitude
        self.signal_magnitude[env_ids] = magnitude * self._uniform(n, self.cfg.signal_scale_range)
        self.direction[env_ids] = torch.deg2rad(self._uniform(n, self.cfg.direction_range))
        self.offset_z[env_ids] = self._uniform(n, self.cfg.offset_z_range)
        self.lead[env_ids] = self._uniform(n, self.cfg.signal_lead_range)

    def _uniform(self, shape, value_range: tuple[float, float]) -> torch.Tensor:
        low, high = value_range
        return torch.rand(shape, device=self.device) * (high - low) + low


@configclass
class SustainedPushCommandCfg(CommandTermCfg):
    """Configuration of :class:`SustainedPushCommand`."""

    class_type: type = SustainedPushCommand

    # the schedule is kept by the term; this only has to outlast an episode
    resampling_time_range: tuple[float, float] = (1.0e6, 1.0e6)

    asset_name: str = MISSING
    """Name of the robot in the scene."""
    body_name: str = "base"
    """Body that is pushed."""

    force_range: tuple[float, float] = (0.0, 30.0)
    """Push force (N). The upper end may be raised by :func:`push_force_curriculum`."""
    push_probability: float = 0.8
    """Probability that an on phase pushes at all."""
    direction_range: tuple[float, float] = (150.0, 210.0)
    """Push direction in the heading frame (deg); 180 pushes the robot backward."""
    offset_z_range: tuple[float, float] = (0.1, 0.3)
    """Height of the push point above the torso CoM (m), fixed in the torso frame."""
    on_duration_range: tuple[float, float] = (2.0, 8.0)
    off_duration_range: tuple[float, float] = (1.0, 5.0)
    first_off_duration_range: tuple[float, float] = (0.5, 4.0)
    ramp_time: float = 0.1
    """Time for the force to build up or fall off (s)."""
    signal_lead_range: tuple[float, float] = (0.0, 0.5)
    """How long the signal turns on before the force (s)."""
    signal_scale_range: tuple[float, float] = (0.8, 1.2)
    """Ratio of the signalled to the actual force: the planned force is only an estimate."""
    bias_force_range: tuple[float, float] = (-2.0, 2.0)
    bias_torque_range: tuple[float, float] = (-2.0, 2.0)
    walking_force_scale: float = 1.0
    """Scale of the push force for environments that are not commanded to stand still."""
    velocity_command_name: str = "base_velocity"
    """Velocity command term that tells which environments stand still."""


def push_force_curriculum(
    env: ManagerBasedRLEnv, env_ids: Sequence[int], command_name: str, start: float, end: float, num_steps: int
) -> float:
    """Raise the upper end of the push force linearly from ``start`` to ``end`` over ``num_steps`` env steps."""
    term: SustainedPushCommand = env.command_manager.get_term(command_name)
    progress = min(env.common_step_counter / max(num_steps, 1), 1.0)
    term.max_force = start + (end - start) * progress
    return term.max_force


def push_force_adaptive_curriculum(
    env: ManagerBasedRLEnv,
    env_ids: Sequence[int],
    command_name: str,
    start: float,
    end: float,
    fall_rate_low: float = 0.10,
    fall_rate_high: float = 0.25,
    step_per_reset: float = 0.0005,
    smoothing: float = 0.995,
) -> dict[str, float]:
    """Keep the push force at the edge of what the policy can take.

    Tracks the share of episodes that end in a fall (moving average over resets) and raises the upper end of
    the push force while falls are rarer than ``fall_rate_low``, lowers it while they are more common than
    ``fall_rate_high``, by ``step_per_reset`` newton per reset environment, within [start, end].
    """
    term: SustainedPushCommand = env.command_manager.get_term(command_name)
    if not hasattr(term, "fall_rate"):
        term.fall_rate = 0.0
        term.max_force = start
    # the first reset (at start-up) ends no episode
    if env.common_step_counter > 0 and len(env_ids) > 0:
        fell = env.termination_manager.terminated[env_ids].float()
        weight = smoothing ** len(env_ids)
        term.fall_rate = weight * term.fall_rate + (1.0 - weight) * fell.mean().item()
        if term.fall_rate < fall_rate_low:
            term.max_force = min(end, term.max_force + step_per_reset * len(env_ids))
        elif term.fall_rate > fall_rate_high:
            term.max_force = max(start, term.max_force - step_per_reset * len(env_ids))
    return {"max_force": term.max_force, "fall_rate": term.fall_rate}


class standing_position_hold(ManagerTermBase):
    """Reward staying where the robot was when it was told to stand still: exp(-distance^2 / std^2).

    The velocity tracking reward barely notices a slow creep under a sustained push; this term anchors the
    position when a zero velocity command starts and rewards staying near it while the command stays zero.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self.anchor = torch.zeros(env.num_envs, 2, device=env.device)
        self.was_standing = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None):
        self.was_standing[slice(None) if env_ids is None else env_ids] = False

    def __call__(
        self,
        env: ManagerBasedRLEnv,
        command_name: str,
        std: float,
        asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ) -> torch.Tensor:
        position = env.scene[asset_cfg.name].data.root_pos_w[:, :2]
        standing = torch.linalg.norm(env.command_manager.get_command(command_name)[:, :3], dim=1) < 1e-3
        start = standing & ~self.was_standing
        self.anchor[start] = position[start]
        self.was_standing = standing
        distance_sq = torch.sum(torch.square(position - self.anchor), dim=1)
        return torch.exp(-distance_sq / std**2) * standing


class standing_feet_displacement(ManagerTermBase):
    """Penalty on how far the feet have moved since the robot was told to stand still (sum of squared metres).

    Under a sustained push a policy can creep back by shuffling its feet; this asks it to keep its footprint
    and lean into the push instead. Caution: the penalty keeps accruing after a step, so a policy can learn
    that falling (which ends the episode) is cheaper than stepping; see :func:`standing_feet_speed`.
    """

    def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        # the reward manager resolves asset_cfg (the foot body ids); the anchor is sized on the first call
        self.anchor = None
        self.was_standing = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)

    def reset(self, env_ids: Sequence[int] | None = None):
        self.was_standing[slice(None) if env_ids is None else env_ids] = False

    def __call__(self, env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
        feet = env.scene[asset_cfg.name].data.body_pos_w[:, asset_cfg.body_ids, :2]
        if self.anchor is None:
            self.anchor = feet.clone()
        standing = torch.linalg.norm(env.command_manager.get_command(command_name)[:, :3], dim=1) < 1e-3
        start = standing & ~self.was_standing
        self.anchor[start] = feet[start]
        self.was_standing = standing
        return torch.sum(torch.square(feet - self.anchor), dim=(1, 2)) * standing


def standing_feet_speed(env: ManagerBasedRLEnv, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalty on the horizontal speed of the feet while standing (sum of squares, m^2/s^2).

    Unlike :class:`standing_feet_displacement` this charges only while a foot moves, so a step taken to
    recover costs once instead of for the rest of the episode (which made falling the cheaper option).
    """
    feet_velocity = env.scene[asset_cfg.name].data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    standing = torch.linalg.norm(env.command_manager.get_command(command_name)[:, :3], dim=1) < 1e-3
    return torch.sum(torch.square(feet_velocity), dim=(1, 2)) * standing


def push_signal(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """The push signal of a :class:`SustainedPushCommand` (planned force / PUSH_SIGNAL_SCALE)."""
    return env.command_manager.get_command(command_name)


def push_force_applied(env: ManagerBasedRLEnv, command_name: str) -> torch.Tensor:
    """The force a :class:`SustainedPushCommand` applies now, scaled like the signal (privileged)."""
    term: SustainedPushCommand = env.command_manager.get_term(command_name)
    return (term.force_magnitude * term.ramp / PUSH_SIGNAL_SCALE).unsqueeze(-1)


__all__ = [
    "PUSH_SIGNAL_SCALE",
    "SustainedPushCommand",
    "SustainedPushCommandCfg",
    "push_force_adaptive_curriculum",
    "push_force_applied",
    "push_force_curriculum",
    "push_signal",
    "standing_feet_displacement",
    "standing_feet_speed",
    "standing_position_hold",
]
