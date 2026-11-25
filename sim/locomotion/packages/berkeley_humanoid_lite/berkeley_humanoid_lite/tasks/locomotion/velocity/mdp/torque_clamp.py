"""The Radxa torque clamp of the real walking test as an action term, and its penalty.

The course drivers cannot be given a torque or current limit over CAN (ODrive 0.5.1 course firmware: Set Limits
changes only the velocity limit), so the Radxa clips the policy's joint targets instead (SOBANG_MotorTest
MotorTestGUI/radxa/walk_policy.py, BridgeIO.clamp, 2026-11-24). Once per policy step, with the encoder readings the
policy observed (q, qd) and the nominal drive gains,

    q_sent = clip(q_target, q + (-L + Kd qd) / Kp, q + (L + Kd qd) / Kp),

so the drive's PD torque Kp (q_sent - q) - Kd qd is within +- L at that instant. The drive then runs its PD on q_sent
until the next target, limited only by its current limit, so the torque can pass L within the 20 ms. The policy
observes its own (unclipped) action as the last action, as on the robot.
"""

from __future__ import annotations

import torch
from collections.abc import Sequence
from typing import TYPE_CHECKING

from isaaclab.envs.mdp.actions.actions_cfg import JointPositionActionCfg
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


class ClampedJointPositionAction(JointPositionAction):
    """Joint position targets clipped once per policy step as the Radxa does (see the module docstring)."""

    cfg: ClampedJointPositionActionCfg

    def __init__(self, cfg: ClampedJointPositionActionCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        # the action manager is built before the startup events, so the actuators hold the nominal gains (the Radxa
        # clamp uses the nominal gains; the gain randomization then acts on the drives only)
        kp = torch.zeros(self.num_envs, self._asset.num_joints, device=self.device)
        kd = torch.zeros_like(kp)
        for actuator in self._asset.actuators.values():
            kp[:, actuator.joint_indices] = actuator.stiffness
            kd[:, actuator.joint_indices] = actuator.damping
        self._kp = kp[:, self._joint_ids].clone()
        self._kd = kd[:, self._joint_ids].clone()
        if not torch.all(self._kp > 0.0):
            raise ValueError("ClampedJointPositionAction needs explicit actuators with stiffness on every joint")
        self._sent = torch.zeros_like(self._raw_actions)
        self._excess = torch.zeros_like(self._raw_actions)
        self._clamped = torch.zeros_like(self._raw_actions, dtype=torch.bool)
        self._needs_clamp = True

    @property
    def excess_torque(self) -> torch.Tensor:
        """|Kp (q_target - q_sent)| per joint (N m): how far the policy's target asked past the limit."""
        return self._excess

    @property
    def clamped(self) -> torch.Tensor:
        return self._clamped

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)
        self._needs_clamp = True

    def apply_actions(self):
        if self._needs_clamp:
            # the readings the policy observed for this step (the first physics substep follows the observation)
            pos, vel = self._asset.delayed_joint_readings()
            q, qd = pos[:, self._joint_ids], vel[:, self._joint_ids]
            limit = self.cfg.torque_limit
            damping = self._kd * qd
            low = q + (damping - limit) / self._kp
            high = q + (damping + limit) / self._kp
            self._sent = torch.minimum(torch.maximum(self.processed_actions, low), high)
            self._excess = (self.processed_actions - self._sent).abs() * self._kp
            self._clamped = self._excess > 1e-6
            self._needs_clamp = False
        self._asset.set_joint_position_target(self._sent, joint_ids=self._joint_ids)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        self._needs_clamp = True


@configclass
class ClampedJointPositionActionCfg(JointPositionActionCfg):
    """JointPositionActionCfg with the Radxa target clamp at torque_limit (N m)."""

    class_type: type = ClampedJointPositionAction
    torque_limit: float = 6.0


def torque_clamp_excess(env: ManagerBasedRLEnv, action_name: str = "joint_pos") -> torch.Tensor:
    """Sum over the joints of the torque the policy's targets asked past the clamp limit (N m), per environment."""
    return env.action_manager.get_term(action_name).excess_torque.sum(dim=1)


def torque_clamp_fraction(env: ManagerBasedRLEnv, action_name: str = "joint_pos") -> torch.Tensor:
    """Fraction of the joints whose target was clipped this step (for logging)."""
    return env.action_manager.get_term(action_name).clamped.float().mean(dim=1)
