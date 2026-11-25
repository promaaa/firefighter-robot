"""Real walking test task with the Radxa torque clamp (user request 2026-11-24, after the first real test).

The drivers keep their 18 A current limit (about 18.1 N m at the hip and knee joints, 22.6 N m at the ankles): it
cannot be changed over CAN with the course firmware and the robot is assembled (no USB). The Radxa clips the joint
targets once per policy step so that the PD torque at that instant is within the limit L (mdp.torque_clamp), and the
same clamp is here, with a light penalty on the torque the targets ask past L. Three tasks: L = 6, 9 and 12 N m.
Trained by fine-tuning policies/real_walk (model_5999): from scratch at 6 N m the policy never learned to step.

Also changed from Velocity-ASMR-Biped-Real-v0: the torso CoM randomization along x is widened forward to -2..+8 cm
(was +-3 cm; -2..+6 cm at 6 N m). The real robot leaned forward 12-15 deg while walking and pressed its toes down with about 3.4 N m per
ankle (MuJoCo: 1.0; a 6 cm forward CoM gives 2.6-3.1), so the torso CoM may be ahead of the model.
"""

from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_real.env_cfg import AsmrBipedRealEnvCfg

DRIVE_LIMIT_6512 = 18.09  # 18 A x 0.067 N m/A x 15 (course driver current_lim)
DRIVE_LIMIT_5010 = 22.57  # 18 A x 0.0836 N m/A x 15
TORSO_COM_X_RANGE = (-0.02, 0.08)
# per N m of torque asked past the limit, summed over the joints. -0.005 (first try, 11-24 evening) made fine-tuning at
# 6 N m worse within 1700 iterations (episodes 580 -> 370 steps): the action noise (about 0.09 rad on the targets,
# ~9 N m at Kp 100) alone asks past the limit whenever a joint works near it, so the penalty pushed the policy away
# from using the torque it has.
CLAMP_PENALTY_WEIGHT = -0.001


@configclass
class AsmrBipedRealClampEnvCfg(AsmrBipedRealEnvCfg):

    torque_clamp: float = 6.0
    torso_com_x: tuple[float, float] = TORSO_COM_X_RANGE

    def __post_init__(self):
        super().__post_init__()
        robot = self.scene.robot
        robot.actuators = {
            "legs": robot.actuators["legs"].replace(effort_limit=DRIVE_LIMIT_6512),
            "ankles": robot.actuators["ankles"].replace(effort_limit=DRIVE_LIMIT_5010),
        }
        joint_pos = self.actions.joint_pos
        self.actions.joint_pos = mdp.ClampedJointPositionActionCfg(
            asset_name=joint_pos.asset_name,
            joint_names=joint_pos.joint_names,
            scale=joint_pos.scale,
            use_default_offset=joint_pos.use_default_offset,
            preserve_order=joint_pos.preserve_order,
            torque_limit=self.torque_clamp,
        )
        self.rewards.torque_clamp_excess = RewTerm(
            func=mdp.torque_clamp_excess, weight=CLAMP_PENALTY_WEIGHT, params={"action_name": "joint_pos"})
        self.events.shift_torso_com.params["x_range"] = self.torso_com_x


# one module-level class per task (train.py pickles the config, so classes cannot be made in a function)
@configclass
class AsmrBipedRealClamp6EnvCfg(AsmrBipedRealClampEnvCfg):
    torque_clamp: float = 6.0
    # +8 cm asks about 10 N m of a single stance ankle; MuJoCo walked the 6 N m clamp at +6 cm
    torso_com_x: tuple[float, float] = (-0.02, 0.06)


@configclass
class AsmrBipedRealClamp9EnvCfg(AsmrBipedRealClampEnvCfg):
    torque_clamp: float = 9.0


@configclass
class AsmrBipedRealClamp12EnvCfg(AsmrBipedRealClampEnvCfg):
    torque_clamp: float = 12.0
