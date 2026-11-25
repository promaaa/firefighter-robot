"""Biped velocity task on the course robot with the course motor driver settings and a 6 N m torque limit.

The task is Velocity-ASMR-Biped-v0 (config/asmr_biped: the same MDP, randomization and payload) with the actuators
of ASMR2026_HUMANOID_FW6_CFG: the PD gains of the course XDrive MINI configuration (6512 Kp 100.6 / Kd 7.99, 5010
Kp 71.6 / Kd 3.58) and a 6 N m torque limit on every joint.
"""

from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped.env_cfg import AsmrBipedEnvCfg
from source.robot_models.robots.asmr2026_humanoid import ASMR2026_HUMANOID_FW6_CFG


@configclass
class AsmrBipedFw6EnvCfg(AsmrBipedEnvCfg):

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = ASMR2026_HUMANOID_FW6_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
