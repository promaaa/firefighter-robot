"""The sustained push tasks of config/biped_push on the course robot with our upper-body payload.

Each task is its Berkeley Humanoid Lite counterpart (same push schedule, curriculum, observations and rewards) with
the robot and the payload of config/asmr_biped. The push acts at a height above the torso CoM, so on the course
robot it acts at the same height above its own (payload-loaded) torso CoM.
"""

from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped.env_cfg import AsmrBipedEventsCfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_push.env_cfg import (
    BerkeleyHumanoidLiteBipedBrace2EnvCfg,
    BerkeleyHumanoidLiteBipedBraceEnvCfg,
    BerkeleyHumanoidLiteBipedPushEnvCfg,
    BerkeleyHumanoidLiteBipedPushSignalEnvCfg,
    BerkeleyHumanoidLiteBipedPushSignalHold2EnvCfg,
    BerkeleyHumanoidLiteBipedPushSignalHoldEnvCfg,
)
from source.robot_models.robots.asmr2026_humanoid import ASMR2026_HUMANOID_CFG


def use_course_robot(cfg):
    cfg.scene.robot = ASMR2026_HUMANOID_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")


@configclass
class AsmrBipedPushEnvCfg(BerkeleyHumanoidLiteBipedPushEnvCfg):
    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        use_course_robot(self)


@configclass
class AsmrBipedPushSignalEnvCfg(BerkeleyHumanoidLiteBipedPushSignalEnvCfg):
    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        use_course_robot(self)


@configclass
class AsmrBipedPushSignalHoldEnvCfg(BerkeleyHumanoidLiteBipedPushSignalHoldEnvCfg):
    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        use_course_robot(self)


@configclass
class AsmrBipedPushSignalHold2EnvCfg(BerkeleyHumanoidLiteBipedPushSignalHold2EnvCfg):
    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        use_course_robot(self)


@configclass
class AsmrBipedBraceEnvCfg(BerkeleyHumanoidLiteBipedBraceEnvCfg):
    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        use_course_robot(self)


@configclass
class AsmrBipedBrace2EnvCfg(BerkeleyHumanoidLiteBipedBrace2EnvCfg):
    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        use_course_robot(self)
