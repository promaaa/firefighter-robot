"""The dynamics gap study tasks of config/biped_s2r on the course robot with our upper-body payload.

Same randomization as the Berkeley Humanoid Lite tasks, in the same absolute units: foot friction 0.8 +- 0.4 alpha,
torso mass +- 1.5 alpha kg (never below -2.5 kg), leg link mass and inertia x (1 +- 0.2 alpha), torso CoM
+- 3 alpha cm along x and z. The payload is added last, on top of the randomized torso, as in config/asmr_biped.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped.env_cfg import (
    PAYLOAD_MASS_KG,
    PAYLOAD_POSITION_M,
    PAYLOAD_SIZE_M,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_s2r.env_cfg import (
    TASKS,
    BerkeleyHumanoidLiteBipedS2REnvCfg,
    DynamicsEventsCfg,
)
from source.robot_models.robots.asmr2026_humanoid import ASMR2026_HUMANOID_CFG

__all__ = ["TASKS"]


@configclass
class AsmrDynamicsEventsCfg(DynamicsEventsCfg):
    """Dynamics randomization events plus the payload, the last startup term."""

    add_payload = EventTerm(
        func=mdp.add_body_payload,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="base"),
            "mass": PAYLOAD_MASS_KG,
            "position": PAYLOAD_POSITION_M,
            "size": PAYLOAD_SIZE_M,
        },
    )


@configclass
class AsmrBipedS2REnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    """Course robot S2R task; the subclass name (S2R<task>EnvCfg) selects the randomization in TASKS."""

    events: AsmrDynamicsEventsCfg = AsmrDynamicsEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = ASMR2026_HUMANOID_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")


# one module-level class per task (train.py pickles the config, so classes cannot be made in a function)
@configclass
class S2RDR000EnvCfg(AsmrBipedS2REnvCfg):
    pass


@configclass
class S2RDR050EnvCfg(AsmrBipedS2REnvCfg):
    pass


@configclass
class S2RDR100EnvCfg(AsmrBipedS2REnvCfg):
    pass


@configclass
class S2RDR200EnvCfg(AsmrBipedS2REnvCfg):
    pass


@configclass
class S2RFriction100EnvCfg(AsmrBipedS2REnvCfg):
    pass


@configclass
class S2RInertia100EnvCfg(AsmrBipedS2REnvCfg):
    pass
