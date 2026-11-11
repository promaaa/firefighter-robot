"""Biped velocity task on the ASMR 2026 course robot (asmr2026-humanoid-v1) with our upper-body payload.

The MDP is the Berkeley Humanoid Lite biped task's (config/biped): the same observations, actions (around the
default joint pose), rewards, terminations and randomization. The joint and body names are the same, so the terms
apply unchanged. What changes is the robot (geometry, masses, actuators and the standing pose as the default pose;
see berkeley_humanoid_lite_assets.robots.asmr2026_humanoid) and the payload.

The payload is our upper body (hose aiming servos, hose, nozzle, camera): 1 kg as a solid 10 cm box resting at the
center of the torso top plate. The course torso (5.57 kg) already includes the frame, electronics and batteries.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.env_cfg import (
    BerkeleyHumanoidLiteBipedEnvCfg,
    EventsCfg,
)
from source.robot_models.robots.asmr2026_humanoid import ASMR2026_HUMANOID_CFG

PAYLOAD_MASS_KG = 1.0
PAYLOAD_SIZE_M = (0.10, 0.10, 0.10)
# box center in the base frame: the top plate (140 x 250 mm) is centered on x = y = 0 at z = 0.8338 m
TORSO_TOP_Z_M = 0.8338
PAYLOAD_POSITION_M = (0.0, 0.0, TORSO_TOP_Z_M + PAYLOAD_SIZE_M[2] / 2)


@configclass
class AsmrBipedEventsCfg(EventsCfg):
    """Biped events plus the payload, the last startup term so it sits on the randomized torso."""

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
class AsmrBipedEnvCfg(BerkeleyHumanoidLiteBipedEnvCfg):

    events: AsmrBipedEventsCfg = AsmrBipedEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = ASMR2026_HUMANOID_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
