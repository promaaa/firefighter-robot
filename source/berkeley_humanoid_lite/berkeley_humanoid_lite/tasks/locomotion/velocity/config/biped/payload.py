"""Upper-body payload of our robot: hose aiming servos, hose, nozzle and camera on top of the torso.

The Berkeley Humanoid Lite torso (3.16 kg) is taken to include its computer and battery (the reported 16 kg of
the full robot matches the 16.33 kg of its URDF). The payload is 1 kg, modeled as a solid 10 cm box resting
at the center of the torso top: the torso mesh top is 0.155 m above the torso CoM, so the box center is
0.205 m above it.

Add ``PAYLOAD_EVENT`` to a task's events as the last startup term (see mdp.add_body_payload).
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp

PAYLOAD_MASS_KG = 1.0
PAYLOAD_SIZE_M = (0.10, 0.10, 0.10)
# box center in the base frame: torso CoM (-0.0046, 0.0004, 0.6379) + torso top 0.155 m + half the box height
PAYLOAD_POSITION_M = (-0.0046, 0.0004, 0.6379 + 0.155 + PAYLOAD_SIZE_M[2] / 2)

PAYLOAD_EVENT = EventTerm(
    func=mdp.add_body_payload,
    mode="startup",
    params={
        "asset_cfg": SceneEntityCfg("robot", body_names="base"),
        "mass": PAYLOAD_MASS_KG,
        "position": PAYLOAD_POSITION_M,
        "size": PAYLOAD_SIZE_M,
    },
)
