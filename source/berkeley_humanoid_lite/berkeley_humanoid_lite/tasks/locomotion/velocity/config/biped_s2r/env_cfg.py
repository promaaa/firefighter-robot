"""Biped tasks for the sim-to-real dynamics gap study: how wide should dynamics randomization be?

All tasks are the biped velocity task with the actuation taken as exact (no actuator gain or joint offset
randomization) and only these properties randomized, scaled by one width factor ``alpha``:

* foot-ground friction coefficient: NOMINAL_FRICTION +- 0.4 alpha (at least 0.1)
* torso mass: +- 1.5 alpha kg (an unknown upper-body payload), never below 3.16 - 2.5 kg (at alpha 2 a lighter
  torso makes the simulation blow up)
* leg link mass and inertia: x (1 +- 0.2 alpha) (3D-printed parts, fasteners, cables)
* torso CoM: +- 3 alpha cm along x and z

alpha = 0 trains on the nominal dynamics only; alpha = 1 is a plausible engineering uncertainty. Each task can
randomize the friction, the inertial properties, or both. Observation noise, reset perturbations and the small
random reset wrench are kept as in the biped task.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.env_cfg import (
    BerkeleyHumanoidLiteBipedEnvCfg,
    EventsCfg,
)

NOMINAL_FRICTION = 0.8
# half-widths of the randomization at alpha = 1
FRICTION_HALF_WIDTH = 0.4
MIN_FRICTION = 0.1
TORSO_MASS_HALF_WIDTH_KG = 1.5
MIN_TORSO_MASS_ADD_KG = -2.5  # the torso (3.16 kg) keeps at least 0.66 kg
LEG_MASS_HALF_WIDTH = 0.2
TORSO_COM_HALF_WIDTH_M = 0.03


@configclass
class DynamicsEventsCfg(EventsCfg):
    """Biped events plus leg mass scaling and torso CoM shifting."""

    scale_leg_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names="leg_.*"),
            "mass_distribution_params": (1.0, 1.0),
            "operation": "scale",
        },
        mode="startup",
    )
    shift_torso_com = EventTerm(
        func=mdp.randomize_body_com_offset,
        params={"asset_cfg": SceneEntityCfg("robot", body_names="base")},
        mode="startup",
    )


def dynamics_ranges(alpha: float, friction: bool = True, inertia: bool = True) -> dict:
    """Randomization ranges at width ``alpha`` (zero width for a group that is not randomized)."""
    a_f = alpha if friction else 0.0
    a_i = alpha if inertia else 0.0
    return {
        "friction": (max(MIN_FRICTION, NOMINAL_FRICTION - FRICTION_HALF_WIDTH * a_f),
                     NOMINAL_FRICTION + FRICTION_HALF_WIDTH * a_f),
        "torso_mass_add": (max(MIN_TORSO_MASS_ADD_KG, -TORSO_MASS_HALF_WIDTH_KG * a_i), TORSO_MASS_HALF_WIDTH_KG * a_i),
        "leg_mass_scale": (1.0 - LEG_MASS_HALF_WIDTH * a_i, 1.0 + LEG_MASS_HALF_WIDTH * a_i),
        "torso_com": (-TORSO_COM_HALF_WIDTH_M * a_i, TORSO_COM_HALF_WIDTH_M * a_i),
    }


# task name: (alpha, randomize friction, randomize inertial properties)
TASKS = {
    "DR000": (0.0, True, True),
    "DR050": (0.5, True, True),
    "DR100": (1.0, True, True),
    "DR200": (2.0, True, True),
    "Friction100": (1.0, True, False),
    "Inertia100": (1.0, False, True),
}


@configclass
class BerkeleyHumanoidLiteBipedS2REnvCfg(BerkeleyHumanoidLiteBipedEnvCfg):
    """Biped task randomizing the dynamics as set by TASKS for the subclass name (S2R<task>EnvCfg)."""

    events: DynamicsEventsCfg = DynamicsEventsCfg()

    def __post_init__(self):
        super().__post_init__()
        name = type(self).__name__.removeprefix("S2R").removesuffix("EnvCfg")
        ranges = dynamics_ranges(*TASKS[name]) if name in TASKS else dynamics_ranges(0.0)
        events = self.events
        # actuation is taken as exact
        events.add_all_joint_default_pos = None
        events.scale_all_actuator_torque_constant = None
        # friction (the terrain's 1.0 multiplies it)
        events.physics_material.params["static_friction_range"] = ranges["friction"]
        events.physics_material.params["dynamic_friction_range"] = ranges["friction"]
        # inertial properties
        events.add_base_mass.params["mass_distribution_params"] = ranges["torso_mass_add"]
        events.scale_leg_mass.params["mass_distribution_params"] = ranges["leg_mass_scale"]
        events.shift_torso_com.params["x_range"] = ranges["torso_com"]
        events.shift_torso_com.params["z_range"] = ranges["torso_com"]


# one module-level class per task (train.py pickles the config, so classes cannot be made in a function)
@configclass
class S2RDR000EnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    pass


@configclass
class S2RDR050EnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    pass


@configclass
class S2RDR100EnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    pass


@configclass
class S2RDR200EnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    pass


@configclass
class S2RFriction100EnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    pass


@configclass
class S2RInertia100EnvCfg(BerkeleyHumanoidLiteBipedS2REnvCfg):
    pass
