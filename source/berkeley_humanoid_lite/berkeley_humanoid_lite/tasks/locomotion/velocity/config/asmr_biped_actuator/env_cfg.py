"""Actuator gap study tasks on the course robot: how wide should the PD gain randomization be?

Every task is the course S2R task at alpha = 0 (config/asmr_biped_s2r, as Velocity-ASMR-Biped-S2R-DR000-v0: nominal
friction, masses and CoM, the payload in the nominal robot, no joint offset randomization) plus randomization of the
PD gains only: at startup each joint's stiffness and damping are scaled by independent factors drawn log-uniformly
from [1 / r, r], so a gain r times too high is as likely as one r times too low. The friction feed-forward of the
actuator model and the torque limits stay nominal.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_s2r.env_cfg import AsmrBipedS2REnvCfg

# task name: gain ratio r (factors in [1 / r, r])
GAIN_TASKS = {
    "Gain125": 1.25,
    "Gain150": 1.5,
    "Gain200": 2.0,
}


@configclass
class AsmrBipedActuatorGapEnvCfg(AsmrBipedS2REnvCfg):
    """Course S2R task at alpha = 0 plus PD gain randomization as set by GAIN_TASKS for the subclass name
    (Actuator<task>EnvCfg; the name is not an S2R task, so the dynamics stay nominal)."""

    def __post_init__(self):
        super().__post_init__()
        ratio = GAIN_TASKS[type(self).__name__.removeprefix("Actuator").removesuffix("EnvCfg")]
        self.events.scale_all_actuator_torque_constant = EventTerm(
            func=mdp.randomize_actuator_gains,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=[".*"]),
                "stiffness_distribution_params": (1.0 / ratio, ratio),
                "damping_distribution_params": (1.0 / ratio, ratio),
                "operation": "scale",
                "distribution": "log_uniform",
            },
            mode="startup",
        )


# one module-level class per task (train.py pickles the config, so classes cannot be made in a function)
@configclass
class ActuatorGain125EnvCfg(AsmrBipedActuatorGapEnvCfg):
    pass


@configclass
class ActuatorGain150EnvCfg(AsmrBipedActuatorGapEnvCfg):
    pass


@configclass
class ActuatorGain200EnvCfg(AsmrBipedActuatorGapEnvCfg):
    pass
