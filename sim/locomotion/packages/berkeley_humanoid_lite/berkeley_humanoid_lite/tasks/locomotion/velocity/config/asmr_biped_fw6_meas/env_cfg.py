"""FW6 tasks with the measured actuator friction model (team bench test of the hip yaw 6512 actuators, 2026-11-06).

The robot is ASMR2026_HUMANOID_FW6M_CFG: the course driver gains, a 6 N m torque limit and the measured friction
(Coulomb, low-speed increase, load-dependent increase, no viscous). On top of Velocity-ASMR-Biped-FW6-v0's mass,
ground friction, joint offset randomization and payload, every task draws per joint at startup:
- PD gains: Kp and Kd scaled by independent factors uniform in [0.9, 1.1] (the bench gains were exact to 0.2 %);
- friction: Coulomb x0.83-1.25 (0.30-0.45 N m) on the 6512 and x0.75-1.5 on the unmeasured 5010, viscous
  0-0.02 / 0-0.03 N m s / rad, load term factor 0.5-1.5 / 0.5-2, Stribeck ratio 1-2;
- torque limit (MeasT68*): 6-8 N m, for a driver current limit above the 6 N m of training;
- dynamics (MeasT68DR100): the course S2R randomization at alpha = 1 (config/asmr_biped_s2r): foot friction
  0.8 +- 0.4, torso mass +- 1.5 kg, leg mass x0.8-1.2, torso CoM +- 3 cm along x and z (the FW6 policies fell with a
  torso CoM 6 cm off, since a 6 N m ankle leaves little torque to hold it).
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_fw6.env_cfg import AsmrBipedFw6EnvCfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_s2r.env_cfg import AsmrDynamicsEventsCfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_s2r.env_cfg import dynamics_ranges
from source.robot_models.robots.asmr2026_humanoid import ASMR2026_HUMANOID_FW6M_CFG

# task name: (torque limit factor range on the 6 N m limit or None for fixed, dynamics alpha or None)
MEASURED_TASKS = {
    "MeasT6": (None, None),
    "MeasT68": ((1.0, 8.0 / 6.0), None),
    "MeasT68DR100": ((1.0, 8.0 / 6.0), 1.0),
}
JOINTS_6512 = [".*_hip_roll_joint", ".*_hip_yaw_joint", ".*_hip_pitch_joint", ".*_knee_pitch_joint"]
JOINTS_5010 = [".*_ankle_pitch_joint", ".*_ankle_roll_joint"]


def use_measured_actuators(cfg, effort: tuple[float, float] | None, alpha: float | None):
    """Put the FW6M robot (measured friction model) into a course task config and add the actuator randomization
    of the measured tasks: gains, friction model, the torque limit range (None: fixed) and the course S2R dynamics
    randomization at width alpha (None: the task's own; needs the leg mass and torso CoM events)."""
    cfg.scene.robot = ASMR2026_HUMANOID_FW6M_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
    events = cfg.events
    joints = SceneEntityCfg("robot", joint_names=[".*"])
    events.scale_all_actuator_torque_constant = EventTerm(
        func=mdp.randomize_actuator_gains,
        params={
            "asset_cfg": joints,
            "stiffness_distribution_params": (0.9, 1.1),
            "damping_distribution_params": (0.9, 1.1),
            "operation": "scale",
            "distribution": "uniform",
        },
        mode="startup",
    )
    events.randomize_friction_6512 = EventTerm(
        func=mdp.randomize_actuator_friction_model,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=JOINTS_6512),
            "coulomb_scale_range": (0.30 / 0.36, 0.45 / 0.36),
            "viscous_range": (0.0, 0.02),
            "load_scale_range": (0.5, 1.5),
            "stribeck_ratio_range": (1.0, 2.0),
        },
        mode="startup",
    )
    events.randomize_friction_5010 = EventTerm(
        func=mdp.randomize_actuator_friction_model,
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=JOINTS_5010),
            "coulomb_scale_range": (0.75, 1.5),
            "viscous_range": (0.0, 0.03),
            "load_scale_range": (0.5, 2.0),
            "stribeck_ratio_range": (1.0, 2.0),
        },
        mode="startup",
    )
    if effort is not None:
        events.scale_actuator_effort_limit = EventTerm(
            func=mdp.randomize_actuator_effort_limit,
            params={"asset_cfg": joints, "effort_distribution_params": effort, "distribution": "uniform"},
            mode="startup",
        )
    if alpha is not None:
        ranges = dynamics_ranges(alpha)
        events.physics_material.params["static_friction_range"] = ranges["friction"]
        events.physics_material.params["dynamic_friction_range"] = ranges["friction"]
        events.add_base_mass.params["mass_distribution_params"] = ranges["torso_mass_add"]
        events.scale_leg_mass.params["mass_distribution_params"] = ranges["leg_mass_scale"]
        events.shift_torso_com.params["x_range"] = ranges["torso_com"]
        events.shift_torso_com.params["z_range"] = ranges["torso_com"]


@configclass
class AsmrBipedFw6MeasEnvCfg(AsmrBipedFw6EnvCfg):
    """FW6 task with the measured friction model and the randomization MEASURED_TASKS sets for the subclass name."""

    def __post_init__(self):
        super().__post_init__()
        use_measured_actuators(self, *MEASURED_TASKS[type(self).__name__.removeprefix("Fw6").removesuffix("EnvCfg")])


# one module-level class per task (train.py pickles the config, so classes cannot be made in a function)
@configclass
class Fw6MeasT6EnvCfg(AsmrBipedFw6MeasEnvCfg):
    pass


@configclass
class Fw6MeasT68EnvCfg(AsmrBipedFw6MeasEnvCfg):
    pass


@configclass
class Fw6MeasT68DR100EnvCfg(AsmrBipedFw6MeasEnvCfg):
    # the dynamics randomization events (leg mass, torso CoM) with the payload as the last startup term
    events: AsmrDynamicsEventsCfg = AsmrDynamicsEventsCfg()
