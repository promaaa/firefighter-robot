"""Biped velocity task for the first real walking test of the course robot (2026-11-20).

Starts from Velocity-ASMR-Biped-FW6-MeasT68DR100-v0 (course driver gains, measured friction model, gains +- 10 %,
friction model randomization, the course S2R dynamics randomization at alpha = 1, payload) and changes, after the
joint measurements on the real robot (see source.robot_models.robots.asmr2026_real) and the user's choices:
- torque limit 6 N m on every joint, fixed, the same as on the drivers (no 6-8 N m randomization);
- the BHL-like standing pose (knee 0.4 rad), which is also the rigid-stand pose before the policy starts;
- explicit PD with gear backlash (0-1 deg), encoder zero offset (+- 2 deg, applied to the observed positions and the
  targets together) and action delay (2-6 ms), at a 2 ms physics step;
- the policy observes the encoders (motor side) and the IMU with delays (0-6 ms; the IMU is a BMI088), with the
  joint position noise lowered to +- 0.01 rad (user choice 11-23);
- the link masses weighed on the real robot (asmr2026_real.MEASURED_BODY_MASSES), no payload (no hose unit in the
  first test), the measured armature range (hip/knee 0.018-0.034, ankle 0.0056-0.014 kg m^2), and IMU mounting tilt
  (+- 2 deg) and gyro bias (+- 0.02 rad/s);
- policy at 50 Hz (BHL: 25 Hz): the 20 ms period leaves room for reading 12 joints (p99 10.4 ms) and inference on
  the Radxa, and halves the hold time of each target. The critic observes the true state without delays.
BHL's joint default position randomization is removed: it shifted the observations but not the targets (the action
term copies the default pose before the startup events run), which is not how a zero offset acts.
"""

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.noise import AdditiveUniformNoiseCfg as Unoise

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_fw6.env_cfg import AsmrBipedFw6EnvCfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_fw6_meas.env_cfg import (
    JOINTS_5010,
    JOINTS_6512,
    use_measured_actuators,
)
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_s2r.env_cfg import AsmrDynamicsEventsCfg
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.env_cfg import ObservationsCfg
from source.robot_models.robots.asmr2026_real import ASMR2026_HUMANOID_REAL_CFG, PHYSICS_DT
from source.robot_models.robots.asmr2026_humanoid import ASMR2026_LEG_JOINTS as HUMANOID_LITE_LEG_JOINTS

POLICY_DECIMATION = 10  # 50 Hz at PHYSICS_DT = 2 ms
LEG_JOINTS = SceneEntityCfg("robot", joint_names=HUMANOID_LITE_LEG_JOINTS, preserve_order=True)


@configclass
class RealObservationsCfg:
    """The biped observations (same terms and order) from the delayed sensor readings."""

    @configclass
    class PolicyCfg(ObsGroup):
        velocity_commands = ObsTerm(func=mdp.generated_commands, params={"command_name": "base_velocity"})
        base_ang_vel = ObsTerm(func=mdp.imu_ang_vel, noise=Unoise(n_min=-0.3, n_max=0.3))
        projected_gravity = ObsTerm(func=mdp.imu_projected_gravity, noise=Unoise(n_min=-0.05, n_max=0.05))
        # +- 0.01 rad (BHL: 0.05): the encoders barely add noise; zero offset and backlash are the real errors
        joint_pos = ObsTerm(
            func=mdp.encoder_joint_pos_rel, params={"asset_cfg": LEG_JOINTS}, noise=Unoise(n_min=-0.01, n_max=0.01))
        joint_vel = ObsTerm(
            func=mdp.encoder_joint_vel_rel, params={"asset_cfg": LEG_JOINTS}, noise=Unoise(n_min=-2.0, n_max=2.0))
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = True

    policy: PolicyCfg = PolicyCfg()
    # the true state without delays or noise
    critic: ObservationsCfg.CriticCfg = ObservationsCfg.CriticCfg()


@configclass
class RealEventsCfg(AsmrDynamicsEventsCfg):
    """Dynamics randomization events plus the measured armature range; no payload (the first walking test is without
    the hose unit; the torso was weighed with everything mounted). The zero offset is in the actuator model."""

    add_all_joint_default_pos = None
    add_payload = None
    # joint measurements 2026-11-20: knee 0.022-0.034 (model 0.0228), ankle pitch 0.007-0.014 (model 0.0056), hip not
    # identified (same range as the knee)
    randomize_armature_6512 = EventTerm(
        func=mdp.randomize_actuator_armature,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINTS_6512), "armature_range": (0.018, 0.034)},
        mode="startup",
    )
    randomize_armature_5010 = EventTerm(
        func=mdp.randomize_actuator_armature,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=JOINTS_5010), "armature_range": (0.0056, 0.014)},
        mode="startup",
    )


@configclass
class AsmrBipedRealEnvCfg(AsmrBipedFw6EnvCfg):

    events: RealEventsCfg = RealEventsCfg()
    observations: RealObservationsCfg = RealObservationsCfg()

    def __post_init__(self):
        super().__post_init__()
        # MeasT68DR100's randomization with the torque limit fixed at 6 N m
        use_measured_actuators(self, None, 1.0)
        self.scene.robot = ASMR2026_HUMANOID_REAL_CFG.replace(prim_path="{ENV_REGEX_NS}/robot")
        self.sim.dt = PHYSICS_DT
        self.decimation = POLICY_DECIMATION
        self.sim.render_interval = self.decimation
        self.scene.contact_forces.update_period = self.sim.dt
