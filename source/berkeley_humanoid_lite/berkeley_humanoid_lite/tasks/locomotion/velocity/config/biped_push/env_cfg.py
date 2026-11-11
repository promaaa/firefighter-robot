"""Biped tasks with a sustained push on the torso, e.g. the reaction of a water jet sprayed forward.

* Push:       the velocity task with sustained pushes; the policy must sense the push from its own state.
* PushSignal: the same, and the policy also observes the push signal (the planned force, known ahead
              from the spraying schedule).
* PushSignalHold: PushSignal, continued from a trained PushSignal policy, with a reward for staying in place
              while standing.
* PushSignalHold2: PushSignalHold with a stricter hold (stronger reward, no moving feet) and up to 1 s of
              signal lead, continued from a PushSignalHold policy.
* Brace2:     Brace with the stricter hold and up to 1 s of signal lead, continued from a Brace policy.
* Brace:      standing only, pushed backward, observes the push signal, rewarded for staying in place;
              meant to be switched in while spraying.

In all three the critic also sees the push force actually applied (privileged information).
"""

from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import berkeley_humanoid_lite.tasks.locomotion.velocity.mdp as mdp
from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.env_cfg import (
    BerkeleyHumanoidLiteBipedEnvCfg,
    CommandsCfg,
    ObservationsCfg,
    RewardsCfg,
)

@configclass
class PushCommandsCfg(CommandsCfg):
    """Velocity command and the push schedule."""

    push = mdp.SustainedPushCommandCfg(asset_name="robot", force_range=(0.0, 35.0), walking_force_scale=0.5)


@configclass
class PushObservationsCfg(ObservationsCfg):
    """Policy as in the biped task; the critic also sees the applied push force."""

    @configclass
    class CriticCfg(ObservationsCfg.CriticCfg):
        push_force = ObsTerm(func=mdp.push_force_applied, params={"command_name": "push"})

    critic: CriticCfg = CriticCfg()


@configclass
class SignalObservationsCfg(ObservationsCfg):
    """The policy also observes the push signal (last term); the critic the signal and the applied force."""

    @configclass
    class PolicyCfg(ObservationsCfg.PolicyCfg):
        push_signal = ObsTerm(func=mdp.push_signal, params={"command_name": "push"})

    @configclass
    class CriticCfg(ObservationsCfg.CriticCfg):
        push_signal = ObsTerm(func=mdp.push_signal, params={"command_name": "push"})
        push_force = ObsTerm(func=mdp.push_force_applied, params={"command_name": "push"})

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()


@configclass
class HoldRewardsCfg(RewardsCfg):
    """Biped rewards plus staying in place while standing: the velocity reward barely notices a slow creep."""

    standing_hold = RewTerm(
        func=mdp.standing_position_hold, weight=2.0, params={"command_name": "base_velocity", "std": 0.1}
    )


@configclass
class StrictHoldRewardsCfg(RewardsCfg):
    """Stronger stay-in-place reward, and a penalty on moving the feet while standing (no shuffling back)."""

    standing_hold = RewTerm(
        func=mdp.standing_position_hold, weight=3.0, params={"command_name": "base_velocity", "std": 0.08}
    )
    # charged while the feet move, not for where they are: a displacement penalty kept accruing after a
    # step and taught the policy that falling was cheaper (fall rate 25% -> 50% in the first Brace2 run)
    standing_feet = RewTerm(
        func=mdp.standing_feet_speed,
        weight=-2.0,
        params={"command_name": "base_velocity", "asset_cfg": SceneEntityCfg("robot", body_names=".*_ankle_roll")},
    )


@configclass
class PushCurriculumsCfg:
    """Grow the push force as long as the policy keeps falls rare."""

    push_force = CurrTerm(
        func=mdp.push_force_adaptive_curriculum,
        # some episodes end in a fall even without pushes (reset perturbations, walking), so the band is set
        # above that floor
        params={"command_name": "push", "start": 10.0, "end": 35.0, "fall_rate_low": 0.20, "fall_rate_high": 0.35},
    )


@configclass
class BerkeleyHumanoidLiteBipedPushEnvCfg(BerkeleyHumanoidLiteBipedEnvCfg):
    """Velocity task with sustained pushes; no push signal for the policy."""

    commands: PushCommandsCfg = PushCommandsCfg()
    observations: PushObservationsCfg = PushObservationsCfg()
    # ManagerBasedRLEnvCfg reads "curriculum"; the biped task's "curriculums" field is not used
    curriculum: PushCurriculumsCfg = PushCurriculumsCfg()

    def __post_init__(self):
        super().__post_init__()
        # stand still in a quarter of the episodes, where spraying happens
        self.commands.base_velocity.rel_standing_envs = 0.25
        # the push term applies its own small random wrench in place of this reset event
        self.events.base_external_force_torque = None


@configclass
class BerkeleyHumanoidLiteBipedPushSignalEnvCfg(BerkeleyHumanoidLiteBipedPushEnvCfg):
    """Velocity task with sustained pushes; the policy observes the push signal."""

    observations: SignalObservationsCfg = SignalObservationsCfg()


@configclass
class BerkeleyHumanoidLiteBipedPushSignalHoldEnvCfg(BerkeleyHumanoidLiteBipedPushSignalEnvCfg):
    """PushSignal with a reward for staying in place while standing; meant to continue a PushSignal policy."""

    rewards: HoldRewardsCfg = HoldRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        # the policy it continues already takes about 30 N
        self.curriculum.push_force.params["start"] = 20.0


@configclass
class BerkeleyHumanoidLiteBipedPushSignalHold2EnvCfg(BerkeleyHumanoidLiteBipedPushSignalEnvCfg):
    """PushSignalHold with a stricter hold and up to 1 s of signal lead; meant to continue a PushSignalHold policy."""

    rewards: StrictHoldRewardsCfg = StrictHoldRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.curriculum.push_force.params["start"] = 22.0
        # more time to get set before the force arrives
        self.commands.push.signal_lead_range = (0.0, 1.0)


@configclass
class BerkeleyHumanoidLiteBipedBraceEnvCfg(BerkeleyHumanoidLiteBipedPushSignalEnvCfg):
    """Standing only, pushed backward; the policy observes the push signal and is rewarded for staying put."""

    rewards: HoldRewardsCfg = HoldRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.commands.base_velocity.rel_standing_envs = 1.0
        self.commands.push.push_probability = 0.9
        self.commands.push.direction_range = (165.0, 195.0)
        self.commands.push.force_range = (0.0, 40.0)
        self.curriculum.push_force.params["end"] = 40.0


@configclass
class BerkeleyHumanoidLiteBipedBrace2EnvCfg(BerkeleyHumanoidLiteBipedBraceEnvCfg):
    """Brace with a stricter hold and up to 1 s of signal lead; meant to continue a Brace policy."""

    rewards: StrictHoldRewardsCfg = StrictHoldRewardsCfg()

    def __post_init__(self):
        super().__post_init__()
        self.curriculum.push_force.params["start"] = 20.0
        self.commands.push.signal_lead_range = (0.0, 1.0)
