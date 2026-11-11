from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_push.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLiteBipedBrace2PPORunnerCfg,
    BerkeleyHumanoidLiteBipedBracePPORunnerCfg,
    BerkeleyHumanoidLiteBipedPushPPORunnerCfg,
    BerkeleyHumanoidLiteBipedPushSignalHold2PPORunnerCfg,
    BerkeleyHumanoidLiteBipedPushSignalHoldPPORunnerCfg,
    BerkeleyHumanoidLiteBipedPushSignalPPORunnerCfg,
)


@configclass
class AsmrBipedPushPPORunnerCfg(BerkeleyHumanoidLiteBipedPushPPORunnerCfg):
    experiment_name = "asmr_biped_push"


@configclass
class AsmrBipedPushSignalPPORunnerCfg(BerkeleyHumanoidLiteBipedPushSignalPPORunnerCfg):
    experiment_name = "asmr_biped_push_signal"


@configclass
class AsmrBipedPushSignalHoldPPORunnerCfg(BerkeleyHumanoidLiteBipedPushSignalHoldPPORunnerCfg):
    experiment_name = "asmr_biped_push_signal_hold"


@configclass
class AsmrBipedPushSignalHold2PPORunnerCfg(BerkeleyHumanoidLiteBipedPushSignalHold2PPORunnerCfg):
    experiment_name = "asmr_biped_push_signal_hold2"


@configclass
class AsmrBipedBracePPORunnerCfg(BerkeleyHumanoidLiteBipedBracePPORunnerCfg):
    experiment_name = "asmr_biped_brace"


@configclass
class AsmrBipedBrace2PPORunnerCfg(BerkeleyHumanoidLiteBipedBrace2PPORunnerCfg):
    experiment_name = "asmr_biped_brace2"
    # Without a bound, the course robot's Brace2 policy learned rare actions of 100-200 (typical 3, M4 at most ~20);
    # fed back through the last-action observation they blew up the value loss twice (iterations 7815 and 8585).
    # 20 leaves the actions the M4 policy uses untouched; deploy with action_limit_lower/upper = -20/20.
    clip_actions = 20.0
