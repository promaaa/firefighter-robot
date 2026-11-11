from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLiteBipedPPORunnerCfg,
)


@configclass
class BerkeleyHumanoidLiteBipedPushPPORunnerCfg(BerkeleyHumanoidLiteBipedPPORunnerCfg):
    # fine-tuned from the biped policy (see scripts/rsl_rl/expand_checkpoint.py)
    max_iterations = 1500
    experiment_name = "biped_push"


@configclass
class BerkeleyHumanoidLiteBipedPushSignalPPORunnerCfg(BerkeleyHumanoidLiteBipedPushPPORunnerCfg):
    experiment_name = "biped_push_signal"


# a learned std can step below zero and stop training ("normal expects all elements of std >= 0"),
# as the first Brace run did; the log parameterization cannot (warm-start with expand_checkpoint.py --log_std)
LOG_STD_POLICY = RslRlPpoActorCriticCfg(
    init_noise_std=1.0,
    noise_std_type="log",
    actor_hidden_dims=[256, 128, 128],
    critic_hidden_dims=[256, 128, 128],
    activation="elu",
)


@configclass
class BerkeleyHumanoidLiteBipedBracePPORunnerCfg(BerkeleyHumanoidLiteBipedPushPPORunnerCfg):
    experiment_name = "biped_brace"
    policy = LOG_STD_POLICY


@configclass
class BerkeleyHumanoidLiteBipedPushSignalHoldPPORunnerCfg(BerkeleyHumanoidLiteBipedPushPPORunnerCfg):
    experiment_name = "biped_push_signal_hold"
    policy = LOG_STD_POLICY


@configclass
class BerkeleyHumanoidLiteBipedPushSignalHold2PPORunnerCfg(BerkeleyHumanoidLiteBipedPushPPORunnerCfg):
    experiment_name = "biped_push_signal_hold2"
    policy = LOG_STD_POLICY


@configclass
class BerkeleyHumanoidLiteBipedBrace2PPORunnerCfg(BerkeleyHumanoidLiteBipedPushPPORunnerCfg):
    experiment_name = "biped_brace2"
    policy = LOG_STD_POLICY
