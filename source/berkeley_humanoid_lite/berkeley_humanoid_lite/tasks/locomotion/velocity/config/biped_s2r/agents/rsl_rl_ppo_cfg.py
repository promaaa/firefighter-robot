from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlPpoActorCriticCfg

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped.agents.rsl_rl_ppo_cfg import (
    BerkeleyHumanoidLiteBipedPPORunnerCfg,
)


@configclass
class S2RPPORunnerCfg(BerkeleyHumanoidLiteBipedPPORunnerCfg):
    """Runner for the dynamics gap study: trained from scratch, same budget for every task."""

    # the biped budget, the same for every randomization width (wide randomization may converge slower)
    max_iterations = 6000
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        noise_std_type="log",
        actor_hidden_dims=[256, 128, 128],
        critic_hidden_dims=[256, 128, 128],
        activation="elu",
    )


@configclass
class S2RDR000PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "s2r_dr000"


@configclass
class S2RDR050PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "s2r_dr050"


@configclass
class S2RDR100PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "s2r_dr100"


@configclass
class S2RDR200PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "s2r_dr200"


@configclass
class S2RFriction100PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "s2r_friction100"


@configclass
class S2RInertia100PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "s2r_inertia100"
