from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_fw6.agents.rsl_rl_ppo_cfg import (
    AsmrBipedFw6PPORunnerCfg,
)


@configclass
class AsmrBipedRealPPORunnerCfg(AsmrBipedFw6PPORunnerCfg):
    experiment_name = "asmr_real"
    # as the MeasT68 runs, which diverged without it (deploy with action_limit_lower/upper = -20/20)
    clip_actions = 20.0
