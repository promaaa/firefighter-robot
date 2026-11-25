from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_real.agents.rsl_rl_ppo_cfg import (
    AsmrBipedRealPPORunnerCfg,
)


@configclass
class AsmrBipedRealClamp6PPORunnerCfg(AsmrBipedRealPPORunnerCfg):
    experiment_name = "asmr_real_clamp6"


@configclass
class AsmrBipedRealClamp9PPORunnerCfg(AsmrBipedRealPPORunnerCfg):
    experiment_name = "asmr_real_clamp9"


@configclass
class AsmrBipedRealClamp12PPORunnerCfg(AsmrBipedRealPPORunnerCfg):
    experiment_name = "asmr_real_clamp12"
