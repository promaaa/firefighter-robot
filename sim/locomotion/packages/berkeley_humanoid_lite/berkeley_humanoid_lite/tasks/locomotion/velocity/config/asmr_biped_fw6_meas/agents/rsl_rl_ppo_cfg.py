from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.asmr_biped_fw6.agents.rsl_rl_ppo_cfg import (
    AsmrBipedFw6PPORunnerCfg,
)


@configclass
class AsmrFw6MeasT6PPORunnerCfg(AsmrBipedFw6PPORunnerCfg):
    experiment_name = "asmr_fw6_meas_t6"


@configclass
class AsmrFw6MeasT68PPORunnerCfg(AsmrBipedFw6PPORunnerCfg):
    experiment_name = "asmr_fw6_meas_t68"
    # The first run (2026-11-06_20-41-38_seed42) diverged at iteration ~2950 (NaN action std), as M4b did; it was
    # resumed from model_2900 with the actions clipped to +-20 (deploy with action_limit_lower/upper = -20/20).
    clip_actions = 20.0


@configclass
class AsmrFw6MeasT68DR100PPORunnerCfg(AsmrBipedFw6PPORunnerCfg):
    experiment_name = "asmr_fw6_meas_t68_dr100"
    clip_actions = 20.0
