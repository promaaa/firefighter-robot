from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_s2r.agents.rsl_rl_ppo_cfg import S2RPPORunnerCfg


@configclass
class AsmrS2RDR000PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_s2r_dr000"


@configclass
class AsmrS2RDR050PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_s2r_dr050"


@configclass
class AsmrS2RDR100PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_s2r_dr100"


@configclass
class AsmrS2RDR200PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_s2r_dr200"


@configclass
class AsmrS2RFriction100PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_s2r_friction100"


@configclass
class AsmrS2RInertia100PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_s2r_inertia100"
