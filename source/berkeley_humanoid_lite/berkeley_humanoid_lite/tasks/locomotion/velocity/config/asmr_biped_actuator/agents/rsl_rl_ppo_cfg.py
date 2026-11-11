from isaaclab.utils import configclass

from berkeley_humanoid_lite.tasks.locomotion.velocity.config.biped_s2r.agents.rsl_rl_ppo_cfg import S2RPPORunnerCfg


@configclass
class AsmrActuatorGain125PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_act_gain125"


@configclass
class AsmrActuatorGain150PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_act_gain150"


@configclass
class AsmrActuatorGain200PPORunnerCfg(S2RPPORunnerCfg):
    experiment_name = "asmr_act_gain200"
