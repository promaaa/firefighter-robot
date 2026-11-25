import gymnasium as gym

from . import agents, env_cfg

##
# Register Gym environments
##

for limit in (6, 9, 12):
    gym.register(
        id=f"Velocity-ASMR-Biped-RealClamp{limit}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": getattr(env_cfg, f"AsmrBipedRealClamp{limit}EnvCfg"),
            "rsl_rl_cfg_entry_point": getattr(agents.rsl_rl_ppo_cfg, f"AsmrBipedRealClamp{limit}PPORunnerCfg"),
        },
    )
