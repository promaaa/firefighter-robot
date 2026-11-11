import gymnasium as gym

from . import agents, env_cfg

##
# Register Gym environments: Velocity-ASMR-Biped-S2R-<task>-v0, tasks in env_cfg.TASKS
##

for name in env_cfg.TASKS:
    gym.register(
        id=f"Velocity-ASMR-Biped-S2R-{name}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": getattr(env_cfg, f"S2R{name}EnvCfg"),
            "rsl_rl_cfg_entry_point": getattr(agents.rsl_rl_ppo_cfg, f"AsmrS2R{name}PPORunnerCfg"),
        },
    )
