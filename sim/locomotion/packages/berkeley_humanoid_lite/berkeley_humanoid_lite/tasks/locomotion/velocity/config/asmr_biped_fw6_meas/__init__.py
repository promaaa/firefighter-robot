import gymnasium as gym

from . import agents, env_cfg

##
# Register Gym environments: Velocity-ASMR-Biped-FW6-<task>-v0, tasks in env_cfg.MEASURED_TASKS
##

for name in env_cfg.MEASURED_TASKS:
    gym.register(
        id=f"Velocity-ASMR-Biped-FW6-{name}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": getattr(env_cfg, f"Fw6{name}EnvCfg"),
            "rsl_rl_cfg_entry_point": getattr(agents.rsl_rl_ppo_cfg, f"AsmrFw6{name}PPORunnerCfg"),
        },
    )
