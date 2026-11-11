import gymnasium as gym

from . import agents, env_cfg

##
# Register Gym environments: Velocity-ASMR-Biped-Actuator-<task>-v0, tasks in env_cfg.GAIN_TASKS
##

for name in env_cfg.GAIN_TASKS:
    gym.register(
        id=f"Velocity-ASMR-Biped-Actuator-{name}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": getattr(env_cfg, f"Actuator{name}EnvCfg"),
            "rsl_rl_cfg_entry_point": getattr(agents.rsl_rl_ppo_cfg, f"AsmrActuator{name}PPORunnerCfg"),
        },
    )
