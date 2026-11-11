import gymnasium as gym

from . import agents, env_cfg

##
# Register Gym environments: Velocity-ASMR-Biped-<task>-v0, the course robot versions of config/biped_push.
##

for name in ["Push", "PushSignal", "PushSignalHold", "PushSignalHold2", "Brace", "Brace2"]:
    gym.register(
        id=f"Velocity-ASMR-Biped-{name}-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": getattr(env_cfg, f"AsmrBiped{name}EnvCfg"),
            "rsl_rl_cfg_entry_point": getattr(agents.rsl_rl_ppo_cfg, f"AsmrBiped{name}PPORunnerCfg"),
        },
    )
