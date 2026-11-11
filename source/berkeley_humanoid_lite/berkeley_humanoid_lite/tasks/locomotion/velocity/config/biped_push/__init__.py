import gymnasium as gym

from . import agents, env_cfg

##
# Register Gym environments.
##

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-Push-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.BerkeleyHumanoidLiteBipedPushEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedPushPPORunnerCfg,
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-PushSignal-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.BerkeleyHumanoidLiteBipedPushSignalEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedPushSignalPPORunnerCfg,
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-Brace-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.BerkeleyHumanoidLiteBipedBraceEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedBracePPORunnerCfg,
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-PushSignalHold-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.BerkeleyHumanoidLiteBipedPushSignalHoldEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedPushSignalHoldPPORunnerCfg,
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-PushSignalHold2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.BerkeleyHumanoidLiteBipedPushSignalHold2EnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedPushSignalHold2PPORunnerCfg,
    },
)

gym.register(
    id="Velocity-Berkeley-Humanoid-Lite-Biped-Brace2-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": env_cfg.BerkeleyHumanoidLiteBipedBrace2EnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_ppo_cfg.BerkeleyHumanoidLiteBipedBrace2PPORunnerCfg,
    },
)
