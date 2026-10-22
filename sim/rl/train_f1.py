"""Train a walking policy in f1_env.py with Brax PPO, settings as the BHL RSL-RL config.

    XLA_PYTHON_CLIENT_MEM_FRACTION=0.45 uv run python train_f1.py <run_dir> <steps> [--restore=<params.pkl>]
        [--set=<config key>=<value> ...] [--envs=8192] [--evals=40] [--warm=<policy.onnx>] [--std=0.3]
        [--lr=1e-3] [--kl=0.01] [--entropy=0.008]

Writes <run_dir>/config.json, progress.jsonl, params_<step>.pkl and params_latest.pkl.
--set changes one env config value, for example --set=push.enable=False or --set=reward.feet_slide=-0.2.
--warm starts the policy from an ONNX file with the BHL layout (45-256-128-128-12, ELU), for example the shipped
BHL policy. Then the observations are not normalised (as BHL), so the copied weights see the same input.
"""

import ast
import functools
import json
import os
import pickle
import sys
import time

from brax.training.agents.ppo import networks as ppo_networks
from brax.training.agents.ppo import train as ppo
from flax import linen
from mujoco_playground import wrapper

import f1_env
import jax
import jax.numpy as jp
import onnx
from brax.training.acme import running_statistics, specs
from onnx import numpy_helper

args = [a for a in sys.argv[1:] if not a.startswith("--")]
opts = {a[2:].split("=", 1)[0]: a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--") and "=" in a}
sets = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--set=")]
run_dir, steps = args[0], int(float(args[1]))
os.makedirs(run_dir, exist_ok=True)

cfg = f1_env.default_config()
for s in sets:
  key, value = s.split("=", 1)
  *path, last = key.split(".")
  node = cfg
  for p in path:
    node = node[p]
  node[last] = ast.literal_eval(value)
cfg.naconmax = 16 * int(opts.get("envs", 8192))  # 16 contacts per env: 2 x 4 foot-floor, 8 foot-foot
env = f1_env.F1Joystick(cfg)

NET = dict(policy_hidden_layer_sizes=(256, 128, 128), value_hidden_layer_sizes=(256, 128, 128))  # BHL MLP
ppo_params = dict(  # BHL rsl_rl_ppo_cfg.py: 24 steps per env, 5 epochs, 4 minibatches, lr 1e-3 adaptive KL 0.01
    num_timesteps=steps, num_evals=int(opts.get("evals", 40)), num_envs=int(opts.get("envs", 8192)),
    episode_length=cfg.episode_length, unroll_length=24, num_minibatches=4, num_updates_per_batch=5,
    learning_rate=float(opts.get("lr", 1e-3)), learning_rate_schedule="ADAPTIVE_KL",
    desired_kl=float(opts.get("kl", 0.01)), entropy_cost=float(opts.get("entropy", 0.008)),
    discounting=0.99, gae_lambda=0.95, clipping_epsilon=0.2, max_grad_norm=1.0, vf_loss_coefficient=1.0,
    normalize_observations=not ("warm" in opts or ("restore" in opts and not json.load(open(
        os.path.join(os.path.dirname(opts["restore"]), "config.json")))["ppo"]["normalize_observations"])),
    reward_scaling=1.0, action_repeat=1, num_resets_per_eval=1,  # new start states at each evaluation
    seed=int(opts.get("seed", 0)),
)
ppo_params["batch_size"] = ppo_params["num_envs"] // ppo_params["num_minibatches"]
restore = None
if "restore" in opts:
  with open(opts["restore"], "rb") as f:
    restore = pickle.load(f)
json.dump({"env": cfg.to_dict(), "ppo": ppo_params, "net": NET, "restore": opts.get("restore"),
           "warm": opts.get("warm"), "warm_std": float(opts.get("std", 0.3))},
          open(os.path.join(run_dir, "config.json"), "w"), indent=1)

network_factory = functools.partial(
    ppo_networks.make_ppo_networks, **NET, activation=linen.elu, policy_obs_key="state",
    value_obs_key="privileged_state", distribution_type="normal", init_noise_std=1.0)  # unbounded, as RSL-RL

if "warm" in opts:  # policy weights from the ONNX file, a new critic, an identity normaliser
  init = {i.name: numpy_helper.to_array(i) for i in onnx.load(opts["warm"]).graph.initializer}
  gemm = [n for n in onnx.load(opts["warm"]).graph.node if n.op_type == "Gemm"]
  nets = network_factory(env.observation_size, env.action_size,
                         preprocess_observations_fn=running_statistics.normalize)
  pol = nets.policy_network.init(jax.random.PRNGKey(1))
  val = nets.value_network.init(jax.random.PRNGKey(2))
  names = [("MLP_0", f"hidden_{i}") for i in range(len(gemm) - 1)] + [("Dense_0",)]
  for name, n in zip(names, gemm):
    layer = pol["params"]
    for k in name:
      layer = layer[k]
    layer["kernel"], layer["bias"] = jp.array(init[n.input[1]].T), jp.array(init[n.input[2]])  # torch (out, in)
  pol["params"]["std_param"]["value"] = jp.full(env.action_size, float(opts.get("std", 0.3)))
  norm = running_statistics.init_state({k: specs.Array(v, jp.float32) for k, v in env.observation_size.items()})
  restore = (norm, pol, val)

t0 = time.time()
log = open(os.path.join(run_dir, "progress.jsonl"), "a")


def progress(step, metrics):
  row = {"step": int(step), "time_s": round(time.time() - t0, 1)}
  row.update({k: float(v) for k, v in metrics.items() if (k.startswith("eval/episode_") and not k.endswith("_std")) or k == "eval/avg_episode_length"})
  log.write(json.dumps(row) + "\n")
  log.flush()
  print(f"step {step:>11,}  t {row['time_s']:7.1f}s  reward {row.get('eval/episode_reward', 0):8.3f}  "
        f"len {row.get('eval/avg_episode_length', metrics.get('eval/avg_episode_length', 0)):6.1f}", flush=True)


def save(step, make_policy, params):
  del make_policy
  for name in (f"params_{int(step)}.pkl", "params_latest.pkl"):
    with open(os.path.join(run_dir, name), "wb") as f:
      pickle.dump(params, f)


ppo.train(
    environment=env,
    wrap_env_fn=wrapper.wrap_for_brax_training,
    randomization_fn=functools.partial(f1_env.domain_randomize, mj_model=env.mj_model,
                                       torque_range=tuple(cfg.torque_range)),
    network_factory=network_factory, progress_fn=progress, policy_params_fn=save, restore_params=restore,
    **ppo_params,
)
print(f"done in {time.time() - t0:.0f} s", flush=True)
