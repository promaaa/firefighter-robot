"""Run an ONNX policy (Gemm + Elu MLP, like the shipped BHL file) inside the MJX training env f1_env.py.

    XLA_PYTHON_CLIENT_MEM_FRACTION=0.3 uv run python mjx_check.py [policy.onnx] [torque Nm] [--terms]

Checks that the training simulator agrees with the CPU harness: the shipped policy must walk here too.
No pushes, no recoil, no constant wrench, no latency, no zero error; observation noise on; masses as in bhl_F2, see f1_env.py.
--terms: falls of 32 runs (20 s) at vx 0.3 and stand, with one randomisation term at a time.
"""
import sys

import jax
import jax.numpy as jp
import numpy as np
import onnx
from onnx import numpy_helper

import f1_env

TERMS = "--terms" in sys.argv
sys.argv = [a for a in sys.argv if a != "--terms"]
path = sys.argv[1] if len(sys.argv) > 1 else str(f1_env.HERE.parent / "sim2sim" / "policy_biped_25hz_a.onnx")
limit = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
g = onnx.load(path).graph
w = {i.name: numpy_helper.to_array(i) for i in g.initializer}
layers = []
for n in g.node:
  if n.op_type == "Gemm":
    tb = next((a.i for a in n.attribute if a.name == "transB"), 0)
    W, b = w[n.input[1]], w[n.input[2]]
    layers.append((jp.array(W.T if tb else W), jp.array(b)))


def policy(obs):
  x = obs
  for i, (W, b) in enumerate(layers):
    x = x @ W + b
    if i < len(layers) - 1:
      x = jax.nn.elu(x)
  return x


cfg = f1_env.default_config()
cfg.push.enable = False
cfg.recoil.force = [0.0, 0.0]
cfg.reset_wrench = [0.0, 0.0]
cfg.act_delay_steps = cfg.obs_delay_steps = 0
cfg.zero_offset = 0.0
cfg.naconmax = 16 * 64
env = f1_env.F1Joystick(cfg)
m = env.mjx_model
m = m.tree_replace({"actuator_forcerange": m.actuator_forcerange.at[:, 0].set(-limit).at[:, 1].set(limit),
                    "geom_friction": m.geom_friction.at[0, 0].set(1.0)})
env._mjx_model = m


def rollout(cmd, key, steps=500):
  s = env.reset(key)
  s.info["command"] = jp.array(cmd)
  s.info["cmd_left"] = jp.array(10 ** 6)
  # BHL start of the harness: base at the origin, default joints, no velocity
  data = s.data.replace(qpos=jp.array(env._init_q), qvel=jp.zeros_like(s.data.qvel))
  s = s.replace(data=data)

  def f(s, _):
    a = policy(s.obs["state"])
    s = env.step(s, a)
    v = env._v_yaw(s.data)
    return s, (s.done, v[0], env._wz(s.data))
  _, (done, vx, wz) = jax.lax.scan(f, s, None, length=steps)
  return done, vx, wz


def rollout_terms(cmd, key, wrench, act_d, obs_d, std, push, zero):
  """As rollout, with one disturbance: wrench (x BHL +-2 N, +-2 Nm), delays (5 ms steps), action noise std,
  a velocity push (m/s) every 6 s, joint zero error (x +-0.05 rad)."""
  s = env.reset(key)
  k = jax.random.split(key, 4)
  s.info["command"], s.info["cmd_left"] = cmd, jp.array(10 ** 6)
  s.info["wrench"] = wrench * jp.r_[jax.random.uniform(k[0], (3,), minval=-2, maxval=2),
                                    jax.random.uniform(k[1], (3,), minval=-2, maxval=2)]
  s.info["act_delay"], s.info["obs_delay"] = act_d, obs_d
  s.info["zero_off"] = zero * jax.random.uniform(k[2], (12,), minval=-0.05, maxval=0.05)
  s = s.replace(data=s.data.replace(qpos=jp.array(env._init_q), qvel=jp.zeros_like(s.data.qvel)))

  def f(carry, i):
    s, key = carry
    key, k1, k2 = jax.random.split(key, 3)
    a = policy(s.obs["state"]) + std * jax.random.normal(k1, (12,))
    th = jax.random.uniform(k2, maxval=2 * jp.pi)
    kick = push * (i % 150 == 149) * jp.array([jp.cos(th), jp.sin(th)])
    s = env.step(s.replace(data=s.data.replace(qvel=s.data.qvel.at[0:2].add(kick))), a)
    return (s, key), (s.done, env._v_yaw(s.data)[0])
  _, (done, vx) = jax.lax.scan(f, (s, k[3]), jp.arange(500))
  return done.any(), vx[125:].mean()


if TERMS:
  run = jax.jit(jax.vmap(rollout_terms, in_axes=(None, 0, None, None, None, None, None, None)))
  keys = jax.random.split(jax.random.PRNGKey(0), 32)
  cases = {"none": (0, 0, 0, 0, 0, 0), "wrench +-2 N, +-2 Nm": (1, 0, 0, 0, 0, 0), "command delay 20 ms": (0, 4, 0, 0, 0, 0),
           "observation delay 20 ms": (0, 0, 4, 0, 0, 0), "10 ms + 10 ms delay": (0, 2, 2, 0, 0, 0),
           "action noise 0.3": (0, 0, 0, 0.3, 0, 0), "push 0.6 m/s every 6 s": (0, 0, 0, 0, 0.6, 0),
           "push 0.3 m/s every 6 s": (0, 0, 0, 0, 0.3, 0), "zero error +-0.05 rad": (0, 0, 0, 0, 0, 1)}
  for cname, cmd in [("vx 0.3", (0.3, 0, 0)), ("stand", (0, 0, 0))]:
    for name, args in cases.items():
      done, vx = (np.asarray(x) for x in run(jp.array(cmd, jp.float32), keys, *args))
      print(f"{limit:.1f} Nm  {cname:7s} {name:24s} falls {done.sum():2d}/32  "
            f"vx {vx[~done].mean() if (~done).any() else np.nan:+.2f}", flush=True)
  sys.exit()

run = jax.jit(jax.vmap(rollout, in_axes=(None, 0)))
keys = jax.random.split(jax.random.PRNGKey(0), 16)
for name, cmd in [("stand", (0, 0, 0)), ("vx 0.3", (0.3, 0, 0)), ("turn on the spot 0.5", (0, 0, 0.5)),
                  ("turn on the spot 0.8", (0, 0, 0.8)), ("vx 0.2 + yaw 0.5", (0.2, 0, 0.5))]:
  done, vx, wz = (np.asarray(x) for x in run(jp.array(cmd, jp.float32), keys))
  fell = done.any(axis=1)
  ok = ~fell
  print(f"{name:22s} falls {fell.sum():2d}/16  vx {vx[ok, 125:].mean() if ok.any() else np.nan:+.2f}  "
        f"yaw rate {wz[ok, 125:].mean() if ok.any() else np.nan:+.2f}")
