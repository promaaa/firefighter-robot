"""Export a policy trained with train_f1.py to ONNX, in the same form as BHL policy_biped_25hz_a.onnx.

Usage: python export_onnx.py <run_dir> [params.pkl] [out.onnx]     -> <run_dir>/policy.onnx
Runs of train_f1.py (config.json has "net") use ELU and the layer sizes of config.json. Without "net", the script
assumes a swish 512-256-128 network.
Input "obs" float32 [1, 45], output "actions" float32 [1, 12], opset 11, IR 6 (as the BHL file).
The observation normaliser is folded into the first layer, so the ONNX takes the raw BHL observation.
The output is the mean action (no sampling). The script checks the ONNX against the JAX policy.
"""

import json
import pickle
import sys

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

run_dir = sys.argv[1]
params_file = sys.argv[2] if len(sys.argv) > 2 else f"{run_dir}/params_latest.pkl"
out = sys.argv[3] if len(sys.argv) > 3 else f"{run_dir}/policy.onnx"
cfg = json.load(open(f"{run_dir}/config.json"))
F1 = "net" in cfg  # train_f1.py run
SIZES = tuple(cfg["net"]["policy_hidden_layer_sizes"]) if F1 else (512, 256, 128)
VSIZES = tuple(cfg["net"]["value_hidden_layer_sizes"]) if F1 else (512, 256, 128)
with open(params_file, "rb") as f:
  params = pickle.load(f)
norm, pol = params[0], params[1]["params"]
mean, std = np.asarray(norm.mean["state"], np.float64), np.asarray(norm.std["state"], np.float64)
assert mean.shape == (45,), "the policy input is not the 45 values of the BHL interface"
# Brax policy with a learned scalar std: MLP_0 (hidden layers, all activated) then Dense_0 (action mean).
layers = [pol["MLP_0"][f"hidden_{i}"] for i in range(len(pol["MLP_0"]))] + [pol["Dense_0"]]
n = len(layers)
Ws = [np.asarray(l["kernel"], np.float64) for l in layers]
bs = [np.asarray(l["bias"], np.float64) for l in layers]
if cfg.get("ppo", {}).get("normalize_observations", True):  # a --warm run has no normaliser
  bs[0] = bs[0] - (mean / std) @ Ws[0]  # fold (obs - mean) / std into layer 0
  Ws[0] = Ws[0] / std[:, None]

nodes, inits, x = [], [], "obs"
for i, (W, b) in enumerate(zip(Ws, bs)):
  inits += [numpy_helper.from_array(W.astype(np.float32), f"W{i}"), numpy_helper.from_array(b.astype(np.float32), f"b{i}")]
  last = i == n - 1
  nodes.append(helper.make_node("Gemm", [x, f"W{i}", f"b{i}"], ["actions" if last else f"z{i}"]))
  if not last and F1:  # ELU, as BHL
    nodes.append(helper.make_node("Elu", [f"z{i}"], [f"h{i}"], alpha=1.0))
    x = f"h{i}"
  elif not last:  # Brax default activation: swish(z) = z * sigmoid(z)
    nodes += [helper.make_node("Sigmoid", [f"z{i}"], [f"s{i}"]), helper.make_node("Mul", [f"z{i}", f"s{i}"], [f"h{i}"])]
    x = f"h{i}"
graph = helper.make_graph(nodes, "bhl_payload_policy",
                          [helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, 45])],
                          [helper.make_tensor_value_info("actions", TensorProto.FLOAT, [1, 12])], inits)
model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 11)], producer_name="sim/rl/export_onnx.py")
model.ir_version = 6
onnx.checker.check_model(model)
onnx.save(model, out)

# Check: the ONNX gives the same action as the JAX policy (on observations like the real ones).
import jax  # pylint: disable=g-import-not-at-top
import onnxruntime as ort
from brax.training.acme import running_statistics
from brax.training.agents.ppo import networks as ppo_networks
from flax import linen

net = ppo_networks.make_ppo_networks(
    {"state": (45,), "privileged_state": tuple(norm.mean["privileged_state"].shape)}, 12,
    preprocess_observations_fn=running_statistics.normalize if cfg.get("ppo", {}).get("normalize_observations", True)
    else (lambda x, y: x), policy_hidden_layer_sizes=SIZES,
    value_hidden_layer_sizes=VSIZES, policy_obs_key="state", value_obs_key="privileged_state",
    distribution_type="normal", activation=linen.elu if F1 else linen.swish)
policy = ppo_networks.make_inference_fn(net)(params, deterministic=True)
sess = ort.InferenceSession(out, providers=["CPUExecutionProvider"])
rng = np.random.default_rng(0)
worst = 0.0
for _ in range(50):
  obs = (mean + std * rng.standard_normal(45)).astype(np.float32)[None]
  a_onnx = sess.run(None, {"obs": obs})[0]
  a_jax = np.asarray(policy({"state": obs[0], "privileged_state": np.zeros(norm.mean["privileged_state"].shape, np.float32)}, jax.random.PRNGKey(0))[0])
  worst = max(worst, float(np.abs(a_onnx[0] - a_jax).max()))
assert worst < 1e-3, f"ONNX and JAX differ by {worst}"
print(f"wrote {out}; max difference to JAX {worst:.2e}")
