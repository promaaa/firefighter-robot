# Trained walking policies

Each folder holds `model_<iteration>.pt` (RSL-RL checkpoint), `policy.onnx` (deployment network), `policy.yaml` (deployment config, paths relative to the repository root) and `params/` (`env.yaml`, `agent.yaml` of the training run). Train and evaluate with [sim/locomotion](../sim/locomotion/README.md).

**Every policy is selected by its folder name.** `policy_select.py` in this folder resolves a name to its files; with no name, `baseline` is used. Robot: `[legs] policy` in `robot/config.toml` or `legs/run_policy.py --policy <name>`. Simulation scripts: `--policy <name>`. Python: `from policies import policy_select; policy_select.onnx("baseline")` (also `.config(name)`, `.checkpoint(name)`, `.names()`). A new policy must be a folder here first: `sim/locomotion/scripts/rsl_rl/export_onnx.py --checkpoint <model.pt> --name <name>` creates it. `…` stands for `Velocity-ASMR-Biped`.

| Folder | Training task | Description | Obs |
|---|---|---|---|
| `baseline` | `…-v0` | Baseline walking. Upstream DR (mass, friction, joint offsets, gains x0.8-1.2). The robot runs this one. | 45 |
| `s2r_dr000` ... `s2r_dr200` | `…-S2R-DR{000,050,100,200}-v0` | Dynamics randomisation width alpha 0, 0.5, 1, 2 | 45 |
| `s2r_friction100` | `…-S2R-Friction100-v0` | alpha 1, friction only | 45 |
| `s2r_inertia100` | `…-S2R-Inertia100-v0` | alpha 1, mass and centre of mass only | 45 |
| `push_m2` | `…-Push-v0` | Push DR, no signal | 45 |
| `push_m3` | `…-PushSignal-v0` | Push signal added to the observation | 46 |
| `push_m3h` | `…-PushSignalHold-v0` | M3 + hold-position reward | 46 |
| `push_m3h2` | `…-PushSignalHold2-v0` | M3h improved (best single policy) | 46 |
| `brace_m4` | `…-Brace-v0` | Bracing-only policy | 46 |
| `brace_m4b` | `…-Brace2-v0` | M4 + strict hold reward, signal leads, actions limited to +-20 | 46 |
| `act_gain125` / `150` / `200` | `…-Actuator-Gain{125,150,200}-v0` | Per-joint Kp, Kd randomisation x0.8-1.25 / x0.67-1.5 / x0.5-2 | 45 |
| **`real_clamp12`** | `…-RealClamp12-v0` | **Walked on the real robot (25 Nov 2026), the best one.** Real-robot model, 50 Hz, Radxa torque clamp 12 N m | 45 |
| **`real_clamp9`** | `…-RealClamp9-v0` | **Walked on the real robot (25 Nov 2026).** Clamp 9 N m | 45 |
| **`real_clamp6`** | `…-RealClamp6-v0` | **Walked on the real robot (25 Nov 2026).** Clamp 6 N m; weaker sideways and turning | 45 |
| `real_walk` | `…-Real-v0` | Real-robot model with a 6 N m torque limit; base checkpoint of the clamp policies. On 24 Nov it leaned forward on the robot | 45 |

alpha 1 means torso mass +-1.5 kg, leg mass +-20 %, centre of mass +-3 cm, foot friction 0.8 +- 0.4.

- **`info.json`** says which robot model a policy was trained for (`trained_on`: `asmr2026` or `bhl`). A folder with a `policy.yaml` and no `info.json` is an ASMR policy. `payload` (`none`, `box_1kg`, `f3`) says what was on the torso in training, when `export_onnx.py --payload` recorded it. Code for the ASMR robot (`legs/run_policy.py`, `sim/sim2sim/run.py`) warns about the others and still loads them: their joint convention, default pose and gains differ, so they may behave oddly (`policy_select.onnx(..., strict=True)` refuses instead).
- `f1_v1` and `f3_v1` (45 observations, `trained_on: bhl`) are our JAX policies for the BHL-model F2 and F3 (card in `train_ranges.json`, training code in [archive/rl_f1](../archive/rl_f1/README.md)). `gap_study.py` uses `f1_v1` on the BHL model (FIXME(policy)); no code uses `f3_v1`. They have no `policy.yaml` and have not run on the robot.
- Only 45-observation policies fit the robot software today. The push-signal policies (46) take the push flag (0/1) as the last observation, which `legs/run_policy.py`, `RlController` and `play_mujoco.py` do not provide. Test them in MuJoCo with `sim/locomotion/scripts/sim2sim/eval_sustained_push_mujoco.py`.
- M4 and M4b are meant to be switched in from `baseline` when a push signal arrives. That switch is not implemented in the deployment code.
- **The real walking policies** (`real_clamp*`, `real_walk`) need a 50 Hz loop with their own standing pose (knee 0.4 rad), vel_ff = 0, and the Radxa target clamp at `torque_clamp_nm` of their `policy.yaml`. The real tests used the Radxa loop of the actuator test GUI. `legs/run_policy.py` (25 Hz, course pose, no clamp) cannot run them as it is. Details: [sim/locomotion/results/real_walk.md](../sim/locomotion/results/real_walk.md).
