# Walking policy: training and evaluation

All code that creates and checks the walking policy lives here: Isaac Lab training, ONNX export, and the sim2sim and robustness experiments. It comes from the SOBANG locomotion repository (see [UPSTREAM.md](UPSTREAM.md)). **Nothing here runs on the robot.** The robot only reads the finished network from [`policies/`](../../policies/README.md).

```
sim/locomotion/  ──train, evaluate──▶  policies/<name>/policy.onnx  ──read by──▶  legs/run_policy.py (robot)
                                                                                  sim/sim2sim/        (laptop check)
robot model: source/robot_models  (shared by training, sim2sim and the robot)
```

Results are **tested in simulation** unless a table says otherwise (status words: [CONTRIBUTING](../../CONTRIBUTING.md)).

## Walked on the real robot

**The policies `real_clamp12`, `real_clamp9` and `real_clamp6` walked on the real robot on 25 November 2026** (in a harness; `real_clamp12` best; measured on board). They come from the real walking tasks `Velocity-ASMR-Biped-RealClamp{12,9,6}-v0`, which rebuild the simulation from measurements on the real robot:
- the course driver PD, measured friction and armature, and the weighed link masses;
- encoder zero offsets, backlash, sensor and command delays, and BMI088 errors;
- a forward-shifted torso CoM range;
- the 50 Hz Radxa loop with its torque clamp (the drives keep their 18 A limit).

The page [results/real_walk.md](results/real_walk.md) describes the environment, how the policies were trained, the simulation checks, and what the robot loop must do. Read it before you change these tasks or run the policies on the robot.

```bash
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_tracking_mujoco.py --policy real_clamp12 --name real_clamp12   # MuJoCo with the real-robot effects
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_real_conditions_mujoco.py --policy real_clamp12 --variant com_x=4  # real-test conditions
```

`legs/run_policy.py` is fixed for `baseline` (25 Hz, course pose) and has no torque clamp, so it cannot run these policies as it is.

## Folder

| Path | Content |
|---|---|
| `scripts/` | All commands, grouped by purpose (tables below) |
| `packages/berkeley_humanoid_lite/` | Isaac Lab tasks (`tasks/locomotion/velocity/`: `config/asmr_*` for the course robot, `mdp/` rewards, events, pushes, curriculum) and the MuJoCo environment |
| `packages/berkeley_humanoid_lite_lowlevel/` | Only the policy loop of the upstream lowlevel repo: `RlController`, `Cfg`, `Se2Gamepad` |
| (robot) | The course robot model is `source/robot_models/` at the repository root, not in this folder. The original BHL robot is not in the repository: [UPSTREAM.md](UPSTREAM.md) says how to get it |
| `configs/` | Deployment config templates. `policy_asmr_biped.yaml` is the course robot template; `play.py` writes `policy_latest.yaml` (not in git) |
| `tests/test_layout.py` | Static checks, no GPU and no Isaac Sim |
| `pyproject.toml` | Own uv project: Python 3.10, Isaac Sim 4.5, Isaac Lab 2.1 (the root project is Python 3.12 and does not install them) |

Logs, recordings and checkpoints go to `logs/` at the repository root (not in git; `~/runs/` or the team drive for long-term copies, as in [CONTRIBUTING](../../CONTRIBUTING.md)). Put small result summaries (tables, figures) in `sim/locomotion/results/` with their date, command and status. Trained networks that the robot may use go to `policies/`. Run every command from the repository root.

## Setup (once)

An NVIDIA GPU is needed for training. The first sync downloads several GB.

```bash
uv sync --project sim/locomotion
```

## Train, play, export

```bash
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/train.py --task Velocity-ASMR-Biped-v0 --payload f3 --headless
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/play.py --task Velocity-ASMR-Biped-v0 --payload f3 --checkpoint <model.pt>
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/play_onnx.py --policy baseline --task Velocity-ASMR-Biped-v0 --payload f3
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_tracking_mujoco.py --policy baseline --name baseline
uv run --project sim/locomotion sim/locomotion/scripts/list_envs.py     # registered tasks
```

The default run (4096 environments, 6000 iterations) took about 2 h 20 min on an RTX 4060.

**Payload is chosen by name too.** `train.py`, `play.py` and `play_onnx.py` take `--payload NAME` (`none`, `box_1kg`, `f3`; without it the task's own `box_1kg` stays). `box_1kg` is the 1 kg box of the baseline. `f3` is the 47 parts of our F3 design (about 4.1 kg: batteries, computer, IMU, pan-tilt servos, camera, nozzle, pod), listed one by one in `source/robot_models/robots/asmr2026_payload.py` from `source/robot_models/data/f3_layout.json`. The parts are rigid and have no collision, so Isaac Lab merges them into the base link (mass, centre of mass and inertia by the parallel-axis sum, `mdp.add_body_payload` with `inertia`), and MuJoCo keeps them as separate bodies. Both give the same torso (`tests/test_layout.py` checks it against MuJoCo's own fusion). The course torso (5.57 kg) is taken to hold its own electronics, so nothing is removed from it: F3 is an upper bound.

```bash
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/train.py --task Velocity-ASMR-Biped-v0 --payload f3 --headless
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/export_onnx.py --name baseline_f3 --payload f3     # the last model of the newest run
uv run python sim/sim2sim/run.py --policy baseline_f3 --payload f3     # MuJoCo, no Isaac
```

`export_onnx.py --payload NAME` writes the payload into `info.json` and into the `sim_payload` of `policy.yaml`, so MuJoCo scripts weld the same parts. The saved `params/env.yaml` of a run also records the payload event.

**Which `<model_*.pt>`?** `train.py` writes one run folder per start, `logs/rsl_rl/asmr_biped/<date_time>/`, and saves `model_<iteration>.pt` in it
(`model_0.pt` is the start, the file with the highest number is the last one; a run that was stopped has fewer files).
The two scripts take the file differently:

| Script | `--checkpoint` is | Example |
|---|---|---|
| `play.py` | a file **name** inside the run folder; the newest run is used unless `--load_run <date_time>` is given | `--checkpoint model_5999.pt` |
| `export_onnx.py` | the same as `play.py` (a file name; `--load_run` too), or a **path**, or a run folder (its last model), or nothing (the last model of the newest run) | `--checkpoint model_5999.pt` or no `--checkpoint` |

List the files with `ls logs/rsl_rl/asmr_biped/*` (PowerShell: `Get-ChildItem logs/rsl_rl/asmr_biped/*`). Do not type the `<` and `>`; they only
mark the place of a name. Add `--headless` to `play.py` when the PC has no usable display GPU (it then writes the ONNX and the config but opens no window).



**Policies are chosen by name.** Every evaluation, play and recording script takes `--policy NAME` (a folder in `policies/`, default `baseline`); the `s2r/` scripts use `policies/<name>/model_*.pt`. `export_onnx.py --checkpoint <model.pt> --name NAME` creates `policies/NAME/` from a training run. Copy `params/` of the run into it too.

**To put a new network on the robot:** make `policies/<name>/` (`export_onnx.py` or copy `model_*.pt`, `policy.onnx`, `policy.yaml` and `params/`), then follow [Put a policy on the robot](../../README.md#put-a-new-policy-on-the-robot).

## Tasks

`…` stands for `Velocity-ASMR-Biped`.

| Task | Purpose |
|---|---|
| `…-v0` | Baseline walking. This is the robot's policy |
| `…-{Push, PushSignal, PushSignalHold, PushSignalHold2, Brace, Brace2}-v0` | Backward push (water-jet reaction); the signal variants add a push flag (46 observations) |
| `…-S2R-{DR000, DR050, DR100, DR200, Friction100, Inertia100}-v0` | Width of the dynamics randomisation |
| `…-Actuator-{Gain125, Gain150, Gain200}-v0` | PD gain randomisation |
| `…-FW6-v0`, `…-FW6-{MeasT6, MeasT68, MeasT68DR100}-v0` | Course driver firmware gains with a 6 N m limit; the Meas tasks add the measured friction model (MeasT68DR100 is the base of the real tasks) |
| `…-Real-v0` | Real walking test model (`source/robot_models/robots/asmr2026_real.py`), 50 Hz, 6 N m torque limit. Policy `real_walk` |
| `…-RealClamp{6, 9, 12}-v0` | `Real-v0` with the Radxa target clamp at 6, 9 or 12 N m and a forward torso CoM range. **Walked on the real robot** (policies `real_clamp6`, `real_clamp9`, `real_clamp12`) |

The old names `Velocity-Berkeley-Humanoid-Lite-Biped-*` are no longer registered. Their code (`config/biped*`) stays as the base configuration that the `config/asmr_biped*` tasks extend. The BHL original robot task was removed ([UPSTREAM.md](UPSTREAM.md)). The trained networks of every task are described in [policies/README.md](../../policies/README.md).

## Scripts

"Isaac" needs Isaac Sim and the GPU. "MuJoCo" needs neither.

### `scripts/rsl_rl/`: training and export

| Script | Does | Runs in |
|---|---|---|
| `train.py` | PPO training with RSL-RL (`--payload NAME`) | Isaac |
| `play.py` | Plays a checkpoint, exports ONNX and writes `configs/policy_latest.yaml` (`--payload NAME`) | Isaac |
| `play_onnx.py` | Plays an exported ONNX network in the training environment (`--payload NAME`) | Isaac |
| `export_onnx.py` | Checkpoint to `policies/<name>/policy.onnx` and `policy.yaml` without Isaac Sim; leaves `policy_latest.yaml` alone | none |
| `expand_checkpoint.py` | Warm start for a task whose observation gained terms at the end (e.g. the push signal) | none |

### `scripts/sim2sim/`: Isaac Lab against MuJoCo

| Script | Does | Runs in |
|---|---|---|
| `eval_tracking_isaaclab.py` | 8 velocity commands from standing; the reference | Isaac |
| `eval_tracking_mujoco.py` | The same commands with the deployment stack (`RlController`) | MuJoCo |
| `eval_sustained_push_mujoco.py` | Sustained backward push on the standing robot | MuJoCo |
| `eval_timing_mujoco.py` | Tracking under delay, period and jitter; cross-check of `s2r/eval_timing_gap.py` | MuJoCo |
| `eval_real_conditions_mujoco.py` | Real walking policies under real-test conditions (IMU tilt, forward lean, knee zero, torso CoM, friction, harness): falls, pitch, torque, tracking error | MuJoCo |
| `play_mujoco.py` | Plays a config in the MuJoCo viewer; velocity from a gamepad | MuJoCo |
| `tracking_common.py` | The command protocol shared by the two tracking evaluations | (helper) |

### `scripts/push_test/`: push robustness

| Script | Does | Runs in |
|---|---|---|
| `run_push_test.py` | Standing robot, 8 directions, impulse and sustained push, increasing force | Isaac |
| `eval_sustained_push.py` | Largest steady backward force the standing robot holds, like a water jet's reaction | Isaac |
| `plot_push_radar.py` | Radar chart of push results, two policies | none |
| `record_push.py` | Video of one push trial with the force arrow | Isaac |
| `push_common.py` | Shared setup | (helper) |

### `scripts/s2r/`: simulation-to-reality gaps

Run **one policy per process**. With several policies in one process the results from the second policy on were wrong.

| Script | Does | Runs in |
|---|---|---|
| `eval_dynamics_gap.py` | Foot friction, torso and leg mass, centre of mass different from training | Isaac |
| `eval_actuator_gap.py` | PD gains and torque limit different from training | Isaac |
| `eval_timing_gap.py` | Action and observation delay, period, jitter, skipped calls | Isaac |
| `eval_goal_reaching.py` | Straight walks, turns and standing on the nominal dynamics | Isaac |
| `scenario_fall_rate.py` | Fall rate on the video scenario for several torso masses | Isaac |
| `record_gap_video.py`, `make_gap_videos.sh` | Videos of a policy with a changed torso mass | Isaac |
| `summarize_eval.py`, `make_result_figures.py` | Text tables and figures from the evaluation CSV files | none |
| `video_common.py` | Shared scenario and overlays | (helper) |

### `scripts/scenario/` and `scripts/sim2real/`

| Script | Does | Runs in |
|---|---|---|
| `scenario/record_isaaclab.py`, `scenario/record_mujoco.py` | Same scripted walk (forward, turn, forward, left, backward) as video in both simulators | Isaac / MuJoCo |
| `scenario/scenario_common.py` | Shared scenario, camera and captions | (helper) |
| `sim2real/visualize.py` | Shows observations received by UDP (port 11000) in the MuJoCo viewer. The upstream BHL stack sends them; `legs/run_policy.py` does not | MuJoCo |

## Verify locally

Do these on your own laptop before you push. Levels A and B need no GPU. **Only A was run when this layout was made**; B and C need the Isaac Lab environment and a GPU, so run them once yourself.

**A. Layout and the robot software (any laptop, a few minutes).**

```bash
uv sync --locked --extra sim --extra dev                  # mujoco must be 3.15.0: another version fails legs/test_legs.py
uv run --extra sim --extra dev pytest sim/locomotion/tests/test_layout.py -q    # paths, imports, policy files
uv run --extra sim --extra dev pytest -q                  # aim, actuators, robot
uv run --extra sim --extra dev pytest legs sim/mission/test_mission.py -q
uv run --extra sim --extra dev python legs/run_policy.py --check                # network: 45 observations, 12 actions
uv run --extra sim --extra dev python sim/sim2sim/run.py --check                # robot model stands 5 s
```

`test_layout.py` fails if a script imports a missing module, a `policy.yaml` points at a missing file or a moved path is still used.

**B. Training environment (needs the Isaac Lab uv environment).**

```bash
uv sync --project sim/locomotion
uv run --project sim/locomotion sim/locomotion/scripts/list_envs.py       # all Velocity-* tasks are listed
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_tracking_mujoco.py --policy baseline --name check --seeds 0
```

**C. On a GPU.**

```bash
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/train.py --task Velocity-ASMR-Biped-v0 --headless --num_envs 64 --max_iterations 2
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/export_onnx.py --checkpoint policies/baseline/model_5999.pt --name export_check
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_tracking_isaaclab.py --headless --policy baseline --name check
```

Compare `policies/export_check/policy.onnx` (delete the folder afterwards) with `policies/baseline/policy.onnx` (same output for the same input), and the two `eval_tracking_*` result files (the same trends).

## Known limits

- **Observation size.** The robot software runs 45-observation policies only. The push-signal policies (46) are tested in MuJoCo with `eval_sustained_push_mujoco.py`. Switching from `baseline` to the Brace policy at a push is not implemented on the robot.
- **Timing study.** `eval_timing_gap.py` has one scenario that copies the upstream BHL deployment loop (state read after the targets are sent). The loop of `legs/run_policy.py` is not modelled; its measured period is printed at the end of a run.
- **Policy selection.** `--policy NAME` everywhere, default `baseline`; `Cfg.from_arguments()` (used by `play_mujoco.py`) takes it too. `sim/sim2sim/hw_risks.py` and `gap_study.py` use the ASMR robot and the named policy (`--policy NAME`, `--payload NAME`). `sim/mission` and `sim/payload/final_check.py` also use the ASMR robot and the named policy. The old BHL loop and model builder are in `archive/payload_bhl/` (no user).
- **`mjcf_path` is required.** A deployment config without it makes the MuJoCo scripts stop with an error; use the configs of the course robot (`policies/*/policy.yaml`, `configs/policy_asmr_biped.yaml`).
- **Recorded paths.** `policies/*/params/env.yaml` keep the absolute paths of the training computer. They are a record and are not read back.
- **Not taken over from the SOBANG repository:** the BHL upper-body teleop (`scripts/teleop`), the BHL model comparison and mass sheets (`scripts/hardware`), and the C++/CAN drivers of the upstream lowlevel repo. The robot's drives are in `legs/` and `actuators/`.
