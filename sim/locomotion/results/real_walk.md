# Walking on the real robot: the training environment and its policies

**On 25 November 2026 the robot walked with the policies of this page.** All three torque clamp policies walked on the real robot in a harness, and `real_clamp12` walked best (team report; the run records will be added). These are the first policies from this repository that walked on the real robot. Before them, `baseline` had not been tried on the real robot, and `real_walk` (24 Nov) leaned forward and fell when the torque was limited.

Status words as in [CONTRIBUTING](../../../CONTRIBUTING.md). Commands run from the repository root with `uv run --project sim/locomotion`.

| Date | Policy | Result | Status |
|---|---|---|---|
| 25 Nov | `real_clamp12` | Walked on the real robot in a harness, the best of the three | measured on board |
| 25 Nov | `real_clamp9`, `real_clamp6` | Walked on the real robot in a harness | measured on board |
| 24 Nov | `real_walk` | Drives at their 18 A limit: walked 4-12 s, leaning 12-15 deg forward; the 24 V 12 A supply dropped out twice. With a 6 N m target clamp on the Radxa: more than 20 deg forward within 0.8 s, stopped | measured on board |
| 25 Nov | `real_clamp12` | Isaac Lab, `Velocity-ASMR-Biped-RealClamp12-v0` with all training randomization, vx 0.2 m/s, 256 robots x 10 s: no fall | tested in simulation |
| 24-25 Nov | all three | MuJoCo with the real-robot effects: no fall in 24 tracking runs and no tilt over 20 deg in 50 real-test runs each (tables below) | tested in simulation |

## What the environment models

The tasks `Velocity-ASMR-Biped-Real-v0` and `Velocity-ASMR-Biped-RealClamp{6,9,12}-v0` (`packages/.../config/asmr_biped_real*`) rebuild the simulation from measurements on the real robot. The robot model is `source/robot_models/robots/asmr2026_real.py`.

- **Drives as on the robot.** The course motor driver gains are an explicit PD computed every 2 ms physics step: hip and knee Kp 100.6 / Kd 7.99, ankle Kp 71.6 / Kd 3.58, randomized +-10 %. The joint friction model was measured on our actuators: Coulomb 0.36 N m, more at low speed, and more with load. The armature range was measured: hip and knee 0.018-0.034, ankle 0.0056-0.014 kg m^2.
- **What the robot software sees.**
  - Encoder zero offsets +-2 deg and gear backlash 0-1 deg.
  - Delays: command 2-6 ms, encoders 0-6 ms, IMU 0-6 ms.
  - IMU (BMI088) mounting tilt +-2 deg and gyro bias +-0.02 rad/s.
  - Joint position noise +-0.01 rad.
- **The weighed link masses** (13.0 kg in total, the torso with everything mounted).
- **Randomized body and floor:**
  - Torso mass +-1.5 kg, leg links x0.8-1.2.
  - Torso centre of mass along x: -2 to +8 cm in the clamp tasks (-2 to +6 cm at 6 N m), +-3 cm in `Real-v0`.
  - Floor friction 0.4-1.2.
- **Policy timing and pose.** The policy runs at 50 Hz. The standing pose is BHL-like (knee 0.4 rad). It is also the pose the robot holds before the policy starts.
- **The Radxa torque clamp** (`mdp/torque_clamp.py`, clamp tasks only).
  - The drives keep their 18 A current limit (18.1 N m at the hip and knee, 22.6 N m at the ankle), because the course firmware cannot change it over CAN.
  - The Radxa clips the targets once per policy step instead, with the encoder readings the policy saw and the nominal gains:
    ```
    q_sent = clip(q_target, q + (-L + Kd qd) / Kp, q + (L + Kd qd) / Kp)
    ```
  - The training clips the same way, with L = 6, 9 or 12 N m.
  - A light penalty (-0.001 per N m) acts on the torque the targets ask beyond L.
  - The policy observes its own unclipped action, as on the robot.

## How the policies were trained

| Policy | Training | Iterations |
|---|---|---|
| `real_walk` | `Velocity-ASMR-Biped-Real-v0` from scratch (23 Nov) | 6000 |
| `real_clamp12` | `…-RealClamp12-v0`, fine-tuned from `real_walk` (model_5999) | +4000 (model_9998) |
| `real_clamp9` | `…-RealClamp9-v0`, fine-tuned from `real_walk` | +4000 (model_9998) |
| `real_clamp6` | `…-RealClamp6-v0`, fine-tuned from `real_clamp9` | +100 of 2000 (model_10100) |

- At 6 N m the policy did not learn to step, either from scratch or fine-tuned from `real_walk` (episodes stayed near 300 steps). Fine-tuning from `real_clamp9` worked. Its checkpoint at iteration 10100 passed the MuJoCo checks. The last checkpoint fell in 6 of 24 runs (fast forward, backward).
- A clamp penalty of -0.005 made the fine-tuning worse. The action noise alone asks beyond the limit when a joint works near it, so -0.001 is used.

Reproduce `real_clamp12` (one GPU, about 2 h 15 min per 4000 iterations on an RTX 4060):

```bash
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/train.py --task Velocity-ASMR-Biped-Real-v0 --headless   # or start from policies/real_walk
mkdir -p logs/rsl_rl/asmr_real_clamp12/warmstart && cp policies/real_walk/model_5999.pt logs/rsl_rl/asmr_real_clamp12/warmstart/
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/train.py --task Velocity-ASMR-Biped-RealClamp12-v0 --headless \
    --resume True --load_run warmstart --checkpoint model_5999.pt --max_iterations 4000
uv run --project sim/locomotion sim/locomotion/scripts/rsl_rl/export_onnx.py \
    --checkpoint logs/rsl_rl/asmr_real_clamp12/<run>/model_9998.pt --name my_clamp12 --template real_clamp12 --action_limit 20
```

Do not pass `--payload`: these tasks have no payload, because the weighed torso already holds its parts.

## Simulation results

### MuJoCo tracking

The run uses the real-robot effects, observation noise and the clamp. There are 8 commands x 3 seeds. Commands: forward 0.3, fast 0.5, backward -0.3, sideways +-0.2, turning +-0.8. Command per policy:

```bash
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_tracking_mujoco.py --policy real_clamp12 --name real_clamp12
```

| Policy | Falls | Forward / fast / backward (m/s) | Sideways (m/s) | Turning (rad/s) |
|---|---|---|---|---|
| `real_clamp12` | 0/24 | 0.26 / 0.43 / -0.29 | +0.17 / -0.19 | +0.80 / -0.84 |
| `real_clamp9` | 0/24 | 0.26 / 0.41 / -0.32 | +0.18 / -0.19 | +0.65 / -0.59 |
| `real_clamp6` | 0/24 | 0.25 / 0.36 / -0.26 | +0.09 / -0.12 | +0.40 / -0.56 |

### MuJoCo real-test conditions

Each policy ran 5 variants x vx 0.1 and 0.2 m/s x 5 seeds, 10 s each:
- base
- torso CoM +4 cm
- torso CoM +6 cm
- CoM +4 cm, knee zero 2.5 deg and IMU 3 deg together
- IMU 3 deg and a 5 deg forward lean at the start together

```bash
uv run --project sim/locomotion sim/locomotion/scripts/sim2sim/eval_real_conditions_mujoco.py --policy real_clamp12 --variant com_x=4,knee_zero=2.5,imu_pitch=3
```

| Policy | Falls, tilt > 20 deg | IMU pitch while walking | First target off the standing pose | Mean torque, vx 0.1 / 0.2 |
|---|---|---|---|---|
| `real_clamp12` | 0/50, 0/50 | -0.4 to +4.7 deg | 7 deg | 1.2 / 2.4 N m |
| `real_clamp9` | 0/50, 0/50 | -3.1 to +1.3 deg | 13-16 deg | 2.5 / 3.3 N m |
| `real_clamp6` | 0/50, 0/50 | -2.4 to +2.5 deg | 13 deg | 2.1 / 2.8 N m |

- **Stop limits.** The clipped target minus the encoder angle reached 13-19 deg in the top 1 % and 19-27 deg at most. It stayed above 12 deg for up to 0.28 s. A tracking-error stop at 12 deg for 0.3 s can therefore trip in normal walking: use 30 deg for 0.5 s on the clipped target. The tilt stop is 30 deg; the training ends an episode at 45 deg.
- **Pushes while walking** (0.2 s on the torso, 3 seeds). `real_clamp12` and `real_clamp9` held a 40 N push forward (backward, 1 of 3 fell) and fell at 60 N. `real_clamp6` fell at 40 N. The harness is the real safety.

### What the simulation still does not show

On 24 Nov the real robot leaned forward much more than the simulation, about 12-15 deg against 4 deg. The joint targets the real robot received were replayed open loop in MuJoCo:
- The simulated robot leaned back after 0.4 s, while the real one kept leaning forward.
- The real ankle pitch moved only 30-50 % of the simulated amplitude and lagged by over 100 ms.

None of these reproduced the real lean in MuJoCo: an IMU tilt, a forward lean at the start, knee zero errors, a torso CoM up to 9 cm forward, low friction, a harness, or ankle zero errors. The forward CoM range of the clamp tasks comes from the ankle torque measured while walking: about 3.4 N m pressing the toes down, against 1.0 N m in the simulation and 2.6-3.1 N m with the CoM 6 cm forward.

## Running them on the robot

The real tests ran these policies with the 50 Hz Radxa loop of the actuator test GUI (SOBANG_MotorTest `MotorTestGUI/radxa/walk_policy.py`, with `real_walk.py` and `real_walk_loop.py` of the SOBANG locomotion repository). The deployment configs (`policies/<name>/policy.yaml`) hold what the loop needs.

- Rate: `policy_dt` 0.02 s (50 Hz).
- Joint order: `joints`.
- Standing pose: `default_joint_positions`.
- Actions: `action_scale` 0.25, limits +-20.
- Gains for the clamp: `joint_kp` and `joint_kd`.
- Clamp limit: `torque_clamp_nm`.

On the robot:
- Send the targets with vel_ff = 0.
- Leave the drives at 18 A.
- Clip with `torque_clamp_nm`.
- Use 30 deg for the tilt stop and 30 deg for 0.5 s for the tracking-error stop.

**`legs/run_policy.py` cannot run these policies as it is.** Its rate (25 Hz) and standing pose (the course pose, knee 0.95 rad) are fixed for `baseline`, and it has no target clamp.
