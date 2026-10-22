# Sim-to-real risks of the course motor drives

**Status:** tested in simulation (MuJoCo, CPU only). Nothing here is measured on board.

**Not verified:** The MuJoCo actuator model has no motor dynamics, no CAN jitter and no gearbox backlash.
The model has only a constant joint friction of 0.1 Nm and an armature of 0.005 kg m², from the BHL model.
We did not check the drive trip limits (last table) on an XDrive MINI. They come from the course config and the ODrive v0.5.1 defaults.

**Assumed:** Hip roll, hip yaw, hip pitch and knee use the 6512 motor. Ankle pitch and ankle roll use the 5010 motor.
Source: the BHL low-level config `robot_configuration.backup.json` (torque constant 0.0919 for the 150KV 6512, 0.1176 for the 110KV 5010).

## Setup

- Model: `bhl_F2` (`sim/payload/models/bhl_F2.xml`). This is the final design F2 with two full R2 tanks, 15.99 kg.
- Policy: the shipped BHL policy at 25 Hz, physics at 2 kHz, the training observation noise, and the BHL start with a 4 cm drop.
- `hw_risks.py` imports the `run.py` functions. With the baseline settings, it gives the same result as `run.py` `simulate()` (self-check).
- Commands: the 4 commands of `run.py`: stand, vx 0.3, yaw 0.5, and vx 0.2 + yaw 0.5. Each run lasts 20 s.
- Seeds: 10 seeds per command, so each case has 40 runs. The seed sets the sensor noise and the zero errors.
- Push test: the robot walks at 0.3 m/s. A 0.1 s push hits the torso, backward or to the left.
  A bisection finds the largest push without a fall in the next 3 s (resolution 0.375 N s).
- The 5 push seeds put the push at 3.0 to 3.4 s, so it hits 5 phases of one step. "Worst" is the smallest of the 5 values.
  A worst value of 0.0 means that the robot fell without a push before 6 s.
- Drive model: the drive does the PD. Torque = K × (target − measured position) − D × velocity, clipped to the joint limit.
- Course gains: pos_gain 12.6 and vel_gain 0.223 (6512) give K = 100.6 Nm/rad and D = 8.0 Nm s/rad.
  pos_gain 20 and vel_gain 0.1 (5010) give K = 71.6 Nm/rad and D = 3.6 Nm s/rad.
  current_lim 18 A gives 18.1 Nm (6512) and 22.6 Nm (5010) at the joint.
- TRAP_TRAJ: the drive moves its setpoint to each new target at 4.2 rad/s and 2.1 rad/s² at most (joint side).
  These are trap_traj vel_limit 10 turn/s and accel 5 turn/s², divided by the gear ratio 15.
- Zero error: each joint gets a random offset, uniform in ±1°, ±2° or ±4°. The policy and the drive both see the offset position.
- Command latency: the drive gets the new target L ms after the policy computes it.
  Observation latency: the policy sees the sensor values of 10 ms before.
- Extra cases: 2b tests TRAP_TRAJ, because section 4 of the course config sets it. 5d finds the latency limit.
  7b to 7d support the recommendation.
- 8 kHz check: the real drive runs at 8 kHz. A one-off run of cases 2 and 3 at 8 kHz physics agreed with 2 kHz.
  That run used an earlier payload model and is not in the script.
- Run time: 6.4 min on 8 CPU cores.

Metrics:

- Falls: runs with a torso tilt above 0.78 rad or a large height drop, as in `run.py`. Mean time to fall uses only the runs that fall.
- vx: mean forward speed from 5 s to 20 s. Only runs without a fall count.
- Mean abs torque and peak torque: all 12 joints, from 1 s (after the start drop) to the end or the fall.
- Clipped steps: the share of physics steps where at least one joint asks for more than its limit.
- Action rate: the RMS change of the joint target between two policy steps, in rad. A higher value means a less smooth gait.

![Falls, speed, torque and push per case](hw_risks.png)

## Results

### Case 1: baseline

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |

### Case 2: course default gains, course current limit

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 2: course gains, 18 A, PASSTHROUGH (course file run to the end) | 20 of 40 | 8.3 | 0.10 | -, - | 4.86 | 22.6 | 29 | 0.214 | 4.1 / 7.1 | 1.1 / 5.9 |
| 2b: course gains, 18 A, TRAP_TRAJ (course file section 4) | 40 of 40 | 1.6 | - | -, - | 5.03 | 22.6 | 71 | 0.249 | 0.0 / 0.0 | 0.0 / 0.0 |

### Case 3: course default gains, 6 Nm

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 3: course gains, 6 Nm | 21 of 40 | 8.5 | 0.23 | 0.14, 0.49 | 3.32 | 6.0 | 87 | 0.183 | 1.5 / 6.3 | 0.0 / 3.1 |

### Case 4: joint zero error (baseline gains)

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 4a: zero error ±1° | 1 of 40 | 17.5 | 0.21 | 0.11, 0.43 | 1.88 | 6.0 | 46 | 0.127 | 4.1 / 6.7 | 3.0 / 3.9 |
| 4b: zero error ±2° | 4 of 40 | 11.8 | 0.21 | 0.10, 0.43 | 1.92 | 6.0 | 45 | 0.133 | 3.0 / 6.5 | 1.5 / 3.5 |
| 4c: zero error ±4° | 10 of 40 | 5.9 | 0.21 | 0.11, 0.43 | 2.06 | 6.0 | 48 | 0.145 | 0.0 / 2.9 | 0.0 / 1.9 |

### Case 5: latency (baseline gains)

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 5a: command latency 10 ms | 1 of 40 | 9.3 | 0.22 | 0.12, 0.46 | 1.90 | 6.0 | 47 | 0.127 | 3.0 / 6.8 | 1.5 / 4.0 |
| 5b: command latency 20 ms | 2 of 40 | 8.3 | 0.22 | 0.13, 0.48 | 1.96 | 6.0 | 48 | 0.129 | 3.8 / 5.5 | 2.2 / 3.3 |
| 5c: command latency 40 ms | 13 of 40 | 10.5 | 0.22 | 0.14, 0.42 | 2.26 | 6.0 | 59 | 0.175 | 2.2 / 4.7 | 1.9 / 2.8 |
| 5d: command latency 60 ms (edge test) | 28 of 40 | 4.7 | - | -, - | 2.54 | 6.0 | 68 | 0.205 | 0.0 / 0.0 | 0.0 / 0.0 |
| 5e: observation latency 10 ms | 1 of 40 | 2.6 | 0.22 | 0.12, 0.43 | 1.94 | 6.0 | 47 | 0.130 | 2.6 / 6.8 | 2.6 / 4.4 |

### Case 6: torque limit (baseline gains)

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 6a: torque limit 4 Nm | 27 of 40 | 1.7 | - | -, - | 1.34 | 4.0 | 40 | 0.080 | 0.0 / 0.0 | 0.0 / 0.0 |
| 6b: torque limit 9 Nm | 0 of 40 | - | 0.28 | 0.14, 0.47 | 1.92 | 9.0 | 11 | 0.123 | 11.6 / 11.6 | 3.4 / 5.5 |

### Case 7: combined realistic cases

| Case | Falls | Mean time to fall (s) | vx at cmd 0.3 (m/s) | vx, yaw at cmd 0.2 + 0.5 | Mean abs torque (Nm) | Peak torque (Nm) | Clipped steps (%) | Action rate (rad) | Push back, worst / mean (N s) | Push left, worst / mean (N s) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 19.1 | 0.22 | 0.11, 0.43 | 1.89 | 6.0 | 46 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | 1 of 40 | 5.7 | 0.22 | 0.12, 0.46 | 1.93 | 6.0 | 47 | 0.128 | 4.1 / 5.8 | 1.1 / 2.5 |
| 7b: realistic + 10 ms observation latency | 7 of 40 | 7.9 | 0.21 | 0.12, 0.47 | 2.03 | 6.0 | 49 | 0.139 | 0.8 / 4.2 | 1.1 / 2.9 |
| 7c: course file (case 2) + ±2°, 10 ms command latency | 30 of 40 | 5.9 | - | -, - | 5.65 | 22.6 | 39 | 0.278 | 0.0 / 0.0 | 0.0 / 0.0 |
| 7d: 7b with a 9 Nm limit | 1 of 40 | 2.2 | 0.27 | 0.16, 0.51 | 2.04 | 9.0 | 15 | 0.129 | 0.0 / 8.4 | 0.0 / 2.5 |

### Drive limits that the simulation does not enforce

Above these values the drive stops with an error, and the joint goes limp. Overspeed: 25 rad/s at the joint. Regen: about 72 W per drive. Bus current: about 288 W per drive. Power is the mechanical power at one joint, without motor losses. The peaks include the runs that fall, up to the fall.

| Case | Peak joint speed (rad/s) | Peak power into one drive, regen (W) | Peak power from one drive (W) |
|---|---|---|---|
| 1 | 20.9 | 67 | 125 |
| 2 | 45.3 | 709 | 919 |
| 2b | 5.4 | 55 | 60 |
| 3 | 35.6 | 169 | 210 |
| 4a | 24.1 | 75 | 145 |
| 4b | 21.4 | 104 | 128 |
| 4c | 24.0 | 93 | 136 |
| 5a | 22.5 | 72 | 126 |
| 5b | 21.0 | 91 | 119 |
| 5c | 25.9 | 114 | 148 |
| 5d | 26.5 | 156 | 155 |
| 5e | 19.8 | 83 | 112 |
| 6a | 16.7 | 39 | 67 |
| 6b | 13.4 | 79 | 89 |
| 7 | 18.2 | 85 | 101 |
| 7b | 27.2 | 117 | 163 |
| 7c | 44.1 | 877 | 867 |
| 7d | 18.6 | 88 | 167 |

### All runs per command (falls, mean vx, mean yaw rate of the runs without a fall)

| Case | stand | vx 0.3 | yaw 0.5 | vx 0.2 + yaw 0.5 |
|---|---|---|---|---|
| 1 | 0 falls, vx -0.00, yaw -0.00 | 0 falls, vx +0.22, yaw -0.04 | 0 falls, vx +0.00, yaw +0.02 | 1 falls, vx +0.11, yaw +0.43 |
| 2 | 1 falls, vx -0.00, yaw -0.02 | 7 falls, vx +0.10, yaw -0.11 | 2 falls, vx +0.02, yaw +0.34 | 10 falls |
| 2b | 10 falls | 10 falls | 10 falls | 10 falls |
| 3 | 0 falls, vx +0.00, yaw -0.01 | 4 falls, vx +0.23, yaw -0.07 | 10 falls | 7 falls, vx +0.14, yaw +0.49 |
| 4a | 0 falls, vx -0.00, yaw -0.00 | 0 falls, vx +0.21, yaw -0.04 | 0 falls, vx +0.00, yaw +0.01 | 1 falls, vx +0.11, yaw +0.43 |
| 4b | 0 falls, vx -0.00, yaw -0.00 | 1 falls, vx +0.21, yaw -0.04 | 0 falls, vx +0.00, yaw +0.01 | 3 falls, vx +0.10, yaw +0.43 |
| 4c | 0 falls, vx -0.00, yaw -0.00 | 5 falls, vx +0.21, yaw -0.04 | 1 falls, vx -0.00, yaw +0.02 | 4 falls, vx +0.11, yaw +0.43 |
| 5a | 0 falls, vx -0.00, yaw -0.00 | 0 falls, vx +0.22, yaw -0.04 | 0 falls, vx +0.00, yaw +0.02 | 1 falls, vx +0.12, yaw +0.46 |
| 5b | 0 falls, vx -0.00, yaw -0.00 | 1 falls, vx +0.22, yaw -0.04 | 0 falls, vx -0.00, yaw +0.02 | 1 falls, vx +0.13, yaw +0.48 |
| 5c | 0 falls, vx -0.00, yaw -0.00 | 3 falls, vx +0.22, yaw -0.02 | 2 falls, vx -0.01, yaw +0.21 | 8 falls, vx +0.14, yaw +0.42 |
| 5d | 0 falls, vx -0.00, yaw -0.00 | 10 falls | 8 falls, vx -0.02, yaw +0.14 | 10 falls |
| 5e | 0 falls, vx -0.00, yaw -0.00 | 0 falls, vx +0.22, yaw -0.03 | 0 falls, vx +0.00, yaw +0.02 | 1 falls, vx +0.12, yaw +0.43 |
| 6a | 0 falls, vx -0.00, yaw -0.00 | 10 falls | 7 falls, vx +0.00, yaw +0.01 | 10 falls |
| 6b | 0 falls, vx -0.00, yaw -0.00 | 0 falls, vx +0.28, yaw -0.03 | 0 falls, vx +0.00, yaw +0.02 | 0 falls, vx +0.14, yaw +0.47 |
| 7 | 0 falls, vx -0.00, yaw -0.00 | 0 falls, vx +0.22, yaw -0.04 | 0 falls, vx -0.00, yaw +0.03 | 1 falls, vx +0.12, yaw +0.46 |
| 7b | 0 falls, vx -0.00, yaw -0.00 | 3 falls, vx +0.21, yaw -0.02 | 0 falls, vx -0.00, yaw +0.08 | 4 falls, vx +0.12, yaw +0.47 |
| 7c | 2 falls, vx -0.00, yaw -0.02 | 10 falls | 8 falls, vx +0.01, yaw +0.37 | 10 falls |
| 7d | 0 falls, vx -0.00, yaw -0.00 | 1 falls, vx +0.27, yaw -0.03 | 0 falls, vx -0.00, yaw +0.06 | 0 falls, vx +0.16, yaw +0.51 |

## Findings

1. **The course gains do not suit this policy.** With K ≈ 101 / 72 Nm/rad and 18 A, 20 of 40 runs fall.
   vx reaches only 0.10 of 0.3 m/s. The stiff joints and the policy oscillate: the action rate almost doubles (0.214 against 0.127 rad).
   The peak joint speed reaches 45 rad/s. The drive trips at about 25 rad/s (overspeed), so the real robot would lose joints.
   With a 6 Nm limit (case 3), the course gains still give 21 falls in 40 runs and clip a torque in 87 % of the steps.
2. **TRAP_TRAJ makes the robot fall at once.** All 40 runs fall, after 1.6 s on average.
   The course trajectory limits (2.1 rad/s² at the joint) are far too slow for new targets every 40 ms.
   The last section of the course file sets PASSTHROUGH and saves it. So the input mode depends on how far the course file was run.
3. **Keep the joint zero error within ±2°.** ±1° changes nothing (1 fall in 40 runs, as the baseline). ±2° gives 4 falls. ±4° gives 10 falls.
   ±4° also cuts the worst backward push from 6.0 to 0.0 N s.
   The BHL training randomizes the default joint position by ±0.05 rad (±2.9°). This probably explains why ±2° works and ±4° does not.
4. **Keep the latency at 20 ms or less.** A command latency of 10 ms changes nothing.
   20 ms gives 2 falls in 40 runs, 40 ms gives 13, and 60 ms gives 28.
   An observation latency of 10 ms alone gives 1 fall. The shipped policy saw no delay in training.
5. **4 Nm is too low. 6 Nm works. 9 Nm adds margin.** With 4 Nm, 27 of 40 runs fall.
   With 9 Nm, no run falls, only 11 % of the steps clip, and the mean backward push rises from 8.6 to 11.6 N s.
6. **Small errors add up.** The realistic case 7 (matched gains, ±2°, 10 ms command latency, 6 Nm) falls in 1 of 40 runs.
   With 10 ms observation latency as well (case 7b), 7 of 40 runs fall, and the worst backward push drops from 4.1 to 0.8 N s.
   The course file with the same errors (case 7c) falls in 30 of 40 runs.

## Recommendation

Status: tested in simulation. Set these values on all 12 leg drives:

| Drive setting (ODrive v0.5.1) | Value | Joint value |
|---|---|---|
| `controller.config.control_mode` | `CONTROL_MODE_POSITION_CONTROL` | - |
| `controller.config.input_mode` | `INPUT_MODE_PASSTHROUGH` (1). Never `TRAP_TRAJ` (5) or `POS_FILTER` (3) | the target goes straight to the PD |
| `controller.config.pos_gain` | 10.0 (turn/s)/turn | K = 20 Nm/rad, as trained |
| `controller.config.vel_gain` | 0.0559 Nm/(turn/s) | D = 2 Nm s/rad, as trained |
| `controller.config.vel_integrator_gain` | 0 | no integral term |
| `motor.config.current_lim` | 6.0 A (6512), 4.8 A (5010) | 6 Nm, as trained |

- The 6 Nm limit uses the course torque constants (0.067 and 0.0836 Nm/A). `motor.config.torque_lim` = 0.4 Nm (motor side) gives the same limit, if v0.5.1 has it.
- Start with 6 Nm. 9 Nm (9.0 A and 7.2 A) also worked in simulation and gave more push margin.
  With all realistic errors, 9 Nm gave 1 fall in 40 runs against 7 for 6 Nm (case 7d against 7b). Check the printed gearboxes before you use 9 Nm.
- Zero tolerance: aim for ±1° on every joint. Accept ±2° at most.
  A one-turn error of the motor encoder is 24° at the joint. The start-up check must detect it and stop.
- Latency budget: at most 20 ms from the encoder sample to the new target in the drive, and aim for 10 ms.
  Read the encoders right before each policy step. Send the targets right after it.
- One CAN frame takes about 0.25 ms at 500 kbps. 12 frames per bus per policy step (6 targets, 6 encoder replies) take about 3 ms.
  So the CAN bus alone fits the budget. The USB-CAN adapters and Linux can add more. Log the loop times on the robot.
- For more margin, retrain the policy with a random action delay of 0 to 20 ms. The shipped policy saw no delay in training.

**Not verified:** bench checks before the first walk:

- Regen: the bus will probably trip on overvoltage in normal walking. See the section "Regen and the bus diode" below.
- Run `dump_errors(odrv0)` on each drive after the first walks. Look for overspeed and regen errors.

## Regen and the bus diode

**Status:** tested in simulation.
**Not verified:** the diode layout, the bus capacitance and the motor phase resistance.
**Assumed:** the phase resistance is 0.1 Ω for both motors. We found no datasheet value for the MAD M6C12 150KV or the MAD 5010 110KV.

- The course BOM has a Schottky diode module (MBRT400100, dual, common cathode) in the battery line. It probably joins the two 6S batteries.
- So regen current cannot flow back into the batteries. All 12 drives share one 24 V bus, and only their capacitors can take the energy.
- Capacitance: 5 × 100 µF per XDrive MINI (from photos), so 6 mF for 12 drives.
- Input power of one drive = τ × ω + 1.5 × R × I², with I = τ / (15 × Kt). Kt is 0.067 Nm/A (6512) and 0.0836 Nm/A (5010).
- The bus power is the sum over the 12 drives. When it is negative, the capacitors charge above the battery voltage. The script tracks this energy at each step.
- Start voltage: 25.2 V, a full 6S battery. We ignore the diode drop. The course config trips at 26 V (overvoltage).
- Runs: cases 1 and 7, 4 commands × 10 seeds, 20 s each. Runs that fall count up to the fall.

| Case | Lowest drive input power (W) | Drive current at 24 V (A) | Runs with a drive below −3 A | Lowest bus power, 12 drives (W) | Longest negative bus power (ms) | Largest energy into the capacitors (J) | Peak bus voltage from 25.2 V (V) |
|---|---|---|---|---|---|---|---|
| 1 | -64 | -2.7 | 0 of 40 | -52 | 106 | 1.68 | 34.58 |
| 7 | -79 | -3.3 | 1 of 40 | -54 | 153 | 2.09 | 36.48 |

- One drive alone is at its limit. The worst drive reaches −3.3 A in case 7, against the −3 A trip (1 of 40 runs).
- The shared bus is the real problem. The 12 drives together return up to 2.1 J in one stretch of up to 153 ms.
- Only 0.12 J lifts 6 mF from 25.2 V to 26 V. So the drives would trip on overvoltage in normal walking.
- The guesses do not change this. A one-off check used R = 0.05 to 0.2 Ω, with and without 1.5 W standby power per drive.
  The peak stayed at 26.9 to 38.7 V. That check used an earlier payload model and is not in the script.
- The mean regen power is small: about 1.4 W at most in one run (`docs/hardware/power_calc.md`). So a small resistor can take it.
- The course config sets `brake_resistance` = 0, so the drives themselves do not burn regen energy.

**Recommendation:** add a brake chopper on the shared 24 V bus before the first walk: about 5 Ω, 10 to 25 W, switched on above about 25.5 V.
Raise `dc_max_negative_current` to about −5 A only after the brake chopper works.
