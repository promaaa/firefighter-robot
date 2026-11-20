# Measured joint model, 2026-11-20

Status: **measured on board**. The ROCK 3C sent position targets over CAN and read the joints back, on the real robot hanging in a harness. The goal is a simulation built from the real joints. The same values are in [real_actuator_20261120.json](real_actuator_20261120.json).

## Summary

1. Each joint behaves as a joint torque PD with the course drive gains. The step response is first order with a time constant equal to Kd/Kp: 79 ms for the hip and the knee, 50 ms for the ankle. It is the same for 4° and 8° steps, so the response is linear in this range.
2. The largest delay for walking is this PD lag, not the communication. For a 2 Hz knee target the joint reaches 0.69 of the amplitude, 53–58° late. A simulation with the same gains and the right inertia reproduces it by itself.
3. A command reaches the drive in about 0.5 ms, the current follows after about 2 ms, and the joint starts to move after about 4–6 ms. In the sine test the lag above the first-order PD is about 4.5 ms plus half the command period.
4. Friction (0.30–0.37 N·m) and the knee armature (about 0.022 kg·m²) are close to the course MJCF. The ankle armature may be larger than in the MJCF (0.007–0.014 against 0.0056). The hip armature could not be found with this test.
5. With two slcan CANables, reading 12 joints and sending 12 targets per tick works up to 150 Hz.

## Values for the simulation

| Item | Hip and knee (MAD M6C12, 6512) | Ankle (MAD 5010) | From |
|---|---|---|---|
| PD Kp [N·m/rad] | **100.6** | **71.6** | drive settings; checked with the step time constant and the current |
| PD Kd [N·m·s/rad] | **7.99** | **3.58** | same |
| Torque limit [N·m] | 18.09 | 22.57 | current limit 18 A × Kt × gear ratio 15 |
| frictionloss [N·m] | 0.30–0.35 | 0.37 | end error of the steps (course MJCF 0.377 / 0.317) |
| armature [kg·m²] | 0.022–0.034 (knee); hip not found | 0.007–0.014 | inertia fit (course MJCF 0.0228 / 0.00556) |
| Command delay [ms] | 0.5 (0.4–1) to the drive + about 1 for the current loop + policy time (not measured) | same | current command in the step test |
| Target between commands | held until the next command; average delay half the command period | same | sine test |
| Observation age [ms] | median 1.77, p95 3.37 (12 joints read together) | same | rtt test |
| Highest loop rate | 150 Hz (12 reads + 12 targets, slcan) | same | loop test |

```text
tau = clip( Kp*(q_target - q) + Kd*(v_ff - qdot), -tau_max, +tau_max )     v_ff = 0 when no velocity feed-forward is sent
time constant  Kd/Kp = 1/pos_gain   -> hip, knee 79.4 ms; ankle 50.0 ms
bandwidth      1/(2*pi*Kd/Kp)       -> hip, knee 2.0 Hz;  ankle 3.2 Hz
```

With `vel_integrator_gain` 0, the drive's position-P then velocity-P loop is exactly this PD (Kp = pos_gain·Kd). The input filter is off (input mode passthrough). The velocity limit (1200°/s at the joint) acts only above an error of about 95°.

## Setup

- Robot: ASMR humanoid (course Robot Model v1) in a harness. Only the joint under test is in closed loop; all other joints are idle.
- Computer: ROCK 3C, Debian 12, Python 3.11. The test program opens the CANables on the board, the same path the policy will use.
- CAN: two CANable V2.0 Pro with the slcan firmware, one per leg, 500 kbit/s. Angles and currents are read with RTR frames.
- Drives: MKS XDrive MINI, course ODrive 0.5.1 firmware, CAN Simple. AS5047P 14-bit encoder on the motor shaft, gear ratio 15.
- Guard: idle when the current stays above 8 A or the tracking error above 12° for 0.3 s. No test tripped it.

| Drive setting (axis0) | 6512 (hip, knee) | 5010 (ankle) |
|---|---|---|
| pos_gain [(turn/s)/turn] | 12.6 | 20.0 |
| vel_gain [N·m/(turn/s)] | 0.223 | 0.1 |
| vel_integrator_gain | 0 | 0 |
| torque_constant [N·m/A] | 0.067 | 0.0836 |
| current_lim [A] | 18 | 18 |
| vel_limit [motor turn/s] | 50 | 50 |
| current control bandwidth [rad/s] | 1000 | 1000 |
| encoder bandwidth [rad/s] | 3000 | 3000 |
| control mode / input mode | 3 position / 1 passthrough | 3 position / 1 passthrough |

Joint gains: Kd = vel_gain·15²/(2π), Kp = pos_gain·Kd. Check: right after an 8° knee step the Iq command was 13.1–13.8 A; 100.6 × 0.1396 rad ÷ (15 × 0.067) predicts 14.0 A.

## Communication and loop rate

| Measured on the ROCK 3C | Median [ms] | p95 [ms] | Max [ms] |
|---|---|---|---|
| One encoder request to its reply | 1.73 | 2.21 | 3.04 |
| 12 requests together to the last reply | 4.05 | 6.03 | 7.07 |
| Age of each reading when all 12 are in | 1.77 | 3.37 | 5.51 |

| Target rate [Hz] | Achieved [Hz] | Read 12, median [ms] | Read 12, p99 [ms] | Ticks with a missing reply | Works |
|---|---|---|---|---|---|
| 25 | 25.0 | 5.15 | 10.21 | 0.0 % | yes |
| 50 | 50.0 | 3.86 | 10.45 | 0.0 % | yes |
| 100 | 99.8 | 3.87 | 7.67 | 0.0 % | yes |
| 150 | 147.7 | 3.74 | 7.60 | 0.0 % | yes |
| 200 | 190.2 | 4.45 | 5.22 | 13.9 % | no |
| 300 | 189.3 | 4.44 | 5.35 | 10.3 % | no |
| 400 | 191.1 | 4.42 | 5.29 | 13.2 % | no |

Each tick reads 12 joints and sends 12 targets, 5 s per rate. Above 150 Hz the loop stays near 190 Hz and 10–14 % of the ticks miss a reply: each bus carries about 18 frames per tick, about 3.7 ms at 500 kbit/s. The board had occasional Python stalls of 5–9 ms.

## From command to motion

All three joints behaved the same. After each step command the test asked for the angle and the current together, so both were sampled every 1–2 ms.

| After the command | What happens |
|---|---|
| 0 ms | The ROCK 3C sends `Set_Input_Pos` |
| about 0.5 ms | The drive applies the new target: its Iq command changes. The first sample already shows it, so the true value is between 0.4 and 1 ms. |
| about 2 ms | The measured current follows the command (current loop) |
| about 4–6 ms | The joint has moved 0.1°. Most of this is inertia, which a physics simulation reproduces. |
| about 60 ms | Half of the final motion (hip, knee; ankle about 38 ms) |

## Step response

Each joint got 10 steps out and back (20 steps, 1 s each). "+/−" is the direction of the joint coordinate: for the left knee − bends, for the left hip pitch + moves the leg forward, for the left ankle − points the toes down. Values are medians of the 20 steps.

| Joint | Step | τ +/− [ms] | Kd/Kp [ms] | Dead time +/− [ms] | 0.1° motion [ms] | t50 [ms] | t90 [ms] | 5 % settle [ms] | Overshoot | Friction [N·m] | Peak current +/− [A] |
|---|---|---|---|---|---|---|---|---|---|---|---|
| knee pitch | 4° | 80.9 / 74.7 | 79.4 | 4.00 / 5.25 | 6.2 | 59 | 182 | 217 | 0.2 % | 0.32 | 4.5 / 4.7 |
| knee pitch | 8° | 77.8 / 79.3 | 79.4 | 4.25 / 4.25 | 3.9 | 60 | 185 | 237 | 0.1 % | 0.30 | 9.8 / 10.2 |
| hip pitch | 4° | 84.3 / 84.3 | 79.4 | 4.25 / 2.75 | 6.0 | 65 | 201 | 244 | 0.3 % | 0.30 | 4.5 / 4.0 |
| hip pitch | 8° | 77.8 / 80.9 | 79.4 | 7.00 / 4.50 | 4.0 | 65 | 181 | 221 | 0.2 % | 0.35 | 9.5 / 9.1 |
| ankle pitch | 4° | 42.6 / 48.1 | 50.0 | 5.75 / 4.00 | 4.2 | 38 | 110 | 136 | 0.2 % | 0.37 | 2.5 / 2.1 |
| ankle pitch | 8° | 47.1 / 47.1 | 50.0 | 4.25 / 4.25 | 3.8 | 39 | 112 | 155 | 0.1 % | 0.37 | 4.8 / 4.7 |

- τ and dead time: a first-order-plus-dead-time fit to the mean response of each direction, normalised by the final value. Fit error 0.3–1.3 %.
- t50, t90, settle and overshoot are relative to the final value.
- Friction torque = Kp × the mean end error against the target of both directions. Each step also starts short of its target, so the error measured from the start looks about twice as large.
- The one-sided part of the end error (gravity and similar) was small: 0.05–0.14°.

## Inertia and friction

The first 60 ms after each step were fitted with: the target changes 0.55 ms after the command, the PD torque passes a first-order current lag and drives an inertia J, and the Coulomb friction above acts on it. The link inertia comes from the course MJCF in the same pose (torso fixed, other joints free, armature removed). Armature = J − link inertia.

| Joint | Direction | Start to target | Fitted J [10⁻³ kg·m²] | MJCF link inertia | Armature | Course MJCF armature | Fit error (first 60 ms) |
|---|---|---|---|---|---|---|---|
| knee pitch | + | 3.7° | 31.8 | 9.6 | **22.3** | 22.8 | 0.56 % |
| knee pitch | - | 3.9° | 40.1 | 9.6 | **30.5** | 22.8 | 0.77 % |
| knee pitch | + | 7.8° | 31.8 | 9.6 | **22.2** | 22.8 | 0.42 % |
| knee pitch | - | 7.9° | 43.3 | 9.6 | **33.7** | 22.8 | 0.91 % |
| ankle pitch | + | 3.8° | 17.2 | 3.2 | **14.0** | 5.6 | 0.74 % |
| ankle pitch | - | 3.6° | 10.1 | 3.2 | **6.9** | 5.6 | 1.54 % |
| ankle pitch | + | 7.8° | 14.8 | 3.2 | **11.6** | 5.6 | 0.87 % |
| ankle pitch | - | 7.7° | 12.7 | 3.2 | **9.5** | 5.6 | 0.93 % |
| hip pitch | + | 3.9° | 54.5 | 59.1 | **-4.6** | 22.8 | 2.25 % |
| hip pitch | - | 3.8° | 29.5 | 59.1 | **-29.6** | 22.8 | 0.62 % |
| hip pitch | + | 7.9° | 63.5 | 59.0 | **4.5** | 22.8 | 2.34 % |
| hip pitch | - | 7.7° | 31.8 | 59.0 | **-27.2** | 22.8 | 0.98 % |

- Knee: extending (+) gives 0.022, as in the MJCF (0.0228). Bending (−) gives 0.031–0.034.
- Ankle: 0.007–0.014, larger than the MJCF. The fit is less stable.
- Hip: the armature comes out negative, so it is not usable. The torso in the harness and the idle knee and ankle move with the reaction. A test with the torso fixed is needed.
- The fitted current lag mostly sits at the lower grid limit (0.3 ms): these data cannot fully separate current lag and inertia.

## Sine response, left knee ±4°

Centre −6°, ±4°, 4 cycles at each of 0.5, 1, 2, 4 and 6 Hz. The targets were sent at 50 Hz or 150 Hz and held in between.

| Frequency | Amplitude ratio, 50 / 150 Hz commands | Phase lag, 50 / 150 Hz | First order (τ 79 ms) alone | Lag above first order, 50 / 150 Hz [ms] |
|---|---|---|---|---|
| 0.5 Hz | 0.93 / 0.94 | 19° / 18° | 0.97, 14° | 28.0 / 21.5 |
| 1 Hz | 0.86 / 0.86 | 33° / 31° | 0.89, 27° | 19.2 / 12.5 |
| 2 Hz | 0.69 / 0.69 | 58° / 53° | 0.71, 45° | 17.7 / 11.0 |
| 4 Hz | 0.41 / 0.42 | 84° / 75° | 0.45, 63° | 14.6 / 7.8 |
| 6 Hz | 0.29 / 0.30 | 101° / 88° | 0.32, 72° | 13.7 / 7.7 |

- From 2 to 6 Hz the lag above first order is 13.7–17.7 ms with 50 Hz commands and 7.7–11 ms with 150 Hz commands. This matches about 4.5 ms of dead time plus half the command period (10 ms or 3.3 ms).
- At 0.5–1 Hz another 12–28 ms remain, most likely friction at low speed.
- Command timing on the board during the sine: 50 Hz at most 22.7 ms between commands; 150 Hz, 99 % within 8.7 ms.

## Recommendations for the simulation

- **Actuator:** a joint torque PD with the Kp and Kd above. Training with an ideal position servo or other gains turns the 80 ms PD lag into a sim-to-real gap.
- **Physics step:** 2 ms or less. Inertia and Kd give a fast pole at J/Kd ≈ 4 ms.
- **Action delay:** policy time + 0.5–2 ms, and hold the target until the next command.
- **Observation delay:** 1–4 ms (median 1.8, p95 3.4).
- **Randomisation ranges:** time constant ±10 %; frictionloss 0.25–0.45 N·m; armature 0.018–0.034 (hip, knee) and 0.0056–0.014 kg·m² (ankle); action delay 0.5–3 ms; observation delay 1–5 ms.
- **Loop rate:** 50 Hz works. 100–150 Hz cuts the hold delay from 10 ms to 3.3–5 ms. 150 Hz is the limit with slcan.
- **Torque limit:** the 6 N·m training limit is a 3.4° error at Kp 100.6, but the drives give up to 18 N·m (about 14 N·m in the 8° step). Either clip the simulation at the drive limit (18.09 / 22.57 N·m) or lower `current_lim` on the drives.
- **Less lag:** sending the target velocity as well (`vel_ff` of `Set_Input_Pos`) removes most of the Kd lag. The policy action then becomes position and velocity. The MotorTestGUI walking test already sends `vel_ff`.

## Limits

- Left leg, three joints only, with 4° and 8° steps and a ±4° sine. No torque saturation, no velocity limit. Hip roll and yaw, ankle roll and the right leg were not measured.
- Robot in a harness with the other joints idle, so the inertia fit includes torso and neighbour motion.
- slcan firmware. The runtime in `robot/` expects candleLight + SocketCAN; after a firmware change, repeat the `rtt` and `loop` tests. The joint dynamics stay the same.
- Both knees rest about 2.5° beyond our zero towards extension; the CAD stop is 0°. Zero the knees at the extension stop, or set the simulation knee limit to match.
- Policy compute time, bus voltage and temperature were not measured.

## Commands and raw data

All commands run in `ASMR_Actuator_Test/MotorTestGUI` on the laptop. `latency/latency.py` copies `radxa/latency_test.py` to the ROCK 3C, runs it there, fetches the log and analyses it. Moving tests were run with `--yes` while a person stood at the power switch. See [MotorTestGUI/README.md](../MotorTestGUI/README.md), section 10 (Korean).

| Test | Command | Log folder |
|---|---|---|
| Request to reply, read 12 | `python latency/latency.py run rtt` | `20261120_123858_rtt` |
| Loop rate 25–400 Hz | `python latency/latency.py run loop` | `20261120_124021_loop` |
| Step, knee pitch 4° | `python latency/latency.py run step --joint left_knee_pitch --deg 4 --n 10 --tag 4deg_pair` | `20261120_125738_step_left_knee_pitch_4deg_pair` |
| Step, knee pitch 8° | `python latency/latency.py run step --joint left_knee_pitch --deg 8 --n 10 --tag 8deg_pair` | `20261120_125832_step_left_knee_pitch_8deg_pair` |
| Step, hip pitch 4° | `python latency/latency.py run step --joint left_hip_pitch --deg 4 --n 10 --tag 4deg` | `20261120_130026_step_left_hip_pitch_4deg` |
| Step, hip pitch 8° | `python latency/latency.py run step --joint left_hip_pitch --deg 8 --n 10 --tag 8deg` | `20261120_130100_step_left_hip_pitch_8deg` |
| Step, ankle pitch 4° | `python latency/latency.py run step --joint left_ankle_pitch --deg 4 --n 10 --tag 4deg` | `20261120_125914_step_left_ankle_pitch_4deg` |
| Step, ankle pitch 8° | `python latency/latency.py run step --joint left_ankle_pitch --deg 8 --n 10 --tag 8deg` | `20261120_125950_step_left_ankle_pitch_8deg` |
| Sine, knee, 50 Hz commands | `python latency/latency.py run sine --joint left_knee_pitch --deg 4 --hz 50 --tag 50hz` | `20261120_131139_sine_left_knee_pitch_50hz` |
| Sine, knee, 150 Hz commands | `python latency/latency.py run sine --joint left_knee_pitch --deg 4 --hz 150 --tag 150hz` | `20261120_131243_sine_left_knee_pitch_150hz` |
| Inertia fit | `python latency/fit_inertia.py <the six step folders>` | `model_fit.json` |

The raw logs (16 MB, mostly `samples.csv`) stay on the measuring laptop in `ASMR_Actuator_Test/MotorTestGUI/logs/latency/` and are not in git. Each folder holds `samples.csv` (request and reply times and values), `meta.json` (settings and pose), `summary.json` and plots. Three earlier knee step folders (`124403`, `125200`, `125251`) have a 7–12 ms gap in the angle samples after each command and are not used here.
