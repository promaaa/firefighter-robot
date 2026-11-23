# Walking on the real robot

Goal: the real robot walks 3 m to a fire and turns on the spot, with no fall. Standing only is not enough.
This page lists the likely causes of failure, the measurements that find them, and what we do about each.
Review it at every weekly meeting.

Status: **planned**. The numbers come from the simulation (**tested in simulation**), unless a line says otherwise.

- **Not verified:** The gap study (`sim/sim2sim/gap_study.py`) now runs the ASMR 2026 model with the F3 parts and the baseline policy (`policies/baseline`), but it has not been run again: the numbers in this file come from the old study on the BHL model (`archive/sim2sim_bhl/`). f3_v1 was trained on the BHL model.

## Why walking is hard for us

- Earlier course years did not make the robot walk (information from the team). We do not know why.
- The Berkeley Humanoid Lite (BHL) authors say that their released policy walks on the real biped, "but the performance is not very good" ([BHL issue #35](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite/issues/35)).
- Our robot is heavier than the BHL biped: 11.34 kg bare, 15.99 kg for F2 with full tanks, 14.60 kg for F3 with the trolley.
- A small difference between the simulation and the robot can make a policy fall. This is the "sim-to-real gap".

## What the simulation says

The gap study runs the shipped BHL policy and our policies f1_v1 and f3_v1 on F3 with one error at a time, then with several errors together ([gap_study.md](../../archive/sim2sim_bhl/gap_study.md)).
Each case has 40 runs of 20 s per policy, or 20 runs per policy in the combined cases.
The ranking below counts the shipped policy and f1_v1. f3_v1 trained with these errors (point 7).

1. **F3 helps.** The shipped policy walks at 0.25 m/s on F3 and 0.22 m/s on F2. The knee is at its 6 N·m limit 21 % of the time on F3, against 27 % on F2. With 40 ms of command latency, it falls in 2 of 40 runs on F3, against 13 of 40 on F2.
2. **Five errors cause most falls.** The realistic case R has the middle value of every error. R alone gives 5 falls in 40 runs for the shipped policy and none for f1_v1.

| Rank | Error | Extra falls, alone on F3 (of 80) | Extra falls in R (of 40) |
|---|---|---|---|
| 1 | The joints give 15 % less torque than the model (torque per amp) | +8 | +22 |
| 2 | The torso centre of mass is 2 cm behind the model | +1 | +21 |
| 3 | The hose pulls the robot back with a constant 3 N | +12 | +20 |
| 4 | The IMU is mounted 4° off | +11 | +17 |
| 5 | 40 ms of command latency | +2 | +17 |
| 6 | 20 ms of observation latency | +1 | +8 |
| 7 | Joint zero errors of ±2° | +1 | +4 |

3. **The mechanical details matter less than we thought.** Rotor inertia up to 3 times the training value, joint friction 0.5 N·m, 1° of gear play, foot friction 0.4 and 10 % heavier legs add 0 to 3 falls.
4. **A hose pull that changes is easy, a constant one is not.** A pull from 0 to 3 N that changes every 4 s adds 2 falls. A constant 3 N pull adds 12.
5. **With every error at its worst value (case P), every run falls.** With a 9 N·m torque limit, 77 of 80 runs still fall in P, but R drops from 5 to 1 fall in 40.
6. The common thread is the knee torque. In the worst R cases, the knee is at its limit 40 to 57 % of the time.
7. **Our new policy f3_v1 handles these errors in the simulation.** It starts from f1_v1 and trains with the ranges of the gap study ([archive/rl_f1](../../archive/rl_f1/README.md#f3-the-trolley-version-and-the-policy-f3_v1), policy in `policies/f3_v1`).
   - It does not fall in R. It falls once in the 240 runs of R with one value at its worst.
   - With four drive errors together (inertia, friction, torque, latency), it falls in 1 of 40 runs. The shipped policy falls in 29, f1_v1 in 21.
   - It walks at 0.20 m/s at a 0.3 m/s command. The shipped policy walks at 0.25 m/s.
   - It still falls in all 40 runs of P. P has gear play, which is not in the training.
   - **Not verified:** these cases use the same kinds of errors as its training. So they do not prove that it walks on the robot.
8. **From a quiet stand, the policies need a short forward command to start.** We ran f3_v1 through `legs/run_policy.py` on the fake drives of `legs/`.
   After 3 s of standing, a command of 0.25 or 0.3 m/s did not move the robot. Turning at 0.5 rad/s did not start either.
   With 0.45 m/s for 0.5 s first, it walked 2.2 m in 9 s, and it turned on the spot at 0.5 rad/s. The shipped policy and f1_v1 behave the same way.
   The walk bridge already sends this start command when a forward command follows a stand (`robot/config.toml`, `[walk]`).

## Likely causes and the test that finds each one

| Id | Cause | What the simulation says | Test on the robot |
|---|---|---|---|
| C1 | The drive settings differ from the training | With the course drive file (TRAP_TRAJ input mode, K about 100 N·m/rad), the shipped policy falls in 40 of 40 runs ([hw_risks](../../sim/sim2sim/results/hw_risks.md)) | Read back the settings of all 12 drives before the first stand |
| C2 | The joints give less torque than we think | Rank 1 above | Bench torque test of each actuator ([legs, part G](../../legs/README.md#g-joint-identification-for-the-simulation)) |
| C3 | The torso centre of mass is behind the model | Rank 2 above | Weigh every part, then check with two scales |
| C4 | The hose pulls the robot | Rank 3 above | Spring scale between the hose and the anchor while a teammate pushes the trolley |
| C5 | The IMU is mounted at an angle | Rank 4 above. 2° gives no extra falls. | `legs/imu.py --level` on a level table |
| C6 | Latency in the loop, the CAN replies or the IMU | Ranks 5 and 6 above | Log the tick time and the age of each reading on the ROCK 3C |
| C7 | A joint zero or a joint sign is wrong | Rank 7 above. A wrong zero of 24° gives violent motion. | Bring-up steps 15 to 25 ([legs](../../legs/README.md)) |
| C8 | Rotor inertia, gear friction, gear play, foot friction | 0 to 3 extra falls | Swing test with `legs/sysid.py`, foot tilt test |
| C9 | The bus voltage drops below 21 V or rises above 26 V | The drives trip, and the leg goes limp | Log the bus voltage. Use full packs and the brake resistors. |
| C10 | The policy cannot handle C2 to C9 together | Case P: every run falls, also with f3_v1 | Train with the measured values, then repeat the gap study |
| C11 | The policy does not start to step from a quiet stand | On the fake drives, no policy starts at 0.25 m/s. A start command of 0.45 m/s for 0.5 s fixes it. | Stage T5: start from a stand through the walk bridge |

## What we do

1. **Ask the teaching assistant (TA) about earlier years.** Which drive settings and which policy did they use? How far did they get: hanging, standing, stepping?
2. **Make the robot lighter.** The trolley takes 1.4 kg off the robot (F3, done). One 6S pack instead of two takes 0.6 kg more and halves the run time to about 2.5 h. It lowers the time at the knee limit only from 21 % to 19 %, so we keep it as an option.
3. **Get the torque right.** Measure the torque per amp of each actuator on the bench. Set `current_lim` so that each joint really gets 6 N·m. After a gearbox test at 9 N·m, consider a higher limit.
4. **Put the centre of mass where the model has it.** Weigh every part, build the model with the weighed masses, and check the robot on two scales. When the centre of mass is more than 1 cm off, move a battery or retrain.
5. **Keep the hose slack.** The breakaway clip lets go at 1.5 N. The boom tip stays behind and above the robot. The person who pushes keeps the robot 0.1 to 0.4 m ahead of the boom tip ([trolley.md](../hardware/trolley.md)).
6. **Mount the IMU level.** Correct a tilt above 1° in software.
7. **Keep the loop fast.** The command and observation latency together stay at 20 ms or less.
8. **Use the most robust policy, and train the next one for the measured robot.** Use f3_v1 first on the robot. Keep f1_v1 and the shipped policy as fallbacks.
   f3_v1 already trains with the ranges of the gap study: torque scale 0.8 to 1.2, IMU tilt up to 5°, hose pull up to 4 N, command latency up to 40 ms.
   After the measurements, train f3_v2 with the measured values. Add starts from a quiet, level stand to its training.
   Every new policy passes the gap study before it runs on the robot.
9. **Test in small stages and log everything.** See "Test stages" below. Compare each log with the same run in the simulation.
10. **Make the demo easy for the legs.** See "Demo plan" below.
11. **Protect the robot.** A gantry for the first stages, a person with a strap on the handles later, the e-stop at all times. The trolley cannot catch the robot.
12. **Decide early.** See "Decisions and dates" below.

## Measurements, in this order

| # | What | How | Pass, or where the result goes |
|---|---|---|---|
| 1 | Torque per amp | Bench torque test of each actuator: a lever, a kitchen scale, a known torque command ([legs, part G](../../legs/README.md#g-joint-identification-for-the-simulation)) | Set `current_lim` for a real 6 N·m. Torque scale range of the training. |
| 2 | Centre of mass of the torso | Weigh each part with a 1 g scale and fill [masses.csv](../../sim/payload/masses.csv). Run `uv run python sim/payload/measured.py`. Then stand the robot with its toes on one scale and its heels on another. | Within 1 cm of the model |
| 3 | Hose pull | A luggage scale between the hose and the anchor, while a teammate pushes the trolley 5 m behind a walking person | Mean pull below 1 N |
| 4 | IMU tilt | The torso on a level table: `uv run python legs/imu.py --level` | Under 1°, or a corrected `IMU_TO_BODY` |
| 5 | Latency | The tick log of `legs/run_policy.py` on the ROCK 3C | 20 ms or less in total |
| 6 | Joint zeros | `uv run python legs/bringup.py check` | ±1° or better |
| 7 | Drive settings | `uv run python legs/bringup.py settings`, then read each value back in odrivetool | PASSTHROUGH, K 20, D 2, 6 N·m |
| 8 | Rotor inertia and friction | Swing test of the knee and the ankle pitch with `legs/sysid.py` | Ranges of the training |
| 9 | Foot friction | Tilt a board with the demo floor material, a foot on it with a 2 kg load, until the foot slides. Friction = tan(angle). | 0.6 or more |

### Results

| Date | Measurement | Result | Status | Command |
|---|---|---|---|---|
| 2026-11-20 | 5, CAN part | On the ROCK 3C with two slcan CANables: encoder request to reply 1.73 ms (p95 2.2), all 12 joints 4.1 ms (p95 6.0), age of a reading 1.8 ms (p95 3.4). A command reaches the drive in about 0.5 ms. Reading 12 joints and sending 12 targets per tick works up to 150 Hz. The policy time is not included. | measured on board | `python latency/latency.py run rtt` and `run loop` in `ASMR_Actuator_Test/MotorTestGUI` |
| 2026-11-20 | 8, part | Left knee, hip pitch and ankle pitch with the course drive gains (passthrough, Kp 100.6 / 71.6 N·m/rad, no integrator): the step response is first order with a time constant of Kd/Kp (79 ms, 50 ms), so the drive acts as the joint PD with those gains. Friction 0.30–0.37 N·m. Armature 0.022–0.034 kg·m² (knee) and 0.007–0.014 (ankle); the hip could not be fitted. | measured on board | `python latency/latency.py run step --joint left_knee_pitch --deg 4`, then `python latency/fit_inertia.py <step folders>` |
| 2026-11-20 | Joint response | Left knee, ±4° sine target: at 2 Hz the joint reaches 0.69 of the amplitude, 53–58° late. The lag above the PD is about 4.5 ms plus half the command period. | measured on board | `python latency/latency.py run sine --joint left_knee_pitch --hz 50`, and `--hz 150` |
| 2026-11-23 | 4, part | The robot has the course BMI088 (Grove 6-Axis) on I2C3 of the ROCK 3C (pins 1, 3, 5, 6), not the IM10A of `robot/config.toml`. The board is turned 90° about z: the IMU axes along body x, y, z are +y, −x, +z (checked by tilting the robot). At rest: \|a\| 1.000 g, tilt drift under 0.03° in 10 s, gyro noise 0.2–0.3 °/s, Mahony filter at about 300 Hz (I2C at 100 kHz). The tilt on a level table is not measured yet. | measured on board | `python3 ~/motor_bridge/imu_reader.py --sensor bmi088 --bus 3 --axes +y -x +z` on the ROCK 3C (copied from `ASMR_Actuator_Test/MotorTestGUI/radxa`) |

Details, all runs and the values for a new simulation: [real_actuator_20261120.md](../../ASMR_Actuator_Test/results/real_actuator_20261120.md).

## Test stages

Each stage has a pass criterion. Do not start a stage before the stage above it passes.
Film each test, and keep the log (joint angles, targets, IMU, bus voltage, tick times).

| Stage | Test | Pass |
|---|---|---|
| T0 | Drive settings of all 12 drives, and the bench torque test | PASSTHROUGH, K 20, D 2, a real 6 N·m, velocity limit 50 turn/s |
| T1 | IMU level check and swing tests of the knees and the ankles | Tilt under 1°. The fit error of each swing test is under 1°. |
| T2 | Legs in the air on the gantry, policy on, zero command | Smooth joints, no shaking, every tick under 40 ms |
| T3 | Stand on the gantry, chains slack | 60 s with no fall (milestone M2) |
| T4 | On the gantry: `legs/run_policy.py --yaw 0.8`, then `--vx 0.25`, 10 s each | The feet lift, the robot turns and steps, no fall |
| T5 | Start from a stand through the walk bridge, then walk 2 m at 0.25 m/s. A person holds a slack strap on the top handles. | 5 of 5 runs start to step and do not fall |
| T6 | Walk 3 m with the trolley and the hose | 5 of 5 runs with no fall |

## Demo plan

- The robot starts 2 to 3 m from the fire and walks a short straight path. `aim/controller.py` commands 0.25 m/s.
- The walk bridge first sends 0.45 m/s for 0.5 s, so the robot starts to step from its stand.
- The robot turns while it walks slowly (at least 0.25 m/s), as `aim/controller.py` does. All three policies can turn this way. Only f1_v1 and f3_v1 turn on the spot.
- It stands still while it sprays. One bag gives 4.8 min of spray.
- The floor is flat, dry and not slippery. The trolley and the hose stay behind the robot.
- The person who pushes the trolley keeps the hose slack. A second person holds the e-stop.

## Decisions and dates

| Date | Check | If the check fails |
|---|---|---|
| 7 Nov | Measurements 1 to 4 done. The gap study is repeated with the measured values. | Use the CAD model and the wide training ranges |
| 21 Nov (M2) | T3 and T4 pass | Find the cause with C1 to C11. Ask the TA for help with the actuators. |
| 28 Nov | T5 passes | Choose the demo mode: A walk, or B stand and spray |
| 5 Dec (M3) | 3 of 5 missions succeed in the chosen mode | Demo in mode B or with Plan B |

- **Mode A, walk:** the robot walks to the fire, aims and sprays.
- **Mode B, stand and spray:** the robot stands at the start point. It turns with small steps or with the pan servo, and it sprays fires within the jet range.
- **Plan B, wheeled base:** the camera, the detector, the pan-tilt, the pump and the software go on a wheeled base ([plan](README.md#plan-b)).

## Not verified

- **Not verified:** all numbers on this page come from the simulation, except the Results table under "Measurements, in this order".
- **Not verified:** the gap study models gear play simply (a massless rotor) and applies the hose pull at the nozzle, backward only.
- **Not verified:** the f3_v1 training and the gap study used the F3 model of 29 October (14.58 kg). The model now reads the CAD layout (14.60 kg, centre of mass 0.3 mm further back). We did not run them again.
- **Not verified:** the training of f3_v1 has no gear play, no motor speed limit and no downward hose pull.
- **Assumed:** the drive firmware applies the PD control at the joint as in the simulation (K and D at the joint, no integrator). The step tests of 2026-11-20 (Results) support this for the left knee, hip pitch and ankle pitch.
