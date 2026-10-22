Model `sim/payload/models/bhl_F2.xml`: 15.99 kg, file time 2026-10-21 15:51.

### Commands on bhl_F2 (6 Nm, training noise, BHL start drop, 20 s)

5 seeds per test. Speeds and stepping metrics are means from 5 s to 20 s of the runs without a fall.
Air: share of time with a foot off the ground. Steps: touchdowns per second. Slip: mean speed of a foot on the ground. Drift: distance moved from 5 s to 20 s.

| Test | Policy | Falls | vx (m/s) | vy (m/s) | yaw rate (rad/s) | Drift (m) | Yaw drift (rad) | Air (%) | Steps (1/s) | Slip (m/s) | Max knee (rad) | Peak torque (Nm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.22 | 6.0 |
| stand | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.33 | 6.0 |
| vx 0.1 | shipped | 0 of 5 | +0.00 | -0.00 | -0.00 | 0.01 | -0.02 | 0 | 0.0 | 0.003 | 0.90 | 6.0 |
| vx 0.1 | ours | 0 of 5 | +0.01 | -0.00 | -0.00 | 0.22 | -0.07 | 5 | 0.7 | 0.008 | 0.87 | 6.0 |
| vx 0.2 | shipped | 2 of 5 | +0.12 | -0.01 | -0.03 | 1.84 | -0.38 | 34 | 2.8 | 0.042 | 1.46 | 6.0 |
| vx 0.2 | ours | 0 of 5 | +0.10 | +0.00 | +0.03 | 1.46 | +0.36 | 36 | 3.5 | 0.030 | 1.03 | 6.0 |
| vx 0.3 | shipped | 1 of 5 | +0.22 | -0.01 | -0.04 | 3.23 | -0.63 | 53 | 3.4 | 0.056 | 1.56 | 6.0 |
| vx 0.3 | ours | 0 of 5 | +0.15 | +0.00 | +0.03 | 2.14 | +0.38 | 49 | 3.8 | 0.038 | 1.17 | 6.0 |
| turn on the spot 0.3 | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.32 | 6.0 |
| turn on the spot 0.3 | ours | 0 of 5 | +0.00 | +0.00 | +0.03 | 0.02 | +0.42 | 3 | 0.5 | 0.006 | 0.69 | 6.0 |
| turn on the spot 0.5 | shipped | 0 of 5 | +0.00 | +0.00 | +0.02 | 0.01 | +0.24 | 0 | 0.0 | 0.007 | 0.69 | 6.0 |
| turn on the spot 0.5 | ours | 0 of 5 | +0.02 | +0.04 | +0.39 | 0.03 | +5.84 | 29 | 3.5 | 0.035 | 0.78 | 6.0 |
| turn on the spot 0.8 | shipped | 0 of 5 | -0.02 | +0.02 | +0.47 | 0.04 | +6.90 | 23 | 2.6 | 0.037 | 1.09 | 6.0 |
| turn on the spot 0.8 | ours | 0 of 5 | +0.03 | +0.06 | +0.69 | 0.09 | +10.27 | 47 | 4.6 | 0.042 | 0.93 | 6.0 |
| vx 0.2 + yaw 0.5 | shipped | 3 of 5 | +0.11 | +0.02 | +0.40 | 0.13 | +5.93 | 38 | 3.1 | 0.041 | 1.38 | 6.0 |
| vx 0.2 + yaw 0.5 | ours | 0 of 5 | +0.10 | +0.01 | +0.37 | 0.20 | +5.53 | 41 | 4.0 | 0.036 | 1.17 | 6.0 |
| stand, pan-tilt moving | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.22 | 6.0 |
| stand, pan-tilt moving | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.33 | 6.0 |
| stand, pan-tilt + recoil 0.3 N | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.21 | 6.0 |
| stand, pan-tilt + recoil 0.3 N | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.02 | +0.05 | 0 | 0.0 | 0.001 | 0.35 | 6.0 |
| stand, pan-tilt + recoil 1.3 N | shipped | 0 of 5 | -0.00 | -0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.002 | 0.23 | 6.0 |
| stand, pan-tilt + recoil 1.3 N | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.02 | +0.05 | 0 | 0.0 | 0.001 | 0.44 | 6.0 |
| empty tank: vx 0.3 | shipped | 0 of 5 | +0.23 | -0.01 | -0.05 | 3.35 | -0.74 | 53 | 3.2 | 0.057 | 1.43 | 6.0 |
| empty tank: vx 0.3 | ours | 0 of 5 | +0.16 | +0.00 | +0.03 | 2.32 | +0.41 | 54 | 3.7 | 0.036 | 1.16 | 6.0 |
| empty tank: turn on the spot 0.5 | shipped | 0 of 5 | +0.00 | +0.00 | +0.02 | 0.01 | +0.23 | 0 | 0.0 | 0.007 | 0.76 | 6.0 |
| empty tank: turn on the spot 0.5 | ours | 0 of 5 | +0.02 | +0.04 | +0.42 | 0.05 | +6.23 | 35 | 3.9 | 0.035 | 0.81 | 6.0 |

### Start walking from a stand (bhl_F2, 6 Nm)

Zero command for 2.0 to 3.75 s (by seed), then the walk command for 9.5 s. Distance: forward distance in these 9.5 s. Started: the distance is more than half of command x 9.5 s.

| Command (m/s) | Observation noise | Policy | Falls | Started | Distance, mean (min to max) (m) | Distance at the command (m) |
|---|---|---|---|---|---|---|
| 0.10 | training | shipped | 0 of 8 | 0 of 8 | -0.00 (-0.01 to 0.01) | 0.95 |
| 0.10 | training | ours | 0 of 8 | 0 of 8 | 0.05 (-0.07 to 0.15) | 0.95 |
| 0.15 | training | shipped | 2 of 8 | 0 of 8 | 0.24 (-0.02 to 0.37) | 1.43 |
| 0.15 | training | ours | 0 of 8 | 0 of 8 | 0.47 (0.31 to 0.60) | 1.43 |
| 0.20 | training | shipped | 4 of 8 | 2 of 8 | 0.96 (0.83 to 1.09) | 1.90 |
| 0.20 | training | ours | 0 of 8 | 0 of 8 | 0.79 (0.71 to 0.86) | 1.90 |
| 0.25 | training | shipped | 4 of 8 | 4 of 8 | 1.39 (1.33 to 1.45) | 2.38 |
| 0.25 | training | ours | 0 of 8 | 0 of 8 | 1.04 (0.99 to 1.10) | 2.38 |
| 0.10 | no | shipped | 0 of 8 | 0 of 8 | 0.01 (0.01 to 0.01) | 0.95 |
| 0.10 | no | ours | 0 of 8 | 0 of 8 | -0.01 (-0.01 to -0.01) | 0.95 |
| 0.15 | no | shipped | 0 of 8 | 0 of 8 | -0.01 (-0.01 to -0.01) | 1.43 |
| 0.15 | no | ours | 0 of 8 | 0 of 8 | -0.00 (-0.03 to 0.16) | 1.43 |
| 0.20 | no | shipped | 8 of 8 | 0 of 8 | - (- to -) | 1.90 |
| 0.20 | no | ours | 0 of 8 | 0 of 8 | 0.84 (0.84 to 0.85) | 1.90 |
| 0.25 | no | shipped | 5 of 8 | 2 of 8 | 1.21 (1.14 to 1.29) | 2.38 |
| 0.25 | no | ours | 0 of 8 | 0 of 8 | 1.13 (1.12 to 1.14) | 2.38 |

### Push test (`sim/payload/torso_tank.py`, bhl_F2, 6 Nm)

Largest push with no fall in the next 3 s (N s, 0.1 s on the torso, worst of 3 gait phases).

| Policy | standing, forward | standing, back | standing, left | standing, right | walking 0.3 m/s, forward | walking 0.3 m/s, back | walking 0.3 m/s, left | walking 0.3 m/s, right |
|---|---|---|---|---|---|---|---|---|
| shipped | 3.8 | 5.6 | 4.9 | 4.5 | 9.4 | 5.2 | 3.0 | 3.0 |
| ours | 12.0 | 3.4 | 5.6 | 7.5 | 8.6 | 4.5 | 4.1 | 5.2 |

### Drive cases of `sim/sim2sim/hw_risks.py` on bhl_F2

Each case: 4 commands (stand, vx 0.3, turn on the spot 0.5, vx 0.2 + yaw 0.5) x 10 seeds of 20 s. Push: walking at 0.3 m/s, 5 push times, worst / mean (N s).

| Case | Policy | Falls | vx at 0.3 | yaw rate at turn on the spot 0.5 | vx, yaw at 0.2 + 0.5 | Peak torque (Nm) | Action rate (rad) | Push back (N s) | Push left (N s) |
|---|---|---|---|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | shipped | 1 of 40 | 0.22 | 0.02 | 0.11, 0.43 | 6.0 | 0.127 | 6.0 / 8.6 | 3.0 / 4.5 |
| 1: baseline: K 20, D 2, 6 Nm (as trained) | ours | 0 of 40 | 0.15 | 0.39 | 0.10, 0.39 | 6.0 | 0.115 | 3.8 / 5.1 | 4.9 / 5.9 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | shipped | 1 of 40 | 0.22 | 0.03 | 0.12, 0.46 | 6.0 | 0.128 | 4.1 / 5.8 | 1.1 / 2.5 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | ours | 0 of 40 | 0.15 | 0.40 | 0.11, 0.41 | 6.0 | 0.115 | 3.8 / 5.0 | 3.8 / 5.6 |
| 7b: realistic + 10 ms observation latency | shipped | 7 of 40 | 0.21 | 0.08 | 0.12, 0.47 | 6.0 | 0.139 | 0.8 / 4.2 | 1.1 / 2.9 |
| 7b: realistic + 10 ms observation latency | ours | 0 of 40 | 0.15 | 0.41 | 0.12, 0.42 | 6.0 | 0.115 | 3.0 / 3.6 | 3.0 / 4.4 |
