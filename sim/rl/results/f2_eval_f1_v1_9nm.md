Model `sim/payload/models/bhl_F2.xml`: 15.99 kg, file time 2026-10-21 15:51.

### Commands on bhl_F2 (9 Nm, training noise, BHL start drop, 20 s)

5 seeds per test. Speeds and stepping metrics are means from 5 s to 20 s of the runs without a fall.
Air: share of time with a foot off the ground. Steps: touchdowns per second. Slip: mean speed of a foot on the ground. Drift: distance moved from 5 s to 20 s.

| Test | Policy | Falls | vx (m/s) | vy (m/s) | yaw rate (rad/s) | Drift (m) | Yaw drift (rad) | Air (%) | Steps (1/s) | Slip (m/s) | Max knee (rad) | Peak torque (Nm) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| stand | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.22 | 6.3 |
| stand | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.30 | 7.7 |
| vx 0.1 | shipped | 0 of 5 | +0.00 | -0.00 | -0.00 | 0.01 | -0.02 | 0 | 0.0 | 0.003 | 0.28 | 9.0 |
| vx 0.1 | ours | 0 of 5 | +0.02 | +0.00 | -0.00 | 0.31 | -0.03 | 9 | 1.0 | 0.009 | 0.86 | 9.0 |
| vx 0.2 | shipped | 0 of 5 | +0.11 | -0.01 | -0.03 | 1.64 | -0.43 | 34 | 2.4 | 0.031 | 1.30 | 9.0 |
| vx 0.2 | ours | 0 of 5 | +0.12 | -0.00 | +0.01 | 1.74 | +0.10 | 45 | 3.5 | 0.030 | 1.02 | 9.0 |
| vx 0.3 | shipped | 0 of 5 | +0.28 | -0.01 | -0.03 | 4.11 | -0.50 | 61 | 3.1 | 0.062 | 1.36 | 9.0 |
| vx 0.3 | ours | 0 of 5 | +0.19 | -0.00 | +0.01 | 2.80 | +0.10 | 60 | 3.6 | 0.041 | 1.12 | 9.0 |
| turn on the spot 0.3 | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.32 | 6.9 |
| turn on the spot 0.3 | ours | 0 of 5 | +0.00 | +0.00 | +0.03 | 0.02 | +0.43 | 3 | 0.5 | 0.006 | 0.69 | 9.0 |
| turn on the spot 0.5 | shipped | 0 of 5 | +0.00 | +0.00 | +0.01 | 0.01 | +0.19 | 0 | 0.0 | 0.006 | 0.70 | 9.0 |
| turn on the spot 0.5 | ours | 0 of 5 | +0.02 | +0.05 | +0.41 | 0.04 | +6.17 | 35 | 3.7 | 0.033 | 0.79 | 9.0 |
| turn on the spot 0.8 | shipped | 0 of 5 | -0.02 | +0.03 | +0.52 | 0.07 | +7.75 | 26 | 2.7 | 0.037 | 1.05 | 9.0 |
| turn on the spot 0.8 | ours | 0 of 5 | +0.03 | +0.08 | +0.75 | 0.05 | +11.17 | 57 | 4.6 | 0.039 | 0.90 | 9.0 |
| vx 0.2 + yaw 0.5 | shipped | 0 of 5 | +0.13 | +0.01 | +0.47 | 0.27 | +7.10 | 47 | 3.2 | 0.047 | 1.32 | 9.0 |
| vx 0.2 + yaw 0.5 | ours | 0 of 5 | +0.11 | +0.01 | +0.45 | 0.14 | +6.72 | 53 | 4.2 | 0.035 | 1.04 | 9.0 |
| stand, pan-tilt moving | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.22 | 6.3 |
| stand, pan-tilt moving | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.30 | 7.7 |
| stand, pan-tilt + recoil 0.3 N | shipped | 0 of 5 | +0.00 | +0.00 | -0.00 | 0.01 | -0.00 | 0 | 0.0 | 0.002 | 0.21 | 6.3 |
| stand, pan-tilt + recoil 0.3 N | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.01 | +0.05 | 0 | 0.0 | 0.001 | 0.31 | 7.9 |
| stand, pan-tilt + recoil 1.3 N | shipped | 0 of 5 | -0.00 | -0.00 | -0.00 | 0.01 | -0.01 | 0 | 0.0 | 0.002 | 0.23 | 6.3 |
| stand, pan-tilt + recoil 1.3 N | ours | 0 of 5 | -0.00 | +0.00 | +0.00 | 0.02 | +0.05 | 0 | 0.0 | 0.002 | 0.33 | 8.5 |
| empty tank: vx 0.3 | shipped | 0 of 5 | +0.29 | -0.02 | -0.04 | 4.29 | -0.59 | 64 | 3.0 | 0.061 | 1.34 | 9.0 |
| empty tank: vx 0.3 | ours | 0 of 5 | +0.19 | -0.00 | +0.01 | 2.89 | +0.09 | 63 | 3.6 | 0.040 | 1.14 | 9.0 |
| empty tank: turn on the spot 0.5 | shipped | 0 of 5 | +0.00 | +0.00 | +0.02 | 0.01 | +0.23 | 0 | 0.0 | 0.007 | 0.72 | 9.0 |
| empty tank: turn on the spot 0.5 | ours | 0 of 5 | +0.02 | +0.05 | +0.44 | 0.03 | +6.53 | 41 | 4.0 | 0.033 | 0.82 | 9.0 |

### Start walking from a stand (bhl_F2, 9 Nm)

Zero command for 2.0 to 3.75 s (by seed), then the walk command for 9.5 s. Distance: forward distance in these 9.5 s. Started: the distance is more than half of command x 9.5 s.

| Command (m/s) | Observation noise | Policy | Falls | Started | Distance, mean (min to max) (m) | Distance at the command (m) |
|---|---|---|---|---|---|---|
| 0.10 | training | shipped | 0 of 8 | 0 of 8 | -0.00 (-0.01 to 0.01) | 0.95 |
| 0.10 | training | ours | 0 of 8 | 0 of 8 | 0.07 (-0.07 to 0.15) | 0.95 |
| 0.15 | training | shipped | 0 of 8 | 0 of 8 | 0.02 (-0.01 to 0.08) | 1.43 |
| 0.15 | training | ours | 0 of 8 | 0 of 8 | 0.54 (0.41 to 0.63) | 1.43 |
| 0.20 | training | shipped | 0 of 8 | 2 of 8 | 0.82 (0.55 to 1.11) | 1.90 |
| 0.20 | training | ours | 0 of 8 | 1 of 8 | 0.89 (0.79 to 1.00) | 1.90 |
| 0.25 | training | shipped | 0 of 8 | 8 of 8 | 1.80 (1.63 to 1.92) | 2.38 |
| 0.25 | training | ours | 0 of 8 | 6 of 8 | 1.30 (1.16 to 1.44) | 2.38 |
| 0.10 | no | shipped | 0 of 8 | 0 of 8 | 0.01 (0.01 to 0.01) | 0.95 |
| 0.10 | no | ours | 0 of 8 | 0 of 8 | -0.01 (-0.01 to -0.01) | 0.95 |
| 0.15 | no | shipped | 0 of 8 | 0 of 8 | -0.01 (-0.01 to -0.01) | 1.43 |
| 0.15 | no | ours | 0 of 8 | 0 of 8 | 0.00 (-0.02 to 0.03) | 1.43 |
| 0.20 | no | shipped | 0 of 8 | 0 of 8 | -0.02 (-0.02 to -0.02) | 1.90 |
| 0.20 | no | ours | 0 of 8 | 8 of 8 | 1.14 (1.11 to 1.17) | 1.90 |
| 0.25 | no | shipped | 0 of 8 | 8 of 8 | 1.95 (1.92 to 1.97) | 2.38 |
| 0.25 | no | ours | 0 of 8 | 8 of 8 | 1.50 (1.48 to 1.52) | 2.38 |

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
