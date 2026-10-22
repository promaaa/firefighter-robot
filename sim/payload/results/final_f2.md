# Final design F2: payload check (tested in simulation)

Model: the BHL biped with the parts of `cad/final/out_f2/layout.json`. The BHL battery (603 g) and PC (280 g) are taken out of the BHL torso mass. Policy: the shipped BHL policy `policy_biped_25hz_a.onnx`.

## Statics

| Model | Mass (kg) | Added (kg) | CoM x / y (mm) | CoM height (m) | Peak torque, stand (Nm) | Peak pitch torque, one leg (Nm) |
|---|---|---|---|---|---|---|
| bare biped | 11.34 | +0.00 | -2 / +0 | 0.362 | 1.5 | 4.3 |
| F2: course BOM parts, R2 tanks full | 15.99 | +4.64 | +1 / -0 | 0.448 | 2.5 | 6.6 |
| F2, R2 half | 15.60 | +4.25 | -0 / -0 | 0.443 | 2.4 | 6.3 |
| F2, R2 empty | 15.21 | +3.86 | -1 / -0 | 0.439 | 2.3 | 6.1 |

## Walking, 20 s with observation noise, PD in software as trained

| Model | Limit (Nm) | stand | vx 0.3 | vx 0.2 + yaw 0.5 |
|---|---|---|---|---|
| bare biped | 6 | vx +0.00, wz -0.00 | vx +0.29, wz -0.04 | vx +0.19, wz +0.50 |
| bare biped | 9 | vx +0.00, wz -0.00 | vx +0.30, wz -0.01 | vx +0.21, wz +0.53 |
| F2: course BOM parts, R2 tanks full | 6 | vx +0.00, wz -0.00 | vx +0.21, wz -0.04 | vx +0.11, wz +0.44 |
| F2: course BOM parts, R2 tanks full | 9 | vx +0.00, wz -0.00 | vx +0.28, wz -0.02 | vx +0.13, wz +0.47 |
| F2, R2 half | 6 | vx +0.00, wz -0.00 | vx +0.21, wz -0.03 | falls at 10.2 s |
| F2, R2 half | 9 | vx +0.00, wz -0.00 | vx +0.29, wz -0.02 | vx +0.15, wz +0.51 |
| F2, R2 empty | 6 | vx +0.00, wz -0.00 | vx +0.23, wz -0.04 | vx +0.12, wz +0.47 |
| F2, R2 empty | 9 | vx +0.00, wz -0.00 | vx +0.29, wz -0.02 | vx +0.12, wz +0.43 |

## Push test: largest push with no fall (N s, worst of 3 push times)

| Model | standing, forward | standing, back | standing, left | standing, right | walking 0.3 m/s, forward | walking 0.3 m/s, back | walking 0.3 m/s, left | walking 0.3 m/s, right |
|---|---|---|---|---|---|---|---|---|
| bare biped | 7.5 | 4.5 | 3.8 | 3.8 | 6.0 | 7.9 | 1.9 | 2.6 |
| F2: course BOM parts, R2 tanks full | 3.8 | 5.6 | 4.9 | 4.5 | 9.4 | 5.2 | 3.0 | 3.0 |
| F2, R2 half | 5.6 | 5.6 | 4.5 | 4.5 | 8.2 | 6.0 | 3.4 | 3.4 |
| F2, R2 empty | 9.8 | 5.6 | 4.5 | 4.5 | 8.6 | 6.4 | 3.4 | 3.0 |

## F2 with the drive model of `sim/sim2sim/hw_risks.py`

Each case: 4 commands x 10 seeds of 20 s. Drive power = mechanical + copper loss (R = 0.1 ohm, ASSUMED).

| Case | Falls | Mean vx at 0.3 m/s | Mean motor bus power, walking (W) | Lowest bus power (W) | Energy into the bus capacitors (J) |
|---|---|---|---|---|---|
| 1: baseline: K 20, D 2, 6 Nm (as trained) | 1 of 40 | 0.22 | 22 | -52 | 1.68 |
| 7: realistic: K 20, D 2, 6 Nm, ±2°, 10 ms command latency | 1 of 40 | 0.22 | 22 | -54 | 2.09 |
| 7b: realistic + 10 ms observation latency | 7 of 40 | 0.21 | 22 | -151 | 1.89 |

Motor battery time while walking (two 6S, 80% usable, case 7 power): about 7.9 h. **Not verified:** drive standby power, motor iron losses and the real phase resistance are not in this number. Measure the bus current on the robot.
