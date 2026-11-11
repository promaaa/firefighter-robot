# ASMR2026 Humanoid v1

Robot model of the ASMR 2026 course humanoid: a 12-joint biped (6 joints per leg).

```
mjcf/
  asmr2026_humanoid.xml           MuJoCo model with position actuators and sensors
  scene.xml                       asmr2026_humanoid.xml on a ground plane
mjlab/
  asmr2026_humanoid_mjlab.xml     same geometry without actuators (mjlab adds them)
  scene.xml                       asmr2026_humanoid_mjlab.xml on a ground plane
  asmr2026_humanoid_constants.py  actuator parameters and mjlab robot config
meshes/                           visual meshes (STL)
```

## Quick start

```bash
pip install mujoco
python -m mujoco.viewer --mjcf=mjcf/scene.xml
```

## Frames and joints

- +X forward, +Y left, +Z up.
- Joints per leg, from hip to foot: `hip_roll`, `hip_yaw`, `hip_pitch`, `knee_pitch`, `ankle_pitch`, `ankle_roll`
  (named `leg_left_*_joint` and `leg_right_*_joint`).
- The IMU is the `imu` site. Foot soles are the `left_foot_collision` and `right_foot_collision` boxes.

## Actuators

| Joints | Actuator | Kp | Kd | Torque limit | Armature | Friction (static / viscous) |
|---|---|---|---|---|---|---|
| hip roll, hip yaw, hip pitch, knee pitch | 6512 | 90.01 | 5.730 | 18.09 N·m | 0.0228 kg·m² | 0.377 N·m / 0.090 N·m·s/rad |
| ankle pitch, ankle roll | 5010 | 21.95 | 0.489 | 22.57 N·m | 0.00556 kg·m² | 0.317 N·m / 0.030 N·m·s/rad |

These values were fitted on a single bench actuator of each type and may not match your robot.
Characterize your own actuators and update `mjlab/asmr2026_humanoid_constants.py`.

## Copyright

Copyright (c) 2026 Seunghoon Kang, Soft Robotics and Bionics Laboratory (SRBL), Department of Mechanical Engineering, Seoul National University. All rights reserved.

Provided for use in the 2026-2 Actuation and Sensing Mechanism Design for Robots course.
