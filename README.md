# Firefighter Robot

A low-cost firefighter humanoid robot. The body reuses the legs and torso of the
open-source [Berkeley Humanoid Lite](https://arxiv.org/abs/2504.17249).

The goal is a robot that enters a hazardous area, finds a fire, moves toward it,
and puts it out with a mounted water nozzle or a small pump.

## Roadmap

1. Get the Berkeley-based body to move reliably.
2. Detect and localize fire with a lightweight onboard camera system (RGB, thermal, or both).
3. Estimate the fire position relative to the robot.
4. Navigate and aim toward the fire.
5. Trigger and control the extinguisher.
6. Increase autonomy and robustness step by step.

## Reference

```bibtex
@inproceedings{chi2025berkeleyhumanoidlite,
  author    = {Yufeng Chi and Qiayuan Liao and Junfeng Long and Xiaoyu Huang and Sophia Shao and Borivoje Nikolic and Zhongyu Li and Koushil Sreenath},
  title     = {Demonstrating Berkeley Humanoid Lite: An Open-source, Accessible, and Customizable 3D-printed Humanoid Robot},
  booktitle = {Proceedings of Robotics: Science and Systems},
  year      = {2025},
  doi       = {10.15607/RSS.2025.XXI.062}
}
```
