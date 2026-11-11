# Contributing

This page tells each team member where to put their work and their results, so that everyone can find them.

## Where things go

| Workstream | Code | Results | Plan and test steps |
|---|---|---|---|
| Walking (policy, drives, IMU) | `legs/`, `sim/rl/`, `sim/sim2sim/` | `sim/rl/results/`, `sim/sim2sim/results/` | `legs/README.md` |
| Aim and water jet | `aim/`, `sim/water/` | `sim/water/results/` | `aim/README.md` |
| Fire detector and camera | `vision/` | `vision/results/` | `docs/plan/vision.md` |
| Robot programs on the ROCK 3C | `robot/`, `actuators/` | `robot/results/` | `robot/README.md` (bring-up) |
| Mission simulation | `sim/mission/` | `sim/mission/results/` | `sim/README.md` |
| Mechanics and wiring | `cad/`, `docs/hardware/` | `cad/final/renders/` | `docs/design.md`, `docs/hardware/` |

- One place for each setting: ports, devices, pins and limits are in `robot/config.toml`. Do not copy a value into the code.
- Large or generated files (datasets, checkpoints, logs, camera recordings) stay out of git. Put them in `~/runs/` or on the team drive and write their location in the README of the workstream.
- Images in the repository show the robot, plots or CAD only: no people and no screenshots of personal screens.

## Reporting a result

Write each result in the results table of the plan page of your workstream, with the date, the command that made it and its status. Use only these status words:

| Status | Meaning |
|---|---|
| planned | not done yet |
| built | the code or the part exists, not tested |
| tested in simulation | tested in MuJoCo or another simulator |
| measured on laptop | measured with real hardware on a laptop or desktop |
| measured on board | measured on the ROCK 3C on the robot |

A number without its status and its command does not go into a report.

## Workflow

1. Update `main`: `git switch main && git pull`.
2. Make a branch for your change: `git switch -c <workstream>-<short-name>`, for example `walking-gantry-test`.
3. Run the tests before you commit. They are the same as the CI checks (`.github/workflows/`):

```sh
uv run pytest
```

4. Write the commit message in the imperative ("Add the gantry test results"), with a short body that says what changed and why.
5. Push the branch and open a pull request. Merge it when the CI checks pass.

## On the robot board

See `robot/README.md`, "Bring-up on the robot". The board runs Radxa OS (Debian 12). `robot/setup/install.sh` installs everything, including the NPU runtime, and you can run it again at any time. Use a 5 V 3 A supply: on a computer USB port the board restarts under load.
