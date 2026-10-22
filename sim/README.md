# Simulations

This folder holds the simulations of the final design F2: the robot model, walking, missions, the water jet and synthetic fire images.
All results here are **tested in simulation**. Nothing in this folder is measured on board.

## Setup (once)

Run these commands from the repository root.

1. Install the Python environment:
   ```
   uv sync
   ```
2. Fetch the Berkeley Humanoid Lite (BHL) meshes and URDF. They are not in git:
   ```
   scripts/fetch_bhl_assets.sh
   ```
3. Build the MuJoCo models of the robot. The XML files are not in git, and most simulations need them:
   ```
   uv run python sim/payload/models.py
   ```

## Folders

| Folder | What it simulates | Main command (from the repository root) |
|---|---|---|
| [payload/](payload/README.md) | The robot models with the F2 payload, and the payload check: statics, walking, pushes, drive model | `uv run python sim/payload/final_check.py` |
| [sim2sim/](sim2sim/README.md) | The shipped BHL walking policy loop (shared by `legs/`, `robot/` and the other folders), and the motor drive risks | `uv run python sim/sim2sim/hw_risks.py --check` |
| [mission/](mission/README.md) | Complete missions: walking, robot camera, fire detector, aiming, water droplets and fire model | `MUJOCO_GL=egl uv run python sim/mission/mission.py --seed 1055` |
| [rl/](rl/README.md) | Training and evaluation of walking policies. Its own uv project (JAX, GPU) | `uv run python sim/rl/f1_eval.py sim/rl/policies/f1_v1.onnx --quick` |
| [water/](water/README.md) | Pump, nozzle and jet range of the KLP02 water system | `uv run python sim/water/water.py` |
| [firescenes/](firescenes/README.md) | Synthetic fire images in Blender, for the detector training set | `blender -b --factory-startup -P sim/firescenes/firescenes.py -- samples 0 200` |

## Quick checks

| Check | Command | Time |
|---|---|---|
| Robot models | `uv run python sim/payload/models.py` | 5 s |
| Payload check | `uv run python sim/payload/final_check.py check` | 10 s |
| Policy loop | `uv run python sim/sim2sim/run.py --check` | 10 s |
| Drive model | `uv run python sim/sim2sim/hw_risks.py --check` | 20 s |
| Mission tests | `MUJOCO_GL=egl uv run pytest sim/mission/test_mission.py -q` | 10 s |
| Water model | `uv run python sim/water/water.py check` | 5 s |
| Policy evaluation | `uv run python sim/rl/f1_eval.py sim/rl/policies/f1_v1.onnx --quick` | 15 s |

## Key numbers

- F2 weighs 15.99 kg with full tanks. Its centre of mass is 0.448 m high (`payload/`).
- The shipped policy walks at 0.21 m/s for a 0.3 m/s command at 6 Nm, and at 0.28 m/s at 9 Nm (`payload/`).
- The KLP02 with the 1.0 mm nozzle gives 0.42 L/min and 10.0 m/s at 22.2 V. Both tanks last 1.8 min (`water/`).
- In 50 missions with real-fire flames, falls and an empty tank end most missions (`mission/`).

## Licence

Code: MIT (root `LICENSE`). READMEs and result files: CC BY-SA 4.0.
The BHL policy file is MIT (`sim2sim/BHL_LICENCE`). The BHL meshes and URDF are CC BY-SA 4.0 and stay out of git.
