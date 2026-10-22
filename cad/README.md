# CAD

This folder holds the parametric CAD of the firefighter robot, final design F2, on the Berkeley Humanoid Lite (BHL) legs.
The part code uses build123d (Python). The renders use Blender.

Status: the CAD and the print kit are **built**. The checks are **tested in simulation**: they run on the CAD geometry and the BHL meshes. Nothing is printed or measured.

## Folders and files

| Path | What it is |
|---|---|
| [`final/`](final/README.md) | Design F2: every part of the course bill of materials (BOM) in the course torso, the two R2 water tanks, the checks and the renders. Start here. |
| [`final/kit/`](final/kit/README.md) | The print kit for the hardware team: STL files, cut drawings, print sheet, hardware list and assembly steps |
| [`parts/`](parts/README.md) | Code of the ST3215 pan-tilt and the deck. `final/` uses it. |
| [`torso_tank/`](torso_tank/README.md) | Code of the R2 tank. `final/tank.py` uses it. |
| [`backpack/`](backpack/README.md) | Shared helpers: path to the BHL assets, print bed size, leg check |
| `bhl.py` | Blender helpers: place the BHL meshes from the URDF, add simple shapes, set up the studio, render |
| `pyproject.toml`, `uv.lock` | Python environment of the CAD code (build123d, trimesh, scipy) |

## Rebuild the print kit

Run the commands from the repository root. The first `uv run` installs the environment in `cad/.venv`.

1. When the `third_party/` folder is missing, fetch the BHL meshes and URDF:

```
scripts/fetch_bhl_assets.sh
```

2. Build the tanks and the deck (15 s):

```
uv run --project cad python cad/final/tank.py
```

3. Build the print kit and run its checks (30 s):

```
uv run --project cad --with matplotlib python cad/final/kit.py
```

4. Run the layout checks (3 min):

```
uv run --project cad python cad/final/layout.py
```

`kit.py` and `layout.py` stop with a list of failures when a check fails.
[`final/README.md`](final/README.md) gives the other steps (section drawing, renders), the checks and the results.

## Robot frame

All CAD files use the same frame: x forward, y left, z up, in mm, feet on the floor. The top of the torso frame is at z = 791 mm.
The STL files stack on the robot without a transform.

## Licence

Code: MIT. CAD files, drawings and renders: CC BY-SA 4.0.
The renders show the BHL meshes (CC BY-SA 4.0). Credit the Berkeley Humanoid Lite team when you share them.
`scripts/fetch_bhl_assets.sh` downloads the BHL meshes and URDF. They are not in this repository.
