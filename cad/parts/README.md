# Pan-tilt parts

This folder holds the build123d code of the ST3215 pan-tilt and its deck. Print the parts from `cad/final/kit/`, not from this folder.

- `top_module_sts3215.py` builds the ST3215 pan-tilt and writes `out_sts3215/` (STL files and `assembly.json`). `cad/final/tank.py`, `cad/final/render.py` and `sim/mission/world.py` read these files.
- `top_module.py` holds the shared parameters and helpers: deck, pan axis, payload hole pattern. `top_module_sts3215.py` and `cad/final/` import it. Run alone, it builds the first top module into `out/`, which the repository does not keep.
- `render_module.py` holds the Blender helpers of `cad/final/render.py`.
- Rebuild `out_sts3215/` (10 s): `uv run --project cad python cad/parts/top_module_sts3215.py`

Licence: code MIT, CAD files CC BY-SA 4.0.
