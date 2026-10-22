# Tank library

`body.py` holds the parametric R2 side tank. `cad/final/tank.py` imports it and builds the F2 tanks with its fixes. Print the tanks from `cad/final/kit/`, not from this folder.

- `render.py` holds the Blender helpers (materials, stills, turntable) that `cad/final/render.py` imports.
- `out/assembly.json` holds the masses and centres of the R1 and R2 tank models of the tank study. `render.py` reads it on import.
- Rebuild `out/`: `uv run --project cad python cad/torso_tank/body.py`

Licence: code MIT, CAD files CC BY-SA 4.0.
