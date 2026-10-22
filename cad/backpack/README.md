# Shared CAD helpers

`body.py` holds helpers that the current CAD uses: the path to the Berkeley Humanoid Lite (BHL) assets, `box()` and the print bed size.
It also holds the leg check `sweep()` with the knee limit of 1.4 rad.
`cad/final/layout.py`, `cad/final/kit.py` and `cad/torso_tank/body.py` import it.
Run alone, `body.py` builds the parts of the old backpack study into `out/` (`uv run --project cad python cad/backpack/body.py`). The repository does not keep them.

Licence: code MIT, CAD files CC BY-SA 4.0.
