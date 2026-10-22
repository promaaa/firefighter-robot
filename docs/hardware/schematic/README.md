# Electrical drawings SOB-E

| File | Content |
|---|---|
| `overview.svg` | System overview: one block per part group, power and signal paths |
| `SOB-E-01_power.svg` | Sheet 1, power distribution |
| `SOB-E-02_control.svg` | Sheet 2, control and communication |
| `SOB-E_schematic.pdf` | Sheets 1 and 2 in A3, for printing |

The design notes, the part list and the power-on tests are in [../electrical.md](../electrical.md). Every wire is in [../harness.csv](../harness.csv).

## Conventions

- Symbols follow IEC 60617. Contacts are drawn at rest: e-stop released, push-lock switches open, relays not energised.
- Reference designators: BT battery, F fuse, D diode, K relay, S switch, U module, A drive, M motor, R resistor, RB brake resistor, RV varistor, H lamp, P meter, RS shunt, J connector, X terminal block.
- Net names: `+24V_BAT`, `+24V_BUS` (after K1), `GND_M` (motor side), `+12V_3S`, `+12V_SW` (after S2), `+24V_CTRL`, `+12V_ACT` (after K2), `+5V_PC`, `GND_3S` (control side).
- An off-sheet flag shows the net name and its target as `sheet/zone`, for example `2/C6` = sheet 2, zone C6.
- `W01`, `W02` … are wire numbers in `harness.csv`. Thick lines are 10–12 AWG.
- Orange dashed outline: proposed, confirm before you build it.

## Edit and rebuild

The drawings are Python code. Each sheet is one file: `power.py` (sheet 1), `control.py` (sheet 2), `overview.py`. `sheetlib.py` draws the symbols, the frame and the title block. Coordinates are in mm on the A3 sheet, origin at the bottom left.

Rebuild all files from the repository root:

```
uv run --no-project --with matplotlib --with numpy python docs/hardware/schematic/build.py
```

The cross-references between the sheets are computed from the positions in `power.py` and `control.py`. Install the Inter font to get the same text widths. Without it, matplotlib uses another font.

When you change the design:

1. Change the sheet code and rebuild.
2. Update `harness.csv` and run `uv run python docs/hardware/power_calc.py check`.
3. Add a line to the revision table in `power.py` and `control.py` (`REVS`), and raise the revision letter in the `frame(...)` calls.

## Licence

The drawings are CC BY-SA 4.0. The Python files are MIT.
