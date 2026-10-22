# Hardware documents

| Document | Content |
|---|---|
| [electrical.md](electrical.md) | Power tree, brake resistors, pump power and control, fuses, e-stop chain, computer pins, grounds, power-on test procedure |
| [schematic/](schematic/README.md) | Drawings SOB-E: system overview, sheet 1 power distribution, sheet 2 control and communication (SVG and A3 PDF) |
| [harness.csv](harness.csv) | Every wire: ends, connectors, gauge, colour, length, fuse, current |
| [water.md](water.md) | Water system: tanks, pump, tubes, nozzle, jet range, spray time, bench tests |
| [order_list.md](order_list.md) | Parts to order that are not in the course BOM, with shops and prices |
| [power_calc.py](power_calc.py) | Power numbers from the simulation and the harness check. Output: [power_calc.md](power_calc.md) |

The mechanical design and the print kit are in [cad/final/](../../cad/final/README.md).

Check the wiring after every change:

```
uv run python docs/hardware/power_calc.py check
```
