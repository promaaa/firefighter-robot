"""Build the electrical drawings: the overview, sheets SOB-E-01 and SOB-E-02, and one PDF with both sheets.

    uv run --no-project --with matplotlib --with numpy python docs/hardware/schematic/build.py
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

import control  # noqa: E402
import overview  # noqa: E402
import power  # noqa: E402
from sheetlib import Sheet  # noqa: E402

zone = Sheet.zone
z_control = {"LEFT LEG": zone(*control.CAN_L), "RIGHT LEG": zone(*control.CAN_R), "rock_5v": zone(*control.P5V),
             "estop": zone(*control.ESTOP), "pump": zone(*control.PUMP), "servo": zone(*control.SERVO)}
z_power = {"plus5v": zone(*power.PLUS5V_FLAG), "estop": zone(*power.ESTOP_FLAGS), "pump": zone(*power.PUMP_FLAG),
           "act": zone(*power.ACT_FLAG)}

overview.draw(str(HERE / "overview"))
s1 = power.draw(str(HERE / "SOB-E-01_power"), z_control)
s2 = control.draw(str(HERE / "SOB-E-02_control"), z_power)
with PdfPages(HERE / "SOB-E_schematic.pdf", metadata={"Title": "SOBANG electrical drawings SOB-E-01, SOB-E-02",
                                                      "Author": "Team SOBANG", "Subject": "Rev B, 2026-10-22",
                                                      "CreationDate": None}) as pdf:
    pdf.savefig(s1.fig)
    pdf.savefig(s2.fig)
print("cross references:", z_control, z_power)
