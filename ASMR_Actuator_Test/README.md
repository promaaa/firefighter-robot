# ASMR Actuator Test

Bench tests, drive setup records and a sim-vs-real GUI for the 12 leg actuators. The drives are MKS XDrive MINI boards running the course ODrive 0.5.1 "instant-start" firmware. Everything here runs on a Windows laptop, over USB for setup and over two CANable adapters for CAN.

This folder is a copy of [GeunYoung-Heo/SOBANG_MotorTest](https://github.com/GeunYoung-Heo/SOBANG_MotorTest), which keeps the full history and the raw logs left out here (see [Not in this folder](#not-in-this-folder)). The step-by-step guides are in Korean:

- Drive setup guide: Notion [⚙️ 모터 세팅](https://app.notion.com/p/40f9f50be3fc800f87f4f70a8d9c05ec)
- Measurement results: Notion [📊 액추에이터 측정 결과](https://app.notion.com/p/40f9f50be3fc81b38370ca54d97d96ea)
- Script notes: [BasicTest/NOTES.md](BasicTest/NOTES.md)
- GUI guide: [MotorTestGUI/README.md](MotorTestGUI/README.md)

## Status

| Item | Status | Command | Date |
|---|---|---|---|
| Firmware, configuration and calibration of all 12 drives | measured on laptop | `python setup_driver.py --joint <joint>` (USB) | 2026-11-05 to 11-09 |
| Break-in of all 12 actuators, about 80 min each | measured on laptop | `python 08_break_in.py` | 2026-11-05 to 11-09 |
| Friction, stiffness, damping and load tests, hip roll (ID01, ID02) | measured on laptop | `07_friction_sweep.py`, `03_stiffness.py`, `09_damping.py` | 2026-11-05, 11-06 |
| CAN with all 12 drives, left leg on one CANable and right leg on the other | measured on laptop | `python motor_test_gui.py` | 2026-11-13 |
| Direction and zero of all 12 joints in MotorTestGUI; 11 joints move on command | measured on laptop | `python motor_test_gui.py` | 2026-11-13 |
| Left knee (node 7) moves after an encoder offset calibration over CAN | measured on laptop | `python can_diag.py --channel COM16 --node 7 --calibrate-offset` | 2026-11-13 |
| Sim-vs-real comparison in MotorTestGUI | planned | | |
| BasicTest and MotorTestGUI on the ROCK 3C | planned | | |

## Folder

| Path | What it holds |
|---|---|
| `BasicTest/` | Bench scripts `00`–`09` (environment check, backup, units and zero, stiffness, torque and friction, step response, CAN check, friction sweep, break-in, damping), `setup_driver.py`, `set_node_id.py`, `set_gains.py`, `check_wiring.py`, `can_diag.py` |
| `BasicTest/bench/` | Shared code: USB access with odrivetool, CAN Simple, motor parameters, joint table, simulated drive |
| `BasicTest/data/IDxx/` | One folder per drive (`IDxx` = node xx): setup backups `xdrive_IDxx_after_setup_*.json`, measurement logs and plots |
| `BasicTest/analysis/` | Break-in report of all 12 actuators: script, figures, tables |
| `MotorTestGUI/` | PySide6 + MuJoCo GUI. Left: the real robot from CAN encoder readings. Center: MuJoCo with the same target trajectory. Right: joint jog with guards, plots, direction and zero setting. One or two CAN buses. |

## Drive setup record

These values come from the latest `BasicTest/data/IDxx/xdrive_IDxx_after_setup_*.json` of each drive. The robot code's CAN commands cannot read them, so this is the "drive setup record" that steps 9 and 10 of the [main README](../README.md) ask for.

| Setting | Value on all 12 drives |
|---|---|
| Firmware | [course instant-start](https://github.com/omletkang/xdrive-mini-fw-instant-start/releases), based on ODrive 0.5.1 |
| CAN | CAN Simple, 500 kbps, heartbeat 100 ms (axis1 heartbeat off) |
| `motor.config.current_lim` | **18 A**, as in the course docs. This is neither the 2 N·m motor-test limit nor the 5.97 A / 4.78 A walking limits in the main README. |
| Gains, 6512 (hip, knee) | `pos_gain` 12.6, `vel_gain` 0.223, `vel_integrator_gain` 0. At the joint: Kp 100.6 N·m/rad, Kd 7.99 N·m·s/rad |
| Gains, 5010 (ankle) | `pos_gain` 20, `vel_gain` 0.1, `vel_integrator_gain` 0. At the joint: Kp 71.6 N·m/rad, Kd 3.58 N·m·s/rad |
| Input mode | passthrough (1) |
| At power-on | idle (`startup_closed_loop_control` False) |
| DC bus trips | undervoltage 10 V (set for the 12 V bench supply; the course value is 21 V), overvoltage 26 V |
| Brake resistor | none (0 Ω). On a 24 V supply this leaves about 2 V before the overvoltage trip, so a fast deceleration can trip it. |
| `motor.config.direction` | −1 on ID01, 02, 07, 08, 12 and +1 on the others. It sets the sign between Iq and joint torque. |

## Joints, nodes and buses

| Joint | Node | Motor | Bus | Joint | Node | Motor | Bus |
|---|---|---|---|---|---|---|---|
| left hip roll | 1 | 6512 | left | right hip roll | 2 | 6512 | right |
| left hip yaw | 3 | 6512 | left | right hip yaw | 4 | 6512 | right |
| left hip pitch | 5 | 6512 | left | right hip pitch | 6 | 6512 | right |
| left knee pitch | 7 | 6512 | left | right knee pitch | 8 | 6512 | right |
| left ankle pitch | 11 | 5010 | left | right ankle pitch | 12 | 5010 | right |
| left ankle roll | 13 | 5010 | left | right ankle roll | 14 | 5010 | right |

The ID01 and ID02 backups still show node 3 and 4. They were saved before the two hip roll drives were renumbered with `set_node_id.py` on 2026-11-09. The CAN check on 2026-11-13 found each of nodes 1–14 once, on the bus in the table.

## CAN wiring notes

Checked on 2026-11-13.

- The CANable V2.0 Pro is galvanically isolated (ADM3050E transceiver and B0505S DC/DC). Its screw terminal is L, H, G. Connecting G to the drive supply negative, once per bus, is recommended but not required. Do not use the GND pad of the SWD header: it is on the USB side.
- The XDrive MINI CAN connector has CAN H and CAN L only. The drives share ground through their power wires.
- On our drive CAN cables, **black is CAN H and red is CAN L**. With H and L swapped, no frame arrives at all.
- Termination: fit the 120 Ω jumper on the CANable and on the last drive of each bus only. With the power off, the resistance between L and H at the CANable terminal should then be about 60 Ω. Measured: all jumpers off, about 12 kΩ; one jumper at the foot drive, 120 Ω.

## Known issue: left knee encoder offset

The left knee (node 7) turns freely when idle and its angle reads correctly. A position command, however, only makes the motor twitch: Iq rises to about 17 A and the MotorTestGUI guard stops the joint.

The cause is the encoder offset stored in the drive. The drive uses it to commutate the motor, not to read the position, so a wrong offset leaves the angle reading correct but the motor without torque. It no longer matches the assembled actuator. The right knee (node 8) had the same problem after it was reassembled on 2026-11-09.

1. **Until the fix:** run an offset calibration over CAN after each power-on. The knee swings about ±14° at 5 A, so bend it to about 45° first and keep it clear.

   ```powershell
   python can_diag.py --channel COM16 --node 7 --calibrate-offset
   ```

   This was tested on 2026-11-13: after it, the knee followed commands. ODrive 0.5.1 cannot save configuration over CAN, so the new offset is lost at power-off.
2. **Fix:** recalibrate over USB and save. When the board is open, check that the encoder magnet is firmly fixed.

   ```powershell
   python setup_driver.py --joint left_knee_pitch --from-step 6
   ```

## Run on a laptop

This folder does not use the repository's uv environment. It uses its own Python 3.12 virtual environment on Windows (PowerShell):

```powershell
py -3.12 -m venv "$env:USERPROFILE\odrv051"
& "$env:USERPROFILE\odrv051\Scripts\Activate.ps1"
pip install -r ASMR_Actuator_Test\BasicTest\requirements.txt
pip install -r ASMR_Actuator_Test\MotorTestGUI\requirements.txt
```

USB access to a drive needs the libusb-win32 driver on "ODrive 3.6 Native Interface (Interface 2)". Install it with [Zadig](https://zadig.akeo.ie/).

Check CAN and start the GUI. Use the COM port of each CANable:

```powershell
cd ASMR_Actuator_Test\BasicTest
python 06_can_check.py scan --channel COM16
python 06_can_check.py scan --channel COM17
cd ..\MotorTestGUI
python motor_test_gui.py --virtual
python motor_test_gui.py
```

`--virtual` runs the GUI against a built-in virtual robot with no hardware. Without it, pick the adapter and ports 1 and 2 in the GUI and press Connect.

On first start, MotorTestGUI offers to download the course Robot Model v1 (MJCF and STL). The model is "All rights reserved", so it stays out of git; `fetch_model.py` downloads it again.

## Safety in MotorTestGUI

- Connecting never enables torque. It only sends read requests, and it sets idle any drive it finds in closed loop.
- Torque comes on only with the 토크 켜기 (torque on) button. Esc sets all drives idle.
- Guards: a joint is set idle if target and actual differ by more than 10° for 0.5 s, or if |Iq| stays above 8 A for 1 s. One command moves at most 30°. Once a joint is zeroed, targets stay 2° inside the model range.
- Hang the robot with the feet off the floor; the simulation has a fixed base.

## Relation to the rest of the repository

- `legs/bringup.py calibrate` writes `legs/calibration.json` for the walking policy. MotorTestGUI keeps its own zero in `MotorTestGUI/config/robot_config.json`, which is per laptop and stays out of git. Neither updates the other.
- The GUI simulation still uses the course MJCF gains (6512: Kp 90.0, Kd 5.73; 5010: Kp 21.95, Kd 0.489). The drives run the gains in the setup record. The team decision of 2026-11-06 is to keep the drive gains and match the simulation to them.

## Not in this folder

- Five raw stiffness logs larger than 1 MB (ID01, ID02, 2026-11-05 and 11-06). They are in [SOBANG_MotorTest](https://github.com/GeunYoung-Heo/SOBANG_MotorTest/tree/main/BasicTest/data).
- `BasicTest/tools/`: Zadig and the course firmware `.hex` and `.elf` (GPL-3.0). Download them from the links above.
- Zip archives that duplicate `BasicTest/data`.
