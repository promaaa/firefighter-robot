# Legs: bring-up kit for the 12 leg drives

This folder runs the 12 leg actuators of the Berkeley Humanoid Lite (BHL) over two CAN buses.
It has the joint map, the CAN driver, the zeroing, the checks for the first day, the IMU reader and the policy loop.
Each script also runs on 12 fake drives that move the MuJoCo biped, so you can try everything on a laptop.

**Status:** built, tested in simulation (`uv run pytest legs -q`: 6 tests pass on 2026-10-22). Nothing is measured on board yet.

## Words used here

- **Drive:** the XDrive MINI board on the back of each actuator. Its print says "MKS ODRIVE MINI V1.0".
- **Node id:** the CAN address of a drive (`axis0.config.can_node_id`).
- **Heartbeat:** the CAN message that each drive sends every 100 ms with its error and its state.
- **Zero pose:** all joints at 0 rad. The legs hang straight down, the knees are straight, the soles are flat, the toes point forward.
- **Default pose:** the stand pose of the policy. Hip pitch -0.2 rad, knee 0.4 rad, ankle pitch -0.3 rad, all others 0.
- **K and D:** the joint stiffness (N·m/rad) and damping (N·m·s/rad) of the drive position control.
- **Damping mode:** K = 0 and D = 2 N·m·s/rad. The legs fold slowly and do not hold a pose.

## Files

| File | What it does |
| --- | --- |
| `robot.py` | Joint map (**edit the `JOINTS` table here**), units, gains, zeroing, drive modes |
| `bringup.py` | Checks for the first day: `scan`, `vbus`, `settings`, `angles`, `calibrate`, `check`, `wiggle`, `hold` |
| `run_policy.py` | The walking policy at 25 Hz over CAN. `--udp` takes the walk command from `aim/run.py`. |
| `imu.py` | IM10A IMU reader, and its one-time setup |
| `odrive_can.py` | CAN messages of the drive firmware (ODrive v0.5.1), and a small bus class over python-can |
| `fake_bus.py` | 12 fake drives that move the MuJoCo biped. Each script takes `--fake`. |
| `test_legs.py` | The tests |
| `calibration.json` | Written by `bringup.py calibrate`. It belongs to one robot. |

## Try it without hardware

The fake drives need the MuJoCo models: do steps 1 to 3 of [robot/README.md](../robot/README.md), "Run it on a laptop", once.

```sh
uv run pytest legs -q
```

```sh
uv run python legs/run_policy.py --fake --seconds 10 --knee-max 1.4
```

```sh
uv run python legs/bringup.py --fake scan
```

The fake drives answer the same CAN messages on the in-process "virtual" bus of python-can. They need no sudo and no vcan.
They run the drive control (position, velocity, torque, torque limit) at 2 kHz, and the 1 s watchdog of the real drives.
The fake robot starts in the zero pose on a hoist.

## Joint map (`JOINTS` in `robot.py`)

| Joint | Node | Motor | Sign | When the angle grows | Calibration stop |
| --- | --- | --- | --- | --- | --- |
| left_hip_roll | 1 | 6512 | +1 | the foot swings to the robot's left | foot inward, -10° |
| left_hip_yaw | 3 | 6512 | -1 | the foot swings left, the toes turn right | toes inward, +33.75° |
| left_hip_pitch | 5 | 6512 | +1 | the foot swings backward | leg backward, +56.25° |
| left_knee_pitch | 7 | 6512 | +1 | the knee bends | knee fully straight, 0° |
| left_ankle_pitch | 11 | 5010 | +1 | the toes go down | toes up, -45° |
| left_ankle_roll | 13 | 5010 | -1 | the left edge of the sole goes up | outer edge of the sole down, -15° |
| right_hip_roll | 2 | 6512 | +1 | the foot swings to the robot's left | foot inward, +10° |
| right_hip_yaw | 4 | 6512 | -1 | the foot swings left, the toes turn right | toes inward, -33.75° |
| right_hip_pitch | 6 | 6512 | -1 | the foot swings backward | leg backward, +56.25° |
| right_knee_pitch | 8 | 6512 | -1 | the knee bends | knee fully straight, 0° |
| right_ankle_pitch | 12 | 5010 | -1 | the toes go down | toes up, -45° |
| right_ankle_roll | 14 | 5010 | -1 | the left edge of the sole goes up | outer edge of the sole down, +15° |

- The order is the policy order. The node ids are the ids of BHL and of the 2025 course code. The lab tapes "CAN 11" and "CAN 12" fit them.
- Motor 6512 (MAD M6C12) drives the hip and knee joints, and motor 5010 drives both ankle joints, as in BHL.
- The signs are start values: the opposite of the BHL `joint_axis_directions`, because the BHL gear ratio is -15. Step 15 checks them.
- The limits and the stops come from the BHL URDF and the BHL calibration script.
- Sources: [Berkeley-Humanoid-Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite/tree/984741a3623c93b0583ccfdc479f1f8b1c4d900e) and [Berkeley-Humanoid-Lite-Lowlevel](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite-Lowlevel/tree/652777cc7c49884e7cd7ddfada758dc1979bf627).

## Bring-up checklist

Do the steps in this order. Run each command from the repository root.

### A. Computer and CAN adapters (one time)

1. Flash the candleLight firmware on each CANable 2.0 Pro with the [CANable Updater](https://canable.io/updater/) (Google Chrome).
2. On the ROCK 3C, `robot/setup/install.sh` sets up both buses at each boot. It names them `can_left` and `can_right` by USB serial number ([robot/README.md](../robot/README.md), step 3).
3. On a laptop, start each bus by hand after each boot, at 500 kbit/s with a transmit queue of 100 frames:

```sh
sudo ip link set can0 up type can bitrate 500000
```

```sh
sudo ip link set can0 txqueuelen 100
```

Do the same for `can1`. The default queue holds 10 frames, and the code sends 12 frames on each bus in each cycle.

### B. Wiring (power off)

4. Connect the 6 drives of each leg in one CAN chain with the white JST cables. Put the CANable at one end of the chain.
5. With the battery off, measure the resistance between CAN H and CAN L of each chain. It must be 60 Ω ("Termination resistors").

### C. Drive settings (odrivetool over USB, one drive at a time)

6. When a drive is new, do the course configuration first (`xdrive_config.txt` of the course firmware, with the motor and encoder calibration).
7. Read the CAN id on the tape of each drive. Edit the node column of `JOINTS` in `legs/robot.py`. A drive without a tape gets the default id of its joint and a new tape.
8. Print the settings block of each drive, with a low joint torque limit of 2 N·m:

```sh
uv run python legs/bringup.py settings --torque-limit 2
```

9. Connect one drive by USB, start odrivetool, and paste the block of its joint. Do this for each of the 12 drives.
   odrivetool runs the block line by line. The last line reboots the drive. Here is the block of the left ankle pitch drive:

```
odrv0.axis0.config.can_node_id = 11
odrv0.axis1.config.can_node_id = 63
odrv0.axis1.config.can_heartbeat_rate_ms = 0
odrv0.can.set_baud_rate(500000)
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.controller.config.pos_gain = 10
odrv0.axis0.controller.config.vel_gain = 0.05585
odrv0.axis0.controller.config.vel_integrator_gain = 0
odrv0.axis0.motor.config.current_lim = 1.59
odrv0.axis0.config.watchdog_timeout = 1.0
odrv0.axis0.config.enable_watchdog = True
odrv0.save_configuration()
odrv0.reboot()
```

- The hip and knee drives (6512 motor) get `current_lim = 1.99` for the same 2 N·m.
- `axis1` does not exist on the board, but the firmware has it. Its heartbeat is off, so 12 boards do not send the same CAN id.
- The watchdog stops the motor when no CAN frame comes for 1 s.

### D. First power-on, calibration and zero check (the robot hangs from the gantry)

10. Hang the robot from the gantry with the chains on the two top handles. The feet must not touch the floor. Connect charged 6S packs.
11. List the drives on each bus. Expected: 6 drives on each bus, state 1 (idle), and "encoder replies: 300 of 300".

```sh
uv run python legs/bringup.py scan
```

12. When a node shows "NOT IN THE JOINT MAP", correct the node column of `JOINTS` (step 7). The error WATCHDOG_TIMER_EXPIRED on an idle drive is normal.
13. Read the bus voltage of each drive with `uv run python legs/bringup.py vbus`.
14. Show the joint angles live, with the motors off:

```sh
uv run python legs/bringup.py angles
```

15. Move each joint by hand in the direction "When the angle grows" of the joint map. Its angle must grow.
    - When the angle gets smaller, change the sign of that joint in `JOINTS`.
    - When another joint changes, the node id of that joint is wrong in `JOINTS`.
16. **CAUTION:** `bringup.py wiggle` moves one joint by ±6° (0.1 rad) for 2 s. Keep your hands and the cables away from the leg. Ctrl+C stops it.

```sh
uv run python legs/bringup.py wiggle left_hip_roll
```

17. Type `yes`. Do steps 16 and 17 for the other 11 joints (name or node id).
18. Start the one-time calibration. Push each joint by hand to its calibration stop (joint map, last column) and hold it there for 1 s.
    When all 12 joints were at their stops, press Ctrl+C. The program writes `legs/calibration.json`.

```sh
uv run python legs/bringup.py calibrate
```

19. Put the legs in the zero pose ("Zeroing"). Start the zero check, push each joint to its stop again, then press Ctrl+C.
    Each "stop error" must be smaller than 3°. A joint with "WRONG ZERO" needs step 19 again. Between 3° and 11°, do steps 18 and 19 again.

```sh
uv run python legs/bringup.py check
```

20. **CAUTION:** `bringup.py hold` moves all 12 joints to the default pose in 2 s. Ctrl+C puts the legs in damping mode, and the motors turn off 3 s later.
    Put the legs in the zero pose, start the hold test and type `yes`. Both legs must be mirror images. A joint 24° away from its mirror joint has a wrong zero.

```sh
uv run python legs/bringup.py hold
```

### E. IMU

21. Connect the IM10A to UART4 of the ROCK 3C ([robot/README.md](../robot/README.md), "Header pins"). The port is `/dev/imu`.
22. Set the IMU once to 200 Hz, 230400 baud, gyro and quaternion output:

```sh
uv run python legs/imu.py --setup
```

23. Show the IMU values with `uv run python legs/imu.py`. Tilt the robot forward: "gravity x" must become positive. Tilt it to its left: "gravity y" must become positive.
24. When a sign is wrong, edit `IMU_TO_BODY` in `legs/imu.py`. Column k is the direction of the IMU axis k (X, Y, Z marks) in the torso frame.

### F. Policy

25. In odrivetool, set the policy torque limit of 6 N·m at the joint: `current_lim = 5.97` in each hip and knee drive, `4.78` in each ankle drive. Then `odrv0.save_configuration()`.
26. Hang the robot from the gantry and put the legs in the zero pose.
27. **CAUTION:** `run_policy.py` moves all joints to the default pose, then the policy moves the legs. Ctrl+C puts the legs in damping mode, and the motors turn off 3 s later.

```sh
uv run python legs/run_policy.py --seconds 20 --knee-max 1.4
```

28. When the program prints "Ramp to the default pose", lower the robot until it stands. Keep the chains on, but slack.
29. Press Enter. The policy starts with a zero velocity command, so the robot stands. Press Ctrl+C to stop.
30. When the robot stands well, walk slowly with `--vx 0.2`. The training range is -0.5 to 0.5 m/s.

- F2 needs `--knee-max 1.4`: at full knee range the foot touches a tank. `robot/launch.py` passes `[legs] knee_max` of `robot/config.toml`.
- The policy file is `[legs] policy` of `robot/config.toml`: the shipped BHL policy `sim/sim2sim/policy_biped_25hz_a.onnx`.
- Our policy `sim/rl/policies/f1_v1.onnx` was trained on the F1 model and evaluated on F2. It has the same inputs and outputs, so `[legs] policy` can point to it.
- f1_v1 has not run on the robot yet ([sim/rl/README.md](../sim/rl/README.md)).
- At the stop, `run_policy.py` prints the time of each policy tick. Each tick must stay below 40 ms.
- `run_policy.py` stops by itself on a drive error, a drive out of closed loop, no heartbeat for 0.5 s, no encoder reply 3 times,
  a joint more than 0.2 rad beyond its limits, or a torso tilt above 0.78 rad (the training limit).
- Each stop puts all drives in damping mode for 3 s. Then the motors turn off.

## Zeroing

The encoder (AS5047P) is on the motor side. It knows the angle inside one motor turn only, and one motor turn is 24° at the joint (15:1 gear).
So at power-on, the software knows each joint angle only modulo 24°.

- **One-time calibration (`bringup.py calibrate`):** you push each joint to its stop. The program stores the encoder angle at the zero pose in `legs/calibration.json`. Do it again only after you open an actuator, change a drive or move the encoder magnet.
- **Zeroing at each start (every script, automatically):** each joint must be within ±12° of the zero pose. The program takes the motor turn that puts the joint closest to the zero pose. A joint 13° away gets an angle 24° wrong. `test_legs.py` tests both cases.
- **Before each start:** let the legs hang straight down and straighten them by hand. Turn the feet so that the toes point forward. Lower the gantry until both soles rest flat on the floor, with the chains still carrying the weight.
- Gravity puts only the knee on a stop. In the model, a free ankle hangs 34° toes down, which is more than 12°.
- A wrong zero is always a multiple of 24°. `check` and `hold` find it, and `run_policy.py` stops at a joint more than 0.2 rad beyond its limits.
- A drive that reboots (for example after a power loss) loses its turn count. The code sees it leave closed loop and stops. Zero again.

## Drive gains

The firmware (v0.5.1) cannot change the gains or the current limit over CAN. You set them once in each drive with odrivetool (part C).
At the joint (gear ratio N = 15): K = N²/(2π) × pos_gain × vel_gain and D = N²/(2π) × vel_gain, with N²/(2π) = 35.81.
The joint torque limit is N × current_lim × torque_constant, with 0.067 N·m/A (6512) and 0.0836 N·m/A (5010) from the course config.

| | Course config, 6512 | Course config, 5010 | Policy (both motors) |
| --- | --- | --- | --- |
| `pos_gain` ((turn/s)/turn) | 12.6 | 20 | **10.0** |
| `vel_gain` (N·m/(turn/s)) | 0.223 | 0.1 | **0.05585** |
| K at the joint (N·m/rad) | 100.6 | 71.6 | **20.0** |
| D at the joint (N·m·s/rad) | 7.99 | 3.58 | **2.00** |
| `input_mode` | TRAP_TRAJ | TRAP_TRAJ | **PASSTHROUGH** |
| `current_lim` (A), 6512 / 5010 | 18 | 18 | **1.99 / 1.59** first tests (2 N·m), **5.97 / 4.78** policy (6 N·m) |

The policy needs PASSTHROUGH, so each target goes to the control at once. The code also sends PASSTHROUGH over CAN when it turns the motors on.

## Termination resistors

- A CAN bus needs exactly two 120 Ω resistors, one at each end of the chain. Each leg is one bus, so each leg needs two.
- With the battery off, measure between CAN H and CAN L at any connector. 60 Ω is correct. 120 Ω: one resistor is missing. 40 Ω or less: 3 or more resistors.
- Use one chain with short branches, not a star. A wrong termination gives lost frames ("encoder replies" below 300 of 300) or no frames.

## Undervoltage trip

- The course config trips the drives at 21.0 V. This is 3.5 V per cell of a 6S pack (full 25.2 V, nominal 22.2 V).
- Under load, a half-empty pack can drop below 21.0 V, and the last drive of the XT30 chain sees the largest drop. That joint then goes limp.
- `run_policy.py` sees the error and puts all drives in damping mode. At the start, it gives a warning below 22.0 V.
- Charge the packs before each test. Ask the course staff before you change the trip level.
- The overvoltage trip is 26 V. Braking energy from the motors can push a full pack above it.

## Not verified (needs the real hardware)

- The `gs_usb` driver on the ROCK 3C (the Radxa 6.1 kernel has `CONFIG_CAN_GS_USB=m`), and the mujoco and onnxruntime wheels on ARM.
- The node ids of the lab robot. Only "CAN 11" and "CAN 12" are known from photos. Which motor is on which joint.
- The joint signs (step 15), the direction of the encoder count, and the stops of the printed legs at the BHL angles.
- That the drives answer remote frames in time and do not lose frames in bursts of 12. Each drive has a receive buffer of 3 frames.
- `axis1.config.can_heartbeat_rate_ms = 0`, the 1 s watchdog, and the termination resistors on the drive and CANable boards.
- The real torque constants, so the real K, D and torque limit. A torque measurement at one joint would check them.
- The IM10A at 230400 baud and 200 Hz, its quaternion frame (z up), and the IMU mounting (`IMU_TO_BODY`).
- The policy on the real robot at 6 N·m, and the timing of the 25 Hz loop on the ROCK 3C.

Licence: code MIT (root `LICENSE`), this document CC BY-SA 4.0.
