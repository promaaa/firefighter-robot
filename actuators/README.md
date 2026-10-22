# Actuators: pan-tilt servos, pump and walking bridge

This folder connects `aim/run.py` to the hardware.
The actuator node drives the two pan-tilt servos and the pump on/off line, and sends the measured angles back.
The walking bridge passes the walk command of `aim/run.py` to the walking policy.
The servos are two Waveshare ST3215 (Feetech STS3215, 12 V) on a Waveshare Bus Servo Adapter (A).
The pump line is a ROCK 3C GPIO (`robot/pump.py`). Its pins are in [robot/README.md](../robot/README.md), "Header pins".

- **Not verified:** Nothing here has run on real servos yet.
- **Not verified:** The driver assumes that the servo adapter does not echo the bytes that it sends.

## Status

| Part | Status |
|---|---|
| `st3215.py`: servo protocol (ping, read, write, sync write, state, torque, set id) | built, tested in simulation with a fake serial port |
| `actuator_node.py --mock`: mock servos and a mock pump pin | built, tested in simulation (`robot/sil.py`) |
| `actuator_node.py` with the servos and the pump GPIO | built. On the robot: planned |
| `walk_bridge.py`: walk command into the policy | built, tested in simulation (`legs/run_policy.py --udp`) |
| Servo ids and calibration | planned |

## Files

- `actuator_node.py`: reads the aim commands, drives the servos and the pump line, and sends the measured angles back.
- `st3215.py`: the Feetech STS protocol over pyserial. Its command line lists the servos, changes an id and reads positions.
- `walk_bridge.py`: the class `AimGamepad`. It gives the policy `vx` and `yaw_rate` from `aim/run.py`.
- `test_actuators.py`: the tests.
- `selftest.py` and `pico_fw/main.py`: optional, not used on F2. A first-day check for an external Pico 2 relay board. On F2, use `robot/bench.py`.

## Run it without hardware

Run the tests:

```sh
uv run pytest actuators -q
```

Run the actuator node with the camera node and the aiming code, all on fakes (Ctrl+C stops it):

```sh
uv run python robot/launch.py --sim --bench
```

`uv run python actuators/walk_bridge.py` prints the command that the policy would get.

## Wiring

| From | To | Note |
|---|---|---|
| ROCK 3C USB 2.0 OTG port, switch at HOST | Bus Servo Adapter (A) USB | The adapter USB chip is a CH343P [W3]. It appears as `/dev/servo` ([robot/README.md](../robot/README.md), "Device names"). |
| 3S servo power, 10.5 to 12.6 V ([docs/hardware/electrical.md](../docs/hardware/electrical.md)) | adapter DC jack (5.5 × 2.1 mm) | Input 9 to 12.6 V. It must match the servo voltage [W2]. |
| Adapter jumper cap | position **B** | B = USB control. A = UART control [W2]. |
| Adapter servo connector | pan servo, then tilt servo (daisy chain) | Pan = id 1, tilt = id 2. |

## Set the servo ids

New servos have id 1 [W1]. Two servos with the same id on one bus do not work.

1. Connect only the tilt servo to the adapter.
2. List the servos on the bus. On a laptop, use the `/dev/serial/by-id/...` name of the adapter in place of `/dev/servo`.

```sh
uv run python actuators/st3215.py /dev/servo
```

3. **CAUTION:** The next command changes the id from 1 to 2 in the servo EPROM. You can change it back the same way.

```sh
uv run python actuators/st3215.py /dev/servo --set-id 1 2
```

4. Connect the pan servo too, and list the servos again. You must see id 1 and id 2.

## Calibration

The calibration is in `robot/config.toml` `[actuator.pan]` and `[actuator.tilt]`.
`zero` is the servo position (in steps, 4096 for one turn) at the angle 0. `direction` is 1 or -1.
Positive pan turns left, seen from above. Positive tilt lifts the nozzle.

1. **CAUTION:** The next command switches the servo torque off. The nozzle can fall forward, so hold it.

```sh
uv run python actuators/st3215.py /dev/servo --torque-off
```

2. Turn the pan by hand until the nozzle points straight forward. Write the id 1 position as the pan `zero`.
3. Tilt the nozzle until a spirit level on it shows level. Write the id 2 position as the tilt `zero`. Stop the tool with Ctrl+C.
4. Empty the tanks and start the node. In a second terminal, send pan = 0.2 rad for 3 s:

```sh
uv run python actuators/actuator_node.py
```

```sh
python3 -c "import json,socket,time; s=socket.socket(2,2); [(s.sendto(json.dumps(dict(pan=0.2,tilt=0.0,pump_on=False)).encode(),('127.0.0.1',5601)),time.sleep(0.1)) for _ in range(30)]"
```

5. When the nozzle turns right instead of left, set the pan `direction` to -1 and start the node again.
6. Do the same for the tilt with `tilt=0.2`. The nozzle must go up.
7. Set `min_deg` and `max_deg` so that the nozzle and the hose cannot hit the robot, the tanks or the cables.
   The F2 geometry allows pan -90 to 90 deg and tilt -40 to 45 deg. Then run `robot/bench.py pantilt` (robot/README.md, step 7).
8. After this, do the bore-sight of [aim/README.md](../aim/README.md), "Calibration".

`actuator_node.py --calib file.json` takes the same keys from a JSON file and overrides `config.toml`.

## Actuator node

- It runs at 50 Hz and uses the newest command (UDP 5601, [docs/architecture.md](../docs/architecture.md)).
- It holds the servos and switches the water off when no command arrives for 0.5 s.
- It moves the goal angles at `max_speed_dps` at most, and clips them to `min_deg` and `max_deg`.
- It switches the water on only when the command is fresh, the servo replies are good and the e-stop is not pressed.
- It updates the pump at each tick. `robot/pump.py` switches the pump off after 0.3 s without an update, and after 12 s on.
- Each tick, it sends `{"t", "pan", "tilt", "water", "estop", "volt"}` to UDP 5603, 5604 and 5606. `t` is `time.monotonic()` when it read the angles.
- `"water"` is the real pump output, after the watchdogs. `aim/run.py --feedback` uses `t` for its delay compensation.
- SIGTERM and Ctrl+C switch the pump off. The servos keep their torque and hold the last angles.

## Walking bridge

- `legs/run_policy.py --udp` uses `AimGamepad` with a small stand-in base class, without the BHL code.
- The velocity comes from UDP 5602: `vx` clipped to ±0.5 m/s, `yaw_rate` clipped to ±1 rad/s, no side speed.
- When no message arrives for 0.5 s, the velocity is zero, so the robot stands still.
- Start kick: when `vx` goes from 0 to forward, the bridge sends at least 0.45 m/s for 0.5 s (robot/README.md, "SIL findings").
- `robot/config.toml` `[walk]` sets these values.
- **Not verified:** `AimGamepad` can also replace `Se2Gamepad` of the BHL Lowlevel runtime. We have not run this:

```python
import berkeley_humanoid_lite_lowlevel.robot.humanoid as bhl_humanoid
from walk_bridge import AimGamepad
bhl_humanoid.Se2Gamepad = AimGamepad  # Humanoid() now creates AimGamepad
```

## Servo facts and sources

We read these sources on 2026-10-10. The tests check the packet examples of the manual (U2).

| Fact | Value | Source |
|---|---|---|
| Default baud rate, default id | 1 000 000 baud (register value 0), id 1 | W1, U1 |
| Packet | `FF FF ID LEN INSTR PARAMS CHECKSUM`, LEN = number of params + 2 | U2 |
| Checksum | `~(ID + LEN + INSTR + params) & 0xFF`. The status packet has ERROR in place of INSTR. | U2, U4 |
| Instructions | PING 0x01, READ 0x02, WRITE 0x03, SYNC_WRITE 0x83. Broadcast id 0xFE. | U5, U7, U2 |
| Byte order | little endian (low byte first) | U1, U4 |
| Sign | sign bit plus magnitude: bit 15 for position and speed, bit 10 for load | U3, U6 |
| Registers | id 5, baud 6, torque enable 40, acceleration 41, goal position 42, goal time 44, goal speed 46, lock 55 | U1, U3 |
| Registers | present position 56, speed 58, load 60, voltage 62, temperature 63 | U1, U3 |
| Units | 4096 steps per turn, voltage 0.1 V, temperature 1 °C, load 0.1 %, speed in steps/s | U1, W1 |
| Set the id | one servo on the bus: lock = 0, write the id, lock = 1 | W1, U2 |

- The manual (U2) calls 0x06 RESET. The official libraries call 0x06 RECOVERY and use 0x0A for RESET. The driver uses neither.
- The position offset register (31) uses bit 11 as its sign bit. The driver does not write it.
- The manual has a typo in one ping example (`FF FF FF 01 02 01 FB`). Its table gives the correct `FF FF 01 02 01 FB`.

Sources:

- U1: [Waveshare ST3215 memory table](https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls)
- U2: [Feetech communication protocol manual](https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN%28191218-0923%29.pdf)
- U3: [FTServo_Python sms_sts.py](https://raw.githubusercontent.com/ftservo/FTServo_Python/main/scservo_sdk/sms_sts.py)
- U4: [FTServo_Python protocol_packet_handler.py](https://raw.githubusercontent.com/ftservo/FTServo_Python/main/scservo_sdk/protocol_packet_handler.py)
- U5: [FTServo_Python scservo_def.py](https://raw.githubusercontent.com/ftservo/FTServo_Python/main/scservo_sdk/scservo_def.py)
- U6: [FTServo_Arduino SMS_STS.cpp](https://raw.githubusercontent.com/ftservo/FTServo_Arduino/main/src/SMS_STS.cpp)
- U7: [FTServo_Arduino INST.h](https://raw.githubusercontent.com/ftservo/FTServo_Arduino/main/src/INST.h)
- W1: [Waveshare ST3215 wiki](https://www.waveshare.com/wiki/ST3215_Servo)
- W2: [Waveshare Bus Servo Adapter (A) wiki](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A))
- W3: [Bus Servo Adapter (A) schematic](https://files.waveshare.com/wiki/Bus-Servo-Adapter-(A)/Bus%20Servo%20Adapter%20(A)-Sch.pdf)

## Open points

- No test on real servos, or in the BHL runtime.
- The node does not check the servo load or temperature in its loop. `st3215.py` can read them.
- The node does not check that the measured angle follows the goal, for example when the nozzle is blocked.

Licence: code MIT (root `LICENSE`), this document CC BY-SA 4.0.
