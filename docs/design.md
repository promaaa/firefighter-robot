# Final design F2

<p align="center">
  <img src="../cad/final/renders/f2_front.jpg" alt="Final design F2, front" width="320">
</p>

Sobang-I is the course Berkeley Humanoid Lite biped with a water system and a fire camera on the torso. Two printed water tanks sit on the side faces of the torso posts. A pan-tilt on top of the torso carries the nozzle and the camera.

Status: the CAD and the print kit are **built**. Walking, aiming and the mission are **tested in simulation**. Nothing is printed or measured yet.

| Item | Value | Details |
|---|---|---|
| Mass | 15.99 kg with full tanks, 15.21 kg empty (bare biped 11.34 kg) | [cad/final](../cad/final/README.md) |
| Legs | 12 cycloidal actuators on XDrive MINI drives, 2 CAN buses | [legs](../legs/README.md) |
| Computer | Radxa ROCK 3C (RK3566, NPU) | [robot](../robot/README.md) |
| Batteries | 2 × 6S 5000 mAh (motors), 1 × 3S 6000 mAh (computer, relays, servos) | [electrical.md](hardware/electrical.md) |
| Water | 2 × 0.39 L tanks (0.781 L), Kamoer KLP02 24 V pump with a DMC-101 controller, 1.0 mm nozzle | [water.md](hardware/water.md) |
| Jet | 10.0 m/s at the nozzle. Range at +15°: 5.3 m best estimate, 2.7 m low estimate. 1.8 min of spray per fill (tested in simulation). | [water.md](hardware/water.md) |
| Aiming | pan-tilt with two ST3215 servos, nozzle 0.9 m above the floor | [aim](../aim/README.md) |
| Camera | Arducam B0201, 640 × 480, about 97° horizontal view | [vision](../vision/README.md) |
| Fire detector | YOLO11n, 416 × 320, ONNX on the CPU or RKNN INT8 on the NPU | [vision](../vision/README.md) |
| Safety | e-stop cuts the drives, the pump and the servos. Software knee limit 1.4 rad. | [electrical.md](hardware/electrical.md) |

## Where the parts are

| Level | Parts |
|---|---|
| Top of the torso | deck, pan-tilt, head with the camera and the nozzle |
| Side faces | the two R2 tanks |
| Middle shelf | main relay K1, boost converter, e-stop contact blocks, IMU, pump and filter |
| Rear pod (outside the frame) | ROCK 3C, OR diode, shunt, relay K2, 3S pack, CAN adapters, 5 V buck, meters and switches |
| Bottom | the two 6S packs, the brake resistors on the front rail |

![Section views](../cad/final/out_f2/layout_views.png)

The print kit has 21 printed pieces (about 1.09 kg of PETG and 62 h of printing) and a 77-step assembly guide: [cad/final/kit](../cad/final/kit/README.md).

## Open design points

- **Pump holder:** the CAD holds the DWP-385 of the earlier plan. We will redraw the holder when the KLP02 arrives and is measured. The KLP02 adds about 0.12 kg.
- **Deck screws:** the 2 rear deck screws sit under the edge of the pan base. Fix this before you print the deck and the pan base.
- **Battery places:** our proposal. Check them with the TA.
- **E-stop bracket:** it must move 50 mm up, because the left tank covers its old place.
- **Air at the tee:** when the robot leans, one tank can send air to the pump. Bench test T8 checks it.
- **No place yet** in the CAD for the DMC-101, the opto-isolator and the servo adapter.
