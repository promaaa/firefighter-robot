# Board speed test: Radxa ROCK3 Model C (aarch64, 4 CPUs), host rock-3c

Date 2026-11-11 02:50. CPU temperature at start: 53.1 °C. Load: (0.4775390625, 0.26904296875, 0.119140625).

Camera (/home/radxa/bench.mp4, 640 x 480): 30.0 fps

| Model and runtime | Median (ms) | 90th percentile (ms) | Detector alone (fps) | Camera + detector (fps) | 10 fps target |
|---|---|---|---|---|---|
| NPU int8 416x320 | 43.6 | 43.9 | 23 | 21.2 | PASS |
| CPU 416x320, 4 threads | 246.5 | 253.4 | 4 | 3.8 | FAIL |
| CPU 416x320, 1 thread | 558.2 | 591.1 | 2 | 1.8 | FAIL |
| NPU int8 640x480 | 70.5 | 70.7 | 14 | 13.5 | PASS |
| NPU int8 416x320 p2bench | 66.5 | 66.7 | 15 | 14.2 | PASS |
| NPU int8 640x480 p2bench | 114.8 | 114.9 | 9 | 8.3 | FAIL |

CPU temperature at the end: 60.6 °C. A model passes when the camera + detector loop reaches 10 fps (or keeps up with a slower camera), or, without a camera, when the 90th percentile detector time allows 10 fps.
