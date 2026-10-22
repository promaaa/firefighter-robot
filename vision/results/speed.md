Measured on laptop: Intel(R) Core(TM) i7-9700 CPU @ 3.00GHz (8 cores), onnxruntime 1.31.0, 200 runs each. Load average before the test: 5.99 9.44 17.16. Other jobs shared the CPU, so the median times are pessimistic. Fastest 10 %: the 10th percentile, closer to the time on a free CPU.

| Model | Threads | Network, median (ms) | Network, fastest 10 % (ms) | detect() on a 640 x 480 frame, median (ms) | Frames per second, detect() median |
|---|---|---|---|---|---|
| `fire_yolo11n_320.onnx` | 1 | 17.9 | 17.4 | 20.6 | 49 |
| `fire_yolo11n_320.onnx` | 4 | 6.4 | 6.3 | 9.1 | 110 |
| `fire_yolo11n_320x256.onnx` | 1 | 14.4 | 14.1 | 17.0 | 59 |
| `fire_yolo11n_320x256.onnx` | 4 | 5.4 | 5.3 | 8.0 | 125 |
| `fire_yolo11n_320x256_int8.onnx` | 1 | 10.9 | 10.7 | 13.5 | 74 |
| `fire_yolo11n_320x256_int8.onnx` | 4 | 4.9 | 4.8 | 7.5 | 134 |
| `fire_yolo11n_416.onnx` | 1 | 30.0 | 29.5 | 33.4 | 30 |
| `fire_yolo11n_416.onnx` | 4 | 11.8 | 11.4 | 15.6 | 64 |
| `fire_yolo11n_416x320.onnx` | 1 | 23.4 | 22.8 | 26.7 | 38 |
| `fire_yolo11n_416x320.onnx` | 4 | 8.6 | 8.5 | 12.0 | 83 |
| `fire_yolo11n_416x320_int8.onnx` | 1 | 17.1 | 16.7 | 20.2 | 49 |
| `fire_yolo11n_416x320_int8.onnx` | 4 | 7.7 | 7.3 | 11.0 | 91 |
| `fire_yolo11n_640x480.onnx` | 1 | 53.5 | 52.1 | 55.7 | 18 |
| `fire_yolo11n_640x480.onnx` | 4 | 22.2 | 20.3 | 24.6 | 41 |
