Fixed test subset: dfire 300, fasdd 300, barman 101, coco 150, synthetic 67 images (all test images, near-duplicates included). Threshold 0.28. Real = D-Fire + FASDD + Barman.

| Model | Real mAP50 | Real mAP50-95 | Real precision | Real recall | Real recall, small fires (< 32 px) | FP per image, COCO fire-like | Synthetic mAP50 |
|---|---|---|---|---|---|---|---|
| fire_yolo11n_416x320.onnx | 0.816 | 0.480 | 0.778 | 0.739 | 0.554 | 0.007 | 0.919 |
| all416_fire_yolo11n_416x320_int8.onnx | 0.574 | 0.270 | 0.694 | 0.505 | 0.402 | 0.000 | 0.278 |
| all416_fire_yolo11n_320x256.onnx | 0.796 | 0.462 | 0.797 | 0.726 | 0.533 | 0.013 | 0.880 |
| all416_fire_yolo11n_640x480.onnx | 0.833 | 0.485 | 0.741 | 0.779 | 0.679 | 0.013 | 0.950 |
| cam416_fire_yolo11n_416x320.onnx | 0.812 | 0.479 | 0.797 | 0.747 | 0.571 | 0.013 | 0.892 |
| cam416_fire_yolo11n_640x480.onnx | 0.825 | 0.484 | 0.742 | 0.758 | 0.620 | 0.007 | 0.950 |
| p2bench_fire_yolo11n_416x320.onnx | 0.828 | 0.482 | 0.796 | 0.749 | 0.571 | 0.013 | 0.902 |
| p2bench_fire_yolo11n_640x480.onnx | 0.823 | 0.478 | 0.730 | 0.772 | 0.641 | 0.007 | 0.947 |
| cam416_fire_yolo11n_416x320_fp16.rknn (simulator) | 0.813 | 0.480 | 0.798 | 0.749 | 0.576 | 0.013 | 0.892 |
| cam416_fire_yolo11n_416x320_int8.rknn (simulator) | 0.788 | 0.424 | 0.777 | 0.708 | 0.511 | 0.013 | 0.878 |
| cam416_fire_yolo11n_640x480_fp16.rknn (simulator) | 0.826 | 0.486 | 0.744 | 0.760 | 0.625 | 0.007 | 0.939 |
| cam416_fire_yolo11n_640x480_int8.rknn (simulator) | 0.777 | 0.410 | 0.722 | 0.727 | 0.560 | 0.007 | 0.916 |
| fire_yolo11n_416x320_fp16.rknn (simulator) | 0.820 | 0.480 | 0.782 | 0.743 | 0.565 | 0.007 | 0.919 |
| fire_yolo11n_416x320_int8.rknn (simulator) | 0.775 | 0.424 | 0.728 | 0.720 | 0.500 | 0.007 | 0.882 |
| fire_yolo11n_640x480_fp16.rknn (simulator) | 0.832 | 0.484 | 0.741 | 0.779 | 0.679 | 0.013 | 0.958 |
| fire_yolo11n_640x480_int8.rknn (simulator) | 0.780 | 0.404 | 0.685 | 0.735 | 0.598 | 0.007 | 0.934 |
| p2bench_fire_yolo11n_416x320_fp16.rknn (simulator) | 0.827 | 0.483 | 0.796 | 0.749 | 0.571 | 0.013 | 0.902 |
| p2bench_fire_yolo11n_416x320_int8.rknn (simulator) | 0.803 | 0.426 | 0.773 | 0.752 | 0.571 | 0.020 | 0.863 |
| p2bench_fire_yolo11n_640x480_fp16.rknn (simulator) | 0.824 | 0.477 | 0.727 | 0.772 | 0.641 | 0.007 | 0.947 |
| p2bench_fire_yolo11n_640x480_int8.rknn (simulator) | 0.791 | 0.385 | 0.717 | 0.741 | 0.598 | 0.007 | 0.928 |
