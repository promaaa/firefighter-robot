Fixed test subset: dfire 300, fasdd 300, barman 101, coco 150, synthetic 67 images (all test images, near-duplicates included). Threshold 0.28. Real = D-Fire + FASDD + Barman.

| Model | Real mAP50 | Real mAP50-95 | Real precision | Real recall | Real recall, small fires (< 32 px) | FP per image, COCO fire-like | Synthetic mAP50 |
|---|---|---|---|---|---|---|---|
| fire_yolo11n_416x320.onnx | 0.816 | 0.480 | 0.778 | 0.739 | 0.554 | 0.007 | 0.919 |
| fire_yolo11n_416x320_int8.onnx | 0.574 | 0.270 | 0.694 | 0.505 | 0.402 | 0.000 | 0.278 |
| fire_yolo11n_320x256.onnx | 0.796 | 0.462 | 0.797 | 0.726 | 0.533 | 0.013 | 0.880 |
| fire_yolo11n_640x480.onnx | 0.833 | 0.485 | 0.741 | 0.779 | 0.679 | 0.013 | 0.950 |
| fire_yolo11n_416x320_int8.rknn (simulator) | 0.775 | 0.424 | 0.728 | 0.720 | 0.500 | 0.007 | 0.882 |
| fire_yolo11n_416x320_fp16.rknn (simulator) | 0.820 | 0.480 | 0.782 | 0.743 | 0.565 | 0.007 | 0.919 |
| fire_yolo11n_640x480_int8.rknn (simulator) | 0.780 | 0.404 | 0.685 | 0.735 | 0.598 | 0.007 | 0.934 |
| fire_yolo11n_640x480_fp16.rknn (simulator) | 0.832 | 0.484 | 0.741 | 0.779 | 0.679 | 0.013 | 0.958 |
