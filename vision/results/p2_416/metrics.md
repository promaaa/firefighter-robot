Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.227 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.736 | 0.778 | 0.822 | 0.512 |
| Real test, new scenes only | 5733 | 2284 | 0.620 | 0.751 | 0.756 | 0.468 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.592 | 0.544 | 0.575 | 0.251 |
| Real test, new scenes, small fires | 5733 | 689 | 0.532 | 0.520 | 0.540 | 0.250 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.782 | 0.869 | 0.899 | 0.601 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.649 | 0.851 | 0.841 | 0.556 |
| D-Fire test, all | 4306 | 2878 | 0.653 | 0.666 | 0.701 | 0.355 |
| D-Fire test, new scenes | 1086 | 566 | 0.621 | 0.634 | 0.680 | 0.354 |
| FASDD test, all | 9512 | 5472 | 0.774 | 0.834 | 0.873 | 0.584 |
| FASDD test, new scenes | 4646 | 1716 | 0.620 | 0.790 | 0.785 | 0.506 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.918 | 0.962 | 0.976 | 0.612 |
| Synthetic firescenes test | 67 | 90 | 0.941 | 0.889 | 0.911 | 0.777 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.058 | 3.3 % |
| D-Fire test, no fire | 3191 | 0.025 | 1.9 % |
| FASDD test, no fire | 6199 | 0.096 | 7.0 % |
| Real test, new scenes, no fire | 4738 | 0.126 | 9.2 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.096 | 0.183 | 19 | 0.053 | 0.158 |
| 8-12 | 281 | 0.345 | 0.388 | 61 | 0.197 | 0.230 |
| 12-16 | 402 | 0.438 | 0.500 | 108 | 0.398 | 0.435 |
| 16-24 | 826 | 0.567 | 0.642 | 241 | 0.510 | 0.598 |
| 24-32 | 745 | 0.713 | 0.766 | 260 | 0.688 | 0.735 |
| 32-48 | 1079 | 0.756 | 0.819 | 305 | 0.715 | 0.787 |
| 48-64 | 760 | 0.786 | 0.857 | 235 | 0.804 | 0.843 |
| 64-96 | 1167 | 0.874 | 0.913 | 286 | 0.839 | 0.906 |
| 96-128 | 918 | 0.906 | 0.949 | 213 | 0.930 | 0.953 |
| 128-max | 2173 | 0.936 | 0.972 | 556 | 0.923 | 0.973 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Camera: f = 476 px at 640 x 480 (lens calibration, pinhole HFOV 67.8 deg). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| new scenes, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| all, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| all, IoU 0.25, recall 50 % | 12 px | 4.0 m | 11.9 m |
| new scenes, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
