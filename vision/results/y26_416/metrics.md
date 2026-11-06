Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.266 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.781 | 0.770 | 0.830 | 0.515 |
| Real test, new scenes only | 5733 | 2284 | 0.691 | 0.752 | 0.778 | 0.482 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.627 | 0.522 | 0.572 | 0.251 |
| Real test, new scenes, small fires | 5733 | 689 | 0.584 | 0.517 | 0.554 | 0.257 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.828 | 0.866 | 0.909 | 0.607 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.726 | 0.854 | 0.862 | 0.573 |
| D-Fire test, all | 4306 | 2878 | 0.698 | 0.651 | 0.705 | 0.357 |
| D-Fire test, new scenes | 1086 | 566 | 0.689 | 0.618 | 0.681 | 0.356 |
| FASDD test, all | 9512 | 5472 | 0.818 | 0.829 | 0.882 | 0.588 |
| FASDD test, new scenes | 4646 | 1716 | 0.691 | 0.796 | 0.811 | 0.523 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.934 | 0.943 | 0.953 | 0.622 |
| Synthetic firescenes test | 67 | 90 | 0.952 | 0.889 | 0.920 | 0.795 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.055 | 3.6 % |
| D-Fire test, no fire | 3191 | 0.022 | 1.8 % |
| FASDD test, no fire | 6199 | 0.070 | 5.4 % |
| Real test, new scenes, no fire | 4738 | 0.092 | 7.0 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.125 | 0.202 | 19 | 0.105 | 0.105 |
| 8-12 | 281 | 0.306 | 0.349 | 61 | 0.148 | 0.180 |
| 12-16 | 402 | 0.403 | 0.455 | 108 | 0.352 | 0.380 |
| 16-24 | 826 | 0.558 | 0.633 | 241 | 0.523 | 0.593 |
| 24-32 | 745 | 0.685 | 0.744 | 260 | 0.696 | 0.731 |
| 32-48 | 1079 | 0.752 | 0.814 | 305 | 0.718 | 0.784 |
| 48-64 | 760 | 0.775 | 0.854 | 235 | 0.804 | 0.847 |
| 64-96 | 1167 | 0.867 | 0.912 | 286 | 0.839 | 0.906 |
| 96-128 | 918 | 0.904 | 0.950 | 213 | 0.925 | 0.962 |
| 128-max | 2173 | 0.938 | 0.969 | 556 | 0.930 | 0.969 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Camera: f = 476 px at 640 x 480 (lens calibration, pinhole HFOV 67.8 deg). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| new scenes, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| all, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| all, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
