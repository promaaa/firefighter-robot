Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.243 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.755 | 0.761 | 0.818 | 0.503 |
| Real test, new scenes only | 5733 | 2284 | 0.642 | 0.732 | 0.753 | 0.464 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.629 | 0.493 | 0.557 | 0.239 |
| Real test, new scenes, small fires | 5733 | 689 | 0.564 | 0.466 | 0.517 | 0.241 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.790 | 0.865 | 0.899 | 0.593 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.664 | 0.847 | 0.844 | 0.554 |
| D-Fire test, all | 4306 | 2878 | 0.677 | 0.637 | 0.691 | 0.345 |
| D-Fire test, new scenes | 1086 | 566 | 0.654 | 0.588 | 0.652 | 0.339 |
| FASDD test, all | 9512 | 5472 | 0.789 | 0.823 | 0.870 | 0.575 |
| FASDD test, new scenes | 4646 | 1716 | 0.639 | 0.779 | 0.789 | 0.505 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.917 | 0.943 | 0.977 | 0.602 |
| Synthetic firescenes test | 67 | 90 | 0.952 | 0.889 | 0.912 | 0.754 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.039 | 2.1 % |
| D-Fire test, no fire | 3191 | 0.022 | 1.8 % |
| FASDD test, no fire | 6199 | 0.086 | 6.2 % |
| Real test, new scenes, no fire | 4738 | 0.111 | 8.0 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.077 | 0.125 | 19 | 0.053 | 0.105 |
| 8-12 | 281 | 0.285 | 0.313 | 61 | 0.148 | 0.180 |
| 12-16 | 402 | 0.368 | 0.428 | 108 | 0.287 | 0.343 |
| 16-24 | 826 | 0.521 | 0.586 | 241 | 0.465 | 0.535 |
| 24-32 | 745 | 0.666 | 0.718 | 260 | 0.646 | 0.692 |
| 32-48 | 1079 | 0.734 | 0.797 | 305 | 0.715 | 0.774 |
| 48-64 | 760 | 0.775 | 0.838 | 235 | 0.779 | 0.809 |
| 64-96 | 1167 | 0.868 | 0.913 | 286 | 0.839 | 0.902 |
| 96-128 | 918 | 0.907 | 0.952 | 213 | 0.915 | 0.948 |
| 128-max | 2173 | 0.942 | 0.968 | 556 | 0.926 | 0.969 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Camera: f = 476 px at 640 x 480 (lens calibration, pinhole HFOV 67.8 deg). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| new scenes, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| all, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| all, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.5, recall 50 % | 24 px | 2.0 m | 6.0 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
