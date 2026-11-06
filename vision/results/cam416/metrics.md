Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.242 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.765 | 0.759 | 0.815 | 0.505 |
| Real test, new scenes only | 5733 | 2284 | 0.669 | 0.740 | 0.765 | 0.473 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.595 | 0.495 | 0.532 | 0.232 |
| Real test, new scenes, small fires | 5733 | 689 | 0.532 | 0.492 | 0.525 | 0.243 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.816 | 0.862 | 0.904 | 0.599 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.715 | 0.847 | 0.857 | 0.567 |
| D-Fire test, all | 4306 | 2878 | 0.671 | 0.627 | 0.672 | 0.339 |
| D-Fire test, new scenes | 1086 | 566 | 0.647 | 0.608 | 0.642 | 0.339 |
| FASDD test, all | 9512 | 5472 | 0.807 | 0.825 | 0.874 | 0.580 |
| FASDD test, new scenes | 4646 | 1716 | 0.675 | 0.783 | 0.802 | 0.516 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.909 | 0.952 | 0.949 | 0.606 |
| Synthetic firescenes test | 67 | 90 | 0.929 | 0.878 | 0.891 | 0.754 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.036 | 2.4 % |
| D-Fire test, no fire | 3191 | 0.023 | 1.8 % |
| FASDD test, no fire | 6199 | 0.066 | 5.1 % |
| Real test, new scenes, no fire | 4738 | 0.086 | 6.6 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.067 | 0.163 | 19 | 0.105 | 0.158 |
| 8-12 | 281 | 0.263 | 0.327 | 61 | 0.131 | 0.180 |
| 12-16 | 402 | 0.366 | 0.435 | 108 | 0.352 | 0.407 |
| 16-24 | 826 | 0.536 | 0.619 | 241 | 0.515 | 0.610 |
| 24-32 | 745 | 0.666 | 0.711 | 260 | 0.642 | 0.685 |
| 32-48 | 1079 | 0.746 | 0.814 | 305 | 0.731 | 0.813 |
| 48-64 | 760 | 0.767 | 0.843 | 235 | 0.813 | 0.851 |
| 64-96 | 1167 | 0.868 | 0.913 | 286 | 0.825 | 0.906 |
| 96-128 | 918 | 0.900 | 0.947 | 213 | 0.901 | 0.939 |
| 128-max | 2173 | 0.933 | 0.965 | 556 | 0.915 | 0.964 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Camera: f = 476 px at 640 x 480 (lens calibration, pinhole HFOV 67.8 deg). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 128 px | 0.4 m | 1.1 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| new scenes, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| all, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| all, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
