Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.212 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.757 | 0.767 | 0.822 | 0.503 |
| Real test, new scenes only | 5733 | 2284 | 0.662 | 0.730 | 0.764 | 0.465 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.629 | 0.514 | 0.571 | 0.245 |
| Real test, new scenes, small fires | 5733 | 689 | 0.589 | 0.479 | 0.545 | 0.254 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.795 | 0.865 | 0.901 | 0.591 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.683 | 0.839 | 0.850 | 0.550 |
| D-Fire test, all | 4306 | 2878 | 0.663 | 0.647 | 0.690 | 0.339 |
| D-Fire test, new scenes | 1086 | 566 | 0.642 | 0.588 | 0.655 | 0.337 |
| FASDD test, all | 9512 | 5472 | 0.801 | 0.826 | 0.876 | 0.576 |
| FASDD test, new scenes | 4646 | 1716 | 0.667 | 0.777 | 0.799 | 0.505 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.927 | 0.962 | 0.970 | 0.610 |
| Synthetic firescenes test | 67 | 90 | 0.929 | 0.867 | 0.911 | 0.751 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.042 | 2.4 % |
| D-Fire test, no fire | 3191 | 0.029 | 2.1 % |
| FASDD test, no fire | 6199 | 0.069 | 5.2 % |
| Real test, new scenes, no fire | 4738 | 0.090 | 6.7 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.144 | 0.173 | 19 | 0.105 | 0.158 |
| 8-12 | 281 | 0.302 | 0.335 | 61 | 0.131 | 0.164 |
| 12-16 | 402 | 0.386 | 0.458 | 108 | 0.306 | 0.361 |
| 16-24 | 826 | 0.540 | 0.614 | 241 | 0.473 | 0.573 |
| 24-32 | 745 | 0.686 | 0.730 | 260 | 0.665 | 0.712 |
| 32-48 | 1079 | 0.746 | 0.800 | 305 | 0.708 | 0.764 |
| 48-64 | 760 | 0.762 | 0.836 | 235 | 0.774 | 0.817 |
| 64-96 | 1167 | 0.870 | 0.915 | 286 | 0.825 | 0.902 |
| 96-128 | 918 | 0.910 | 0.949 | 213 | 0.906 | 0.930 |
| 128-max | 2173 | 0.938 | 0.970 | 556 | 0.919 | 0.966 |

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
