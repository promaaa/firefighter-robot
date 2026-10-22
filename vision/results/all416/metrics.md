Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.284 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.781 | 0.745 | 0.809 | 0.503 |
| Real test, new scenes only | 5733 | 2284 | 0.673 | 0.723 | 0.747 | 0.466 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.626 | 0.472 | 0.529 | 0.230 |
| Real test, new scenes, small fires | 5733 | 689 | 0.565 | 0.469 | 0.511 | 0.238 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.825 | 0.850 | 0.897 | 0.597 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.706 | 0.833 | 0.839 | 0.558 |
| D-Fire test, all | 4306 | 2878 | 0.699 | 0.601 | 0.676 | 0.341 |
| D-Fire test, new scenes | 1086 | 566 | 0.667 | 0.588 | 0.656 | 0.346 |
| FASDD test, all | 9512 | 5472 | 0.816 | 0.817 | 0.865 | 0.577 |
| FASDD test, new scenes | 4646 | 1716 | 0.675 | 0.767 | 0.780 | 0.506 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.915 | 0.924 | 0.944 | 0.615 |
| Synthetic firescenes test | 67 | 90 | 0.929 | 0.878 | 0.911 | 0.780 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.045 | 2.1 % |
| D-Fire test, no fire | 3191 | 0.021 | 1.8 % |
| FASDD test, no fire | 6199 | 0.073 | 5.6 % |
| Real test, new scenes, no fire | 4738 | 0.095 | 7.4 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.067 | 0.144 | 19 | 0.105 | 0.158 |
| 8-12 | 281 | 0.238 | 0.310 | 61 | 0.115 | 0.148 |
| 12-16 | 402 | 0.328 | 0.388 | 108 | 0.306 | 0.352 |
| 16-24 | 826 | 0.510 | 0.574 | 241 | 0.465 | 0.539 |
| 24-32 | 745 | 0.654 | 0.703 | 260 | 0.650 | 0.688 |
| 32-48 | 1079 | 0.723 | 0.788 | 305 | 0.695 | 0.770 |
| 48-64 | 760 | 0.741 | 0.816 | 235 | 0.753 | 0.791 |
| 64-96 | 1167 | 0.860 | 0.904 | 286 | 0.825 | 0.902 |
| 96-128 | 918 | 0.893 | 0.937 | 213 | 0.906 | 0.930 |
| 128-max | 2173 | 0.928 | 0.961 | 556 | 0.917 | 0.962 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Assumed camera: f = 283 px at 640 x 480 (HFOV 97.1 deg, pinhole). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 128 px | 0.2 m | 0.7 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.4 m | 1.3 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.3 m | 0.9 m |
| new scenes, IoU 0.25, recall 90 % | 64 px | 0.4 m | 1.3 m |
| all, IoU 0.5, recall 50 % | 16 px | 1.8 m | 5.3 m |
| all, IoU 0.25, recall 50 % | 16 px | 1.8 m | 5.3 m |
| new scenes, IoU 0.5, recall 50 % | 24 px | 1.2 m | 3.5 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 1.8 m | 5.3 m |
