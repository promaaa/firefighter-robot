Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.230 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.735 | 0.773 | 0.819 | 0.505 |
| Real test, new scenes only | 5733 | 2284 | 0.629 | 0.741 | 0.758 | 0.464 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.585 | 0.536 | 0.563 | 0.245 |
| Real test, new scenes, small fires | 5733 | 689 | 0.531 | 0.505 | 0.541 | 0.249 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.783 | 0.864 | 0.898 | 0.594 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.660 | 0.843 | 0.843 | 0.552 |
| D-Fire test, all | 4306 | 2878 | 0.645 | 0.657 | 0.691 | 0.342 |
| D-Fire test, new scenes | 1086 | 566 | 0.631 | 0.610 | 0.663 | 0.337 |
| FASDD test, all | 9512 | 5472 | 0.777 | 0.830 | 0.871 | 0.579 |
| FASDD test, new scenes | 4646 | 1716 | 0.628 | 0.784 | 0.792 | 0.505 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.927 | 0.962 | 0.976 | 0.591 |
| Synthetic firescenes test | 67 | 90 | 0.952 | 0.889 | 0.911 | 0.748 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.052 | 2.4 % |
| D-Fire test, no fire | 3191 | 0.025 | 2.1 % |
| FASDD test, no fire | 6199 | 0.092 | 6.6 % |
| Real test, new scenes, no fire | 4738 | 0.116 | 8.5 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.115 | 0.154 | 19 | 0.000 | 0.053 |
| 8-12 | 281 | 0.345 | 0.384 | 61 | 0.197 | 0.246 |
| 12-16 | 402 | 0.438 | 0.500 | 108 | 0.333 | 0.389 |
| 16-24 | 826 | 0.548 | 0.639 | 241 | 0.494 | 0.606 |
| 24-32 | 745 | 0.707 | 0.762 | 260 | 0.696 | 0.746 |
| 32-48 | 1079 | 0.741 | 0.804 | 305 | 0.698 | 0.761 |
| 48-64 | 760 | 0.779 | 0.845 | 235 | 0.791 | 0.826 |
| 64-96 | 1167 | 0.869 | 0.913 | 286 | 0.815 | 0.892 |
| 96-128 | 918 | 0.901 | 0.943 | 213 | 0.925 | 0.958 |
| 128-max | 2173 | 0.937 | 0.968 | 556 | 0.926 | 0.962 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Camera: f = 476 px at 640 x 480 (lens calibration, pinhole HFOV 67.8 deg). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| new scenes, IoU 0.25, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| all, IoU 0.25, recall 50 % | 12 px | 4.0 m | 11.9 m |
| new scenes, IoU 0.5, recall 50 % | 24 px | 2.0 m | 6.0 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
