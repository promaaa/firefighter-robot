Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.231 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.762 | 0.773 | 0.830 | 0.514 |
| Real test, new scenes only | 5733 | 2284 | 0.670 | 0.734 | 0.772 | 0.474 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.616 | 0.524 | 0.577 | 0.252 |
| Real test, new scenes, small fires | 5733 | 689 | 0.566 | 0.478 | 0.540 | 0.253 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.806 | 0.869 | 0.906 | 0.603 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.701 | 0.845 | 0.859 | 0.563 |
| D-Fire test, all | 4306 | 2878 | 0.666 | 0.659 | 0.704 | 0.354 |
| D-Fire test, new scenes | 1086 | 566 | 0.624 | 0.601 | 0.663 | 0.344 |
| FASDD test, all | 9512 | 5472 | 0.807 | 0.829 | 0.880 | 0.587 |
| FASDD test, new scenes | 4646 | 1716 | 0.682 | 0.777 | 0.807 | 0.516 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.926 | 0.952 | 0.974 | 0.617 |
| Synthetic firescenes test | 67 | 90 | 0.963 | 0.878 | 0.914 | 0.749 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.042 | 1.8 % |
| D-Fire test, no fire | 3191 | 0.022 | 1.7 % |
| FASDD test, no fire | 6199 | 0.063 | 4.9 % |
| Real test, new scenes, no fire | 4738 | 0.082 | 6.2 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.125 | 0.173 | 19 | 0.000 | 0.105 |
| 8-12 | 281 | 0.306 | 0.335 | 61 | 0.131 | 0.164 |
| 12-16 | 402 | 0.405 | 0.480 | 108 | 0.324 | 0.380 |
| 16-24 | 826 | 0.561 | 0.637 | 241 | 0.494 | 0.593 |
| 24-32 | 745 | 0.685 | 0.734 | 260 | 0.642 | 0.685 |
| 32-48 | 1079 | 0.758 | 0.814 | 305 | 0.738 | 0.793 |
| 48-64 | 760 | 0.782 | 0.846 | 235 | 0.779 | 0.821 |
| 64-96 | 1167 | 0.871 | 0.913 | 286 | 0.829 | 0.892 |
| 96-128 | 918 | 0.908 | 0.946 | 213 | 0.906 | 0.939 |
| 128-max | 2173 | 0.936 | 0.966 | 556 | 0.915 | 0.959 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Camera: f = 476 px at 640 x 480 (lens calibration, pinhole HFOV 67.8 deg). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.7 m | 2.2 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.5 m | 1.5 m |
| new scenes, IoU 0.25, recall 90 % | 96 px | 0.5 m | 1.5 m |
| all, IoU 0.5, recall 50 % | 16 px | 3.0 m | 8.9 m |
| all, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
| new scenes, IoU 0.5, recall 50 % | 24 px | 2.0 m | 6.0 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 3.0 m | 8.9 m |
