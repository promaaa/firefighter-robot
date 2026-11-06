Model: `fire_yolo11n_416.onnx`. Confidence threshold: 0.266 (best F1 on the real validation images). IoU 0.5 for precision (P), recall (R) and mAP50. New scenes: test images that are not near-duplicates of a training image (data.py).

| Test set | Images | Fire boxes | Precision | Recall | mAP50 | mAP50-95 |
|---|---|---|---|---|---|---|
| Real test (D-Fire + FASDD + Barman), all images | 13919 | 8455 | 0.773 | 0.748 | 0.808 | 0.500 |
| Real test, new scenes only | 5733 | 2284 | 0.677 | 0.729 | 0.749 | 0.463 |
| Real test, small fires (width < 32 px) | 13919 | 2358 | 0.613 | 0.469 | 0.525 | 0.232 |
| Real test, new scenes, small fires | 5733 | 689 | 0.565 | 0.478 | 0.514 | 0.239 |
| Real test, large fires (width >= 32 px) | 13919 | 6097 | 0.818 | 0.855 | 0.895 | 0.592 |
| Real test, new scenes, large fires | 5733 | 1595 | 0.712 | 0.838 | 0.840 | 0.552 |
| D-Fire test, all | 4306 | 2878 | 0.696 | 0.605 | 0.671 | 0.336 |
| D-Fire test, new scenes | 1086 | 566 | 0.682 | 0.592 | 0.656 | 0.338 |
| FASDD test, all | 9512 | 5472 | 0.805 | 0.820 | 0.866 | 0.576 |
| FASDD test, new scenes | 4646 | 1716 | 0.676 | 0.774 | 0.782 | 0.503 |
| Barman test (candles, lighters, matches), all | 101 | 105 | 0.889 | 0.914 | 0.917 | 0.586 |
| Synthetic firescenes test | 67 | 90 | 0.940 | 0.867 | 0.891 | 0.757 |

False positives on images without fire, at the same threshold:

| Negative set | Images | False positives per image | Images with a false positive |
|---|---|---|---|
| COCO fire-like (hard negatives) | 330 | 0.033 | 2.1 % |
| D-Fire test, no fire | 3191 | 0.022 | 1.9 % |
| FASDD test, no fire | 6199 | 0.074 | 5.7 % |
| Real test, new scenes, no fire | 4738 | 0.092 | 7.0 % |

Recall by box width (real test, same threshold). A box counts as found when a detection overlaps it with IoU >= 0.5 (or >= 0.25):

| Box width (px) | Boxes, all | Recall, all, IoU 0.5 | Recall, all, IoU 0.25 | Boxes, new scenes | Recall, new, IoU 0.5 | Recall, new, IoU 0.25 |
|---|---|---|---|---|---|---|
| 0-8 | 104 | 0.067 | 0.144 | 19 | 0.158 | 0.158 |
| 8-12 | 281 | 0.228 | 0.295 | 61 | 0.148 | 0.180 |
| 12-16 | 402 | 0.326 | 0.393 | 108 | 0.278 | 0.324 |
| 16-24 | 826 | 0.492 | 0.565 | 241 | 0.456 | 0.548 |
| 24-32 | 745 | 0.670 | 0.713 | 260 | 0.681 | 0.712 |
| 32-48 | 1079 | 0.728 | 0.794 | 305 | 0.711 | 0.777 |
| 48-64 | 760 | 0.761 | 0.843 | 235 | 0.791 | 0.838 |
| 64-96 | 1167 | 0.866 | 0.908 | 286 | 0.815 | 0.892 |
| 96-128 | 918 | 0.894 | 0.938 | 213 | 0.911 | 0.948 |
| 128-max | 2173 | 0.929 | 0.961 | 556 | 0.912 | 0.962 |

Smallest box width from which the recall stays at 90 % (or 50 %) or more for all larger boxes (bins with 20 boxes or more), and the largest distance for a flame of that width. Assumed camera: f = 283 px at 640 x 480 (HFOV 97.1 deg, pinhole). Distance d = f x flame width / box width.

| Test images, criterion | Box width | 10 cm flame | 30 cm flame |
|---|---|---|---|
| all, IoU 0.5, recall 90 % | 128 px | 0.2 m | 0.7 m |
| all, IoU 0.25, recall 90 % | 64 px | 0.4 m | 1.3 m |
| new scenes, IoU 0.5, recall 90 % | 96 px | 0.3 m | 0.9 m |
| new scenes, IoU 0.25, recall 90 % | 96 px | 0.3 m | 0.9 m |
| all, IoU 0.5, recall 50 % | 24 px | 1.2 m | 3.5 m |
| all, IoU 0.25, recall 50 % | 16 px | 1.8 m | 5.3 m |
| new scenes, IoU 0.5, recall 50 % | 24 px | 1.2 m | 3.5 m |
| new scenes, IoU 0.25, recall 50 % | 16 px | 1.8 m | 5.3 m |
