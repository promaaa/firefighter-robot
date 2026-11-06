Detection rate of one visible full-strength fire (10 flame pixels or more), by distance and tray diameter.
YOLO 0.28: vision/detector.py at its own threshold. YOLO 0.5: only boxes that aim/ uses (min_conf 0.5).
Baseline: sim/sim2sim/baseline_detector.py. Overlap: any overlap with the true flame box (campaign rule). IoU: 0.25 or more.

| Flames | Tray (cm) | Distance (m) | Images | True box width (px) | YOLO 0.28, overlap | YOLO 0.28, IoU 0.25 | YOLO 0.5, overlap | Baseline, overlap |
|---|---|---|---|---|---|---|---|---|
| procedural | 10 | 1.0 | 40 | 54 | 72 % | 72 % | 38 % | 100 % |
| procedural | 10 | 1.5 | 40 | 37 | 70 % | 70 % | 32 % | 100 % |
| procedural | 10 | 2.0 | 40 | 29 | 78 % | 78 % | 40 % | 100 % |
| procedural | 10 | 2.5 | 40 | 22 | 85 % | 85 % | 62 % | 100 % |
| procedural | 10 | 3.0 | 40 | 20 | 85 % | 72 % | 48 % | 100 % |
| procedural | 10 | 3.5 | 35 | 17 | 74 % | 69 % | 37 % | 100 % |
| procedural | 10 | 4.0 | 35 | 13 | 66 % | 63 % | 31 % | 100 % |
| procedural | 10 | 4.5 | 33 | 10 | 70 % | 70 % | 21 % | 88 % |
| procedural | 10 | 5.0 | 35 | 10 | 34 % | 29 % | 0 % | 77 % |
| procedural | 15 | 1.0 | 40 | 85 | 68 % | 68 % | 48 % | 100 % |
| procedural | 15 | 1.5 | 40 | 58 | 72 % | 72 % | 57 % | 100 % |
| procedural | 15 | 2.0 | 40 | 44 | 88 % | 88 % | 65 % | 100 % |
| procedural | 15 | 2.5 | 40 | 35 | 80 % | 80 % | 60 % | 100 % |
| procedural | 15 | 3.0 | 40 | 27 | 90 % | 88 % | 68 % | 100 % |
| procedural | 15 | 3.5 | 35 | 27 | 71 % | 71 % | 31 % | 100 % |
| procedural | 15 | 4.0 | 40 | 18 | 88 % | 88 % | 57 % | 100 % |
| procedural | 15 | 4.5 | 40 | 18 | 92 % | 92 % | 60 % | 100 % |
| procedural | 15 | 5.0 | 35 | 17 | 80 % | 77 % | 37 % | 100 % |
| procedural | 20 | 1.0 | 40 | 121 | 78 % | 78 % | 68 % | 100 % |
| procedural | 20 | 1.5 | 40 | 78 | 78 % | 78 % | 68 % | 100 % |
| procedural | 20 | 2.0 | 40 | 60 | 75 % | 75 % | 60 % | 100 % |
| procedural | 20 | 2.5 | 40 | 48 | 75 % | 75 % | 57 % | 100 % |
| procedural | 20 | 3.0 | 40 | 36 | 85 % | 85 % | 72 % | 100 % |
| procedural | 20 | 3.5 | 40 | 34 | 78 % | 78 % | 55 % | 100 % |
| procedural | 20 | 4.0 | 40 | 30 | 82 % | 82 % | 65 % | 100 % |
| procedural | 20 | 4.5 | 40 | 27 | 75 % | 75 % | 65 % | 100 % |
| procedural | 20 | 5.0 | 35 | 24 | 80 % | 77 % | 51 % | 100 % |
| procedural | 30 | 1.0 | 40 | 136 | 90 % | 90 % | 85 % | 100 % |
| procedural | 30 | 1.5 | 40 | 133 | 82 % | 82 % | 78 % | 100 % |
| procedural | 30 | 2.0 | 40 | 75 | 92 % | 92 % | 80 % | 100 % |
| procedural | 30 | 2.5 | 40 | 70 | 85 % | 85 % | 78 % | 100 % |
| procedural | 30 | 3.0 | 40 | 59 | 82 % | 82 % | 75 % | 100 % |
| procedural | 30 | 3.5 | 40 | 44 | 88 % | 88 % | 88 % | 100 % |
| procedural | 30 | 4.0 | 40 | 42 | 95 % | 95 % | 82 % | 100 % |
| procedural | 30 | 4.5 | 40 | 40 | 90 % | 90 % | 88 % | 100 % |
| procedural | 30 | 5.0 | 35 | 30 | 86 % | 86 % | 69 % | 100 % |
| real | 10 | 1.0 | 40 | 52 | 60 % | 60 % | 55 % | 98 % |
| real | 10 | 1.5 | 40 | 36 | 62 % | 62 % | 50 % | 90 % |
| real | 10 | 2.0 | 40 | 29 | 68 % | 65 % | 45 % | 88 % |
| real | 10 | 2.5 | 40 | 24 | 70 % | 68 % | 32 % | 80 % |
| real | 10 | 3.0 | 40 | 20 | 60 % | 50 % | 25 % | 75 % |
| real | 10 | 3.5 | 35 | 17 | 63 % | 63 % | 37 % | 74 % |
| real | 10 | 4.0 | 35 | 15 | 46 % | 43 % | 26 % | 71 % |
| real | 10 | 4.5 | 35 | 13 | 57 % | 51 % | 31 % | 57 % |
| real | 10 | 5.0 | 35 | 12 | 34 % | 20 % | 9 % | 69 % |
| real | 15 | 1.0 | 40 | 84 | 85 % | 85 % | 78 % | 100 % |
| real | 15 | 1.5 | 40 | 59 | 75 % | 75 % | 75 % | 98 % |
| real | 15 | 2.0 | 40 | 47 | 82 % | 82 % | 60 % | 88 % |
| real | 15 | 2.5 | 40 | 36 | 88 % | 88 % | 68 % | 85 % |
| real | 15 | 3.0 | 40 | 30 | 88 % | 88 % | 50 % | 85 % |
| real | 15 | 3.5 | 35 | 26 | 57 % | 57 % | 31 % | 80 % |
| real | 15 | 4.0 | 40 | 22 | 85 % | 82 % | 40 % | 85 % |
| real | 15 | 4.5 | 40 | 19 | 72 % | 60 % | 42 % | 80 % |
| real | 15 | 5.0 | 35 | 18 | 51 % | 43 % | 9 % | 69 % |
| real | 20 | 1.0 | 40 | 104 | 95 % | 95 % | 90 % | 100 % |
| real | 20 | 1.5 | 40 | 76 | 95 % | 95 % | 80 % | 100 % |
| real | 20 | 2.0 | 40 | 59 | 88 % | 88 % | 60 % | 100 % |
| real | 20 | 2.5 | 40 | 47 | 80 % | 80 % | 55 % | 100 % |
| real | 20 | 3.0 | 40 | 40 | 90 % | 90 % | 70 % | 98 % |
| real | 20 | 3.5 | 40 | 34 | 80 % | 80 % | 60 % | 90 % |
| real | 20 | 4.0 | 40 | 29 | 68 % | 68 % | 45 % | 85 % |
| real | 20 | 4.5 | 40 | 27 | 80 % | 80 % | 42 % | 88 % |
| real | 20 | 5.0 | 35 | 23 | 91 % | 91 % | 49 % | 86 % |
| real | 30 | 1.0 | 40 | 156 | 90 % | 90 % | 88 % | 100 % |
| real | 30 | 1.5 | 40 | 118 | 88 % | 88 % | 88 % | 100 % |
| real | 30 | 2.0 | 40 | 86 | 82 % | 82 % | 70 % | 100 % |
| real | 30 | 2.5 | 40 | 67 | 88 % | 88 % | 85 % | 100 % |
| real | 30 | 3.0 | 40 | 57 | 80 % | 80 % | 78 % | 100 % |
| real | 30 | 3.5 | 40 | 52 | 85 % | 85 % | 80 % | 100 % |
| real | 30 | 4.0 | 40 | 49 | 88 % | 88 % | 80 % | 100 % |
| real | 30 | 4.5 | 40 | 39 | 78 % | 78 % | 57 % | 100 % |
| real | 30 | 5.0 | 35 | 35 | 83 % | 80 % | 54 % | 100 % |

Real flames, YOLO detector at 0.28, by true flame box width (all trays and distances):

| Box width (px) | Images | Detected, overlap | Detected, IoU 0.25 | YOLO on real photos, IoU 0.25 (vision/README.md) |
|---|---|---|---|---|
| 0-8 | 5 | 0 % | 0 % | 15 % |
| 8-12 | 15 | 0 % | 0 % | 32 % |
| 12-16 | 91 | 46 % | 38 % | 39 % |
| 16-24 | 222 | 69 % | 64 % | 58 % |
| 24-32 | 235 | 74 % | 71 % | 71 % |
| 32-48 | 322 | 78 % | 78 % | 79 % |
| 48-64 | 209 | 84 % | 84 % | 82 % |
| 64-96 | 173 | 91 % | 91 % | 90 % |
| 96-128 | 70 | 89 % | 89 % | 94 % |
| 128-more | 58 | 95 % | 95 % | 96 % |

False detections (boxes that overlap no flame), per image, all images:

| Flames | YOLO 0.28 | YOLO 0.5 | Baseline |
|---|---|---|---|
| procedural | 0.000 | 0.000 | 0.627 |
| real | 0.000 | 0.000 | 0.628 |

By real clip (phase % 5). Procedural: the same scenes, trays and placements with the procedural flames:

| Real clip | Images | YOLO 0.28, real | YOLO 0.28, procedural | YOLO 0.5, real | Baseline, real | Baseline, procedural |
|---|---|---|---|---|---|---|
| basket | 305 | 80 % | 73 % | 58 % | 100 % | 100 % |
| campfire_a | 310 | 72 % | 81 % | 53 % | 86 % | 100 % |
| campfire_b | 265 | 88 % | 79 % | 67 % | 65 % | 98 % |
| free_flame | 265 | 48 % | 74 % | 28 % | 97 % | 98 % |
| wood_fire | 255 | 95 % | 93 % | 76 % | 100 % | 100 % |

Images: 2840 (2798 with a visible flame). Hidden flames (furniture between) are not in the rates.
