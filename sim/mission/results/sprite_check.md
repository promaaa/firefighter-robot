Detection rate of one visible full-strength fire (10 flame pixels or more), by distance and tray diameter.
YOLO 0.28: vision/detector.py at its own threshold. YOLO 0.5: only boxes that aim/ uses (min_conf 0.5).
Baseline: sim/sim2sim/baseline_detector.py. Overlap: any overlap with the true flame box (campaign rule). IoU: 0.25 or more.

| Flames | Tray (cm) | Distance (m) | Images | True box width (px) | YOLO 0.28, overlap | YOLO 0.28, IoU 0.25 | YOLO 0.5, overlap | Baseline, overlap |
|---|---|---|---|---|---|---|---|---|
| procedural | 10 | 1.0 | 40 | 32 | 62 % | 60 % | 42 % | 100 % |
| procedural | 10 | 1.5 | 40 | 21 | 85 % | 80 % | 50 % | 100 % |
| procedural | 10 | 2.0 | 40 | 16 | 70 % | 62 % | 42 % | 100 % |
| procedural | 10 | 2.5 | 40 | 12 | 50 % | 42 % | 10 % | 100 % |
| procedural | 10 | 3.0 | 40 | 10 | 42 % | 32 % | 25 % | 78 % |
| procedural | 10 | 3.5 | 35 | 9 | 43 % | 40 % | 6 % | 66 % |
| procedural | 10 | 4.0 | 35 | 8 | 31 % | 31 % | 14 % | 34 % |
| procedural | 10 | 4.5 | 31 | 6 | 23 % | 23 % | 3 % | 6 % |
| procedural | 10 | 5.0 | 35 | 5 | 3 % | 3 % | 0 % | 0 % |
| procedural | 15 | 1.0 | 40 | 50 | 62 % | 62 % | 35 % | 100 % |
| procedural | 15 | 1.5 | 40 | 34 | 78 % | 78 % | 57 % | 100 % |
| procedural | 15 | 2.0 | 40 | 26 | 80 % | 78 % | 42 % | 100 % |
| procedural | 15 | 2.5 | 40 | 19 | 100 % | 95 % | 62 % | 100 % |
| procedural | 15 | 3.0 | 40 | 14 | 80 % | 70 % | 48 % | 100 % |
| procedural | 15 | 3.5 | 35 | 16 | 57 % | 51 % | 26 % | 100 % |
| procedural | 15 | 4.0 | 40 | 10 | 68 % | 65 % | 30 % | 95 % |
| procedural | 15 | 4.5 | 40 | 9 | 45 % | 45 % | 22 % | 82 % |
| procedural | 15 | 5.0 | 35 | 9 | 29 % | 29 % | 26 % | 74 % |
| procedural | 20 | 1.0 | 40 | 71 | 78 % | 78 % | 65 % | 100 % |
| procedural | 20 | 1.5 | 40 | 46 | 78 % | 72 % | 65 % | 100 % |
| procedural | 20 | 2.0 | 40 | 35 | 75 % | 75 % | 52 % | 100 % |
| procedural | 20 | 2.5 | 40 | 28 | 68 % | 68 % | 40 % | 100 % |
| procedural | 20 | 3.0 | 40 | 20 | 90 % | 90 % | 62 % | 100 % |
| procedural | 20 | 3.5 | 40 | 20 | 62 % | 52 % | 45 % | 100 % |
| procedural | 20 | 4.0 | 40 | 16 | 72 % | 68 % | 57 % | 100 % |
| procedural | 20 | 4.5 | 40 | 14 | 75 % | 68 % | 60 % | 100 % |
| procedural | 20 | 5.0 | 35 | 14 | 40 % | 34 % | 20 % | 100 % |
| procedural | 30 | 1.0 | 40 | 94 | 82 % | 75 % | 78 % | 100 % |
| procedural | 30 | 1.5 | 40 | 79 | 75 % | 72 % | 68 % | 100 % |
| procedural | 30 | 2.0 | 40 | 52 | 88 % | 88 % | 78 % | 100 % |
| procedural | 30 | 2.5 | 40 | 42 | 85 % | 85 % | 75 % | 100 % |
| procedural | 30 | 3.0 | 40 | 36 | 80 % | 80 % | 65 % | 100 % |
| procedural | 30 | 3.5 | 40 | 26 | 88 % | 88 % | 80 % | 100 % |
| procedural | 30 | 4.0 | 40 | 25 | 92 % | 92 % | 85 % | 100 % |
| procedural | 30 | 4.5 | 40 | 20 | 80 % | 80 % | 70 % | 100 % |
| procedural | 30 | 5.0 | 35 | 17 | 80 % | 80 % | 66 % | 100 % |
| real | 10 | 1.0 | 40 | 31 | 65 % | 60 % | 32 % | 88 % |
| real | 10 | 1.5 | 40 | 21 | 68 % | 55 % | 42 % | 82 % |
| real | 10 | 2.0 | 40 | 16 | 60 % | 52 % | 25 % | 75 % |
| real | 10 | 2.5 | 40 | 14 | 48 % | 42 % | 20 % | 75 % |
| real | 10 | 3.0 | 40 | 12 | 52 % | 52 % | 22 % | 75 % |
| real | 10 | 3.5 | 35 | 10 | 40 % | 40 % | 3 % | 66 % |
| real | 10 | 4.0 | 35 | 9 | 34 % | 31 % | 11 % | 49 % |
| real | 10 | 4.5 | 35 | 8 | 26 % | 26 % | 14 % | 51 % |
| real | 10 | 5.0 | 35 | 7 | 0 % | 0 % | 0 % | 40 % |
| real | 15 | 1.0 | 40 | 49 | 72 % | 72 % | 55 % | 90 % |
| real | 15 | 1.5 | 40 | 35 | 75 % | 75 % | 62 % | 88 % |
| real | 15 | 2.0 | 40 | 28 | 65 % | 65 % | 52 % | 85 % |
| real | 15 | 2.5 | 40 | 21 | 78 % | 70 % | 28 % | 85 % |
| real | 15 | 3.0 | 40 | 17 | 65 % | 62 % | 35 % | 72 % |
| real | 15 | 3.5 | 35 | 15 | 40 % | 40 % | 14 % | 63 % |
| real | 15 | 4.0 | 40 | 13 | 60 % | 60 % | 12 % | 60 % |
| real | 15 | 4.5 | 40 | 11 | 42 % | 32 % | 28 % | 55 % |
| real | 15 | 5.0 | 35 | 10 | 43 % | 43 % | 9 % | 46 % |
| real | 20 | 1.0 | 40 | 61 | 68 % | 68 % | 62 % | 100 % |
| real | 20 | 1.5 | 40 | 44 | 78 % | 78 % | 68 % | 95 % |
| real | 20 | 2.0 | 40 | 34 | 75 % | 75 % | 50 % | 98 % |
| real | 20 | 2.5 | 40 | 28 | 68 % | 68 % | 30 % | 88 % |
| real | 20 | 3.0 | 40 | 23 | 72 % | 65 % | 38 % | 78 % |
| real | 20 | 3.5 | 40 | 20 | 52 % | 52 % | 12 % | 78 % |
| real | 20 | 4.0 | 40 | 17 | 75 % | 72 % | 28 % | 68 % |
| real | 20 | 4.5 | 40 | 16 | 78 % | 75 % | 40 % | 70 % |
| real | 20 | 5.0 | 35 | 14 | 46 % | 46 % | 3 % | 63 % |
| real | 30 | 1.0 | 40 | 92 | 88 % | 88 % | 88 % | 100 % |
| real | 30 | 1.5 | 40 | 70 | 85 % | 85 % | 78 % | 100 % |
| real | 30 | 2.0 | 40 | 51 | 82 % | 82 % | 62 % | 100 % |
| real | 30 | 2.5 | 40 | 40 | 88 % | 88 % | 72 % | 100 % |
| real | 30 | 3.0 | 40 | 34 | 75 % | 75 % | 52 % | 100 % |
| real | 30 | 3.5 | 40 | 30 | 78 % | 78 % | 62 % | 95 % |
| real | 30 | 4.0 | 40 | 29 | 80 % | 80 % | 68 % | 95 % |
| real | 30 | 4.5 | 40 | 24 | 65 % | 65 % | 52 % | 90 % |
| real | 30 | 5.0 | 35 | 21 | 60 % | 60 % | 43 % | 91 % |

Real flames, YOLO detector at 0.28, by true flame box width (all trays and distances):

| Box width (px) | Images | Detected, overlap | Detected, IoU 0.25 | YOLO on real photos, IoU 0.25 (vision/README.md) |
|---|---|---|---|---|
| 0-8 | 59 | 8 % | 8 % | 15 % |
| 8-12 | 151 | 32 % | 30 % | 32 % |
| 12-16 | 223 | 54 % | 52 % | 39 % |
| 16-24 | 327 | 64 % | 58 % | 58 % |
| 24-32 | 217 | 69 % | 67 % | 71 % |
| 32-48 | 236 | 86 % | 86 % | 79 % |
| 48-64 | 102 | 77 % | 77 % | 82 % |
| 64-96 | 67 | 79 % | 79 % | 90 % |
| 96-128 | 17 | 100 % | 100 % | 94 % |
| 128-more | 1 | 100 % | 100 % | 96 % |

False detections (boxes that overlap no flame), per image, all images:

| Flames | YOLO 0.28 | YOLO 0.5 | Baseline |
|---|---|---|---|
| procedural | 0.000 | 0.000 | 0.325 |
| real | 0.000 | 0.000 | 0.332 |

By real clip (phase % 5). Procedural: the same scenes, trays and placements with the procedural flames:

| Real clip | Images | YOLO 0.28, real | YOLO 0.28, procedural | YOLO 0.5, real | Baseline, real | Baseline, procedural |
|---|---|---|---|---|---|---|
| basket | 305 | 69 % | 61 % | 40 % | 100 % | 89 % |
| campfire_a | 310 | 55 % | 70 % | 41 % | 64 % | 92 % |
| campfire_b | 265 | 76 % | 75 % | 50 % | 49 % | 94 % |
| free_flame | 265 | 32 % | 59 % | 10 % | 87 % | 90 % |
| wood_fire | 255 | 84 % | 72 % | 53 % | 99 % | 90 % |

Images: 2840 (2796 with a visible flame). Hidden flames (furniture between) are not in the rates.
