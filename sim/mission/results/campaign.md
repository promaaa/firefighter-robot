Comparison (the same mission seeds in every configuration). Robot: final design F2 (bhl_F2.xml, R2 tanks full).

| Configuration | Jet and fire model | Leg torque limit | Detector | Success | All fires out | Falls | Missions with a false spray | Median time to put out all fires (s) | Median water used (L) | Water that hit a fire |
|---|---|---|---|---|---|---|---|---|---|---|
| f2_kit_real | v0 9.2 m/s at 11.1 V, 0.39 L/min, 0.749 L, trays 10-20 cm | 6 Nm | baseline_detector.py, real flames | 5 of 50 | 8 | 20 | 22 | 73.1 | 0.275 | 48 % |
| f2_real_conf03 | v0 9.2 m/s at 11.1 V, 0.39 L/min, 0.749 L, trays 10-20 cm | 6 Nm | detector.py, real flames, min_conf 0.3 | 8 of 50 | 8 | 12 | 0 | 66.4 | 0.246 | 77 % |

Success: every fire out, and no fall or ABORT before the end of the mission. Times and water: successful missions only.

| Configuration | Missions | Success | 95 % interval | All fires out | Fires out | Median time to put out all fires (s) | 90th percentile (s) | Median water used (L) | Falls | Missions with a false spray | False detections per minute | Targets given up: not a fire / a fire |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f2_kit_real: baseline colour detector, shipped BHL policy, 6 Nm, real-fire flames | 50 | 5 (10 %) | 4 % to 21 % | 8 | 32 of 102 | 73.1 | 102.8 | 0.275 | 20 | 22 | 2309.7 | 21 / 13 |
| f2_real_conf03: YOLO11n detector (vision/detector.py) with aim/ min_conf 0.3, shipped BHL policy, 6 Nm, real-fire flames | 50 | 8 (16 %) | 8 % to 29 % | 8 | 37 of 102 | 66.4 | 80.7 | 0.246 | 12 | 0 | 0.0 | 1 / 18 |

Why the missions failed:

| Configuration | reason | missions |
|---|---|---|
| f2_kit_real | fall | 20 |
| f2_kit_real | tank empty | 19 |
| f2_kit_real | search found no fire | 5 |
| f2_kit_real | time limit | 1 |
| f2_real_conf03 | tank empty | 15 |
| f2_real_conf03 | search found no fire | 12 |
| f2_real_conf03 | fall | 12 |
| f2_real_conf03 | time limit | 3 |

Success by number of fires and by scene:

| Configuration | 1 fire | 2 fires | 3 fires | indoor | outdoor |
|---|---|---|---|---|---|
| f2_kit_real | 3 of 17 | 2 of 14 | 0 of 19 | 3 of 34 | 2 of 16 |
| f2_real_conf03 | 5 of 17 | 3 of 14 | 0 of 19 | 6 of 34 | 2 of 16 |

Fires put out, by tray diameter (all fires of all missions, also missions that ended in a fall):

| Configuration | 10-12.5 cm | 12.5-15 cm | 15-17.5 cm | 17.5-20 cm | 20-25 cm | 25-30 cm |
|---|---|---|---|---|---|---|
| f2_kit_real | 10 of 19 | 11 of 28 | 11 of 35 | 0 of 20 | - | - |
| f2_real_conf03 | 11 of 19 | 15 of 28 | 11 of 35 | 0 of 20 | - | - |

Localisation error of single detections (floor-plane ray, aim/ geometry, true robot pose), by distance from the robot:

| Configuration | Distance (m) | Detections | Median error (m) | 90th percentile (m) |
|---|---|---|---|---|
| f2_kit_real | 0-1 | 12631 | 0.108 | 0.487 |
| f2_kit_real | 1-1.5 | 21182 | 0.136 | 0.765 |
| f2_kit_real | 1.5-2 | 33103 | 0.152 | 0.547 |
| f2_kit_real | 2-2.5 | 4012 | 0.147 | 0.530 |
| f2_kit_real | 2.5-3 | 1606 | 0.279 | 0.730 |
| f2_kit_real | 3-4 | 10588 | 0.523 | 2.610 |
| f2_kit_real | 4-5 | 1295 | 0.200 | 0.635 |
| f2_kit_real | 5-6 | 147 | 0.479 | 0.724 |
| f2_kit_real | 6-8 | 68 | 0.668 | 0.782 |
| f2_real_conf03 | 0-1 | 127 | 0.066 | 0.246 |
| f2_real_conf03 | 1-1.5 | 1977 | 0.043 | 0.097 |
| f2_real_conf03 | 1.5-2 | 18052 | 0.100 | 0.181 |
| f2_real_conf03 | 2-2.5 | 3066 | 0.105 | 0.228 |
| f2_real_conf03 | 2.5-3 | 1234 | 0.177 | 0.383 |
| f2_real_conf03 | 3-4 | 2264 | 0.172 | 0.430 |
| f2_real_conf03 | 4-5 | 1122 | 0.193 | 0.464 |
| f2_real_conf03 | 5-6 | 156 | 0.462 | 0.715 |
| f2_real_conf03 | 6-8 | 133 | 0.460 | 0.867 |

Detection rate: share of camera images with a visible fire (10 flame pixels or more) in which the detector found it:

| Configuration | Distance (m) | Images | Detected |
|---|---|---|---|
| f2_kit_real | 0-1 | 1972 | 98 % |
| f2_kit_real | 1-1.5 | 4524 | 99 % |
| f2_kit_real | 1.5-2 | 24461 | 84 % |
| f2_kit_real | 2-2.5 | 2977 | 99 % |
| f2_kit_real | 2.5-3 | 1212 | 95 % |
| f2_kit_real | 3-4 | 3397 | 89 % |
| f2_kit_real | 4-5 | 1783 | 66 % |
| f2_kit_real | 5-6 | 227 | 65 % |
| f2_kit_real | 6-8 | 157 | 43 % |
| f2_real_conf03 | 0-1 | 379 | 33 % |
| f2_real_conf03 | 1-1.5 | 2937 | 67 % |
| f2_real_conf03 | 1.5-2 | 33506 | 53 % |
| f2_real_conf03 | 2-2.5 | 4750 | 67 % |
| f2_real_conf03 | 2.5-3 | 1916 | 64 % |
| f2_real_conf03 | 3-4 | 2961 | 75 % |
| f2_real_conf03 | 4-5 | 1498 | 75 % |
| f2_real_conf03 | 5-6 | 262 | 60 % |
| f2_real_conf03 | 6-8 | 189 | 70 % |

Largest distance at which each fire was detected, over all fires:

| Configuration | Fires | Never detected | Median largest detection distance (m) | Fires detected from 4 m or more |
|---|---|---|---|---|
| f2_kit_real | 102 | 25 | 3.27 | 26 |
| f2_real_conf03 | 102 | 14 | 3.33 | 33 |

False detections by nearest object (all missions):

| Configuration | object | detections |
|---|---|---|
| f2_kit_real | other | 205675 |
| f2_kit_real | cone | 6831 |
| f2_kit_real | ball | 4573 |
| f2_kit_real | lamp | 2073 |
| f2_kit_real | poster | 40 |
| f2_kit_real | extinguisher | 2 |
| f2_kit_real | red_box | 1 |
| f2_real_conf03 | other | 2 |

Run time: f2_kit_real: median 79 s wall for 126 s simulated, f2_real_conf03: median 80 s wall for 122 s simulated.
Crashed missions (not in the tables): f2_kit_real: 0, f2_real_conf03: 0.
