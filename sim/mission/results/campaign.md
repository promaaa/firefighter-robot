Comparison (the same mission seeds in every configuration). Robot: final design F2 (bhl_F2.xml, R2 tanks full).

| Configuration | Jet and fire model | Leg torque limit | Detector | Success | All fires out | Falls | Missions with a false spray | Median time to put out all fires (s) | Median water used (L) | Water that hit a fire |
|---|---|---|---|---|---|---|---|---|---|---|
| f2_kit_real | v0 9.2 m/s at 11.1 V, 0.39 L/min, 0.749 L, trays 10-20 cm | 6 Nm | baseline_detector.py, real flames | 3 of 50 | 5 | 19 | 16 | 49.4 | 0.198 | 54 % |
| f2_real_conf03 | v0 9.2 m/s at 11.1 V, 0.39 L/min, 0.749 L, trays 10-20 cm | 6 Nm | detector.py, real flames, min_conf 0.3 | 5 of 50 | 6 | 13 | 0 | 64.7 | 0.269 | 78 % |

Success: every fire out, and no fall or ABORT before the end of the mission. Times and water: successful missions only.

| Configuration | Missions | Success | 95 % interval | All fires out | Fires out | Median time to put out all fires (s) | 90th percentile (s) | Median water used (L) | Falls | Missions with a false spray | False detections per minute | Targets given up: not a fire / a fire |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| f2_kit_real: baseline colour detector, shipped BHL policy, 6 Nm, real-fire flames | 50 | 3 (6 %) | 2 % to 16 % | 5 | 27 of 102 | 49.4 | 117.3 | 0.198 | 19 | 16 | 1936.0 | 15 / 11 |
| f2_real_conf03: YOLO11n detector (vision/detector.py) with aim/ min_conf 0.3, shipped BHL policy, 6 Nm, real-fire flames | 50 | 5 (10 %) | 4 % to 21 % | 6 | 25 of 102 | 64.7 | 87.7 | 0.269 | 13 | 0 | 0.3 | 1 / 16 |

Why the missions failed:

| Configuration | reason | missions |
|---|---|---|
| f2_kit_real | fall | 19 |
| f2_kit_real | tank empty | 17 |
| f2_kit_real | search found no fire | 7 |
| f2_kit_real | time limit | 4 |
| f2_real_conf03 | tank empty | 16 |
| f2_real_conf03 | fall | 13 |
| f2_real_conf03 | search found no fire | 11 |
| f2_real_conf03 | time limit | 5 |

Success by number of fires and by scene:

| Configuration | 1 fire | 2 fires | 3 fires | indoor | outdoor |
|---|---|---|---|---|---|
| f2_kit_real | 2 of 17 | 0 of 14 | 1 of 19 | 1 of 34 | 2 of 16 |
| f2_real_conf03 | 3 of 17 | 2 of 14 | 0 of 19 | 5 of 34 | 0 of 16 |

Fires put out, by tray diameter (all fires of all missions, also missions that ended in a fall):

| Configuration | 10-12.5 cm | 12.5-15 cm | 15-17.5 cm | 17.5-20 cm | 20-25 cm | 25-30 cm |
|---|---|---|---|---|---|---|
| f2_kit_real | 12 of 19 | 10 of 28 | 5 of 35 | 0 of 20 | - | - |
| f2_real_conf03 | 5 of 19 | 12 of 28 | 8 of 35 | 0 of 20 | - | - |

Localisation error of single detections (floor-plane ray, aim/ geometry, true robot pose), by distance from the robot:

| Configuration | Distance (m) | Detections | Median error (m) | 90th percentile (m) |
|---|---|---|---|---|
| f2_kit_real | 0-1 | 12539 | 0.110 | 0.553 |
| f2_kit_real | 1-1.5 | 10940 | 0.098 | 0.647 |
| f2_kit_real | 1.5-2 | 25503 | 0.134 | 0.407 |
| f2_kit_real | 2-2.5 | 1824 | 0.187 | 0.364 |
| f2_kit_real | 2.5-3 | 3981 | 0.193 | 1.510 |
| f2_kit_real | 3-4 | 11977 | 0.796 | 2.791 |
| f2_kit_real | 4-5 | 2013 | 0.537 | 0.683 |
| f2_kit_real | 5-6 | 211 | 0.606 | 0.960 |
| f2_kit_real | 6-8 | 114 | 1.194 | 1.470 |
| f2_real_conf03 | 0-1 | 957 | 0.127 | 0.142 |
| f2_real_conf03 | 1-1.5 | 1541 | 0.050 | 0.151 |
| f2_real_conf03 | 1.5-2 | 15939 | 0.095 | 0.201 |
| f2_real_conf03 | 2-2.5 | 2187 | 0.099 | 0.258 |
| f2_real_conf03 | 2.5-3 | 1310 | 0.156 | 0.440 |
| f2_real_conf03 | 3-4 | 2342 | 0.153 | 0.387 |
| f2_real_conf03 | 4-5 | 922 | 0.186 | 0.422 |
| f2_real_conf03 | 5-6 | 107 | 1.119 | 1.929 |
| f2_real_conf03 | 6-8 | 26 | 0.292 | 0.735 |

Detection rate: share of camera images with a visible fire (10 flame pixels or more) in which the detector found it:

| Configuration | Distance (m) | Images | Detected |
|---|---|---|---|
| f2_kit_real | 0-1 | 4586 | 100 % |
| f2_kit_real | 1-1.5 | 6311 | 78 % |
| f2_kit_real | 1.5-2 | 24066 | 79 % |
| f2_kit_real | 2-2.5 | 3138 | 51 % |
| f2_kit_real | 2.5-3 | 3327 | 87 % |
| f2_kit_real | 3-4 | 4598 | 76 % |
| f2_kit_real | 4-5 | 3496 | 52 % |
| f2_kit_real | 5-6 | 389 | 54 % |
| f2_kit_real | 6-8 | 249 | 46 % |
| f2_real_conf03 | 0-1 | 1523 | 63 % |
| f2_real_conf03 | 1-1.5 | 3210 | 45 % |
| f2_real_conf03 | 1.5-2 | 39167 | 40 % |
| f2_real_conf03 | 2-2.5 | 3539 | 61 % |
| f2_real_conf03 | 2.5-3 | 1896 | 67 % |
| f2_real_conf03 | 3-4 | 4194 | 55 % |
| f2_real_conf03 | 4-5 | 2103 | 44 % |
| f2_real_conf03 | 5-6 | 286 | 37 % |
| f2_real_conf03 | 6-8 | 114 | 23 % |

Largest distance at which each fire was detected, over all fires:

| Configuration | Fires | Never detected | Median largest detection distance (m) | Fires detected from 4 m or more |
|---|---|---|---|---|
| f2_kit_real | 102 | 18 | 3.29 | 26 |
| f2_real_conf03 | 102 | 14 | 3.08 | 25 |

False detections by nearest object (all missions):

| Configuration | object | detections |
|---|---|---|
| f2_kit_real | other | 156679 |
| f2_kit_real | cone | 7938 |
| f2_kit_real | lamp | 4745 |
| f2_kit_real | ball | 4257 |
| f2_kit_real | red_box | 641 |
| f2_kit_real | extinguisher | 395 |
| f2_kit_real | poster | 242 |
| f2_real_conf03 | lamp | 21 |
| f2_real_conf03 | cone | 5 |

Run time: f2_kit_real: median 112 s wall for 114 s simulated, f2_real_conf03: median 117 s wall for 124 s simulated.
Crashed missions (not in the tables): f2_kit_real: 0, f2_real_conf03: 0.
