Written by `data.py`. Box width in pixels after the resize to 640 px on the long side. Small box: width under 32 px.

| Split | Source | Images | With fire | Without fire | Fire boxes | Small boxes | Median box width (px) | Most common original sizes |
|---|---|---|---|---|---|---|---|---|
| train | dfire | 15621 | 4129 | 11492 | 10232 | 3936 | 42 | 1280x720 (6560), 640x360 (3015), 480x360 (538) |
| train | fasdd | 76101 | 26021 | 50080 | 60108 | 17797 | 57 | 640x360 (18549), 640x640 (9823), 640x480 (6356) |
| train | barman | 7777 | 7763 | 14 | 7784 | 3608 | 35 | 640x360 (7777) |
| train | coco | 6285 | 0 | 6285 | 0 | 0 | - | 640x480 (1358), 640x427 (748), 480x640 (465) |
| train | synthetic | 203 | 202 | 1 | 270 | 115 | 38 | 1280x720 (203) |
| val | dfire | 1600 | 578 | 1022 | 1575 | 732 | 36 | 1280x720 (810), 640x360 (278), 480x360 (54) |
| val | fasdd | 3000 | 1078 | 1922 | 2377 | 670 | 60 | 640x360 (770), 640x640 (355), 640x480 (267) |
| val | barman | 444 | 443 | 1 | 462 | 265 | 25 | 640x360 (444) |
| val | coco | 500 | 0 | 500 | 0 | 0 | - | 640x480 (124), 640x427 (55), 480x640 (33) |
| test | dfire | 4306 | 1115 | 3191 | 2878 | 1175 | 40 | 1280x720 (1797), 640x360 (839), 480x360 (156) |
| test | fasdd | 9512 | 3313 | 6199 | 5472 | 1113 | 88 | 640x360 (2893), 640x640 (1004), 640x480 (796) |
| test | barman | 101 | 100 | 1 | 105 | 51 | 33 | 640x360 (101) |
| test | coco | 330 | 0 | 330 | 0 | 0 | - | 640x480 (82), 640x427 (31), 480x640 (23) |
| test | synthetic | 67 | 67 | 0 | 90 | 36 | 38 | 1280x720 (67) |

Test images that are near-duplicates of a training image (dHash within 10 bits of 64 and mean grey difference of 32 x 32 thumbnails <= 10.0): barman 100, coco 3, dfire 3220, fasdd 4866, synthetic 3

Mean grey difference from each test image to the most similar training image (2-unit bins, 20 = 20 or more): 0: 6241, 2: 1146, 4: 387, 6: 241, 8: 177, 10: 118, 12: 106, 14: 91, 16: 80, 18: 88, 20: 5641
