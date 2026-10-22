# Data card: fire detection dataset

**Not verified:** We did not check the labels by hand. The labels come from the dataset authors.
**Assumed:** "Research and education use" is allowed for all sources (see the licences below). The dataset is not for commercial use, because two sources are non-commercial.

The dataset has one class: fire. `data.py` drops the smoke boxes of D-Fire and FASDD.
`data.py` builds the dataset. It downloads every source without an account, converts it, and writes the counts to [results/data_stats.md](results/data_stats.md).
The dataset is at `~/datasets/firefighter/yolo/` (6 GB of images). It is not in git. The downloads take 26 GB in `raw/` and 8 GB in `extracted/`.
Training adds a 50 GB image cache (`.npy` files next to the images). You can delete the cache after training.

## Terms

- YOLO format: one text file for each image. Each line is `class x_centre y_centre width height`, divided by the image size. Class 0 is fire.
- Box width: the width of a fire box in pixels, after the image is resized to 640 px on its long side. This is the size in the 640 x 480 robot camera image.
- Small fire: a fire box with a width under 32 px.
- Hard negative: an image without fire that shows something like fire (a sunset, an orange object, a lamp).
- Near-duplicate: a test image that shows the same scene as a training image, for example the next frame of the same video.

## Sources

| Source | What | Licence | Where we download it |
|---|---|---|---|
| D-Fire [1] | 21 527 photos and surveillance-camera frames, mostly outdoor and wildfire. Fire and smoke boxes | CC0 1.0 (LICENSE file of the original repository) | Hugging Face mirror `hanvithSai/gavin-dfire-raw` (unchanged copy of the original train and test split) |
| FASDD_CV [2] | 95 126 images of flames and smoke: indoor, urban, wildfire, and many negatives with fire-like objects | CC BY 4.0 | Hugging Face mirror `seawsurf/fire_smoke_dataset_fasdd_cv` (Roboflow export `forestfiresmoke/fasdd_cv-dx83j` v3, resized to 640 px) |
| Barman flames [3] | 8 452 indoor images of candle, lighter and match flames, often small, on floors and tables | CC BY-NC-SA 4.0 (non-commercial, share-alike) | Hugging Face mirror `baizhanquan/FireDetectionDataset-flame-indoor-characteristic-flame-barman` |
| COCO 2017 [4] | General photos. We keep only fire-like images without fire (see below) as hard negatives | Annotations CC BY 4.0. Each image has its own Flickr licence ([results/coco_licences.json](results/coco_licences.json)): 26 % of the training picks are CC BY or CC BY-SA, the rest are non-commercial or no-derivatives | Hugging Face mirror `pcuenq/coco-2017-mirror` (the official zip files) |
| Firescenes | Our synthetic Blender images: rooms or outdoor ground with 1 to 3 fires and fire-like objects (`sim/firescenes/`) | Our own work | `sim/firescenes/out/`, or the GitHub release `firescenes-v1` (link in `data.py`) |

`data.py` pins each download URL to one revision.

`data.py` keeps a COCO image as a hard negative when a caption names a fire-coloured object or a light.
Examples: a sunset, an orange object, a lamp, neon, a traffic light, a fire hydrant, a fire truck.
It drops the image when any caption names a possible real fire.
Examples: a fire, a flame, a candle, smoke, a fireplace, a grill, a stove, an oven.
Captions can miss a real fire, so a few COCO "negatives" can contain a small fire.

## Splits

- D-Fire: the original test split is our test split. The original training split gives our validation split: blocks of 100 neighbouring images, 1 block in 10. Neighbouring frames of the same camera stay together.
- FASDD: the Roboflow train, valid and test splits. We keep 3 000 random images of the 9 513 valid images, so that validation is shorter.
- Barman: the Roboflow train, valid and test splits. `data.py` drops the 130 training images without a label file, because their status is unknown.
- COCO: fire-like images from `train2017` go to training (500 of them to validation). Fire-like images from `val2017` are the hard-negative test set.
- Firescenes: one image in four goes to test (67 images), the rest to training (203 images).

Each training epoch uses 62 719 images (`train.py all416`):

- all D-Fire, Barman and COCO training images,
- the 26 021 FASDD images with fire and 6 000 random FASDD images without fire,
- the 203 synthetic images, 5 times.

## Counts

From [results/data_stats.md](results/data_stats.md):

| Split | Source | Images | With fire | Without fire | Fire boxes | Small boxes (< 32 px) | Median box width (px) |
|---|---|---|---|---|---|---|---|
| train | D-Fire | 15 621 | 4 129 | 11 492 | 10 232 | 3 936 | 42 |
| train | FASDD | 76 101 | 26 021 | 50 080 | 60 108 | 17 797 | 57 |
| train | Barman | 7 777 | 7 763 | 14 | 7 784 | 3 608 | 35 |
| train | COCO fire-like | 6 285 | 0 | 6 285 | 0 | 0 | - |
| train | Firescenes | 203 | 202 | 1 | 270 | 115 | 38 |
| val | D-Fire | 1 600 | 578 | 1 022 | 1 575 | 732 | 36 |
| val | FASDD | 3 000 | 1 078 | 1 922 | 2 377 | 670 | 60 |
| val | Barman | 444 | 443 | 1 | 462 | 265 | 25 |
| val | COCO fire-like | 500 | 0 | 500 | 0 | 0 | - |
| test | D-Fire | 4 306 | 1 115 | 3 191 | 2 878 | 1 175 | 40 |
| test | FASDD | 9 512 | 3 313 | 6 199 | 5 472 | 1 113 | 88 |
| test | Barman | 101 | 100 | 1 | 105 | 51 | 33 |
| test | COCO fire-like | 330 | 0 | 330 | 0 | 0 | - |
| test | Firescenes | 67 | 67 | 0 | 90 | 36 | 38 |

Class balance: the training list of one epoch has 38 923 images with fire and 23 796 without fire (38 % without fire).

Image sizes before the resize: D-Fire is mostly 1280 x 720 and 640 x 360. FASDD is already 640 px on the long side (640 x 360, 640 x 640, 640 x 480).
Barman is 640 x 360. COCO is mostly 640 x 480. Firescenes is 1280 x 720.

## Near-duplicates between test and training images

The public splits are not clean. Many test images are frames of the same video or the same camera as a training image.
FASDD also contains D-Fire images, so a D-Fire test image can be in the FASDD training split.

`data.py` checks each test image against all training images in two steps:

1. A difference hash (dHash): the 8 x 9 grey image gives 64 bits. Candidates are training images within 10 bits.
2. The mean grey difference of 32 x 32 grey thumbnails (0 to 255). The test image is a near-duplicate when the difference is 10 or less.

We chose the limit of 10 by eye, on pairs of test and training images. Pairs up to about 10 show the same scene.
Some same-scene pairs are above 10, so the check misses a few.

| Test source | Test images | Near-duplicates | Left: "new scenes" | New-scene images with fire | New-scene fire boxes |
|---|---|---|---|---|---|
| D-Fire | 4 306 | 3 220 (75 %) | 1 086 | 203 | 566 |
| FASDD | 9 512 | 4 866 (51 %) | 4 646 | 791 | 1 716 |
| Barman | 101 | 100 (99 %) | 1 | 1 | 2 |
| COCO fire-like | 330 | 3 | 327 | 0 | 0 |
| Firescenes | 67 | 3 | 64 | 64 | 87 |

Of the D-Fire near-duplicates, 1 082 match a FASDD training image, not a D-Fire one.
The Barman test images are frames of the training scenes, so the Barman results show only "same scenes".
`evaluate.py` reports every result twice: on all test images (as in the papers) and on the new scenes only. The new-scene numbers are closer to a new room on the robot.

## References

1. P. V. A. B. de Venâncio, A. C. Lisboa, A. V. Barbosa. An automatic fire detection system based on deep convolutional neural networks for low-power, resource-constrained devices. Neural Computing and Applications, 2022. <https://github.com/gaiasd/DFireDataset>
2. M. Wang et al. FASDD: An open-access 100,000-level flame and smoke detection dataset for deep learning in fire detection. Geo-spatial Information Science, 2024. <https://doi.org/10.1080/10095020.2024.2347922>
3. S. Barman. Flame dataset (candle, lighter, match stick flames), Kaggle, version 1. <https://www.kaggle.com/datasets/sreemantabarman/flame-dataset-candlelightermatch-stick-flames>
4. T.-Y. Lin et al. Microsoft COCO: Common objects in context. ECCV 2014. <https://cocodataset.org>
