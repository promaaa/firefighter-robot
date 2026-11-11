# Vision on the real camera and board

Goal: the robot sees the demo fire (candles) at the demo distance, finds its position within 10 cm and aims the jet.
The robot probably does not walk, so the tests below use a robot that stands still and turns its pan-tilt.
This page lists the tests with the Arducam B0201 camera and the ROCK 3C board, in order, with their commands and pass criteria.
Review it at every weekly meeting.

Status: **planned**, unless a line says otherwise. Status words: built, tested in simulation, measured on laptop, measured on board, planned.

## Why candles are hard for our camera

- The B0201 has a wide lens with strong barrel distortion. At 640 x 480, the lens calibration (V3, 5 Nov) gives f = 476 px at the image centre and a real field of view of 80° x 59°. The old model value was f = 283 px.
- A candle flame is about 10 mm wide. Its width in the image is f × 0.010 m / distance: 4.8 px at 1 m, 3.2 px at 1.5 m. In the 1920 x 1080 mode, f = 1084 px: 10.8 px at 1 m.
- The screen bench with an ideal camera (`screen_bench.py --fake`, 3 Nov) gives 50 % detection from a flame width of 16 px with the 416 x 320 model and 11 px with the 640 x 480 input.
  With f = 476 px, that is one candle up to about 0.3 to 0.4 m. The robot sprays from 0.7 to 2 m.
- **Assumed:** the flame box of a candle is 10 mm wide. The range test (V4) measures it.

We can change five things without new hardware. The tests compare them on the same recordings:

| Id | Change | How we test it |
|---|---|---|
| A | A bigger demo fire: a group of candles. The team chooses the number, the layout and the distance. | V4 with `--fires candle,group` |
| B | More pixels: the 1920 x 1080 mode, with the detector on 640 x 480 tiles (a digital zoom) | V4 with `--modes 640x480,1920x1080`, then `report --tiled` |
| C | A model trained for small flames: `small416` (training images scaled by 0.1 to 1.9) | Done on 5 Nov: no gain (Results below). We keep the old model. |
| D | A short exposure: the flame keeps its colour and shape, the room gets dark | Screen bench, 6 Nov: at 1 ms every detector finds about twice as many flames as with the automatic exposure (Model changes below). V4 with `--exposures auto,10,3` checks it with real flames. |
| E | A colour detector for far flames | V4 report with `--detector sim/sim2sim/baseline_detector.py` |

## Test stages

| Id | Test | When | Who, time | Output | Pass criterion |
|---|---|---|---|---|---|
| V1 | Camera check on the lab desktop | 5 Nov | Marc, 20 min | camera modes, USB id, frame rate, a snapshot | 15 fps or more at 640 x 480 MJPEG |
| V2 | Screen bench with the B0201 | night of 5 Nov | nobody (4 h) | `report.md`, `report.png`, camera delay | 90 % or more of the images kept |
| V3 | Lens calibration | 6 Nov | Marc, 30 min | `vision/camera_b0201.json`, `results/calib_640x480.jpg` | RMS error under 0.5 px |
| V4 | Range test with real candles | 7 to 10 Nov | Marc and one helper, 1 h | `report.md`, `range.png`, `truth.jpg` | none: it is a measurement |
| V5 | Choice of the detector, camera mode and exposure | after V2 and V4 | laptop, 1 h | a decision in this page | the demo fire found in 90 % of the frames at the demo distance |
| V6 | Board speed and NPU accuracy | the day the ROCK 3C arrives | Marc, 1 h | `vision/results/board_bench_<host>.md`, NPU rows of the V2 and V4 reports | 10 fps or more for camera + detector. NPU detection rate within 5 points of ONNX |
| V7 | Camera node on the board | after V6 | Marc, 30 min | frame rate and detections of `robot/bench.py camera` | 15 fps, no frame gap over 200 ms |
| V8 | Training with our own B0201 images | week of 19 Nov | laptop GPU, 3 h | a new model | better detection than V5 on clips that are not in the training |
| V9 | Fire position from the head camera | week of 19 Nov, head on the robot | Marc and the hardware team, 1 h | position error from `aim/run.py` | 10 cm or less at 1.5 m |
| V10 | Spray at candles in bench mode | week of 26 Nov | the team | candles out or not | 3 of 5 runs (milestone M3) |
| V11 | Demo rehearsal in the demo room | week of 3 Dec | the team | | 5 of 5 runs. No spray in 5 min without fire |

## How to run the tests

Run all commands from the repository root, unless a step says otherwise.

### V1 camera check

1. Plug the B0201 into a USB port of the lab desktop.
2. Find the device name. The B0201 is usually the first `/dev/videoN` in the list.

```sh
v4l2-ctl --list-devices
```

3. Run the camera test. Use the device name from step 2.

```sh
uv run python robot/bench.py camera --device /dev/video0
```

4. Write the USB id that the test prints into `robot/config.toml` `[camera] usb`.
5. Save the list of camera modes for the report:

```sh
v4l2-ctl -d /dev/video0 --list-formats-ext > vision/results/b0201_modes.txt
```

6. Copy the snapshot that the test printed to `vision/results/b0201_first_image.png`.
7. Look at the live image. Close the window before the next test, because only one program can use the camera.

```sh
ffplay -f v4l2 -input_format mjpeg -video_size 1920x1080 /dev/video0
```

The M12 lens is focused at the factory. Turn it only when a page of text at 1.5 m is not sharp.

### V2 screen bench (overnight)

1. Fix the camera about 30 cm from the centre of the lower screen (DP-2), at the height of the screen centre and square to it. Tape the cable. This live view on the upper screen helps: the screen must fill the red frame, with its centre on the yellow square. Press `q` to close it.

```sh
hyprctl dispatch 'hl.dsp.focus({ monitor = "HDMI-A-1" })' && ffplay -f v4l2 -input_format mjpeg -video_size 640x480 -i /dev/video0 -vf "drawbox=x=32:y=78:w=576:h=324:color=red:t=2,drawbox=x=312:y=232:w=16:h=16:color=yellow:t=fill,scale=1280:960"
```

2. Set the screen brightness to its maximum.
3. Check the framing and the camera delay (1 min, no sweep). `setup.jpg` in the new run folder shows the white screen in a red box and one candle.

```sh
cd vision && uv run python screen_bench.py run --monitor DP-2 --repeat 0
```

4. Start the run. It shows the images full screen on DP-2 for a few hours, then it runs the two models on the saved frames. The current commands are below the lessons of the first run.

5. In the morning, read `report.md` and `report.png` in the run folder. `screen_bench.py report RUN --model FILE` adds a model later. The detections of the other models stay in their `det_*.json` files.
6. Put the median camera delay of `report.md` into `robot/config.toml` `[camera] latency_s`. It is an upper limit, because it also contains the screen delay.

The first image set (`--stimuli v1`) holds only two flames: six frames of one candle video and six of one tea light video. For the ideal camera, the frames of one video give the same 50 % width, and the two flames differ by a factor 2 (11 px and 6 px). So the second set (`--stimuli v2`) has eight flames, two frames each (`results/video_sources.md`). Its ideal camera and its run:

```sh
cd vision && uv run python screen_bench.py run --fake --stimuli v2 --repeat 3 --model models/fire_yolo11n_416x320.onnx --model ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_640x480.onnx
```

```sh
cd vision && uv run python screen_bench.py run --monitor DP-2 --stimuli v2 --repeat 3 --model models/fire_yolo11n_416x320.onnx --model ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_640x480.onnx --baseline ~/runs/firefighter/screen_bench/2026-11-05_1437_fake
```

`report.md` then also gives the 50 % width of each source video.

Lessons of the first full run (5 Nov, run `2026-11-05_1411`):

- The camera turned by 18 px between 14:33 and 14:48, and then stayed. The screen mask of the setup then held the bright wall, so 85 % of the true boxes were wrong. Now each true box is the difference of two camera frames (white box minus black screen), so a moved camera or the mouse pointer does not change it. The bench also moves the mouse pointer to the other screen.
- The automatic exposure brightens the dark screen, so the flame turns into a white blob with a halo. On 16 flames at the screen centre, a fixed 1 ms exposure found 13 at 45 px and 12 at 16 px, against 7 and 7 with the automatic exposure (8 px: 3 in both cases). `--exposure 10` sets 1 ms for a run.
- The camera sees the white screen as blue (R 161, G 174, B 252). A fixed white balance does not make it grey (6500 K: R 142, G 195, B 161), and RGB gains on the frames lowered the detection (7 to 2 of 16 at 45 px). So the bench keeps the automatic white balance and does not correct the colours.

The two runs of the night of 5 Nov (eight flames, 4320 images each, about 2 h 40 each):

```sh
cd vision && uv run python screen_bench.py run --monitor DP-2 --stimuli v2 --repeat 2 --model models/fire_yolo11n_416x320.onnx --model ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_640x480.onnx --baseline ~/runs/firefighter/screen_bench/2026-11-05_1437_fake
```

```sh
cd vision && uv run python screen_bench.py run --monitor DP-2 --stimuli v2 --repeat 2 --exposure 10 --model models/fire_yolo11n_416x320.onnx --model ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_640x480.onnx --baseline ~/runs/firefighter/screen_bench/2026-11-05_1437_fake
```

### V3 lens calibration

1. Print `vision/results/calib_board_9x6_25mm_A4.pdf` on A4 at 100 % (no scaling): 10 x 7 squares of 25 mm, 9 x 6 inner corners. `calibrate.py --board-pdf FILE` writes it again, with the same `--cols`, `--rows` and `--square` as the calibration.
2. Measure the line under the board. It must be 100 mm. When it is not, measure one square and use that size in step 3.
3. Glue the page flat on a rigid board. Then start the live calibration with the square size in metres.

```sh
cd vision && uv run python calibrate.py --camera /dev/video0 --square 0.025
```

4. Move the board or the camera: near, far, and tilted by 30 to 45° in many views. The window shows the corners of the saved views as red dots. Fill the whole image with them, also the corners.
   Hold each position still: the program saves only new and still positions, at most one each second. The window title counts them.
   On 5 Nov, series with views tilted by 25° or less gave f from 455 to 479 px, and series with tilts up to 34 to 38° gave 501 to 513 px. The fit of all views matched a board at a measured distance.
5. Read the RMS error and the `fy / fx` line at the end. Below 0.5 px is good. `fy / fx` must be close to 1, because the 640 x 480 mode is a crop (V1).
6. Do the same in the 1920 x 1080 mode. The result goes into `camera_b0201_1920x1080.json`.

```sh
cd vision && uv run python calibrate.py --camera /dev/video0 --square 0.025 --width 1920 --height 1080
```

7. Put the new `hfov_deg` into `robot/config.toml` `[camera]`. `vision/detector.py` and `aim/run.py` read `vision/camera_b0201.json` themselves.
8. Commit `vision/camera_b0201.json`, `vision/camera_b0201_1920x1080.json` and the two `vision/results/calib_*.jpg` figures.

The saved views stay in `~/runs/firefighter/calib/`. `calibrate.py --images FOLDER --images FOLDER2` calibrates again from them.
The calibration of 5 Nov uses the 640 x 480 folders `2026-11-05_1321`, `_1323`, `_1325`, `_1329` and the 1920 x 1080 folders `2026-11-05_1326`, `_1333`.

### V4 range test with real candles

**CAUTION:** Light candles only in a room where open flames are allowed. Put each candle on a fireproof plate and keep water or an extinguisher near it. A lit candle that falls can start a real fire.

Setup:

- Fix the camera at 0.9 m above the floor, the height of the robot camera. Point it down by about 40°, so that a candle at 0.3 m and a candle at 3 m are both in the image. The measured vertical field of view is 59°: at 40° down, the floor is in the image from 0.30 m to about 4 m. Check it in the live view before the first clip.
- Put a tape measure on the floor, from the point under the lens. Mark 0.3, 0.5, 1, 1.5, 2 and 3 m.
- Prepare one candle, and the group that the team proposes for the demo fire (for example 5 tea lights).
- The laptop stays behind the camera. Step out of the image during each clip.

1. Record. The script says what to place at each step and waits for Enter. It records 2 camera modes and 3 exposures at each step: automatic, 1 ms and 0.3 ms (about 45 s). 1 ms was the best exposure on the screen bench (V2). A real flame is much brighter than the screen, so 0.3 ms checks whether the flame core still saturates at 1 ms.

```sh
uv run python vision/range_test.py record --camera /dev/video0 --note "room, light, candle type"
```

2. Record the room without fire for false alarms, with the automatic exposure and at 1 ms. Turn the camera slowly by hand during each 60 s clip.

```sh
uv run python vision/range_test.py record --camera /dev/video0 --fires none --seconds 60 --modes 640x480 --exposures auto,10
```

3. Draw a box around the flames, once per position and camera mode. SESSION is the folder that step 1 printed.

```sh
uv run python vision/range_test.py label SESSION
```

4. Make the report with all detectors.

```sh
uv run python vision/range_test.py report SESSION --model vision/models/fire_yolo11n_416x320.onnx --model ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_640x480.onnx --model ~/runs/firefighter/vision/cam416/onnx/fire_yolo11n_640x480.onnx --model ~/runs/firefighter/vision/p2cam416/onnx/fire_yolo11n_640x480.onnx --model ~/runs/firefighter/vision/p2bench/onnx/fire_yolo11n_416x320.onnx --model ~/runs/firefighter/vision/p2bench/onnx/fire_yolo11n_640x480.onnx --tiled ~/runs/firefighter/vision/all416/onnx/fire_yolo11n_640x480.onnx --detector sim/sim2sim/baseline_detector.py
```

5. Look at `truth.jpg` first. Each green box must be on a flame. Run `label SESSION --redo` for a wrong one.
6. In `report.md`, read the table "Confirmed false alarms". It counts each time 3 frames with a box fall within 0.6 s in the clips without a flame. With this rule, the robot walks to a fire (`aim/controller.py`). Take the lowest threshold without one, for the detector, mode and exposure that the robot will use.
7. Run the command of step 4 again with `--conf T` at the end, where T is that threshold. The tables then show the distances that this threshold keeps. The detections are saved, so it is fast.

### V6 board speed and NPU accuracy

1. Do the bring-up steps 1 and 2 of [robot/README.md](../../robot/README.md#bring-up-on-the-robot): SD card, then `install.sh`. It also installs rknn-toolkit-lite2.
2. Install the NPU runtime library ([vision/README.md](../../vision/README.md#on-the-robot-board-rk3566), step 1).
3. On the laptop, copy the 640 x 480 NPU models and the NPU models of `cam416` and `p2bench` to the board. They are not in git. Use the board address from `hostname -I`. `board_bench.py` times `p2bench` too: its P2 head needs more computation than YOLO11n. `cam416` is the same network as the robot model, so it has the same speed.

```sh
rsync -a vision/models/fire_yolo11n_640x480_int8.rknn vision/models/fire_yolo11n_640x480_fp16.rknn ~/runs/firefighter/vision/rknn/cam416_fire_yolo11n_*_int8.rknn ~/runs/firefighter/vision/rknn/p2bench_fire_yolo11n_*_int8.rknn radxa@BOARD:firefighter-robot/vision/models/
```

4. Plug the B0201 into the board. Run the speed test on the board.

```sh
uv run python vision/board_bench.py
```

5. On the laptop, copy the V2 and V4 recordings to the board.

```sh
rsync -a ~/runs/firefighter/screen_bench ~/runs/firefighter/range_test radxa@BOARD:runs/firefighter/
```

6. On the board, add the NPU rows to the V4 report. Do the same with `screen_bench.py report` for V2.

```sh
uv run python vision/range_test.py report SESSION --model vision/models/fire_yolo11n_416x320_int8.rknn --model vision/models/fire_yolo11n_640x480_int8.rknn --model vision/models/cam416_fire_yolo11n_640x480_int8.rknn --model vision/models/p2bench_fire_yolo11n_416x320_int8.rknn --model vision/models/p2bench_fire_yolo11n_640x480_int8.rknn
```

7. Commit `vision/results/board_bench_<host>.md`.

### V7 camera node on the board

1. Run the camera test on the board with the NPU model:

```sh
FIRE_MODEL=vision/models/fire_yolo11n_416x320_int8.rknn uv run python robot/bench.py camera
```

2. Hold a lit candle at the demo distance in front of the camera. Run the test again. The number of detections must grow.

## Model changes after V2 and V4

The two screen bench runs (V2) and the candle test (V4) decide which change we train. Each new model is then tested on the frames that we already recorded, without a new camera session:
`screen_bench.py report RUN --model NEW.onnx` (screen bench), `range_test.py report SESSION --model NEW.onnx` (real candles) and `evaluate.py` (test images).
We train on the lab GPU servers or on the local RTX 3090.

| Id | Result | What it means | Change (`vision/train.py` variant) |
|---|---|---|---|
| M1 | V2: the B0201 needs 1.5 times the flame width of the ideal camera, or more | The camera blurs, compresses, and over- or underexposes small flames | `cam416`: start from all416, training images changed like the B0201 does (blur, exposure from x0.3 to x3 with a halo around the saturated light, white balance error, noise, JPEG). Then a fine-tune with the screen bench frames: the white box labels them. Test it on flames of other videos. |
| M2 | V2 and V4: even the ideal camera needs 11 px or more, and one candle at the demo distance gives about 5 px | The model is too coarse for very small flames | `p2_416`: YOLOv8n with an output at stride 4 (P2). `y26_416`: YOLO26n. Without a new model: 1920 x 1080 tiles (lever B). |
| M3 | V2: many boxes on LED candles, or boxes away from the flame | False alarms | Add frames without fire (LED candles, the demo room of V4) to the training as negatives. |
| M4 | V4: our candles are missed although they are wide enough | Our candles do not look like the training flames | V8: fine-tune with our labelled V4 frames. |

Check on 5 Nov: YOLOv8n-P2 (3.4 M parameters) and YOLO26n (2.4 M) with COCO weights export to ONNX, and rknn-toolkit2 2.5.0 builds them for the RK3566 (fp16), with the same setting as `rknn_convert.py` (`disable_rules=["seperate_conv_rs_split"]`). Their speed on the board is not measured. INT8 check on 6 Nov in the Rockchip simulator (`rknn_convert.py`, then `evaluate.py --deploy ... --rknn ...`, 918 fixed test images, [deploy_check.md](../../vision/results/deploy_check.md)): `p2bench` (YOLOv8n-P2, 416 x 320) keeps mAP50 0.803 in INT8 (ONNX 0.828) and the same small-fire recall (0.571). Our YOLO11n loses more in INT8: 0.775 (ONNX 0.816). The same check on 6 Nov for the other candidates, mAP50 INT8 (ONNX): `p2bench` 640 x 480 0.791 (0.823), `cam416` 416 x 320 0.788 (0.812), `cam416` 640 x 480 0.777 (0.825), our model at 640 x 480 0.780 (0.833). `p2bench` keeps the best INT8 mAP50 at both sizes. Measured on laptop, in the simulator.

Training on 5 Nov, on the lab server cauchy (RTX 3090, GPUs 0 to 2, tmux session `firefighter`, logs in `~/runs/firefighter/vision/logs/` there): `cam416` (15 epochs), `p2_416` and `y26_416` (20 epochs each), with `--workers 8 --cache none`. The project lock has torch for CUDA 13, but the cauchy driver (570) takes CUDA 12.8 at most, so cauchy has its own environment `~/envs/firefighter-vision` with torch 2.14.1+cu126 and Ultralytics 8.4.177. Each training took 31 to 35 min. `export.py` and `evaluate.py` then ran there too (they need `aim/detection.py` next to `vision/`). Results: see the table below and `vision/results/<variant>/metrics.md`. The ONNX files keep the name `fire_yolo11n_*` for all three architectures: the run name gives the architecture.

`p2cam416` trained later on 5 Nov on the lab server fermat (TITAN RTX, GPU 0, 15 epochs in 40 min), because the cauchy GPUs were then in use by other people. fermat does not share its home folder with cauchy, and its glibc (2.27) does not take the newest torch, so fermat has its own environment `~/envs/firefighter-vision` with torch 2.4.1+cu121. The script `~/projects/firefighter-robot/after_p2cam.sh` there ran the export, the evaluation and the screen bench report.

| Model (416 x 416 input, test images) | mAP50, all | New scenes | Small fires (< 32 px) | New scenes, small fires | Barman (candles, lighters, matches) | Fire-like images with a false fire (COCO) | New scenes without fire, images with a false fire |
|---|---|---|---|---|---|---|---|
| all416 (YOLO11n, now on the robot) | 0.809 | 0.747 | 0.529 | 0.511 | 0.944 | 2.1 % | 7.4 % |
| cam416 (all416 + camera-like images) | 0.815 | 0.765 | 0.532 | 0.525 | 0.949 | 2.4 % | 6.6 % |
| p2_416 (YOLOv8n-P2) | 0.822 | 0.756 | 0.575 | 0.540 | 0.976 | 3.3 % | 9.2 % |
| y26_416 (YOLO26n) | 0.830 | 0.778 | 0.572 | 0.554 | 0.953 | 3.6 % | 7.0 % |
| p2cam416 (p2_416 + camera-like images) | 0.830 | 0.772 | 0.577 | 0.540 | 0.974 | 1.8 % | 6.2 % |
| p2bench (p2cam416 + B0201 screen bench frames, 6 Nov) | 0.819 | 0.758 | 0.563 | 0.541 | 0.976 | 2.4 % | 8.5 % |

Each model has its own threshold (best F1 on the validation images): 0.284, 0.242, 0.227, 0.266, 0.231, 0.230. Measured on laptop (computed on cauchy and fermat). The screen bench frames of the night of 5 Nov test the same models with the real camera.

The same models on the screen bench frames (`vision/bench_compare.py RUN`), fire frames found by true flame width, each model at its own threshold. Ideal camera (`2026-11-05_1437_fake`) against the B0201 with the automatic exposure (`2026-11-05_1834`, 4261 of 4320 images kept):

| Model and input | Ideal, 8 to 15 px | Ideal, 16 to 31 px | B0201 auto, 8 to 15 px | B0201 auto, 16 to 31 px | B0201 auto, 32 px and more | B0201 auto, LED candle frames with a box |
|---|---|---|---|---|---|---|
| all416, 416 x 320 | 68 % | 99 % | 7 % | 30 % | 26 % | 3 % |
| all416, 640 x 480 | 91 % | 100 % | 10 % | 19 % | 7 % | 0 % |
| cam416, 416 x 320 | 72 % | 99 % | 10 % | 31 % | 32 % | 8 % |
| cam416, 640 x 480 | 89 % | 99 % | 21 % | 41 % | 21 % | 4 % |
| p2_416, 416 x 320 | 74 % | 99 % | 16 % | 41 % | 32 % | 6 % |
| p2_416, 640 x 480 | 93 % | 100 % | 33 % | 43 % | 26 % | 6 % |
| y26_416, 416 x 320 | 70 % | 98 % | 2 % | 23 % | 24 % | 3 % |
| y26_416, 640 x 480 | 89 % | 100 % | 6 % | 23 % | 15 % | 0 % |
| p2cam416, 416 x 320 | 76 % | 97 % | 18 % | 52 % | 62 % | 6 % |
| p2cam416, 640 x 480 | 90 % | 100 % | 42 % | 72 % | 47 % | 7 % |

With the automatic exposure, the flames turn into white blobs (V2 lessons). YOLO26n, the best on the test images, is the worst here. The P2 model and the camera-like images help the most, so `p2cam416` joins both (p2_416 + CameraLike, 15 epochs, lab server fermat, 5 Nov). It is the only model that reaches 50 %: at 640 x 480 it finds 42 % of the 8 to 15 px flames and 72 % of the 16 to 31 px flames (p2_416: 33 % and 43 %). Measured on laptop.

The same models with the B0201 at a fixed 1 ms exposure (`2026-11-05_2130`, 4316 of 4320 images kept), each model at its own threshold:

| Model and input | 8 to 15 px | 16 to 31 px | 32 px and more | LED candle frames with a box |
|---|---|---|---|---|
| all416, 416 x 320 | 18 % | 53 % | 54 % | 5 % |
| all416, 640 x 480 | 35 % | 42 % | 34 % | 1 % |
| cam416, 416 x 320 | 30 % | 77 % | 84 % | 15 % |
| cam416, 640 x 480 | 64 % | 92 % | 73 % | 17 % |
| p2_416, 416 x 320 | 40 % | 65 % | 46 % | 6 % |
| p2_416, 640 x 480 | 54 % | 53 % | 28 % | 4 % |
| y26_416, 416 x 320 | 28 % | 66 % | 66 % | 2 % |
| y26_416, 640 x 480 | 46 % | 63 % | 41 % | 0 % |
| p2cam416, 416 x 320 | 53 % | 80 % | 83 % | 22 % |
| p2cam416, 640 x 480 | 73 % | 88 % | 72 % | 28 % |

At 1 ms, every model finds more flames than with the automatic exposure: our detector (all416, 416 x 320) finds 53 % of the 16 to 31 px flames instead of 30 %. But the camera-like images also give more boxes on LED candles: 15 % to 28 % of these frames. So the threshold of the validation images does not fit the robot. `bench_compare.py RUN --led 0.02` sets each model's threshold so that at most 2 % of the LED-candle frames get a box. Then all models have the same false alarms:

| Model and input | Automatic: threshold | Automatic: 8 to 15 px | Automatic: 16 to 31 px | 1 ms: threshold | 1 ms: 8 to 15 px | 1 ms: 16 to 31 px |
|---|---|---|---|---|---|---|
| all416, 416 x 320 | 0.31 | 7 % | 29 % | 0.37 | 13 % | 47 % |
| all416, 640 x 480 | 0.11 | 18 % | 29 % | 0.23 | 38 % | 45 % |
| cam416, 416 x 320 | 0.41 | 6 % | 24 % | 0.43 | 15 % | 67 % |
| cam416, 640 x 480 | 0.34 | 17 % | 35 % | 0.43 | 48 % | 86 % |
| p2_416, 416 x 320 | 0.34 | 11 % | 34 % | 0.36 | 32 % | 59 % |
| p2_416, 640 x 480 | 0.36 | 25 % | 35 % | 0.36 | 45 % | 46 % |
| y26_416, 416 x 320 | 0.30 | 1 % | 21 % | 0.24 | 30 % | 68 % |
| y26_416, 640 x 480 | 0.10 | 12 % | 34 % | 0.10 | 53 % | 69 % |
| p2cam416, 416 x 320 | 0.36 | 12 % | 42 % | 0.55 | 19 % | 61 % |
| p2cam416, 640 x 480 | 0.36 | 33 % | 61 % | 0.61 | 27 % | 68 % |

With the same false alarms, `p2cam416` at 640 x 480 is the best with the automatic exposure. At 1 ms, `cam416` at 640 x 480 is the best on 16 to 31 px flames (86 %), and `y26_416`, `cam416` and `p2_416` at 640 x 480 find about half of the 8 to 15 px flames. YOLO26n at 640 x 480 has so few boxes on LED candles that its threshold stops at 0.1, the lowest score in the det files. The LED candles are only two videos, so these thresholds are a first value: the false alarms of the demo room (V4) set the final threshold. Measured on laptop (computed on fermat).

`p2bench` (M1 second step and M3) trained in the night of 5 to 6 Nov on fermat (GPUs 0 to 2, 22 min each, script `~/projects/firefighter-robot/bench_ft.sh` there). It starts from `p2cam416` and adds the B0201 frames of both screen bench runs to the training images, 5 epochs (`train.py`, `bench_list`: 15 166 frames). The true box is the label. An LED candle frame has no label, so it teaches "no fire". Flames under 4 px are left out. Two more models check it fairly: `p2bench_a` leaves out the videos of fold a (the three Jahobr candles, the Giles Laurent tea light, LED candle 1), and `p2bench_b` leaves out fold b (the four other flame videos, LED candle 2). Each one is tested on the frames of its left-out fold: `bench_compare.py RUN --videos <fold> --led 0.02`. The three Jahobr candles look alike, so they stay in one fold.

Left-out videos only, each model at the threshold that gives at most 2 % LED-candle frames with a box. Each cell: flames 8 to 15 px / 16 to 31 px found.

| Model and input | Automatic, fold a | Automatic, fold b | 1 ms, fold a | 1 ms, fold b |
|---|---|---|---|---|
| all416, 416 x 320 (now on the robot) | 10 / 43 % | 4 / 17 % | 21 / 69 % | 5 / 25 % |
| cam416, 640 x 480 | 37 / 55 % | 2 / 21 % | 66 / 97 % | 30 / 75 % |
| p2cam416, 416 x 320 | 20 / 59 % | 3 / 24 % | 41 / 91 % | 12 / 35 % |
| p2cam416, 640 x 480 | 53 / 76 % | 15 / 43 % | 72 / 98 % | 5 / 19 % |
| p2bench of the fold, 416 x 320 | 52 / 67 % | 31 / 53 % | 93 / 95 % | 56 / 79 % |
| p2bench of the fold, 640 x 480 | 82 / 81 % | 43 / 58 % | 100 / 100 % | 64 / 78 % |

On flames that it did not see, the model trained with the B0201 frames finds the most flames in every column, with the same false alarms. Example, 1 ms, fold b, 416 x 320: 56 % of the 8 to 15 px flames, against 12 % for `p2cam416` and 5 % for our detector now. At 1 ms, the fold models also put fewer boxes on the LED candle that they did not see: 3 % to 15 % of these frames at their own threshold, against 17 % to 36 % for `p2cam416`. On the test images, `p2bench` loses a little (table above): mAP50 0.819 instead of 0.830, and 8.5 % instead of 6.2 % of the new scenes without fire get a false fire. In INT8 (Rockchip simulator) it keeps mAP50 0.803. Measured on laptop (computed on fermat).

INT8 on the same left-out frames (6 Nov, Rockchip simulator, `rknn_convert.py RUN/onnx/FILE --bench RUN_DIR --videos <fold>`, files `det_*_int8_sim.json` in the run folder). 1 ms, each model at the threshold that gives at most 2 % LED-candle frames with a box. Each cell: flames 8 to 15 px / 16 to 31 px found.

| Model and input | Fold a, ONNX | Fold a, INT8 | Fold b, ONNX | Fold b, INT8 |
|---|---|---|---|---|
| all416, 416 x 320 (now on the robot) | 21 / 69 % | 20 / 63 % | 5 / 25 % | 4 / 25 % |
| all416, 640 x 480 | 57 / 57 % | 57 / 53 % | 19 / 33 % | 19 / 31 % |
| p2bench of the fold, 416 x 320 | 93 / 95 % | 95 / 96 % | 56 / 79 % | 54 / 80 % |
| p2bench of the fold, 640 x 480 | 100 / 100 % | 100 / 100 % | 64 / 78 % | 64 / 79 % |

In INT8, the `p2bench` fold models change by 2 points at most, and their thresholds by 0.04 at most. Our detector loses up to 6 points. At each model's own threshold, `p2bench` changes by 1 point at most, and our detector at 640 x 480 loses 8 points on fold a. So the gain of `p2bench` on small flames stays in INT8. The full `p2bench` saw all these videos, so its fold models stand in for it. The simulator is not the NPU: V6 step 6 repeats the check on the board. Measured on laptop (simulator).

Limits: the left-out flames are still images on the same screen, filmed in the same night, so the model can learn the look of this screen. The gain on real candles can be smaller: only V4 shows it. `p2bench` (all videos) is the model for the robot, but its own screen bench numbers are not a fair test, because it saw all the videos.

Proposal for the decision of 11 Nov (Marc): a fixed 1 ms exposure, if V4 confirms it with real flames. `p2bench` if V4 confirms its gain on real candles, else `cam416`. The confirmed false alarms of the demo room (V4 steps 2 and 6: 3 frames with a box within 0.6 s) set the threshold, and the board speed (V6) sets the input size.

## Decisions and dates

| Date | Decision | Who | Based on |
|---|---|---|---|
| 10 Nov | The demo fire: number and layout of the candles, and the distance to the robot | the team | V4 |
| 11 Nov | Camera mode, exposure and detector of the robot | Marc | V2, V4, V5 |
| board day + 1 | NPU or CPU, 416 x 320 or 640 x 480 input | Marc | V6 |
| 25 Nov | Keep the model of V8 or not | Marc | V8 |

Requirement R1 of the [plan](README.md#requirements-and-verification) is for a 10 cm flame at 3 m. When the team has chosen the demo fire, R1 becomes: "the camera sees the demo fire at the demo distance in 90 % of the frames, with no false fire in 5 min of the demo room".

## Results

| Test | Date | Result |
|---|---|---|
| Camera check V1 (`robot/bench.py camera`) | 5 Nov | USB id 0c45:0261. 640 x 480 MJPEG: 14.8 fps over 20 s (15 fps set). 1920 x 1080 MJPEG: 30 fps. 640 x 480 is the centre 1440 x 1080 of the 1920 x 1080 image, scaled by 1/2.25: edge correlation 0.92 (whole image squeezed: 0.39). Measured on laptop. |
| Lens calibration V3 (`calibrate.py`) | 5 Nov | 640 x 480: f = 476.1 px (fy 477.3), cx 298.3, cy 222.6, k1 -0.39, RMS 0.51 px, 100 views in 4 hand-held series. Real field of view 80° x 59°. 1920 x 1080: f = 1083.6 px (481.6 px at 640 x 480 scale), RMS 1.53 px, 50 views. Check: the board at a measured 400 mm comes out at 400.8 mm (640 x 480) and 406.4 mm (1920 x 1080). The same board in both modes: x640 = 0.4445 x1080 - 106.0 (crop model: 0.4444 x1080 - 106.9), residual 0.08 px. The radial model stops rising at about 390 px from the image centre (the far corners, 0.5 % of the image): `detector.undistort` stops there. Measured on laptop. |
| Screen bench setup check V2 (`--repeat 0`, run `2026-11-05_1403`) | 5 Nov | Screen at [39, 82, 601, 428] in the camera image, 0.344 camera px per screen px, delay from the screen to the frame: median 251 ms (20 changes). The first check found a bug: with the lens distortion, the box around the screen also holds the bright desk and wall, so the white square came out 562 px wide (scale 2.08). The bench now uses only the lit screen pixels (`screen_mask.png`). Measured on laptop. |
| First full screen bench run V2 (run `2026-11-05_1411`, 2 flames) | 5 Nov | Not valid. The camera turned by 18 px after 20 min, so only 1167 of 7776 true boxes were right, most of them from the first 20 min. The bench now measures each true box against a black frame. Measured on laptop. |
| Exposure check on the screen bench (16 flames of set v2, screen centre, one frame each) | 5 Nov | Flames found by the 416 x 320 model at 45, 16 and 8 px: automatic exposure 7, 7, 3. Fixed 1 ms: 13, 12, 3. Fixed 3 ms: 10, 8, 3. Fixed 8 ms: 6 at 45 px. Measured on laptop. |
| Screen bench V2, B0201 with the automatic exposure (run `2026-11-05_1834`, set v2: eight flames, two LED candles) | 5 Nov | 4261 of 4320 images kept. Our detector (416 x 320) finds 7 % of the 8 to 15 px flames and 30 % of the 16 to 31 px flames (ideal camera: 68 % and 99 %). Best model: `p2cam416` at 640 x 480, 42 % and 72 %. See "Model changes". Measured on laptop. |
| Screen bench V2, B0201 at a fixed 1 ms exposure (run `2026-11-05_2130`) | 6 Nov | 4316 of 4320 images kept. Our detector finds 18 % and 53 %. With at most 2 % LED-candle frames with a box: `cam416` at 640 x 480 finds 48 % and 86 %. See "Model changes". Measured on laptop. |
| `p2bench`: B0201 screen bench frames in the training, two folds of left-out videos | 6 Nov | 1 ms, 416 x 320, 2 % LED-candle frames with a box: 56 % to 93 % of the 8 to 15 px flames of videos that the model did not see (our detector now: 5 % to 21 %). Test images: mAP50 0.819 (`p2cam416`: 0.830). INT8 in the Rockchip simulator: 0.803. Measured on laptop. |
| INT8 check on the same left-out screen bench frames (Rockchip simulator, 1 ms) | 6 Nov | At 2 % LED-candle frames with a box, the `p2bench` fold models change by 2 points at most: 8 to 15 px flames at 416 x 320, 95 % and 54 % (ONNX 93 % and 56 %). Our detector loses up to 6 points. Measured on laptop (simulator). |
| Ideal camera on the screen bench (`--fake`) | 3 Nov | 50 % detection from 16 px (candle, 416 x 320) and 11 px (candle, 640 x 480). Tea light: 8 px and 6 px. Measured on laptop. |
| `small416` on the same ideal-camera frames | 5 Nov | Candle: 50 % from 23 px (416 x 320) and 16 px (640 x 480), so worse than the old model. Tea light: 8 px and 6 px, the same. Measured on laptop. |
| `small416` on the test images ([metrics.md](../../vision/results/small416/metrics.md)) | 5 Nov | mAP50 0.808 (old model 0.809), small fires 0.525 (0.529), new scenes 0.749 (0.747). False fires on fire-like images: 2.1 % (2.1 %). Measured on laptop. |
| Board speed V6 (`board_bench.py`, video input) | 11 Nov | Camera + detector on the ROCK 3C NPU, INT8: our detector 21.2 fps at 416 x 320 and 13.5 fps at 640 x 480. `p2bench` 14.2 fps at 416 x 320, 8.3 fps at 640 x 480 (below 10 fps). CPU only, 4 threads: 3.8 fps. Setup needed: NPU overlay and librknnrt 2.5.0 (`install.sh` step 11), and a 5 V 3 A supply: on a computer USB port the board restarted under load. Measured on board. |
| Camera node V7 (`robot/bench.py camera`) | 11 Nov | B0201 on the board, 1 ms exposure, candle video on the screen: 14.5 fps, the flame found in 267 of 267 frames with `fire_yolo11n_416x320_int8.rknn`. `p2bench` on the NPU boxes the whole candle (INT8 and fp16), while ONNX on the laptop boxes the flame: open issue, the robot keeps our detector. NPU accuracy on the bench frames: not run yet. Measured on board. |

Licence: this document is CC BY-SA 4.0.
