"""Tests for the detector pre/post-processing, the FireDetection output and the calibration. Run: cd vision && uv run pytest -q"""
import json

import cv2
import numpy as np
import onnx
from onnx import TensorProto, helper

import calibrate
import detector


def fake_model(path, boxes, scores, h=320, w=416):
    """ONNX model that ignores its input and returns fixed boxes (cx, cy, w, h) and scores, like the exported network."""
    a = len(scores)
    b = np.asarray(boxes, np.float32).T[None]
    s = np.asarray(scores, np.float32)[None, None]
    g = helper.make_graph(
        [helper.make_node("Shape", ["images"], ["shp"]),
         helper.make_node("Constant", [], ["boxes"], value=helper.make_tensor("b", TensorProto.FLOAT, b.shape, b.ravel())),
         helper.make_node("Constant", [], ["scores"], value=helper.make_tensor("s", TensorProto.FLOAT, s.shape, s.ravel()))],
        "fake", [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, h, w])],
        [helper.make_tensor_value_info("boxes", TensorProto.FLOAT, [1, 4, a]),
         helper.make_tensor_value_info("scores", TensorProto.FLOAT, [1, 1, a])])
    onnx.save(helper.make_model(g, opset_imports=[helper.make_opsetid("", 12)], ir_version=8), path)


def test_letterbox_640x480():
    x, s, (px, py) = detector.letterbox(np.zeros((480, 640, 3), np.uint8), 416, 320)
    assert x.shape == (1, 3, 320, 416) and x.dtype == np.float32
    assert s == 0.65 and (px, py) == (0, 4)
    assert np.allclose(x[0, :, :4], 114 / 255) and np.allclose(x[0, :, 4:316], 0)


def test_decode_nms_and_split_outputs():
    # two overlapping boxes (keep the better one), one separate box, one box under the threshold
    out = np.array([[100, 100, 40, 40, 0.9], [102, 101, 40, 40, 0.8], [300, 200, 20, 30, 0.7], [50, 50, 10, 10, 0.1]], np.float32)
    one = out.T[None]
    b1, s1 = detector.decode([one], conf=0.35, iou=0.5)
    b2, s2 = detector.decode([one[:, :4], one[:, 4:]], conf=0.35, iou=0.5)
    assert np.allclose(s1, [0.9, 0.7]) and np.allclose(b1, b2) and np.allclose(s1, s2)
    assert np.allclose(b1[0], [80, 80, 120, 120])


def test_detect_output(tmp_path):
    # box (195, 138)-(221, 190) in the 416 x 320 input: scale 0.65, 4 px border at the top
    fake_model(tmp_path / "m.onnx", [[208, 164, 26, 52]], [0.8])
    det = detector.FireDetector(tmp_path / "m.onnx", calib=None)
    (d,) = det(np.zeros((480, 640, 3), np.uint8), 12.5)
    assert d.timestamp == 12.5 and d.camera == "rgb" and abs(d.confidence - 0.8) < 1e-6
    assert np.allclose(d.bbox, (195 / 0.65, 134 / 0.65, 221 / 0.65, 186 / 0.65))
    assert np.isclose(d.u, 320) and np.isclose(d.v, 186 / 0.65)  # bottom centre: the fire base on the floor
    det.conf = 0.9
    assert det(np.zeros((480, 640, 3), np.uint8), 0.0) == []


CALIB = dict(width=640, height=480, fx=300.0, fy=301.0, cx=322.0, cy=238.0, dist=[-0.30, 0.10, 0.001, -0.0005, -0.015])


def test_undistort_round_trip():
    """Distort ideal pixels with OpenCV's lens model, then undistort() must give them back."""
    K = np.array([[CALIB["fx"], 0, CALIB["cx"]], [0, CALIB["fy"], CALIB["cy"]], [0, 0, 1]])
    xy = np.random.default_rng(0).uniform(-1.0, 1.0, (300, 2)) * [1.0, 0.75]  # normalised ideal coordinates
    ideal = xy * [CALIB["fx"], CALIB["fy"]] + [CALIB["cx"], CALIB["cy"]]
    dist = cv2.projectPoints(np.c_[xy, np.ones(len(xy))], np.zeros(3), np.zeros(3), K, np.array(CALIB["dist"]))[0][:, 0]
    assert np.abs(detector.undistort(dist, CALIB) - ideal).max() < 0.05


def test_undistort_wide_lens_corners():
    """B0201 lens (5 Nov calibration): exact where the model rises, and a finite point at the far corners."""
    c = dict(fx=476.1, fy=477.3, cx=298.3, cy=222.6, dist=[-0.392, 0.184, 0.001, 0.001, -0.048])
    K = np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]])
    xy = np.random.default_rng(1).uniform(-1.0, 1.0, (500, 2)) * [1.2, 0.9]  # up to 50 deg from the axis
    ideal = xy * [c["fx"], c["fy"]] + [c["cx"], c["cy"]]
    dist = cv2.projectPoints(np.c_[xy, np.ones(len(xy))], np.zeros(3), np.zeros(3), K, np.array(c["dist"]))[0][:, 0]
    inside = np.hypot(*xy.T) < 1.2  # the radial model of this lens rises up to an ideal radius of about 1.3
    assert np.abs(detector.undistort(dist[inside], c) - ideal[inside]).max() < 0.05
    corners = detector.undistort(np.array([[0.0, 0.0], [639.0, 0.0], [0.0, 479.0], [639.0, 479.0]]), c)
    assert np.isfinite(corners).all() and (np.abs(corners - [c["cx"], c["cy"]]) < 3 * c["fx"]).all()


def test_calibrate_synthetic_views():
    """Render the board in 15 poses with a known camera and lens distortion. calibrate() must find them again."""
    K = np.array([[CALIB["fx"], 0, CALIB["cx"]], [0, CALIB["fy"], CALIB["cy"]], [0, 0, 1]])
    D = np.array(CALIB["dist"])
    sq = 60  # px per square in the flat board image
    board = np.kron((np.indices((7, 10)).sum(0) % 2).astype(np.uint8) * 255, np.ones((sq, sq), np.uint8))
    board = cv2.copyMakeBorder(board, sq, sq, sq, sq, cv2.BORDER_CONSTANT, value=255)
    # ideal pinhole pixel of each output (distorted) pixel
    uv = np.stack(np.meshgrid(np.arange(640.0), np.arange(480.0)), -1).reshape(-1, 1, 2)
    ideal = cv2.undistortPoints(uv, K, D, P=K).reshape(480, 640, 2).astype(np.float32)
    rng = np.random.default_rng(1)
    grays = []
    for _ in range(15):
        rvec = rng.uniform(-0.5, 0.5, 3)
        tvec = np.array([rng.uniform(-0.26, 0.01), rng.uniform(-0.18, 0.0), rng.uniform(0.28, 0.42)])
        corners = np.array([[-1, -1], [11, -1], [11, 8], [-1, 8]], np.float32) * 0.025  # outer corners with the border, m
        img_c = cv2.projectPoints(np.c_[corners, np.zeros(4)], rvec, tvec, K, None)[0].reshape(4, 2).astype(np.float32)
        src = np.array([[0, 0], [board.shape[1], 0], [board.shape[1], board.shape[0]], [0, board.shape[0]]], np.float32)
        Hm = cv2.getPerspectiveTransform(src, img_c)
        flat = cv2.warpPerspective(board, Hm, (640, 480), borderValue=128)
        grays.append(cv2.remap(flat, ideal[..., 0], ideal[..., 1], cv2.INTER_LINEAR, borderValue=128))
    r = calibrate.calibrate(grays, 9, 6, 0.025)
    assert r["views"] >= 10 and r["rms"] < 0.5
    assert abs(r["fx"] / CALIB["fx"] - 1) < 0.02 and abs(r["cx"] - CALIB["cx"]) < 5
    assert abs(r["dist"][0] - CALIB["dist"][0]) < 0.05
    json.dumps(r)


def test_find_corners_full_hd():
    """1920 x 1080: find() searches on a 640 px wide copy and refines on the full image. Each corner within 0.2 px of the truth."""
    sq = 90
    board = np.kron((np.indices((7, 10)).sum(0) % 2).astype(np.uint8) * 255, np.ones((sq, sq), np.uint8))
    board = cv2.copyMakeBorder(board, sq, sq, sq, sq, cv2.BORDER_CONSTANT, value=255)
    h, w = board.shape
    Hm = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), np.float32([[500, 200], [1500, 260], [1450, 900], [450, 820]]))
    img = cv2.GaussianBlur(cv2.warpPerspective(board, Hm, (1920, 1080), borderValue=128), (0, 0), 1.0)
    inner = np.stack(np.meshgrid(np.arange(1, 10), np.arange(1, 7)), -1).reshape(-1, 1, 2) * sq + sq - 0.5
    truth = cv2.perspectiveTransform(inner.astype(np.float32), Hm).reshape(-1, 2)
    c = calibrate.find(img, 9, 6, fast=True).reshape(-1, 2)
    assert len(c) == 54 and np.linalg.norm(c[:, None] - truth[None], axis=2).min(1).max() < 0.2


def test_camera_like_in_the_training_pipeline(tmp_path):
    """train.CameraLike changes the image and keeps the boxes. add_camera_like puts it into an Ultralytics dataset, also
    after close_mosaic, and the dataset still gives samples."""
    import random

    import pytest
    pytest.importorskip("ultralytics")  # the training env only: the CI fire-detector job does not install it
    import train
    from ultralytics.cfg import get_cfg
    from ultralytics.data import YOLODataset
    random.seed(0)
    np.random.seed(0)
    img = np.full((64, 64, 3), 120, np.uint8)
    img[20:30, 20:30] = 250
    out = train.CameraLike(p=1.0)(dict(img=img.copy(), instances="boxes stay"))
    assert out["img"].shape == img.shape and out["img"].dtype == np.uint8 and out["instances"] == "boxes stay"
    assert np.abs(out["img"].astype(int) - img).mean() > 1
    (tmp_path / "images").mkdir()
    (tmp_path / "labels").mkdir()
    for i in range(4):
        cv2.imwrite(str(tmp_path / f"images/{i}.jpg"), img)
        (tmp_path / f"labels/{i}.txt").write_text("0 0.39 0.39 0.16 0.16\n")
    hyp = get_cfg()
    ds = YOLODataset(img_path=str(tmp_path / "images"), imgsz=64, augment=True, hyp=hyp, data={"names": {0: "fire"}, "channels": 3})
    train.add_camera_like(ds, hyp)
    assert any(isinstance(x, train.CameraLike) for x in ds.transforms.transforms)
    ds.close_mosaic(hyp)
    assert any(isinstance(x, train.CameraLike) for x in ds.transforms.transforms)
    assert ds[0]["img"].shape[0] == 3


def test_screen_bench_true_box():
    """screen_bench: the white box shows where the source flame lands, at the aimed width, also near the screen edges.
    A bright spot in every frame (the mouse pointer, a wall seen after the camera turned) does not change the true box."""
    import screen_bench as sb
    from PIL import Image, ImageDraw

    class Spot(sb.FakeRig):
        def look(self, img, settle=0.0, n=1):
            f = super().look(img, settle, n)[0].copy()
            f[230:240, 600:620] = 250
            return [f] * n

    rig, box = Spot(), [100, 50, 120, 90]  # source flame 20 px wide, 40 px high
    src = Image.new("RGB", (300, 200))
    ImageDraw.Draw(src).rectangle([100, 50, 119, 89], fill="white")
    for width, gx, gy in [(4, 0.5, 0.5), (16, 0.1, 0.9), (45, 0.9, 0.1)]:
        tb = sb.target_box(box, width, gx, gy, sb.K_FAKE, rig.size)
        t = sb.true_box(rig, tb, 0.0)
        assert sb.bright_box(sb.grey(rig.look(sb.mask(tb, rig.size))[0]))[2] >= 620  # the old way: the spot is in the box
        assert abs(t[2] - t[0] - width) <= 1 and abs(t[3] - t[1] - 2 * width) <= 1
        seen = sb.bright_box(sb.grey(sb.FakeRig().look(sb.compose(src, box, tb, rig.size))[0]))
        assert max(abs(a - b) for a, b in zip(t, seen)) <= 1, (t, seen)


def test_range_test_truth_and_tiles(tmp_path):
    """range_test: the true box is the bright blob inside the label box, and tiled boxes come back in image pixels."""
    import range_test as rt
    f = np.full((480, 640, 3), 40, np.uint8)
    f[200:212, 300:304] = 250  # a small flame, 4 x 12 px
    assert rt.truth(f, [290, 190, 315, 220]) == [300, 200, 304, 212]
    assert rt.truth(f, None) is None
    fake_model(tmp_path / "m.onnx", [[208, 164, 26, 52]], [0.8])  # box (300, 206.2)-(340, 286.2) in a 640 x 480 tile
    b, s = rt.tiled(detector.FireDetector(tmp_path / "m.onnx", calib=None), np.zeros((1080, 1920, 3), np.uint8), 0.5)
    assert len(b) == 9 and np.allclose(s, 0.8)
    assert sorted({round(x) for x in b[:, 0]}) == [300, 940, 1580] and sorted({round(y) for y in b[:, 1]}) == [206, 506, 806]


def test_bench_compare_rates_and_videos(tmp_path):
    """bench_compare.rates: a fire frame counts when a box at the model threshold touches the true box; videos filters the trials."""
    import bench_compare as bc
    stim = [dict(video="a_candle.webm"), dict(video="b_led.webm"), dict(video="c_candle.webm")]
    (tmp_path / "setup.json").write_text(json.dumps(dict(stimuli=stim)))
    trials = [dict(stim=0, fire=True, ok=True, truth=[100, 100, 110, 130], frames=["f0"]),   # 10 px, found
              dict(stim=0, fire=True, ok=True, truth=[200, 100, 220, 140], frames=["f1"]),   # 20 px, box too weak
              dict(stim=2, fire=True, ok=True, truth=[300, 100, 312, 130], frames=["f2"]),   # 12 px, box elsewhere
              dict(stim=1, fire=False, ok=True, truth=[50, 50, 60, 70], frames=["f3"]),      # LED candle with a box
              dict(stim=2, fire=True, ok=False, truth=None, frames=["f4"])]                  # not valid: skipped
    (tmp_path / "trials.jsonl").write_text("".join(json.dumps(t) + "\n" for t in trials))
    det = {"f0": [[101, 99, 111, 128, 0.9]], "f1": [[200, 100, 220, 140, 0.05]], "f2": [[400, 100, 410, 130, 0.9]],
           "f3": [[0, 0, 5, 5, 0.9]], "f4": []}
    (tmp_path / "det_x_fire_yolo11n_640x480.json").write_text(json.dumps(det))
    r = bc.rates(tmp_path, "x_fire_yolo11n_640x480")  # unknown run x: threshold CONF
    assert r[(8, 16)] == 0.5 and r[(16, 32)] == 0.0 and r[(4, 8)] is None and r["led"] == 1.0
    r = bc.rates(tmp_path, "x_fire_yolo11n_640x480", videos=["a_"])
    assert r[(8, 16)] == 1.0 and r["led"] is None
    assert len(bc.trials(tmp_path, ["c_", "b_"])) == 2
    assert bc.led_threshold(tmp_path, "x_fire_yolo11n_640x480", 0.0) > 0.9  # no LED frame may keep its 0.9 box
    assert bc.rates(tmp_path, "x_fire_yolo11n_640x480", conf=0.91)[(8, 16)] == 0.0
    assert bc.led_threshold(tmp_path, "x_fire_yolo11n_640x480", 1.0) == 0.1
