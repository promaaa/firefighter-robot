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
