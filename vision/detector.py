"""Fire detector plug-in: a YOLO11n network (ONNX) run with onnxruntime. Needs only onnxruntime, numpy and pillow.

Interface (the same for every detector module, FireDetection is in aim/detection.py): detect(rgb, t) -> list[FireDetection]
rgb: H x W x 3 uint8 RGB image. t: time of the image, s.
u, v of each detection is the bottom centre of the box, because the fire base is on the floor.
When camera_b0201.json (from calibrate.py) exists and matches the image size, u, v and the box are corrected for the
lens distortion: they are pixels of an ideal pinhole camera with the calibrated fx, fy, cx, cy (as aim/geometry.py assumes).

The environment variable FIRE_MODEL selects another model file (.onnx, or .rknn on the robot board).

Use with the camera node (from the repo root):
  uv run python robot/camera_node.py --source video:FILE --detector vision/detector.py
"""
import json
import os
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "aim"))
from detection import FireDetection  # noqa: E402

HERE = Path(__file__).resolve().parent
# Input 416 x 320 (W x H): a 640 x 480 image becomes 416 x 312 (4 px border). On the robot board, set
# FIRE_MODEL=vision/models/fire_yolo11n_416x320_int8.rknn to use the NPU.
MODEL = Path(os.environ.get("FIRE_MODEL", HERE / "models/fire_yolo11n_416x320.onnx"))
CALIB = HERE / "camera_b0201.json"
CONF = 0.28  # best F1 score on the real validation images. aim/ then keeps only boxes from its min_conf (0.3 for F2)
NMS_IOU = 0.5  # boxes that overlap more than this with a better box are removed
PAD = 114  # grey letterbox border, as in training


def letterbox(rgb, w, h):
    """Resize to fit in w x h (keep the aspect ratio), pad with grey. Returns the 1x3xHxW float input, scale, (pad_x, pad_y)."""
    h0, w0 = rgb.shape[:2]
    s = min(w / w0, h / h0)
    nw, nh = round(w0 * s), round(h0 * s)
    px, py = (w - nw) // 2, (h - nh) // 2
    canvas = np.full((h, w, 3), PAD, np.uint8)
    canvas[py:py + nh, px:px + nw] = np.asarray(Image.fromarray(rgb).resize((nw, nh), Image.BILINEAR))
    return canvas.transpose(2, 0, 1)[None].astype(np.float32) / 255.0, s, (px, py)


def nms(boxes, scores, iou):
    """Greedy non-maximum suppression. boxes: N x 4 (x0, y0, x1, y1). Returns the kept indices, best first."""
    order = scores.argsort()[::-1]
    area = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    keep = []
    while order.size:
        i, rest = order[0], order[1:]
        keep.append(i)
        xx0 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy0 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx1 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy1 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(xx1 - xx0, 0, None) * np.clip(yy1 - yy0, 0, None)
        order = rest[inter / (area[i] + area[rest] - inter + 1e-9) <= iou]
    return np.array(keep, dtype=int)


def decode(outputs, conf, iou, max_det=100):
    """Network output -> (boxes N x 4 in input pixels, scores N). Accepts one (1, 5, A) output or (1, 4, A) + (1, 1, A)."""
    out = np.concatenate([o.reshape(o.shape[0], -1, o.shape[-1]) for o in outputs], axis=1)[0]  # 5 x A: cx, cy, w, h, score
    keep = out[4] >= conf
    cx, cy, w, h, score = out[:, keep]
    boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
    k = nms(boxes, score, iou)[:max_det]
    return boxes[k], score[k]


def undistort(pts, c, iters=10):
    """Distorted pixels (N x 2) -> pixels of the ideal pinhole camera with the same fx, fy, cx, cy.
    c: calibration dict with fx, fy, cx, cy and dist = k1, k2, p1, p2, k3 (OpenCV model). Fixed-point iteration, like
    cv2.undistortPoints."""
    k1, k2, p1, p2, k3 = c["dist"]
    xd = (pts[:, 0] - c["cx"]) / c["fx"]
    yd = (pts[:, 1] - c["cy"]) / c["fy"]
    x, y = xd.copy(), yd.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
        x = (xd - (2 * p1 * x * y + p2 * (r2 + 2 * x * x))) / rad
        y = (yd - (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)) / rad
    return np.stack([x * c["fx"] + c["cx"], y * c["fy"] + c["cy"]], axis=1)


class FireDetector:
    def __init__(self, model=MODEL, conf=CONF, threads=4, camera="rgb", calib=CALIB):
        self.conf, self.camera = conf, camera
        self.calib = json.loads(Path(calib).read_text()) if calib and Path(calib).exists() else None
        if str(model).endswith(".rknn"):  # robot board: the NPU, through rknn-toolkit-lite2. Built, not tested on the board
            from rknnlite.api import RKNNLite
            self.rknn = RKNNLite()
            if self.rknn.load_rknn(str(model)) != 0 or self.rknn.init_runtime() != 0:
                raise RuntimeError(f"cannot load {model} on the NPU")
            w, _, h = re.search(r"_(\d+)(x(\d+))?", Path(model).stem).groups()  # fire_yolo11n_416x320_int8.rknn: W 416, H 320
            self.w, self.h = int(w), int(h or w)
            return
        self.rknn = None
        import onnxruntime as ort  # here, so that the board can run the .rknn model without onnxruntime
        so = ort.SessionOptions()
        so.intra_op_num_threads = threads
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(str(model), so, providers=["CPUExecutionProvider"])
        self.input = self.sess.get_inputs()[0].name
        self.h, self.w = self.sess.get_inputs()[0].shape[2:]

    def run(self, x):
        if self.rknn:  # the RKNN model takes H x W x 3 uint8 and divides by 255 itself
            return self.rknn.inference(inputs=[np.round(x[0].transpose(1, 2, 0) * 255).astype(np.uint8)[None]])
        return self.sess.run(None, {self.input: x})

    def boxes(self, rgb, conf=None):
        """Boxes (N x 4, x0 y0 x1 y1, image pixels) and scores, best first."""
        x, s, (px, py) = letterbox(rgb, self.w, self.h)
        b, sc = decode(self.run(x), self.conf if conf is None else conf, NMS_IOU)
        b = (b - [px, py, px, py]) / s
        h0, w0 = rgb.shape[:2]
        return np.clip(b, 0, [w0, h0, w0, h0]), sc

    def __call__(self, rgb, t):
        b, sc = self.boxes(rgb)
        uv = np.stack([(b[:, 0] + b[:, 2]) / 2, b[:, 3]], axis=1)
        c = self.calib
        if c and (c["width"], c["height"]) == (rgb.shape[1], rgb.shape[0]) and len(b):
            uv = undistort(uv, c)
            corners = undistort(b.reshape(-1, 2), c).reshape(-1, 4)  # (x0, y0) and (x1, y1): a close box for small fires
            b = np.concatenate([np.minimum(corners[:, :2], corners[:, 2:]), np.maximum(corners[:, :2], corners[:, 2:])], 1)
        return [FireDetection(t, self.camera, float(u), float(v), tuple(float(x) for x in box), float(s))
                for (u, v), box, s in zip(uv, b, sc)]


_detector = None


def detect(rgb, t):
    global _detector
    if _detector is None:
        _detector = FireDetector()
    return _detector(rgb, t)
