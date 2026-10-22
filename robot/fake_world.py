"""Fake world for robot/launch.py --sim: fires on the floor and a synthetic robot camera that sees them.

The robot pose is the true MuJoCo torso pose from legs/run_policy.py --fake (sim_truth port). Without it (--bench),
the robot stands at the origin. The pan and tilt angles come from the actuator node (feedback_sim port).
The pump state is the mock pump pin (a file), so a hung actuator node still sprays here, as on the robot.
A fire loses health while the jet lands within hit_radius of it, and goes out after burn_s seconds of water.
The camera model and the jet model are the ones of aim/ (aim/run.py robot_setup), so they match the controller exactly.
"""
import json
import math
import socket
import sys
import time
from pathlib import Path

import numpy as np

from robot_config import CONF, HOST

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "aim"))
from ballistics import jet_path  # noqa: E402
from geometry import camera_pose, nozzle_exit, project, robot_to_world, world_to_robot  # noqa: E402
from run import robot_setup  # noqa: E402  aim/run.py: the same camera size, field of view and jet speed as the controller

FLAME_HEIGHT = 0.3  # m
OUTER, INNER = (255, 120, 15), (255, 220, 60)  # flame colours, RGB


def _socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, port))
    s.setblocking(False)
    return s


def _latest(sock):
    m = None
    while True:
        try:
            m = json.loads(sock.recv(65536))
        except BlockingIOError:
            return m
        except ValueError:
            pass


class FakeWorld:
    def __init__(self, pump_file=None, fires=None, seed=0):
        s = CONF["sim"]
        self.cfg, self.cc = robot_setup(CONF["robot"]["concept"])
        self.k = self.cfg["cameras"]["rgb"]["K"]
        self.fires = np.array(s["fires"] if fires is None else fires, float).reshape(-1, 2)
        self.health = np.ones(len(self.fires))
        self.radius, self.hit, self.burn = s["fire_radius"], s["hit_radius"], s["burn_s"]
        self.pump_file = Path(pump_file) if pump_file else None
        self.truth_rx, self.fb_rx = _socket(CONF["ports"]["sim_truth"]), _socket(CONF["ports"]["feedback_sim"])
        self.pose, self.pan, self.tilt = (0.0, 0.0, 0.0, 0.0, 0.0), 0.0, self.cc.search_tilt
        self.rng, self.t_last = np.random.default_rng(seed), None
        h, w = self.k.height, self.k.width
        self.vv, self.uu = np.mgrid[0:h, 0:w]
        self.background = np.empty((h, w, 3), np.uint8)
        self.background[: h // 3] = (150, 150, 145)  # wall
        self.background[h // 3:] = (95, 95, 90)  # floor

    def water(self):
        if not self.pump_file:
            return False
        try:
            return self.pump_file.read_text().strip() == "1"
        except FileNotFoundError:
            return False

    def update(self):
        """Read the newest true pose and pan-tilt angles, then put water on the fires. Returns the landing point or None."""
        if m := _latest(self.truth_rx):
            self.pose = (m["x"], m["y"], m["yaw"], m["roll"], m["pitch"])
        if m := _latest(self.fb_rx):
            self.pan, self.tilt = m["pan"], m["tilt"]
        now = time.monotonic()
        dt, self.t_last = (now - self.t_last if self.t_last else 0.0), now
        if not self.water():
            return None
        land = self.landing()
        for i, (x, y) in enumerate(self.fires):
            if self.health[i] > 0 and math.hypot(land[0] - x, land[1] - y) < self.hit:
                self.health[i] = max(0.0, self.health[i] - dt / self.burn)
        return land

    def landing(self):
        """Where the jet reaches the floor, world (x, y)."""
        att = self.pose[3:5]
        e, d = nozzle_exit(self.cfg, self.pan, self.tilt, att)
        hor = d[:2] / max(np.linalg.norm(d[:2]), 1e-9)
        r = jet_path(self.cc.v0, math.asin(np.clip(d[2], -1, 1)), self.cc.drag, z_stop=-e[2], dt=5e-3)[-1][0]
        return robot_to_world(self.pose, (*(e[:2] + r * hor), 0.0))[:2]

    def render(self):
        """The robot camera image (H x W x 3 uint8) with the burning fires as orange ellipses on a grey room."""
        img = self.background.copy()
        att = self.pose[3:5]
        _, cam_t = camera_pose(self.cfg, "rgb", self.pan, self.tilt, att)
        vv, uu = self.vv, self.uu
        for (x, y), h in zip(self.fires, self.health):
            if h <= 0:
                continue
            p = world_to_robot(self.pose, (x, y, 0.0))
            base = project(self.cfg, "rgb", self.pan, self.tilt, p, att)
            top = project(self.cfg, "rgb", self.pan, self.tilt, p + (0, 0, FLAME_HEIGHT * (1 + 0.1 * self.rng.standard_normal())), att)
            if base is None or top is None:
                continue
            rx = max(1.0, self.radius * self.k.fx / np.linalg.norm(p - cam_t))
            ry = max(1.0, (base[1] - top[1]) / 2)
            cu, cv = base[0], base[1] - ry  # the bottom of the flame touches the fire base point
            for scale, colour in ((1.0, OUTER), (0.55, INNER)):
                inside = ((uu - cu) / (rx * scale)) ** 2 + ((vv - cv - ry * (1 - scale)) / (ry * scale)) ** 2 <= 1
                img[inside] = colour
        return img


if __name__ == "__main__":  # a quick look: write one image of the default fire seen from the start pose
    from PIL import Image
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "fake_world.png")
    Image.fromarray(FakeWorld().render()).save(out)
    print(f"wrote {out}")
