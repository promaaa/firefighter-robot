"""Minimal runtime: fire detections in over UDP, controller at 10 Hz, commands out as JSON lines.

python run.py [F2|F1|A|B|E] [--feedback] [--no-walk]

Input: UDP datagrams on 127.0.0.1:DET_PORT. Each line is one FireDetection as a JSON object (interface v1),
or one frame message {"timestamp", "camera", "frame": true, "detections": [...]} (interface v2, see aim/README.md).
Pose: {"timestamp", "x", "y", "yaw"} lines on POSE_PORT (odometry frame), optional "roll" and "pitch" (IMU, rad). Without a pose for POSE_MAX_AGE,
run.py integrates its own walk command (dead reckoning). --no-walk keeps the pose at (0, 0, 0) for bench tests.
Output: one JSON line per tick on stdout, and the same line as a UDP datagram to each port in CMD_PORTS.
Without --feedback, a mock actuator: the pan-tilt reaches each command at once.
With --feedback, the measured angles come from actuators/actuator_node.py on UDP FEEDBACK_PORT.
Ports, the camera size and field of view, and ControllerConfig overrides come from robot/config.toml.
"""
import argparse
import json
import math
import socket
import sys
import time
from dataclasses import asdict, fields
from pathlib import Path

from controller import Controller, ControllerConfig
from detection import FireDetection
from geometry import CONCEPTS, intrinsics_from_fov

sys.path.append(str(Path(__file__).resolve().parents[1] / "robot"))
from robot_config import CONF, HOST  # noqa: E402  ports, camera size and controller overrides: robot/config.toml

_P = CONF["ports"]
DET_PORT = _P["detections"]  # detections in
CMD_PORTS = [_P["actuator_cmd"], _P["walk_cmd"], _P["supervisor"]]  # commands out: actuator driver (pan, tilt, pump_on), walking bridge (vx, yaw_rate), robot/launch.py
FEEDBACK_PORT = _P["feedback"]  # measured pan and tilt from actuators/actuator_node.py (with --feedback)
POSE_PORT = _P["pose"]  # robot pose in the odometry frame
FEEDBACK_MAX_AGE = 0.5  # s, older feedback is not used
V_REF, V_EXP = 11.1, 0.73  # F1 only: its pump ran on the 3S pack, so v0 = v0(11.1 V) * (V / 11.1)^0.73
# Limit: F2 v0 is fixed. Its pump follows the motor bus, v0 ~ 10.0 (V / 22.2)^0.8 (docs/hardware/water.md): send legs/robot.py vbus() with the pose to scale it.
POSE_MAX_AGE = 0.5  # s, older pose: dead reckoning
RATE = 10.0  # Hz, controller rate
FIELDS = {f.name for f in fields(FireDetection)}


def _check_time(t, now):
    t = float(t)
    if not 0 <= now - t <= 1.0:
        raise ValueError(f"timestamp {t:.3f} is not within 1 s before now ({now:.3f}): use time.monotonic()")
    return t


def _detection(obj, cameras, now):
    d = FireDetection(**{k: v for k, v in obj.items() if k in FIELDS})
    if d.camera not in cameras:
        raise ValueError(f"unknown camera, expected one of {sorted(cameras)}")
    d.bbox = tuple(float(x) for x in d.bbox)
    if len(d.bbox) != 4:
        raise ValueError("bbox needs 4 numbers")
    for k in ("u", "v", "confidence"):
        setattr(d, k, float(getattr(d, k)))
    for k in ("max_temp_c", "depth_m"):
        if getattr(d, k) is not None:
            setattr(d, k, float(getattr(d, k)))
    d.timestamp = _check_time(d.timestamp, now)
    return d


def parse(datagram, cameras, now):
    """(detections, frame messages) from one datagram. Bad lines are reported on stderr and skipped."""
    dets, frames = [], 0
    for line in datagram.decode(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            obj = json.loads(line)
            if isinstance(obj, dict) and obj.get("frame") is True:  # interface v2: one message per camera image
                if obj["camera"] not in cameras:
                    raise ValueError(f"unknown camera, expected one of {sorted(cameras)}")
                base = dict(timestamp=_check_time(obj["timestamp"], now), camera=obj["camera"])
                dets += [_detection({**base, **d}, cameras, now) for d in obj["detections"]]
                frames += 1
            else:  # interface v1: one detection per line
                dets.append(_detection(obj, cameras, now))
        except (ValueError, TypeError, AttributeError, KeyError) as e:
            print(f"bad detection {line[:200]!r}: {e}", file=sys.stderr)
    return dets, frames


class Odometry:
    """Robot pose (x, y, yaw, roll, pitch): from POSE_PORT messages, else dead reckoning of the walk command.

    Dead reckoning keeps the last roll and pitch."""

    def __init__(self, walk=True):
        self.pose, self.t_msg, self.walk, self.reckoning = (0.0, 0.0, 0.0, 0.0, 0.0), -math.inf, walk, False

    def message(self, m, now):
        pose = tuple(float(m.get(k, 0.0) if k in ("roll", "pitch") else m[k]) for k in ("x", "y", "yaw", "roll", "pitch"))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError("pose is not finite")
        self.pose, self.t_msg = pose, _check_time(m["timestamp"], now)

    def advance(self, now, vx, yaw_rate, dt):
        """After each command: integrate it when the pose input is older than POSE_MAX_AGE."""
        fresh = now - self.t_msg < POSE_MAX_AGE
        if fresh or not self.walk:
            self.reckoning = False
            return
        if not self.reckoning:
            print("run.py: no fresh pose on udp %d, dead reckoning from the walk command" % POSE_PORT, file=sys.stderr)
            self.reckoning = True
        x, y, yaw, roll, pitch = self.pose
        yaw += yaw_rate * dt
        self.pose = (x + vx * math.cos(yaw) * dt, y + vx * math.sin(yaw) * dt, yaw, roll, pitch)


def _socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, port))
    s.setblocking(False)
    return s


def _drain(sock):
    out = []
    while sock:
        try:
            out.append(sock.recv(65536))
        except BlockingIOError:
            break
    return out


def robot_setup(concept):
    """The concept geometry with the camera size and field of view of config.toml [camera], and the ControllerConfig
    with the overrides of config.toml [aim.controller] (for example v0 from the jet test)."""
    cam = CONF["camera"]
    cfg = dict(CONCEPTS[concept], cameras={k: dict(v) for k, v in CONCEPTS[concept]["cameras"].items()})
    if "rgb" in cfg["cameras"]:
        cfg["cameras"]["rgb"]["K"] = intrinsics_from_fov(cam["width"], cam["height"], cam["hfov_deg"])
    return cfg, ControllerConfig(**{**cfg.get("controller", {}), **CONF.get("aim", {}).get("controller", {})})


def main(concept="F2", feedback=False, walk=True):
    cfg, cc = robot_setup(concept)
    ctl, v0_ref = Controller(cfg, cc), cc.v0
    rx, pose_rx = _socket(DET_PORT), _socket(POSE_PORT)
    fb_rx = _socket(FEEDBACK_PORT) if feedback else None
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    odo = Odometry(walk)
    pan, tilt = 0.0, 0.0  # mock actuator: measured angles = last command
    fb, warned, v2 = None, False, False
    tick = time.monotonic()
    print(f"run.py: concept {concept}, detections on udp {HOST}:{DET_PORT}, pose on {POSE_PORT}, commands to udp ports {CMD_PORTS}",
          file=sys.stderr)
    while True:
        datagrams = _drain(rx)
        now = time.monotonic()
        dets, frames = [], 0
        for g in datagrams:
            d, f = parse(g, cfg["cameras"], now)
            dets, frames = dets + d, frames + f
        v2 = v2 or frames > 0  # the detector sends frame messages: a tick without one is a late frame, not "no fire"
        for g in _drain(pose_rx):
            try:
                odo.message(json.loads(g), now)
            except (ValueError, KeyError, TypeError) as e:
                print(f"bad pose: {e}", file=sys.stderr)
        for g in _drain(fb_rx):
            try:
                m = json.loads(g)
                fb = (float(m["t"]), float(m["pan"]), float(m["tilt"]))
                if concept == "F1" and m.get("volt"):  # the servos read the 3S voltage that drove the F1 pump
                    cc.v0 = v0_ref * (float(m["volt"]) / V_REF) ** V_EXP
            except (ValueError, KeyError, TypeError) as e:
                print(f"bad feedback: {e}", file=sys.stderr)
        t_angles = None
        if fb and 0 <= now - fb[0] < FEEDBACK_MAX_AGE:
            t_angles, pan, tilt = fb
        elif fb_rx and not warned:
            print("run.py: no fresh feedback, using the commanded angles", file=sys.stderr)
            warned = True
        cmd = ctl.step(now, dets, odo.pose if walk else (0.0, 0.0, 0.0, *odo.pose[3:]), pan, tilt, t_angles, frames if v2 else None)
        pan, tilt = cmd.pan, cmd.tilt
        odo.advance(now, cmd.vx, cmd.yaw_rate, 1 / RATE)
        line = json.dumps(dict(t=round(now, 3), **asdict(cmd)))
        print(line, flush=True)
        for port in CMD_PORTS:
            try:
                tx.sendto(line.encode(), (HOST, port))
            except OSError:
                pass  # nobody listens: stdout is enough
        if cmd.state == "DONE":
            break
        tick += 1 / RATE
        time.sleep(max(0.0, tick - time.monotonic()))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("concept", nargs="?", default="F2", choices=sorted(CONCEPTS))
    ap.add_argument("--feedback", action="store_true", help="use the measured angles of actuators/actuator_node.py")
    ap.add_argument("--no-walk", action="store_true", help="bench test: the robot does not walk, pose stays (0, 0, 0)")
    a = ap.parse_args()
    try:
        main(a.concept, a.feedback, not a.no_walk)
    except KeyboardInterrupt:
        pass
