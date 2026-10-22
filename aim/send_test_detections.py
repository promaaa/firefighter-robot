"""Fake camera, detector and walking robot to test run.py without hardware.

python send_test_detections.py [F2|F1|A|B|E] [--feedback] [--v2]

It sends synthetic detections to run.py at 15 frames per second: one line per detection (interface v1),
or with --v2 one frame message per image, also when it sees no fire.
It reads the walk command of run.py (UDP 5602), moves a simulated robot, and sends its pose (UDP 5605).
It reads the pan and tilt commands of run.py (UDP 5601) to know where the camera points.
With --feedback, it reads the measured angles of actuators/actuator_node.py instead (UDP 5604).
The fire burns until the water has been on for 3 s, then the detections stop.
"""
import argparse
import json
import math
import socket
import time
from dataclasses import asdict

import numpy as np

from geometry import CONCEPTS
from run import CMD_PORTS, DET_PORT, HOST, POSE_PORT, RATE
from sim_demo import TURN_VX, fake_detections

FIRE = np.array([2.6, 0.8, 0.0])  # m, odometry frame, robot starts at (0, 0) facing +x
FPS = 15.0
BURN = 3.0  # s of water that puts the fire out
FEEDBACK_PORT = 5604  # second feedback port of actuators/actuator_node.py


def _socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, port))
    s.setblocking(False)
    return s


def _drain(sock):
    out = []
    while True:
        try:
            out.append(json.loads(sock.recv(65536)))
        except BlockingIOError:
            return out


def main(concept="F2", seconds=90.0, feedback=False, v2=False):
    cfg = CONCEPTS[concept]
    cam = next(iter(cfg["cameras"]))
    rx_angles = _socket(FEEDBACK_PORT if feedback else CMD_PORTS[0])  # this fake camera sits on the pan-tilt
    rx_walk = _socket(CMD_PORTS[1])  # the walking bridge port: this fake robot walks
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rng = np.random.default_rng()
    pan, tilt, pumped, state, sent, t_fb = 0.0, 0.0, 0.0, "", 0, None
    pose, vx, yaw_rate = np.zeros(3), 0.0, 0.0
    t_prev = time.monotonic()
    end = t_prev + seconds
    while time.monotonic() < end and state != "DONE":
        for m in _drain(rx_angles):
            pan, tilt = m["pan"], m["tilt"]
            if feedback:  # measured angles and the real water state, with their time
                pumped += m["water"] * (m["t"] - t_fb) if t_fb else 0.0
                t_fb = m["t"]
        for m in _drain(rx_walk):
            vx, yaw_rate, state = m["vx"], m["yaw_rate"], m["state"]
            pumped += 0.0 if feedback else m["pump_on"] / RATE
        now = time.monotonic()
        dt, t_prev = now - t_prev, now
        pose[2] += yaw_rate * min(1.0, abs(vx) / TURN_VX) * dt  # the policy does not turn on the spot
        pose[:2] += vx * dt * np.array([math.cos(pose[2]), math.sin(pose[2])])
        tx.sendto(json.dumps(dict(timestamp=now, x=pose[0], y=pose[1], yaw=pose[2])).encode(), (HOST, POSE_PORT))
        dets = fake_detections(cfg, cam, pan, tilt, pose, FIRE, now, rng) if pumped < BURN else []
        if v2:
            msg = dict(timestamp=now, camera=cam, frame=True, detections=[{k: v for k, v in asdict(d).items() if k not in ("timestamp", "camera")} for d in dets])
            tx.sendto(json.dumps(msg).encode(), (HOST, DET_PORT))
        elif dets:
            tx.sendto("\n".join(json.dumps(asdict(d)) for d in dets).encode(), (HOST, DET_PORT))
        sent += len(dets)
        time.sleep(1 / FPS)
    print(f"sent {sent} detections, water on for {pumped:.1f} s, robot at ({pose[0]:.2f}, {pose[1]:.2f}), "
          f"last state {state or 'none received'}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("concept", nargs="?", default="F2", choices=sorted(CONCEPTS))
    ap.add_argument("--feedback", action="store_true", help="read the angles from actuators/actuator_node.py")
    ap.add_argument("--v2", action="store_true", help="send one frame message per image (interface v2)")
    a = ap.parse_args()
    main(a.concept, feedback=a.feedback, v2=a.v2)
