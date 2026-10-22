"""Actuator node: aim commands in, pan-tilt servos and water pump out, measured angles back.

python actuator_node.py --mock                     mock servos and a mock pump
python actuator_node.py                            servos on config.toml [actuator] servo_port, pump on the [pump] GPIO

In:  UDP 127.0.0.1:CMD_PORT, the JSON command lines of aim/run.py (pan, tilt in rad, pump_on).
Out: ST3215 servos (ids in the calibration), the pump GPIO (robot/pump.py), and one JSON line per tick to each FEEDBACK_PORTS:
     {"t": time.monotonic() when the angles were read, "pan": rad, "tilt": rad, "water": pump output on, "estop": bool}
Ports, rate, timeout, calibration and pump pin come from robot/config.toml.
"""
import argparse
import json
import math
import signal
import socket
import sys
import time
from pathlib import Path

from st3215 import ServoError

sys.path.append(str(Path(__file__).resolve().parents[1] / "robot"))
from robot_config import CONF, HOST  # noqa: E402

_P, _A = CONF["ports"], CONF["actuator"]
CMD_PORT = _P["actuator_cmd"]  # aim/run.py commands
FEEDBACK_PORTS = [_P["feedback"], _P["feedback_sim"], _P["supervisor"]]  # aim/run.py --feedback, fake camera or tests, launch.py
RATE = _A["rate_hz"]  # Hz
CMD_TIMEOUT = _A["cmd_timeout_s"]  # s, water off and servos hold when no command arrives for this time
TICKS_PER_RAD = 4096 / (2 * math.pi)

# Calibration: ticks at angle 0, sign, limits and speed for each axis. Calibration steps: actuators/README.md.
DEFAULT_CALIB = {k: dict(_A[k]) for k in ("pan", "tilt")}


def clip_rad(c, rad):
    return min(max(rad, math.radians(c["min_deg"])), math.radians(c["max_deg"]))


def to_ticks(c, rad):
    return round(c["zero"] + c["direction"] * clip_rad(c, rad) * TICKS_PER_RAD)


def to_rad(c, ticks):
    return (ticks - c["zero"]) * c["direction"] / TICKS_PER_RAD


def step_toward(current, goal, max_step):
    return current + min(max(goal - current, -max_step), max_step)


class MockServos:
    """Two servos that move to the goal at their maximum speed."""

    def __init__(self, calib):
        self.calib, self.angles, self.volt = calib, {"pan": 0.0, "tilt": 0.0}, None

    def move(self, goals, dt):
        for k, g in goals.items():
            self.angles[k] = step_toward(self.angles[k], g, math.radians(self.calib[k]["max_speed_dps"]) * dt)

    def read(self):
        return self.angles["pan"], self.angles["tilt"]


class Servos:
    def __init__(self, calib, device):
        from st3215 import Bus
        self.calib, self.bus = calib, Bus.open(device)
        for c in calib.values():
            if not self.bus.ping(c["id"]):
                sys.exit(f"servo id {c['id']} does not answer on {device}")
            self.bus.torque(c["id"], True)

    def move(self, goals, dt):
        self.bus.sync_write_goal({self.calib[k]["id"]: (to_ticks(self.calib[k], g), round(math.radians(self.calib[k]["max_speed_dps"]) * TICKS_PER_RAD), 0)
                                  for k, g in goals.items()})

    def read(self):
        st = {k: self.bus.state(self.calib[k]["id"]) for k in ("pan", "tilt")}
        self.volt = st["pan"]["voltage"]  # the 3S voltage: the jet speed of the F1 pump follows it (aim/run.py)
        return tuple(to_rad(self.calib[k], st[k]["position"]) for k in ("pan", "tilt"))


def run(servos, pump, calib, seconds=None):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind((HOST, CMD_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    goals = dict(zip(("pan", "tilt"), servos.read()))
    pan, tilt = goals["pan"], goals["tilt"]
    cmd, t_cmd, dt = None, -math.inf, 1 / RATE
    tick = start = time.monotonic()
    while seconds is None or tick - start < seconds:
        while True:
            try:
                c = json.loads(rx.recv(65536))
                c = dict(pan=float(c["pan"]), tilt=float(c["tilt"]), pump_on=c["pump_on"] is True)
                if not (math.isfinite(c["pan"]) and math.isfinite(c["tilt"])):
                    raise ValueError("pan or tilt is not finite")
                cmd, t_cmd = c, time.monotonic()
            except BlockingIOError:
                break
            except (ValueError, KeyError, TypeError) as e:
                print(f"bad command line: {e}", file=sys.stderr)
        fresh = cmd is not None and time.monotonic() - t_cmd < CMD_TIMEOUT
        if fresh:  # software rate limit on the goal, the servo limits its speed too
            for k in goals:
                goals[k] = step_toward(goals[k], clip_rad(calib[k], float(cmd[k])), math.radians(calib[k]["max_speed_dps"]) * dt)
        try:
            servos.move(goals, dt)
            pan, tilt = servos.read()
            servo_ok = True
        except (ServoError, OSError) as e:  # a lost servo reply: water off, no feedback this tick
            print(f"servo bus: {e}", file=sys.stderr)
            servo_ok = False
        t_read = time.monotonic()
        pump.set(fresh and servo_ok and cmd["pump_on"])  # the pump checks the e-stop and its watchdogs
        line = json.dumps(dict(t=round(t_read, 4), pan=round(pan, 5), tilt=round(tilt, 5), water=pump.on, estop=pump.estop,
                               volt=getattr(servos, "volt", None)))
        for port in FEEDBACK_PORTS if servo_ok else ():
            try:
                tx.sendto(line.encode(), (HOST, port))
            except OSError:
                pass
        tick += dt
        time.sleep(max(0.0, tick - time.monotonic()))


def _stop(*_):
    raise KeyboardInterrupt  # SIGTERM (launch.py, systemd) stops the node like Ctrl+C: pump off, servos keep holding


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mock", action="store_true", help="no hardware: simulated servos and pump")
    ap.add_argument("--pump-file", help="with --mock: keep the mock pump pin in this file (launch.py --sim)")
    ap.add_argument("--servo-port", default=_A["servo_port"])
    ap.add_argument("--calib", help="calibration JSON, same keys as DEFAULT_CALIB (overrides config.toml)")
    ap.add_argument("--seconds", type=float, help="stop after this time (for tests)")
    a = ap.parse_args()
    calib = DEFAULT_CALIB
    if a.calib:
        with open(a.calib) as f:
            calib = {k: dict(DEFAULT_CALIB[k], **v) for k, v in json.load(f).items()}
    from pump import open_pump  # robot/pump.py
    signal.signal(signal.SIGTERM, _stop)
    servos = MockServos(calib) if a.mock else Servos(calib, a.servo_port)
    pump = open_pump(a.mock, a.pump_file, log=lambda m: print(m, file=sys.stderr))
    print(f"actuator_node: {'mock' if a.mock else 'hardware'}, commands on udp {CMD_PORT}, feedback to {FEEDBACK_PORTS}", file=sys.stderr)
    try:
        run(servos, pump, calib, a.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        pump.close()


if __name__ == "__main__":
    main()
