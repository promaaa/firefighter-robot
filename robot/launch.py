"""Start, watch and stop all robot nodes. One command for the whole robot software.

    uv run python robot/launch.py --sim              all fake: legs on fake drives (MuJoCo), mock servos and pump, synthetic camera
    uv run python robot/launch.py --sim --bench      the same without legs: the robot stands still at the origin
    uv run python robot/launch.py --bench            real pan-tilt, pump and camera. No legs.
    uv run python robot/launch.py --full             the whole robot
    uv run python robot/launch.py --send ready       the robot stands: start the walking policy (the same as Enter)
    uv run python robot/launch.py --send stop        safe stop, from another terminal or over ssh

Order: actuator node and camera node, then the legs (zero, ramp to the default pose, wait for "ready", policy),
then aim/run.py when the legs send their first pose (with --bench: when the actuator and the camera send).
Watch: every node sends to the supervisor port (heartbeat). A node that exits, or that sends nothing for its
heartbeat_timeout_s (config.toml [launch]), starts a safe stop. aim/run.py exits by itself at DONE: the mission is done,
the robot stands and the pump stays off until you stop it (or --stop-when-done).
Safe stop: kill a hung node, stop aim/run.py, send "pump off, hold the angles" to the actuator node, legs damping
for 3 s then motors off, stop the camera, stop the actuator node (servos keep their torque and hold), force the pump pin low.
Logs: robot/logs/<time>-<mode>/: launch.log (events), telemetry.jsonl (every heartbeat), one log per node.
Environment: ROBOT_CONFIG=file picks another config, ROBOT_PORT_OFFSET=n moves all ports (both reach every node).
Exit code: 0 after a stop by the operator, 1 after a fault, 2 when a node did not start.
"""
import argparse
import ctypes
import json
import os
import select
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from robot_config import CONF, HOST, ROOT, path

PORTS, L = CONF["ports"], CONF["launch"]


def _pdeathsig():  # each node gets SIGTERM when launch.py dies, so no node runs on without its supervisor
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGTERM)  # 1 = PR_SET_PDEATHSIG


def kind(m):
    """Which node sent this supervisor message."""
    for key, name in (("cmd", "cmd"), ("truth", "truth"), ("frame", "camera"), ("water", "actuator"), ("state", "aim"), ("yaw", "legs")):
        if key in m:
            return name
    return "unknown"


class Launch:
    def __init__(self, a):
        self.a, self.legs = a, not a.bench
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        mode = ("sim-" if a.sim else "") + ("bench" if a.bench else "full")
        self.dir = Path(a.log_dir) if a.log_dir else path(L["log_dir"]) / f"{stamp}-{mode}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.pump_file = self.dir / "pump_pin" if a.sim else None
        self.log_f = open(self.dir / "launch.log", "a", buffering=1)
        self.tel = open(self.dir / "telemetry.jsonl", "a", buffering=1)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind((HOST, PORTS["supervisor"]))
        self.rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.procs, self.started, self.last, self.first = {}, {}, {}, {}
        self.angles, self.state, self.water, self.pin = (0.0, 0.0), None, None, None
        self.done, self.stop_req = False, None
        self.t0 = time.monotonic()

    # ---- logging
    def log(self, msg):
        line = f"{time.monotonic() - self.t0:8.2f} s  {msg}"
        print(f"[launch] {line}", flush=True)
        self.log_f.write(line + "\n")

    def _pump_lines(self, name, proc):
        quiet = name == "aim"  # aim/run.py prints one JSON line per tick: log file only
        with open(self.dir / f"{name}.log", "a", buffering=1) as f:
            for line in proc.stdout:
                f.write(f"{time.monotonic() - self.t0:8.2f}  {line}")
                if not (quiet and line.startswith("{")):
                    print(f"[{name}] {line}", end="", flush=True)

    # ---- nodes
    def argv(self, name):
        a, py = self.a, [sys.executable, "-u"]
        if name == "actuator":
            return py + ["actuators/actuator_node.py"] + (["--mock", "--pump-file", str(self.pump_file)] if a.sim else [])
        if name == "camera":  # the detector comes from config.toml [camera] detector (camera_node.detector_file)
            src = a.camera or ("sim" if a.sim else "real")
            fires = ";".join(f"{x},{y}" for x, y in CONF["sim"]["bench_fires" if a.bench else "fires"])
            return py + ["robot/camera_node.py", "--source", src] + (["--pump-file", str(self.pump_file), "--fires", fires] if src == "sim" else [])
        if name == "legs":
            lg = CONF["legs"]
            return py + ["legs/run_policy.py", "--udp", "--seconds", str(lg["max_run_s"]), "--knee-max", str(lg["knee_max"])] \
                + (["--fake"] if a.sim else [])
        if name == "aim":
            return py + ["aim/run.py", CONF["robot"]["concept"], "--feedback"] + (["--no-walk"] if a.bench else [])
        raise KeyError(name)

    def start(self, name):
        p = subprocess.Popen(self.argv(name), cwd=ROOT, env=dict(os.environ, PYTHONUNBUFFERED="1"),
                             stdin=subprocess.PIPE if name == "legs" else subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             start_new_session=True, preexec_fn=_pdeathsig)  # own session: Ctrl+C reaches launch.py only
        self.procs[name], self.started[name] = p, time.monotonic()
        threading.Thread(target=self._pump_lines, args=(name, p), daemon=True).start()
        self.log(f"started {name}, pid {p.pid}: {' '.join(self.argv(name)[2:])}")

    def alive(self, name):
        p = self.procs.get(name)
        return p is not None and p.poll() is None

    def signal(self, name, sig):
        if self.alive(name):
            try:
                self.procs[name].send_signal(sig)
            except ProcessLookupError:
                pass

    # ---- messages
    def read_messages(self):
        while True:
            try:
                data = self.rx.recv(65536)
            except BlockingIOError:
                break
            try:
                m = json.loads(data)
            except ValueError:
                continue
            now, src = time.monotonic(), kind(m)
            if src == "cmd":
                self.command(m.get("cmd"))
                continue
            self.last[src] = now
            self.first.setdefault(src, now)
            self.tel.write(json.dumps(dict(rt=round(now - self.t0, 3), src=src, **m)) + "\n")
            if src == "aim" and m["state"] != self.state:
                self.log(f"aim state {self.state} -> {m['state']}")
                self.state = m["state"]
            if src == "actuator":
                self.angles = (m["pan"], m["tilt"])
                if m["water"] != self.water:
                    self.water = m["water"]
                    self.log(f"pump {'ON' if self.water else 'off'} (actuator node)")
        if self.pump_file and self.pump_file.exists():  # the mock pump pin, independent of the actuator node
            pin = self.pump_file.read_text().strip()
            if pin != self.pin:
                self.pin = pin
                self.tel.write(json.dumps(dict(rt=round(time.monotonic() - self.t0, 3), src="pump_pin", on=pin == "1")) + "\n")

    def command(self, cmd):
        if cmd == "ready":
            self.ready()
        elif cmd == "stop" and not self.stop_req:
            self.stop_req = ("stop command (launch.py --send stop)", False, None)

    def ready(self):
        if self.alive("legs"):
            self.log("ready: the policy starts")
            try:
                self.procs["legs"].stdin.write("\n")
                self.procs["legs"].stdin.flush()
            except OSError:
                pass

    # ---- watch
    def check(self):
        """A fault as (reason, hung node or None), or None."""
        now = time.monotonic()
        for name, p in self.procs.items():
            rc = p.poll()
            if rc is None:
                continue
            if name == "aim" and rc == 0:
                if not self.done:
                    self.done = True
                    self.log("mission done (aim/run.py reached DONE). The robot stands, the pump is off. "
                             "Ctrl+C or `launch.py --send stop` stops the robot.")
                    if self.a.stop_when_done:
                        self.stop_req = ("mission done (--stop-when-done)", False, None)
                continue
            return f"{name} exited with code {rc}", None
        for name in self.procs:
            if name == "aim" and self.done:
                continue
            timeout = L["heartbeat_timeout_s"][name]
            if name in self.last and now - self.last[name] > timeout:
                return f"{name}: no heartbeat for {now - self.last[name]:.2f} s (limit {timeout} s): hung", name
            if name not in self.first and name != "legs" and now - self.started[name] > L["first_heartbeat_s"]:
                return f"{name}: no heartbeat {L['first_heartbeat_s']} s after the start", name
        return None

    def hold(self):
        """Pump off and hold the measured angles: the command of the supervisor while it stops the robot."""
        msg = json.dumps(dict(pan=self.angles[0], tilt=self.angles[1], pump_on=False, vx=0.0, yaw_rate=0.0, state="STOP"))
        for port in (PORTS["actuator_cmd"], PORTS["walk_cmd"]):
            try:
                self.tx.sendto(msg.encode(), (HOST, port))
            except OSError:
                pass

    def wait(self, names, seconds):
        """Wait until these nodes exit, while the actuator node gets hold commands. Kill the ones that stay."""
        end = time.monotonic() + seconds
        while any(self.alive(n) for n in names) and time.monotonic() < end:
            self.hold()
            self.read_messages()
            time.sleep(0.05)
        for n in names:
            if self.alive(n):
                self.log(f"{n} did not stop in {seconds:g} s: kill")
                self.signal(n, signal.SIGKILL)
                self.procs[n].wait()

    def safe_stop(self, reason, fault, hung=None):
        from pump import force_off
        self.log(f"{'FAULT' if fault else 'STOP'}: {reason}. Safe stop.")
        if hung:  # a hung (or stopped) process does not react to SIGTERM
            self.log(f"kill {hung} (hung)")
            self.signal(hung, signal.SIGKILL)
            self.procs[hung].wait()
            if hung == "actuator":
                self.log(force_off(self.a.sim, self.pump_file))
        self.signal("aim", signal.SIGTERM)
        self.wait(["aim"], 1.0)
        self.hold()
        if self.alive("legs"):
            self.log("legs: damping for 3 s, then motors off")
            self.signal("legs", signal.SIGTERM)
            self.wait(["legs"], L["stop_wait_s"])
        self.signal("camera", signal.SIGTERM)
        self.wait(["camera"], 2.0)
        self.signal("actuator", signal.SIGTERM)  # pump off on exit, the servos keep their last goal
        self.wait(["actuator"], 2.0)
        self.log(force_off(self.a.sim, self.pump_file))
        self.read_messages()
        codes = {n: p.returncode for n, p in self.procs.items()}
        self.log(f"all nodes stopped: exit codes {codes}")
        return 1 if fault else 0

    def run(self):
        stop = lambda *_: setattr(self, "stop_req", self.stop_req or ("Ctrl+C or SIGTERM", False, None))  # noqa: E731
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        self.log(f"mode {'sim ' if self.a.sim else ''}{'bench (no legs)' if self.a.bench else 'full'}, logs in {self.dir}")
        if self.pump_file:
            self.pump_file.write_text("0")
        self.start("actuator")
        self.start("camera")
        if self.legs:
            self.start("legs")
            if not self.a.sim:
                self.log("When the robot stands on the floor, press Enter here (or run `launch.py --send ready`).")
        tty = sys.stdin.isatty()
        try:
            while not self.stop_req:
                self.read_messages()
                if tty and select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    self.ready()
                if "aim" not in self.procs and {"actuator", "camera"} <= self.first.keys() and (not self.legs or "legs" in self.first):
                    self.start("aim")
                if (f := self.check()) is not None:
                    self.stop_req = (f[0], True, f[1])
                time.sleep(0.05)
        except Exception as e:  # noqa: BLE001  any error in the supervisor: stop the robot first
            self.stop_req = (f"supervisor error {e!r}", True, None)
        reason, fault, hung = self.stop_req
        code = self.safe_stop(reason, fault, hung)
        if fault and "aim" not in self.procs:
            code = 2
        return code


def send(cmd):
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(json.dumps(dict(cmd=cmd)).encode(), (HOST, PORTS["supervisor"]))
    print(f"sent {cmd} to launch.py on udp {PORTS['supervisor']}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0], epilog="Details: robot/README.md")
    ap.add_argument("--sim", action="store_true", help="fake backends: legs fake bus (MuJoCo), mock servos and pump, synthetic camera")
    ap.add_argument("--bench", action="store_true", help="no legs: aim/run.py --no-walk")
    ap.add_argument("--full", action="store_true", help="the whole robot, real hardware")
    ap.add_argument("--camera", help="camera source: real, sim, folder:PATH or video:PATH (default: sim with --sim, else real)")
    ap.add_argument("--stop-when-done", action="store_true", help="safe stop when aim/run.py reaches DONE")
    ap.add_argument("--log-dir", help="log folder (default: config.toml [launch] log_dir / time-mode)")
    ap.add_argument("--send", choices=("ready", "stop"), help="send a command to a running launch.py, then exit")
    a = ap.parse_args(argv)
    if a.send:
        return send(a.send)
    if a.full == (a.sim or a.bench):
        ap.error("give --full, or --sim and/or --bench")
    return Launch(a).run()


if __name__ == "__main__":
    sys.exit(main())
