"""Bench tests for the first hardware days. Each test prints PASS, FAIL or WARN with a hint. Procedures: robot/README.md.

    uv run python robot/bench.py preflight              every device present and set up. Run it first, every day.
    uv run python robot/bench.py pantilt                sweep pan and tilt to their limits, measure the position error
    uv run python robot/bench.py pump --seconds 2       timed sprays. You type the jet length: v0 for config.toml.
    uv run python robot/bench.py imu                    IMU rate, gravity at rest, tilt forward and left
    uv run python robot/bench.py camera                 formats, exposure, frame rate, one snapshot, detections
    uv run python robot/bench.py can                    link state, the 12 heartbeats, bus load with policy-like traffic

Add --mock to run a test without hardware (fake drives, mock servos and pump, synthetic camera and IMU).
Add --yes to answer yes to every question (with --mock only).
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from robot_config import CONF, ROOT, path

sys.path.append(str(ROOT / "actuators"))
sys.path.append(str(ROOT / "legs"))
sys.path.append(str(ROOT / "aim"))
DATA = Path(__file__).resolve().parent / "bench_data"  # measured data sheets: commit them
SNAP = path(CONF["launch"]["log_dir"]) / "bench"
G = 9.81
results = []


def report(status, what, hint=""):
    results.append(status)
    print(f"{status}: {what}" + (f"\n  Hint: {hint}" if hint else ""), flush=True)


class Ask:
    def __init__(self, yes, mock_answers=None):
        self.yes, self.mock = yes, mock_answers or {}

    def ok(self, q):
        if self.yes:
            print(f"  {q} yes")
            return True
        return input(f"  {q} [y/N] ").strip().lower().startswith("y")

    def number(self, q, key=None):
        if self.yes:
            v = self.mock[key]() if callable(self.mock.get(key)) else self.mock.get(key, 0.0)
            print(f"  {q} {v:.3f}")
            return v
        while True:
            try:
                return float(input(f"  {q} ").strip().replace(",", "."))
            except ValueError:
                print("  Type a number, for example 1.25")


# ---------------------------------------------------------------- preflight
def preflight(a, ask):
    c = CONF
    for key, what in (("can.left_serial", "left CANable serial"), ("can.right_serial", "right CANable serial"), ("camera.usb", "camera USB id")):
        sec, k = key.split(".")
        if c[sec][k]:
            report("PASS", f"config.toml {key} = {c[sec][k]}")
        else:
            report("WARN", f"config.toml {key} is empty ({what})", "bash robot/setup/install.sh --list-usb, then put the value in config.toml and run install.sh again")
    try:
        import gpiod  # noqa: F401
        report("PASS", "Python package gpiod is installed")
    except ImportError:
        report("FAIL", "Python package gpiod is missing", "uv sync --extra board")
    servo = c["actuator"]["servo_port"]
    if Path(servo).exists():
        try:
            import st3215
            bus = st3215.Bus.open(servo)
            ids = [c["actuator"][k]["id"] for k in ("pan", "tilt")]
            missing = [i for i in ids if not bus.ping(i)]
            report("FAIL" if missing else "PASS", f"servos on {servo}: " + (f"no reply from id {missing}" if missing else f"ids {ids} answer"),
                   "12 V on the adapter, jumper at B, ids set (actuators/README.md)" if missing else "")
        except OSError as e:
            report("FAIL", f"cannot open {servo}: {e}", "Add your user to the dialout group (install.sh step 2), log out and in")
    else:
        report("FAIL", f"{servo} does not exist", "Plug the servo adapter. Check the udev rule: install.sh step 4")
    imu = c["imu"]["port"]
    if Path(imu).exists():
        n = imu_frames(imu, c["imu"]["baud"], 0.5)
        report("PASS" if n else "FAIL", f"IMU on {imu}: {n} frames in 0.5 s",
               "" if n else "Check the wires (IMU TX to the ROCK 3C RX), the 3.3 V supply and the baud rate (legs/imu.py --setup)")
    else:
        report("FAIL", f"{imu} does not exist", f"Enable the overlay {c['imu']['overlay']} and reboot (install.sh step 8)")
    cam = c["camera"]["device"]
    report("PASS" if Path(cam).exists() else "FAIL", f"camera device {cam}", "" if Path(cam).exists() else "Plug the camera. Set config.toml [camera] usb and run install.sh")
    for ch in c["can"]["channels"]:
        st = can_link(ch)
        if st is None:
            report("FAIL", f"CAN interface {ch} does not exist", "Plug the CANable. Set the serials in config.toml and run install.sh. Check: lsmod | grep gs_usb")
        else:
            ok = st["up"] and st["state"] == "ERROR-ACTIVE" and st["bitrate"] == c["can"]["bitrate"]
            report("PASS" if ok else "FAIL", f"CAN {ch}: {st}", "" if ok else "sudo systemctl restart robot-can@" + ch + " ; check the 24 V supply of the drives and the termination (legs/README.md)")
    chip, line = c["pump"]["chip"], c["pump"]["line"]
    try:
        from pump import GpiodLine
        GpiodLine(chip, line).release()
        report("PASS", f"pump pin {chip} line {line} can be used (output, low)")
    except Exception as e:  # noqa: BLE001
        report("FAIL", f"pump pin {chip} line {line}: {e!r}", "gpiodetect and gpioinfo show the chips. Is another program using the pin? Is your user in the gpio group?")
    gov = Path("/sys/devices/system/cpu/cpufreq/policy0/scaling_governor")
    g = gov.read_text().strip() if gov.exists() else "unknown"
    report("PASS" if g == "performance" else "WARN", f"CPU governor {g}", "" if g == "performance" else "install.sh step 7, then reboot")
    tz = Path("/sys/class/thermal/thermal_zone0/temp")
    if tz.exists():
        t = int(tz.read_text()) / 1000
        report("PASS" if t < 70 else "WARN", f"SoC temperature {t:.0f} C", "" if t < 70 else "Add a heatsink or a fan. The SoC slows down when it is hot.")


def can_link(ch):
    try:
        out = subprocess.run(["ip", "-details", "-statistics", "-json", "link", "show", ch], capture_output=True, text=True, check=True).stdout
    except (OSError, subprocess.CalledProcessError):
        return None
    j = json.loads(out)[0]
    info = j.get("linkinfo", {}).get("info_data", {})
    xs = j.get("linkinfo", {}).get("info_xstats", {})
    return dict(up="UP" in j.get("flags", []), state=info.get("state"), bitrate=info.get("bittiming", {}).get("bitrate"),
                txqlen=j.get("txqlen"), bus_off=xs.get("bus_off"), restarts=xs.get("restarts"))


def imu_frames(port, baud, seconds):
    """Number of valid quaternion frames (0x55 0x59, checksum good) that the IMU sends in this time."""
    import serial
    data, end = b"", time.monotonic() + seconds
    with serial.Serial(port, baud, timeout=0.05) as s:
        while time.monotonic() < end:
            data += s.read(max(1, s.in_waiting))
    return sum(1 for i in range(len(data) - 10) if data[i] == 0x55 and data[i + 1] == 0x59 and sum(data[i:i + 10]) & 0xFF == data[i + 10])


# ---------------------------------------------------------------- pan-tilt
def pantilt(a, ask):
    from actuator_node import DEFAULT_CALIB, MockServos, Servos
    calib = DEFAULT_CALIB
    servos = MockServos(calib) if a.mock else Servos(calib, CONF["actuator"]["servo_port"])
    lim = {k: (calib[k]["min_deg"], calib[k]["max_deg"]) for k in ("pan", "tilt")}
    if not ask.ok(f"CAUTION: the pan-tilt moves to its limits {lim} (deg) at {a.speed:g} deg/s. Is the area around the nozzle clear?"):
        return report("SKIP", "pan-tilt sweep not confirmed")
    dt, worst_static, worst_dyn = 1 / CONF["actuator"]["rate_hz"], {}, 0.0
    goal = dict(zip(("pan", "tilt"), servos.read()))
    for axis in ("pan", "tilt"):
        lo, hi = (math.radians(v) for v in lim[axis])
        for target in (hi, lo, 0.0):
            while abs(goal[axis] - target) > 1e-6:  # ramp the goal at the test speed
                goal[axis] += max(-math.radians(a.speed) * dt, min(math.radians(a.speed) * dt, target - goal[axis]))
                servos.move(goal, dt)
                meas = dict(zip(("pan", "tilt"), servos.read()))
                worst_dyn = max(worst_dyn, abs(meas[axis] - goal[axis]))
                time.sleep(0 if a.mock else dt)
            t_end = time.monotonic() + (0 if a.mock else 1.5)  # hold, then read the static error
            while time.monotonic() < t_end:
                servos.move(goal, dt)
                time.sleep(dt)
            err = abs(dict(zip(("pan", "tilt"), servos.read()))[axis] - target)
            worst_static[f"{axis} {math.degrees(target):+.0f} deg"] = round(math.degrees(err), 2)
            if target != 0.0 and not a.yes:
                ask.ok(f"{axis} at {math.degrees(target):+.0f} deg: the nozzle, the hose and the cables touch nothing?")
    bad = {k: v for k, v in worst_static.items() if v > 2.0}
    report("FAIL" if bad else "PASS", f"static error at each limit (deg): {worst_static}",
           "Check the nozzle balance, the hose and cable pull and the 12 V current. Reduce the limits in config.toml [actuator]." if bad else "")
    report("PASS", f"largest error while moving at {a.speed:g} deg/s: {math.degrees(worst_dyn):.1f} deg (lag, for information)")


# ---------------------------------------------------------------- pump and jet
def v0_from_shot(x, h, tilt):
    """Jet speed from the landing distance x (m) of a shot from height h (m) at elevation tilt (rad), no drag."""
    d = h + x * math.tan(tilt)
    return x / math.cos(tilt) * math.sqrt(G / (2 * d)) if d > 0 and x > 0 else float("nan")


def pump(a, ask):
    from pump import open_pump
    tilt = math.radians(a.tilt)
    servos = None
    if not a.no_servo:
        from actuator_node import DEFAULT_CALIB, MockServos, Servos
        servos = MockServos(DEFAULT_CALIB) if a.mock else Servos(DEFAULT_CALIB, CONF["actuator"]["servo_port"])
    h = a.height if a.height else ask.number("Height of the nozzle exit above the floor (m):", "height")
    print(f"Jet test: {a.runs} sprays of {a.seconds:g} s at tilt {a.tilt:g} deg, nozzle at {h:.3f} m.")
    print("CAUTION: the pump sprays water. Point the nozzle at a clear floor area. Keep the electronics dry.")
    p = open_pump(a.mock, log=print)
    sheet = (SNAP / "jet_test_mock.csv") if a.mock else (DATA / "jet_test.csv")  # mock data never goes into the real sheet
    sheet.parent.mkdir(parents=True, exist_ok=True)
    new = not sheet.exists()
    v0s = []
    try:
        with open(sheet, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["date_time", "run", "spray_s", "tilt_deg", "nozzle_height_m", "landing_m", "v0_m_s", "supply_v", "note"])
            for run in range(1, a.runs + 1):
                if servos:  # hold pan 0 and the test tilt during the spray
                    servos.move({"pan": 0.0, "tilt": tilt}, 1.0)
                if not ask.ok(f"Spray {run}: water in the tank, area clear?"):
                    break
                end = time.monotonic() + a.seconds
                while time.monotonic() < end:
                    p.set(True)  # the max_spray_s watchdog of the pump also limits this
                    time.sleep(0.02)
                p.set(False)
                x = ask.number("Where did the jet land? Distance on the floor from below the nozzle exit (m), 0 = no value:", "landing")
                v = ask.number("Supply voltage at the pump during the spray (V), 0 = not measured:", "supply")
                v0 = v0_from_shot(x, h, tilt) if x > 0 else float("nan")
                v0s += [v0] if v0 == v0 else []
                w.writerow([datetime.now().isoformat(timespec="seconds"), run, a.seconds, a.tilt, h, x, round(v0, 3) if v0 == v0 else "", v or "", a.note])
                print(f"  run {run}: landing {x:.2f} m, v0 = {v0:.2f} m/s (no drag)")
    finally:
        p.close()
    if v0s:
        report("PASS", f"jet speed v0 = {np.mean(v0s):.2f} m/s (mean of {len(v0s)}, spread {np.ptp(v0s):.2f} m/s). Data in {sheet.relative_to(ROOT)}")
        print(f"  Put this line in robot/config.toml under [aim.controller]:  v0 = {np.mean(v0s):.2f}")
    else:
        report("WARN", "no landing distance typed: no v0")


# ---------------------------------------------------------------- IMU
class MockImu:
    """Upright for 1.5 s, then tilted forward by 20 deg, then to the left by 20 deg."""

    def __init__(self):
        self.t0 = time.monotonic()

    def read(self):
        from scipy.spatial.transform import Rotation
        t = time.monotonic() - self.t0
        r = Rotation.identity() if t < 1.5 else Rotation.from_euler("y", 20, degrees=True) if t < 3.0 else Rotation.from_euler("x", -20, degrees=True)
        return r.as_quat(scalar_first=True), np.zeros(3)


def imu(a, ask):
    from scipy.spatial.transform import Rotation
    port, baud = CONF["imu"]["port"], CONF["imu"]["baud"]
    if a.mock:
        dev = MockImu()
    else:
        n = imu_frames(port, baud, 2.0)
        report("PASS" if n > 300 else "FAIL", f"IMU rate {n / 2:.0f} Hz on {port} at {baud} baud (expected 200 Hz)",
               "" if n > 300 else "Run once: uv run python legs/imu.py --setup (200 Hz, 230400 baud)")
        import imu as im
        dev = im.Im10a(port, baud)
        time.sleep(0.3)

    def gravity(seconds=0.5):
        gs, ws, end = [], [], time.monotonic() + seconds
        while time.monotonic() < end:
            q, w = dev.read()
            gs.append(Rotation.from_quat(q, scalar_first=True).inv().apply([0, 0, -1.0]))
            ws.append(w)
            time.sleep(0.01)
        return np.mean(gs, axis=0), np.abs(ws).max()

    ask.ok("Hold the robot upright and still. Ready?")
    g, w = gravity()
    flat = np.linalg.norm(g - [0, 0, -1]) < 0.1
    report("PASS" if flat else "FAIL", f"upright: gravity {np.round(g, 2)} (expected 0, 0, -1)",
           "" if flat else "The IMU is not flat in the torso: edit IMU_TO_BODY in legs/imu.py")
    report("PASS" if w < 0.05 else "WARN", f"gyro at rest: largest {w:.3f} rad/s (expected below 0.05)")
    for name, axis, word in (("forward", 0, "x"), ("left", 1, "y")):
        print(f"  Tilt the robot {name} by about 20 deg and hold it (20 s at most).")
        end, got = time.monotonic() + 20, None
        while time.monotonic() < end:
            g, _ = gravity(0.2)
            if abs(g[axis]) > 0.25:
                got = g
                break
        if got is None:
            report("FAIL", f"tilt {name}: gravity {word} did not change", "Check the IMU frames and IMU_TO_BODY in legs/imu.py")
        else:
            report("PASS" if got[axis] > 0 else "FAIL", f"tilt {name}: gravity {np.round(got, 2)}, {word} must be positive",
                   "" if got[axis] > 0 else f"Change the sign of the IMU axis that points {name} in IMU_TO_BODY (legs/imu.py)")


# ---------------------------------------------------------------- camera
def camera(a, ask):
    import camera_node as cn
    dev = CONF["camera"]["device"]
    if not a.mock:
        for args in (["--list-formats-ext"], ["--list-ctrls"]):
            out = subprocess.run(["v4l2-ctl", "-d", dev, *args], capture_output=True, text=True)
            print(out.stdout or out.stderr)
        if a.exposure:  # manual exposure (control names of the Linux 6.1 UVC driver)
            subprocess.run(["v4l2-ctl", "-d", dev, "-c", "auto_exposure=1", "-c", f"exposure_time_absolute={a.exposure}"], check=False)
    detect = cn.load_detector(cn.detector_file(source="sim" if a.mock else "real"))
    frames, closer, _ = cn.open_source("sim" if a.mock else "real", fires=[[1.5, 0.2]] if a.mock else None)
    times, dets, bright, snap = [], 0, [], None
    t_end = time.monotonic() + a.seconds
    try:
        for t, rgb in frames:
            times.append(t)
            d = detect(rgb, t)
            dets += len(d)
            bright.append(float(rgb.mean()))
            if snap is None or d:
                snap = (rgb.copy(), d)
            if time.monotonic() > t_end:
                break
    finally:
        if closer:
            closer.close()
    fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0
    gap = max(np.diff(times)) if len(times) > 1 else float("inf")
    want = CONF["camera"]["fps"]
    slow = "WARN" if a.mock else "FAIL"  # the mock camera speed depends on the laptop load: information only
    report("PASS" if fps >= 0.9 * want else slow, f"frame rate {fps:.1f} fps (config {want}), largest gap {gap * 1000:.0f} ms",
           "" if fps >= 0.9 * want else "Use MJPEG (config input_format), a shorter exposure (--exposure), or a lower resolution")
    m = float(np.mean(bright)) if bright else 0.0
    report("PASS" if 40 < m < 215 else "WARN", f"mean brightness {m:.0f} of 255", "" if 40 < m < 215 else "Set the exposure: --exposure N (v4l2-ctl --list-ctrls shows the range)")
    report("PASS", f"{dets} detections in {len(times)} frames (point the camera at a flame to see some)")
    if snap is not None:
        from PIL import Image, ImageDraw
        SNAP.mkdir(parents=True, exist_ok=True)
        img = Image.fromarray(snap[0])
        draw = ImageDraw.Draw(img)
        for d in snap[1]:
            draw.rectangle(list(d.bbox if hasattr(d, "bbox") else d["bbox"]), outline=(0, 255, 0), width=3)
        out = SNAP / f"camera_{datetime.now():%Y%m%d-%H%M%S}.png"
        img.save(out)
        print(f"  snapshot with the detection boxes: {out}")


# ---------------------------------------------------------------- CAN
def frame_bits(dlc, rtr=False):
    """Bits of one standard CAN frame on the wire, with about 10 % bit stuffing."""
    return 1.1 * (47 + (0 if rtr else 8 * dlc))


def can(a, ask):
    import odrive_can as oc
    import robot as rb  # legs/robot.py
    buses, sim = rb.open_buses(a.mock)
    try:
        nodes = {b.name: [] for b in buses}
        for b in buses:
            if not a.mock:
                st = can_link(b.name)
                report("PASS" if st and st["state"] == "ERROR-ACTIVE" else "FAIL", f"{b.name} link: {st}")
            b.poll(2.0)
            hb = {n: v for (n, cmd), v in b.last.items() if cmd == oc.HEARTBEAT}
            nodes[b.name] = sorted(hb)
        found = {n for ns in nodes.values() for n in ns}
        expected = {j[1] for j in rb.JOINTS}
        report("PASS" if expected <= found else "FAIL", f"heartbeats: {nodes}, expected nodes {sorted(expected)}",
               "" if expected <= found else f"missing {sorted(expected - found)}: check the chain, the 24 V and the node ids (legs/bringup.py scan)")
        bitrate = CONF["can"]["bitrate"]
        for b in buses:  # policy-like traffic: one encoder request to each drive at 25 Hz, for 2 s (read only)
            ns = nodes[b.name]
            if not ns:
                continue
            sent = got = 0
            t0 = time.monotonic()
            for _ in rb.ticks(rb.RATE):
                got += len(b.ask(ns, oc.GET_ENCODER_ESTIMATES))
                sent += len(ns)
                if time.monotonic() - t0 > 2.0:
                    break
            dt = time.monotonic() - t0
            bits = sent * frame_bits(0, rtr=True) + got * frame_bits(8) + len(ns) * 10 * dt * frame_bits(8)
            policy_bits = bits + sent * frame_bits(8)  # the policy also sends one position target per drive and tick
            report("PASS" if got >= 0.99 * sent else "FAIL", f"{b.name}: {got} of {sent} encoder replies at 25 Hz; "
                   f"bus load {100 * bits / dt / bitrate:.1f} %, with the position targets of the policy about {100 * policy_bits / dt / bitrate:.1f} %",
                   "" if got >= 0.99 * sent else "Lost frames: check the termination (60 ohm) and txqueuelen (legs/README.md)")
    finally:
        for b in buses:
            b.close()
        if sim:
            sim.close()


TESTS = dict(preflight=preflight, pantilt=pantilt, pump=pump, imu=imu, camera=camera, can=can)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=list(TESTS))
    ap.add_argument("--mock", action="store_true", help="no hardware")
    ap.add_argument("--yes", action="store_true", help="answer yes to every question (with --mock only)")
    ap.add_argument("--speed", type=float, default=30.0, help="pantilt: sweep speed (deg/s)")
    ap.add_argument("--seconds", type=float, default=None, help="pump: spray time (default 2 s). camera: test time (default 5 s)")
    ap.add_argument("--runs", type=int, default=3, help="pump: number of sprays")
    ap.add_argument("--tilt", type=float, default=0.0, help="pump: nozzle tilt (deg, 0 = level)")
    ap.add_argument("--height", type=float, help="pump: nozzle exit height above the floor (m)")
    ap.add_argument("--no-servo", action="store_true", help="pump: do not move the pan-tilt (hold the nozzle by hand or in a clamp)")
    ap.add_argument("--note", default="", help="pump: a note for the data sheet (nozzle, tube length)")
    ap.add_argument("--exposure", type=int, help="camera: manual exposure value (v4l2 exposure_time_absolute)")
    a = ap.parse_args(argv)
    if a.yes and not a.mock:
        ap.error("--yes only with --mock: on the robot, each move needs your answer")
    a.seconds = a.seconds or (2.0 if a.test == "pump" else 5.0)
    model_x = lambda: 1.2 * math.sqrt(2 * 0.3 / G) * 6.0  # noqa: E731  mock jet: v0 6 m/s, 0.3 m high, plus 20 %
    ask = Ask(a.yes, dict(height=0.3, landing=model_x, supply=12.0))
    print(f"bench {a.test}{' (mock)' if a.mock else ''}")
    TESTS[a.test](a, ask)
    print(f"Summary: {results.count('PASS')} PASS, {results.count('FAIL')} FAIL, {results.count('WARN')} WARN, {results.count('SKIP')} SKIP")
    return 1 if "FAIL" in results else 0


if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    sys.exit(main())
