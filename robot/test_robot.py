"""Fast tests of the robot runtime: config, pump watchdogs, fake world, launch with a fault, bench mocks, install files.

    uv run pytest robot -q

The longer end-to-end test with the legs is robot/sil.py.
"""
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

import pump as pm
import robot_config as rc

ROOT = rc.ROOT
OFFSET = str(30000 + os.getpid() % 2000 * 10)  # own ports, so a running pipeline or another test run does not disturb this one
ENV = dict(os.environ, ROBOT_PORT_OFFSET=OFFSET)


def test_config_ports_unique_and_offset(monkeypatch):
    c = rc.load()
    ports = list(c["ports"].values())
    assert len(set(ports)) == len(ports)
    base = c["ports"]["detections"] - int(os.environ.get("ROBOT_PORT_OFFSET", "0"))
    monkeypatch.setenv("ROBOT_PORT_OFFSET", "7")
    assert rc.load()["ports"]["detections"] == base + 7
    assert rc.path("sim/sim2sim/policy_biped_25hz_a.onnx").exists(), "legs.policy must exist"
    sys.path.insert(0, str(ROOT / "aim"))
    from run import robot_setup
    spray = robot_setup(c["robot"]["concept"])[1].max_spray
    assert c["pump"]["max_spray_s"] > spray, f"the pump backstop must be longer than one controller spray ({spray} s)"


def test_pump_watchdogs():
    line, logs = pm.MemoryLine(), []
    p = pm.Pump(line, max_spray_s=0.3, keepalive_s=0.15, log=logs.append)
    p.set(True)
    assert p.on and line.get() == 1
    time.sleep(0.3)  # no update for longer than keepalive_s: the watchdog thread switches off
    assert not p.on and line.get() == 0 and "keepalive" in logs[-1]
    end = time.monotonic() + 0.5
    while time.monotonic() < end:  # updates at 50 Hz, but on for longer than max_spray_s: off, and it stays off
        p.set(True)
        time.sleep(0.02)
    assert not p.on and "max_spray_s" in logs[-1]
    p.set(False)  # the spray ends: the next spray may start
    p.set(True)
    assert p.on
    p.close()
    assert line.get() == 0


def test_pump_estop_and_file_line(tmp_path):
    estop = pm.MemoryLine()
    f = tmp_path / "pin"
    p = pm.Pump(pm.FileLine(f), estop=estop, log=lambda m: None)
    p.set(True)
    assert f.read_text() == "1"
    estop.set(1)  # pressed (input high)
    p.set(True)
    assert not p.on and f.read_text() == "0" and p.estop
    p.close()
    assert pm.force_off(mock=True, pump_file=f).startswith("pump line forced low") and f.read_text() == "0"


def test_fake_world_sees_aims_and_puts_out(tmp_path, monkeypatch):
    for k in ("sim_truth", "feedback_sim"):
        monkeypatch.setitem(rc.CONF["ports"], k, 0)  # any free port
    import fake_world
    from camera_node import detector_file, load_detector
    sys.path.insert(0, str(ROOT / "aim"))
    from ballistics import solve_aim
    from geometry import localise
    pin = tmp_path / "pin"
    pin.write_text("0")
    w = fake_world.FakeWorld(pin, fires=[[1.5, 0.4]])
    dets = load_detector(detector_file("", "sim"))(w.render(), time.monotonic())
    assert len(dets) == 1
    p = localise(w.cfg, dets[0], w.pan, w.tilt)
    assert np.hypot(p[0] - 1.5, p[1] - 0.4) < 0.1, p
    aim = solve_aim(w.cfg, np.array([1.5, 0.4, 0.0]), w.cc.v0, w.cc.drag)
    w.pan, w.tilt = aim.pan, aim.tilt
    assert np.hypot(*(w.landing() - [1.5, 0.4])) < 0.05
    w.update()
    pin.write_text("1")
    end = time.monotonic() + 0.5
    while time.monotonic() < end:
        w.update()
        time.sleep(0.02)
    assert 0.75 < w.health[0] < 0.9  # 0.5 s of 3 s burn time
    w.health[0] = 0.0
    assert not load_detector(detector_file("", "sim"))(w.render(), time.monotonic())


def telemetry(d):
    p = d / "telemetry.jsonl"
    return [json.loads(x) for x in p.read_text().splitlines() if x.endswith("}")] if p.exists() else []


def test_launch_bench_sim_sprays_then_stops_on_a_dead_node(tmp_path):
    """launch.py --sim --bench: the pump switches on. Then the camera node dies: safe stop, pump pin low, exit code 1."""
    p = subprocess.Popen([sys.executable, "-u", "robot/launch.py", "--sim", "--bench", "--log-dir", str(tmp_path)],
                         cwd=ROOT, env=ENV, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        end, on = time.monotonic() + 40, None
        while time.monotonic() < end and on is None:
            time.sleep(0.2)
            on = next((r for r in telemetry(tmp_path) if r["src"] == "pump_pin" and r["on"]), None)
        assert on, "the pump did not switch on: " + (tmp_path / "launch.log").read_text()
        log = (tmp_path / "launch.log").read_text()
        cam = int(log.split("started camera, pid ")[1].split(":")[0])
        os.kill(cam, signal.SIGKILL)
        assert p.wait(20) == 1
    finally:
        if p.poll() is None:
            p.kill()
    log = (tmp_path / "launch.log").read_text()
    assert "FAULT: camera exited" in log and "all nodes stopped" in log, log
    assert (tmp_path / "pump_pin").read_text() == "0"
    assert {"SEARCH", "AIM", "SPRAY"} <= {r["state"] for r in telemetry(tmp_path) if r["src"] == "aim"}


def test_launch_needs_a_mode():
    import launch
    with pytest.raises(SystemExit):
        launch.main(["--sim", "--full"])
    with pytest.raises(SystemExit):
        launch.main([])


@pytest.mark.parametrize("test", ["pantilt", "pump", "imu", "camera"])
def test_bench_mocks(test, tmp_path):
    r = subprocess.run([sys.executable, "robot/bench.py", test, "--mock", "--yes"], cwd=ROOT, env=ENV,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == 0 and "0 FAIL" in r.stdout, r.stdout + r.stderr
    if test == "pump":
        assert "jet_test_mock.csv" in r.stdout, "mock data must not go into the real data sheet"


def test_bench_yes_needs_mock():
    r = subprocess.run([sys.executable, "robot/bench.py", "pantilt", "--yes"], cwd=ROOT, env=ENV, capture_output=True, text=True)
    assert r.returncode == 2


def test_install_renders_valid_files(tmp_path):
    r = subprocess.run(["bash", "robot/setup/install.sh", "--render", str(tmp_path)], cwd=ROOT, capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    rules = (tmp_path / "70-robot.rules").read_text()
    assert 'SYMLINK+="servo"' in rules and 'KERNEL=="ttyS4", SYMLINK+="imu"' in rules
    unit = (tmp_path / "robot-can@.service").read_text()
    assert "bitrate 500000" in unit and "txqueuelen 100" in unit
    assert "[Install]" in (tmp_path / "robot@.service").read_text()
    if shutil.which("udevadm") and subprocess.run(["udevadm", "verify", "--help"], capture_output=True).returncode == 0:  # systemd 254 or newer
        v = subprocess.run(["udevadm", "verify", "--resolve-names=never", str(tmp_path / "70-robot.rules")], capture_output=True, text=True)
        assert v.returncode == 0, v.stdout + v.stderr


def test_v0_from_shot():
    import bench
    v0, h = 6.0, 0.3
    x = v0 * math.sqrt(2 * h / 9.81)  # level shot
    assert bench.v0_from_shot(x, h, 0.0) == pytest.approx(v0)
    t = math.radians(20)  # shot at 20 deg: land where z = -h
    tf = (v0 * math.sin(t) + math.sqrt((v0 * math.sin(t)) ** 2 + 2 * 9.81 * h)) / 9.81
    assert bench.v0_from_shot(v0 * math.cos(t) * tf, h, t) == pytest.approx(v0)


def test_detector_choice(tmp_path, monkeypatch):
    """[camera] detector, else vision/detector.py when it exists, else the baseline. The synthetic camera uses [sim] detector."""
    import camera_node as cn
    monkeypatch.setattr(cn, "ROOT", tmp_path)
    base = tmp_path / "sim" / "sim2sim" / "baseline_detector.py"
    assert cn.detector_file("", "real") == base
    (tmp_path / "vision").mkdir()
    (tmp_path / "vision" / "detector.py").write_text("def detect(rgb, t):\n    return []\n")
    assert cn.detector_file("", "real") == tmp_path / "vision" / "detector.py"
    assert cn.detector_file("", "folder") == tmp_path / "vision" / "detector.py"
    assert cn.detector_file("my/det.py", "real") == ROOT / "my" / "det.py"
    assert cn.detector_file("", "sim") == ROOT / rc.CONF["sim"]["detector"]
    assert cn.load_detector(tmp_path / "vision" / "detector.py")(None, 0.0) == []
    (tmp_path / "bad.py").write_text("raise FileNotFoundError('model.onnx')\n")
    with pytest.raises(SystemExit, match="cannot load the detector"):
        cn.load_detector(tmp_path / "bad.py")
