import math

import numpy as np
import pytest

from ballistics import G, jet_path, solve_aim, v0_from_flow, v0_from_horizontal_shot, _height_at
from controller import Controller
from detection import FireDetection
from geometry import CONCEPTS, TargetFilter, localise, nozzle_exit, project
from sim_demo import fake_detections

CAMS = [(c, cam) for c in CONCEPTS for cam in CONCEPTS[c]["cameras"]]


@pytest.mark.parametrize("concept,cam", CAMS)
def test_projection_round_trip(concept, cam):
    cfg, rng = CONCEPTS[concept], np.random.default_rng(0)
    k = cfg["cameras"][cam]["K"]
    for _ in range(50):
        pan, tilt = rng.uniform(-1, 1), rng.uniform(-0.6, 0.3)
        u, v, depth = rng.uniform(0, k.width), rng.uniform(0, k.height), rng.uniform(0.5, 8)
        p = localise(cfg, FireDetection(0, cam, u, v, (0, 0, 0, 0), 1, depth_m=depth), pan, tilt)
        assert np.allclose(project(cfg, cam, pan, tilt, p), (u, v), atol=1e-6)
        # same pixel through the floor intersection, at the floor height of p
        q = localise(cfg, FireDetection(0, cam, u, v, (0, 0, 0, 0), 1), pan, tilt, fire_height=p[2])
        if q is not None:
            assert np.allclose(q, p, atol=1e-6)


def jet_miss(cfg, aim, target, v0, drag):
    """Distance between the target and the jet where the jet reaches the target's horizontal distance."""
    e, d = nozzle_exit(cfg, aim.pan, aim.tilt)
    rel = np.asarray(target, dtype=float) - e
    dist = math.hypot(rel[0], rel[1])
    lateral = abs(d[0] * rel[1] - d[1] * rel[0]) / math.hypot(d[0], d[1])
    if drag == 0:
        z = dist * math.tan(aim.tilt) - G * dist ** 2 / (2 * v0 ** 2 * math.cos(aim.tilt) ** 2)
    else:
        z = _height_at(jet_path(v0, aim.tilt, drag, z_stop=rel[2] - 1, dt=1e-4), dist)[0]
    return math.hypot(lateral, z - rel[2])


TARGETS = [(1.5, 0, 0), (2, 0.5, 0), (3, -1, 0), (4, 2, 0.3), (1, 1, 0), (2.5, -2.5, 0.1)]


@pytest.mark.parametrize("concept", list(CONCEPTS))
@pytest.mark.parametrize("drag", [0.0, 0.05])
def test_solver_hits_target(concept, drag):
    cfg, v0 = CONCEPTS[concept], 8.0
    for target in TARGETS:
        aim = solve_aim(cfg, target, v0, drag)
        assert aim.in_range, (target, aim)
        assert jet_miss(cfg, aim, target, v0, drag) < 0.02, (target, aim)
        assert aim.tof > 0 and aim.max_range > math.hypot(target[0], target[1]) - 0.3


def test_out_of_range():
    cfg = CONCEPTS["A"]
    assert solve_aim(cfg, (15, 0, 0), 8.0).reason == "too far"
    assert solve_aim(cfg, (8, 0, 0), 8.0, drag=0.05).reason == "too far"
    assert solve_aim(cfg, (0.2, 0, 0), 8.0).reason == "too close"
    assert solve_aim(cfg, (-2, 0, 0), 8.0).reason == "pan limit"
    assert not solve_aim(cfg, (15, 0, 0), 8.0).in_range


def test_v0_helpers():
    v0, h = 8.0, 0.9
    x = v0 * math.sqrt(2 * h / G)  # landing distance of a horizontal shot
    assert v0_from_horizontal_shot(x, h) == pytest.approx(v0)
    assert v0_from_flow(2 / 60000, 0.002) == pytest.approx(10.61, abs=0.01)  # 2 L/min through 2 mm


def test_filter_rejects_outlier():
    f = TargetFilter()
    for _ in range(10):
        f.update((3.0, 1.0, 0.0))
    assert not f.update((6.0, -2.0, 0.0))
    assert np.allclose(f.x, (3.0, 1.0, 0.0))


def run_static(fire, frames, conf=None, start=0.0, ctl=None, fire_on=True):
    """Robot standing still at the origin, servos follow the command at once."""
    cfg = CONCEPTS["A"]
    ctl = ctl or Controller(cfg)
    rng = np.random.default_rng(0)
    pan, tilt, cmds = 0.0, 0.0, []
    for i in range(frames):
        t = start + 0.1 * i
        dets = fake_detections(cfg, "rgb", pan, tilt, (0, 0, 0), np.array(fire), t, rng, p_miss=0, p_outlier=0) if fire_on else []
        for d in dets:
            d.confidence = conf if conf is not None else d.confidence
        cmd = ctl.step(t, dets, (0, 0, 0), pan, tilt)
        pan, tilt = cmd.pan, cmd.tilt
        cmds.append(cmd)
    return ctl, cmds


def test_no_spray_at_low_confidence():
    ctl, cmds = run_static((1.8, 0.3, 0), 100, conf=0.3)
    assert not any(c.pump_on for c in cmds)
    assert ctl.state == "SEARCH"


def test_no_spray_too_close():
    ctl = Controller(CONCEPTS["A"])
    ctl.cc.min_range = 2.0  # fire at 1.8 m is too close; the robot asks to back up but never moves here
    ctl, cmds = run_static((1.8, 0.3, 0), 100, ctl=ctl)
    assert not any(c.pump_on for c in cmds)
    assert any(c.vx < 0 for c in cmds) and all(c.yaw_rate == 0 for c in cmds if c.vx < 0)


def test_spray_then_stop_when_fire_gone():
    ctl, cmds = run_static((1.8, 0.3, 0), 40)
    assert cmds[-1].pump_on and ctl.state == "SPRAY"
    gone = ctl.cc.gone_frames
    check, search = round(ctl.cc.check_time * 10), round(ctl.cc.search_time * 10)
    ctl, cmds = run_static((1.8, 0.3, 0), gone + check + search + 5, start=4.0, ctl=ctl, fire_on=False)
    off_after = next(i for i, c in enumerate(cmds) if not c.pump_on)
    assert off_after * 0.1 <= ctl.cc.max_age + 1e-9  # no recent detection: pump off
    assert cmds[gone].state == "CHECK"
    assert cmds[gone + check + 1].state == "SEARCH"  # look for other fires before DONE
    assert cmds[gone + check + search - 2].state == "SEARCH" and ctl.state == "DONE"


def test_check_and_done_hold_commanded_angles():
    """A sagging tilt servo must not drag the target: CHECK and DONE hold the last commanded angles."""
    cfg, fire, sag = CONCEPTS["A"], np.array([1.8, 0.3, 0.0]), 0.02  # rad, measured = commanded - sag
    ctl, rng = Controller(cfg), np.random.default_rng(0)
    ctl.cc.search_time = 2.0
    pan, tilt, cmds = 0.0, 0.0, []
    for i in range(200):
        t = 0.1 * i
        dets = fake_detections(cfg, "rgb", pan, tilt, (0, 0, 0), fire, t, rng, p_miss=0, p_outlier=0) if i < 40 else []
        cmd = ctl.step(t, dets, (0, 0, 0), pan, tilt)
        pan, tilt = cmd.pan, cmd.tilt - sag
        cmds.append(cmd)
    last_spray = [c for c in cmds if c.state == "SPRAY"][-1]
    held = [c for c in cmds if c.state in ("CHECK", "DONE")]
    assert held and ctl.state == "DONE"
    assert all(c.tilt == last_spray.tilt and c.pan == last_spray.pan for c in held if c.state == "CHECK")
    assert min(c.tilt for c in cmds) >= CONCEPTS["A"]["tilt_limits"][0]  # no creep to the limit


def test_two_fires_both_sprayed_then_done():
    """Two fires in view. After the first one is out, the controller searches again and sprays the second."""
    from sim_demo import landing_point
    cfg = CONCEPTS["A"]
    ctl, rng = Controller(cfg), np.random.default_rng(0)
    ctl.cc.search_time = 3.0
    fires = {k: np.array(f) for k, f in (("a", (1.8, 0.3, 0.0)), ("b", (1.6, -0.6, 0.0)))}
    health = {"a": 1.0, "b": 1.0}
    pan, tilt, states = 0.0, 0.0, []
    for i in range(600):
        t = 0.1 * i
        dets = [d for k, f in fires.items() if health[k] > 0
                for d in fake_detections(cfg, "rgb", pan, tilt, (0, 0, 0), f, t, rng, p_miss=0, p_outlier=0)]
        cmd = ctl.step(t, dets, (0, 0, 0), pan, tilt)
        if cmd.pump_on:
            hit = landing_point(cfg, pan, tilt, ctl.cc.v0, ctl.cc.drag)
            for k, f in fires.items():
                if np.linalg.norm(hit[:2] - f[:2]) < 0.25:
                    health[k] -= 0.1 / 3.0
        pan, tilt = cmd.pan, cmd.tilt
        states.append(cmd.state)
        if cmd.state == "DONE":
            break
    assert health["a"] <= 0 and health["b"] <= 0, health
    assert ctl.sprays == 2 and states[-1] == "DONE"
    assert states[-round(ctl.cc.search_time * 10) - 1:-1] == ["SEARCH"] * round(ctl.cc.search_time * 10)


def test_delay_compensation():
    """Image 100 ms older than the control step, pan moving: the estimate uses the angles at the image time."""
    cfg, fire, delay, dt = CONCEPTS["A"], np.array([2.0, 0.2, 0.0]), 0.1, 0.04
    ctl, rng, tilt = Controller(cfg), np.random.default_rng(0), -0.3

    def pan_at(t):
        return 0.3 * math.sin(1.5 * t)

    worst = worst_uncompensated = 0.0
    for i in range(100):
        now = i * dt
        t_img = now - delay
        dets = []
        if t_img >= 0:
            dets = fake_detections(cfg, "rgb", pan_at(t_img), tilt, (0, 0, 0), fire, t_img, rng, noise_px=0, p_miss=0, p_outlier=0)
            assert dets
            worst_uncompensated = max(worst_uncompensated, np.linalg.norm(localise(cfg, dets[0], pan_at(now), tilt) - fire))
        ctl.step(now, dets, (0, 0, 0), pan_at(now), tilt)
        if ctl.filter.x is not None:
            worst = max(worst, np.linalg.norm(ctl.filter.x - fire))
    assert worst < 0.03
    assert worst_uncompensated > 0.05  # the test would catch a missing compensation


def test_parse_detection_json():
    from run import parse
    good = '{"timestamp": 9.9, "camera": "rgb", "u": 320, "v": 300, "bbox": [300, 280, 340, 320], "confidence": 0.9}'
    future = good.replace("9.9", "10.5")
    bad = ['{"camera": "rgb"}', "not json", good.replace('"rgb"', '"ir"')]
    dets, frames = parse("\n".join([good, future] + bad).encode(), {"rgb": None}, now=10.0)
    assert len(dets) == 1 and dets[0].bbox == (300.0, 280.0, 340.0, 320.0) and dets[0].depth_m is None and frames == 0


def test_parse_interface_v2_frames():
    import json
    from run import parse
    det = {"u": 320, "v": 300, "bbox": [300, 280, 340, 320], "confidence": 0.9}
    frame = {"timestamp": 9.9, "camera": "rgb", "frame": True, "detections": [det, dict(det, u=100)]}
    empty = {"timestamp": 9.95, "camera": "rgb", "frame": True, "detections": []}
    bad = dict(empty, camera="ir")
    v1 = dict(det, timestamp=9.8, camera="rgb")
    lines = "\n".join(json.dumps(m) for m in (frame, empty, bad, v1)).encode()
    dets, frames = parse(lines, {"rgb": None}, now=10.0)
    assert frames == 2  # the bad frame does not count
    assert [d.u for d in dets] == [320.0, 100.0, 320.0] and dets[0].timestamp == 9.9 and dets[0].camera == "rgb"


def test_pose_input_and_dead_reckoning():
    from run import Odometry
    odo = Odometry()
    odo.message({"timestamp": 9.9, "x": 1.0, "y": 2.0, "yaw": 0.5}, now=10.0)
    odo.advance(10.0, 0.25, 0.3, 0.1)  # fresh pose: no integration
    assert odo.pose == (1.0, 2.0, 0.5, 0.0, 0.0) and not odo.reckoning
    odo.message({"timestamp": 9.9, "x": 1.0, "y": 2.0, "yaw": 0.5, "roll": 0.01, "pitch": -0.02}, now=10.0)
    assert odo.pose[3:] == (0.01, -0.02)
    for i in range(10):  # no pose message for 1 s: integrate the walk command
        odo.advance(10.5 + 0.1 * i, 0.25, 0.0, 0.1)
    assert odo.reckoning and abs(odo.pose[0] - (1.0 + 0.25 * math.cos(0.5))) < 1e-9
    assert abs(odo.pose[1] - (2.0 + 0.25 * math.sin(0.5))) < 1e-9
    bench = Odometry(walk=False)
    bench.advance(100.0, 0.25, 0.3, 0.1)
    assert bench.pose == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_confirmation_counts_frames_in_a_time_window():
    """Detections in every other tick (late frames): 3 within 0.6 s still confirm the fire."""
    cfg, fire = CONCEPTS["A"], np.array([1.8, 0.3, 0.0])
    ctl, rng = Controller(cfg), np.random.default_rng(0)
    states = []
    for i in range(10):
        t = 0.1 * i
        dets = fake_detections(cfg, "rgb", 0.0, ctl.cc.search_tilt, (0, 0, 0), fire, t, rng, p_miss=0, p_outlier=0) if i % 2 == 0 else []
        states.append(ctl.step(t, dets, (0, 0, 0), 0.0, ctl.cc.search_tilt).state)
    assert states[-1] == "APPROACH" or "APPROACH" in states


def test_late_frames_are_not_misses():
    ctl = Controller(CONCEPTS["A"])
    ctl.step(0.0, [], (0, 0, 0), 0.0, 0.0, frames=0)  # v2 detector, frame late
    assert ctl.missed == 0
    ctl.step(0.1, [], (0, 0, 0), 0.0, 0.0, frames=1)  # v2 frame with no fire
    ctl.step(0.2, [], (0, 0, 0), 0.0, 0.0)  # old format: each step without a fire is a miss
    assert ctl.missed == 2


@pytest.mark.parametrize("concept", ["A", "B", "E"])
@pytest.mark.parametrize("fire", [(2.5, 3.0, 0.0), (-3.0, 0.0, 0.0)])  # in front on the left, and behind
def test_no_turn_on_the_spot(concept, fire):
    """The BHL policy cannot turn on the spot. The toy simulation has the same limit and must still put out the fire."""
    from sim_demo import run
    _, ctl, _, log = run(concept, fire=fire)
    assert not [r["cmd"] for r in log if abs(r["cmd"].yaw_rate) > 0.1 and abs(r["cmd"].vx) < 0.1]
    assert log[-1]["health"] <= 0 and ctl.state == "DONE"


def test_off_centre_fire_uses_pan_not_body():
    ctl, cmds = run_static((1.3, 0.75, 0), 60)  # 1.5 m away, 30 deg to the left
    after_search = [c for c in cmds if c.state != "SEARCH"]
    assert after_search and all(c.yaw_rate == 0 and c.vx == 0 for c in after_search)
    assert any(c.pump_on for c in cmds) and abs(cmds[-1].pan - math.radians(30)) < math.radians(3)


def test_too_close_steps_back_once_then_sprays_once():
    """Fire too close for the jet: step back past the margin, then one spray, no SPRAY <-> APPROACH switching."""
    import itertools
    from sim_demo import run
    _, ctl, _, log = run("A", fire=(0.75, 0.0, 0.0), t_max=80)
    states = [k for k, _ in itertools.groupby(r["cmd"].state for r in log)]
    first_spray = next(i for i, r in enumerate(log) if r["cmd"].state == "SPRAY")
    assert any(r["cmd"].vx < 0 for r in log[:first_spray])
    assert ctl.sprays == 1 and log[-1]["health"] <= 0 and ctl.state == "DONE"
    assert ("SPRAY", "APPROACH") not in set(zip(states, states[1:]))



def test_torso_tilt_in_localisation_and_aim():
    """Camera and nozzle poses include the torso roll and pitch (base rotation = yaw * pitch * roll)."""
    cfg, fire, att = CONCEPTS["A"], np.array([2.0, 0.3, 0.0]), (0.04, 0.08)
    pan, tilt = 0.15, -0.35
    u, v = project(cfg, "rgb", pan, tilt, fire, att)
    det = FireDetection(0, "rgb", u, v, (0, 0, 0, 0), 1)
    assert np.linalg.norm(localise(cfg, det, pan, tilt, att=att) - fire) < 1e-6
    assert np.linalg.norm(localise(cfg, det, pan, tilt) - fire) > 0.3  # a level model would be far off
    aim = solve_aim(cfg, fire, 8.0, att=att)
    e, d = nozzle_exit(cfg, aim.pan, aim.tilt, att)
    rel = fire - e
    dist = math.hypot(rel[0], rel[1])
    elev = math.asin(d[2])
    lateral = abs(d[0] * rel[1] - d[1] * rel[0]) / math.hypot(d[0], d[1])
    z = dist * math.tan(elev) - G * dist ** 2 / (2 * 8.0 ** 2 * math.cos(elev) ** 2)
    assert aim.in_range and math.hypot(lateral, z - rel[2]) < 0.02
    assert abs(aim.tilt - solve_aim(cfg, fire, 8.0).tilt) > 0.05  # the servo angles change with the tilt


def test_no_spray_when_torso_tilted_or_just_started():
    """A steady tilt is compensated. A large tilt, a fast tilt change and the first 2 s block the spray."""
    cfg, fire = CONCEPTS["A"], np.array([1.8, 0.3, 0.0])
    ctl = Controller(cfg)
    pan, tilt, pumps = 0.0, 0.0, []
    for i in range(160):
        t = 0.1 * i
        pitch = -0.09  # steady 5 deg nose-up, as in the software-in-the-loop test
        if 60 <= i < 62:
            pitch = 0.0  # a 0.09 rad change in 0.1 s (52 deg/s) at 6.0 s: a stumble
        if 100 <= i < 102:
            pitch = 0.3  # 17 deg at 10.0 s
        u, v = project(cfg, "rgb", pan, tilt, fire, (0.0, pitch))  # the camera tilts with the torso
        dets = [FireDetection(t, "rgb", u, v, (u - 5, v - 5, u + 5, v + 5), 0.9)]
        cmd = ctl.step(t, dets, (0.0, 0.0, 0.0, 0.0, pitch), pan, tilt)
        pan, tilt = cmd.pan, cmd.tilt
        pumps.append(cmd.pump_on)
    assert not any(pumps[:20])  # first 2 s after start
    assert all(pumps[30:60])  # steady 5 deg tilt: the pump runs
    assert not any(pumps[60:82])  # fast change at 6.0 s, then 2 s
    assert any(pumps[84:100])
    assert not any(pumps[100:122])  # large tilt at 10.0 s, then 2 s
