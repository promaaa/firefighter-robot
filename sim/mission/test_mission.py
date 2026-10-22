"""Self-checks for the mission simulation. Run from the repository root:

    MUJOCO_GL=egl uv run pytest sim/mission/test_mission.py -q
"""
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE)]
import world  # noqa: E402
from ballistics import jet_path  # noqa: E402
from detection import FireDetection  # noqa: E402
from geometry import localise, nozzle_exit, project  # noqa: E402

CFG = world.CFG


def model(seed=4, **over):
    spec = {**world.random_spec(seed), **over}
    m = mujoco.MjModel.from_xml_string(world.build_xml(spec))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    return spec, m, d


def place(m, d, x, y, yaw, pan, tilt):
    """Robot upright with its base origin at the floor-standing height, pan and tilt joints set."""
    d.qpos[:3] = x, y, -world.DZ
    d.qpos[3:7] = math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)
    d.qpos[m.joint("pan").qposadr[0]], d.qpos[m.joint("tilt").qposadr[0]] = pan, tilt
    mujoco.mj_forward(m, d)


def test_sim_camera_and_nozzle_match_aim_geometry():
    """The MuJoCo camera and nozzle of the scene are where aim/geometry.py (F2) puts them: same pixel, same jet axis."""
    spec, m, d = model(cam_pitch_err=0.0)
    cam = m.camera("rgb").id
    for pan, tilt in [(0.0, 0.0), (0.6, -0.3), (-1.2, 0.4)]:
        place(m, d, 0.0, 0.0, 0.0, pan, tilt)
        for p in [(2.0, 0.3, 0.0), (1.5, -0.8, 0.2), (3.5, 1.0, 0.0)]:
            pc = d.cam_xmat[cam].reshape(3, 3).T @ (np.array(p) - d.cam_xpos[cam])
            uv_sim = (world.W / 2 + world.K.fy * pc[0] / -pc[2], world.H / 2 - world.K.fy * pc[1] / -pc[2])
            uv_aim = project(CFG, "rgb", pan, tilt, p)
            assert np.allclose(uv_sim, uv_aim, atol=0.5), (pan, tilt, p, uv_sim, uv_aim)
        e, dvec = nozzle_exit(CFG, pan, tilt)
        assert np.allclose(d.site_xpos[m.site("nozzle").id], e, atol=1e-6)
        assert np.allclose(d.xmat[m.body("tilt").id].reshape(3, 3)[:, 0], dvec, atol=1e-6)


def test_leg_drive_is_the_hw_risks_pd():
    """Leg actuators: tau = clip(20 (goal - measured) - 2 qd, +-6 Nm), with ctrl = goal - zero error (hw_risks case 7)."""
    _, m, d = model()
    rng = np.random.default_rng(0)
    d.qvel[6:18] = rng.uniform(-3, 3, 12)
    d.ctrl[:12] = d.qpos[7:19] + rng.uniform(-0.4, 0.4, 12)
    mujoco.mj_forward(m, d)
    want = np.clip(20 * (d.ctrl[:12] - d.qpos[7:19]) - 2 * d.qvel[6:18], -6, 6)
    assert np.allclose(d.actuator_force[:12], want, atol=1e-9)


def test_droplets_follow_the_aim_ballistics():
    """With no spread, the droplet model lands where aim/ballistics.jet_path lands (same v0, drag)."""
    spec = {**world.random_spec(4), "v0_scale": 1.0, "drag": 0.11, "outdoor": True}
    w = world.Water(spec, np.zeros((0, 2, 3)), np.random.default_rng(0))
    old = world.SPREAD
    world.SPREAD = 0.0
    try:
        el = math.radians(5)
        dvec = np.array([math.cos(el), 0, math.sin(el)])
        for k in range(600):  # pump on for 0.1 s: 40 droplets
            w.step(k * world.DROP_DT, k * world.DROP_DT < 0.1, np.array([0, 0, 0.9]), dvec, [], [])
    finally:
        world.SPREAD = old
    x = np.mean([p[0] for p in w.landed])
    ref = jet_path(w.v0, el, 0.11, z_stop=-0.9)[-1][0]
    assert len(w.landed) == 40 and abs(x - ref) < 0.06, (x, ref)  # 3 % speed noise per droplet


def test_fire_model():
    sp = world.random_spec(4)
    f = [dict(x=0, y=0, d=0.2, phase=0)]
    need = sp["water_per_area"] * math.pi * 0.01  # kg to put out a full 0.2 m fire
    s = world.fire_step(f, [1.0], [0.5 * need], 0.0, sp)
    assert abs(s[0] - 0.5) < 1e-9
    s = world.fire_step(f, s, [0.0], 3.0, sp)  # unattended: regrows
    assert abs(s[0] - (0.5 + 3.0 / sp["regrow"])) < 1e-9
    assert world.fire_step(f, [0.1], [need], 0.0, sp) == [0.0]
    assert world.fire_step(f, [0.0], [0.0], 100.0, sp) == [0.0]  # out stays out
    # docs/hardware/water.md section 7: the lowest water rate that beats the regrowth is 9 to 15 L/(m^2 min)
    assert 9 <= sp["water_per_area"] / sp["regrow"] * 60 <= 15


def test_spec_is_self_contained_and_the_robot_is_f2():
    """A mission spec stores its jet and fire model, and its scene model keeps the 15.99 kg of bhl_F2 (full tanks)."""
    sp = world.random_spec(1086)
    assert sp["robot"] == "F2" and all(sp[k] == v for k, v in world.DESIGNS["f2"].items() if k not in ("tray", "drag"))
    m = mujoco.MjModel.from_xml_string(world.build_xml(sp))
    assert abs(m.body_subtreemass[1] - 15.99) < 0.05, m.body_subtreemass[1]


@pytest.mark.parametrize("kind", ["procedural", "real"])
def test_flame_is_drawn_and_hidden_by_a_box(kind):
    """A fire in front of the camera gives flame pixels. A box between them hides the flame (depth test)."""
    spec, m, d = model(seed=4)
    spec["fires"] = [dict(x=2.0, y=0.0, d=0.25, phase=0)]
    sprites = world.flame_sprites(kind)
    R = np.stack([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], 1).astype(float)  # looks along +x
    rgb = np.full((world.H, world.W, 3), 90, np.uint8)
    far = np.full((world.H, world.W), 10.0, np.float32)
    img, info = world.composite_fires(rgb, far, np.array([0, 0, 0.93]), R, world.K.fy, spec["fires"], [1.0], 0.0, sprites)
    n, box = info[0]
    assert n > 300 and box is not None
    assert img[box[1]:box[3], box[0]:box[2], 0].mean() > 150  # bright, red channel high
    near = far.copy()
    near[:, :] = 1.0  # a wall 1 m away
    _, info = world.composite_fires(rgb, near, np.array([0, 0, 0.93]), R, world.K.fy, spec["fires"], [1.0], 0.0, sprites)
    assert info[0][0] == 0


@pytest.mark.parametrize("kind", ["procedural", "real"])
def test_floor_localisation_of_the_flame_base(kind):
    """The bottom centre of the true flame box, put on the floor by aim/, lands near the tray centre."""
    spec, m, d = model(seed=4, cam_pitch_err=0.0)
    place(m, d, 0.0, 0.0, 0.0, 0.0, -0.2)
    cam = m.camera("rgb").id
    fire = dict(x=2.2, y=0.4, d=0.2, phase=0)
    rgb = np.zeros((world.H, world.W, 3), np.uint8)
    _, info = world.composite_fires(rgb, np.full((world.H, world.W), 50.0, np.float32), d.cam_xpos[cam].copy(),
                                    d.cam_xmat[cam].reshape(3, 3).copy(), world.K.fy, [fire], [1.0], 0.0, world.flame_sprites(kind))
    b = info[0][1]
    p = localise(CFG, FireDetection(0, "rgb", (b[0] + b[2]) / 2, b[3], b, 1.0), 0.0, -0.2)
    assert math.hypot(p[0] - 2.2, p[1] - 0.4) < 0.12, p


def test_random_missions_are_valid():
    for seed in range(20):
        sp = world.random_spec(seed)
        x, y, _ = sp["start"]
        assert 1 <= len(sp["fires"]) <= 3
        for f in sp["fires"]:
            assert 1.5 - 1e-9 <= math.hypot(f["x"] - x, f["y"] - y) <= 5.0 + 1e-9 and 0.1 <= f["d"] <= 0.3
        m = mujoco.MjModel.from_xml_string(world.build_xml(sp))
        assert m.nu == 14 and m.joint("pan").range[1] > 1.5


def test_real_sprites_loop_and_mip_levels():
    """Each real clip: 60 frames with alpha, mip levels that halve the size and keep the mean alpha (area average)."""
    clips = world.flame_sprites("real")
    assert len(clips) >= 4
    for levels in clips:
        f = levels[0]
        assert f.shape[0] == 60 and f.shape[1] == 2 * f.shape[2] and 0.1 < f[..., 3].mean() < 0.7
        assert levels[1].shape[1] == f.shape[1] // 2 and abs(levels[1][..., 3].mean() - f[..., 3].mean()) < 1e-3
        step = np.abs(np.diff(f[..., 3], axis=0)).mean((1, 2))  # the flame moves fast: compare the loop seam with real steps
        assert np.abs(f[0, ..., 3] - f[-1, ..., 3]).mean() <= 1.05 * step.max()  # the cross-fade: no jump at the loop end
