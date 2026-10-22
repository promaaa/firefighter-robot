"""One firefighting mission with the final robot F2: walking policy + rendered robot camera with fire sprites + detector +
aim/ controller + ST3215 pan-tilt + water droplets + fire model.

    MUJOCO_GL=egl uv run python sim/mission/mission.py --seed 3                      one mission, prints the result
    MUJOCO_GL=egl uv run python sim/mission/mission.py --seed 3 --video hero.mp4     the same, with the video
    MUJOCO_GL=egl uv run python sim/mission/mission.py --seed 3 --sil                aim/run.py and the actuator node as
                                                                                     separate processes over UDP
Options: --detector FILE (detect(rgb, t)), --policy FILE (ONNX, BHL interface), --outdoor / --indoor.
"""
import argparse
import importlib.util
import json
import math
import os
import subprocess
import sys
import time
from collections import deque
from dataclasses import asdict
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT / "aim")]
import world  # noqa: E402
from controller import Controller  # noqa: E402
from geometry import localise, robot_to_world, world_to_robot  # noqa: E402


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


s2s = load_module(ROOT / "sim" / "sim2sim" / "run.py", "s2s_run")  # sim/sim2sim/run.py (aim/ also has a run.py)
ACTION_SCALE, DEFAULT, NOISE, POLICY, TILT_LIMIT = s2s.ACTION_SCALE, s2s.DEFAULT, s2s.NOISE, s2s.POLICY, s2s.TILT_LIMIT
observation, quat_rotate_inverse = s2s.observation, s2s.quat_rotate_inverse

CFG = world.CFG
BASELINE = ROOT / "sim" / "sim2sim" / "baseline_detector.py"
OUT = HERE / "results"
RUNS = Path.home() / "runs" / "firefighter" / "mission"
PHYSICS_DT, POLICY_DT, AIM_DT, CHUNK = 0.0005, 0.04, 0.1, 10  # s, s, s, physics steps per chunk (5 ms: servos, droplets)
ZERO_DEG, CMD_DELAY = 2.0, 20  # hw_risks case 7: leg zero error +-2 deg, 10 ms command latency (20 physics steps)
SERVO_SPEED = 4.7  # rad/s, ST3215 at 12 V: 0.222 s / 60 deg (45 rpm), Waveshare listing
DAMP_D = 2.0  # Nm s/rad, legs after a fall or ABORT: damping only (K = 0)
FPS = 30
PIVOT_H = 0.64 + world.DZ  # m, aim/geometry.BASE_Z in the base frame: aim/ tilts the torso about this point


def load_detector(path):
    mod = load_module(path, "detector")
    if hasattr(mod, "FireDetector"):  # vision/detector.py: one onnxruntime thread, so parallel missions do not fight for cores
        mod._detector = mod.FireDetector(threads=1)
    return mod.detect


class Policy:
    """Any ONNX walking policy with the BHL interface: obs 1 x 45 in, actions 1 x 12 out."""

    def __init__(self, path):
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        self.s = ort.InferenceSession(str(path), so, providers=["CPUExecutionProvider"])
        self.inp = self.s.get_inputs()[0].name
        assert self.s.get_inputs()[0].shape[-1] == 45, "policy input is not the BHL 45-value observation"

    def __call__(self, obs):
        return self.s.run(None, {self.inp: obs})[0][0]


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def draw_box(img, box, color, w=2):
    u0, v0, u1, v1 = [int(round(c)) for c in box]
    h, wd = img.shape[:2]
    u0, u1 = np.clip([u0, u1], 0, wd - 1)
    v0, v1 = np.clip([v0, v1], 0, h - 1)
    img[v0:v0 + w, u0:u1 + 1] = img[max(v1 - w + 1, 0):v1 + 1, u0:u1 + 1] = color
    img[v0:v1 + 1, u0:u0 + w] = img[v0:v1 + 1, max(u1 - w + 1, 0):u1 + 1] = color


def add_sphere(scene, pos, r, rgba):
    if scene.ngeom < scene.maxgeom:
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([r, r, r]),
                            np.asarray(pos, float), np.eye(3).flatten(), np.asarray(rgba, np.float32))
        scene.ngeom += 1


def match(dets, info):
    """Label each detection: index of the fire whose visible flame box it overlaps (most overlap), or -1."""
    out = []
    for det in dets:
        u0, v0, u1, v1 = det.bbox
        best, area = -1, 0.0
        for i, (n, box) in enumerate(info):
            if box is None:
                continue
            a = max(0, min(u1, box[2] + 3) - max(u0, box[0] - 3)) * max(0, min(v1, box[3] + 3) - max(v0, box[1] - 3))
            if a > area:
                best, area = i, a
        out.append(best)
    return out


def false_label(det, d, cam_id, points):
    """Name of the distractor nearest to a false detection (projected), or 'other'."""
    best, bd = "other", 80.0
    for name, p in points.items():
        pc = d.cam_xmat[cam_id].reshape(3, 3).T @ (np.asarray(p) - d.cam_xpos[cam_id])
        if pc[2] > -0.05:
            continue
        u, v = world.W / 2 + world.K.fy * pc[0] / -pc[2], world.H / 2 - world.K.fy * pc[1] / -pc[2]
        u0, v0, u1, v1 = det.bbox
        dist = 0.0 if u0 - 10 <= u <= u1 + 10 and v0 - 10 <= v <= v1 + 10 else math.hypot(u - (u0 + u1) / 2, v - (v0 + v1) / 2)
        if dist < bd:
            best, bd = name, dist
    return best


class Video:
    """Third-person follow camera (960 x 540, 30 fps) with the fire sprites, droplets, robot camera inset and labels."""

    def __init__(self, m, path, crf=26, speed=1.0, title="", room=None):
        self.w, self.h = 960, 540
        self.room = room  # (lx, ly) of an indoor room: the camera stays inside it
        self.furn = [g for g in range(m.ngeom) if m.geom(g).name.startswith("furn")]
        self.r = mujoco.Renderer(m, self.h, self.w, max_geom=4000)
        self.cam = mujoco.MjvCamera()
        self.cam.type, self.cam.distance, self.cam.elevation = mujoco.mjtCamera.mjCAMERA_FREE, 3.6, -24.0
        self.look, self.az = None, None
        self.speed, self.title, self.n = speed, title, 0
        self.ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{self.w}x{self.h}",
                                    "-r", str(FPS), "-i", "-", "-pix_fmt", "yuv420p", "-crf", str(crf), "-preset", "slow", str(path)],
                                   stdin=subprocess.PIPE)
        try:
            self.font = ImageFont.truetype("DejaVuSans.ttf", 18)
            self.small = ImageFont.truetype("DejaVuSans.ttf", 14)
        except OSError:
            self.font = self.small = ImageFont.load_default()

    def _inside(self, az, el, dist):
        a, e = math.radians(az), math.radians(el)
        cx = self.look[0] - dist * math.cos(e) * math.cos(a)
        cy = self.look[1] - dist * math.cos(e) * math.sin(a)
        return abs(cx) < self.room[0] / 2 - 0.15 and abs(cy) < self.room[1] / 2 - 0.15

    def frame(self, m, d, base, target, drops, fires, strengths, t, sprites, inset, state, text_lines):
        p = d.xpos[base]
        yaw = yaw_of(d.xmat[base].reshape(3, 3))
        goal = np.array([p[0], p[1], 0.45]) if target is None else (np.array([p[0], p[1], 0.45]) + np.array([target[0], target[1], 0.2])) / 2
        self.look = goal if self.look is None else self.look + 0.04 * (goal - self.look)
        if state == "ABORT" and getattr(self, "dist", None):  # a fallen robot turns fast: keep the last camera angles
            self.cam.lookat[:] = self.look
            alpha = m.geom_rgba[self.furn, 3].copy()
            m.geom_rgba[self.furn, 3] = 0.35
            self.r.update_scene(d, self.cam)
            m.geom_rgba[self.furn, 3] = alpha
            return self._finish(m, d, drops, fires, strengths, t, sprites, inset, text_lines)
        # behind the robot and to its side, looking along the robot-to-fire line (or the heading)
        az = math.degrees(yaw if target is None else math.atan2(target[1] - p[1], target[0] - p[0])) + 50.0
        dist = 3.4 if target is None else float(np.clip(1.8 + 0.6 * math.hypot(target[0] - p[0], target[1] - p[1]), 3.0, 4.6))
        elev = -30.0
        if self.room:  # the room is convex: a camera inside it always sees the robot. Turn around the robot to find such a pose
            for off in (0, 30, -30, 60, -60, 90, -90, 120, -120, 180):
                ok = [e for e in (-30.0, -40.0, -50.0) if self._inside(az + off, e, dist)]
                if ok:
                    az, elev = az + off, ok[0]
                    break
        self.az = az if self.az is None else self.az + 0.03 * math.remainder(az - self.az, 360)
        self.el = elev if getattr(self, "el", None) is None else self.el + 0.05 * (elev - self.el)
        while self.room and dist > 1.0 and not self._inside(self.az, self.el, dist):  # last resort: come closer
            dist -= 0.1
        self.cam.lookat[:], self.cam.azimuth, self.cam.elevation = self.look, self.az, self.el
        self.cam.distance = self.dist = dist
        alpha = m.geom_rgba[self.furn, 3].copy()
        m.geom_rgba[self.furn, 3] = 0.35  # furniture half transparent in this view only
        self.r.update_scene(d, self.cam)
        m.geom_rgba[self.furn, 3] = alpha
        return self._finish(m, d, drops, fires, strengths, t, sprites, inset, text_lines)

    def _finish(self, m, d, drops, fires, strengths, t, sprites, inset, text_lines):
        for q in drops[::2]:
            add_sphere(self.r.scene, q, 0.012, (0.55, 0.75, 1.0, 0.85))
        rgb, depth = world.render_rgbd(self.r)
        gl = self.r.scene.camera[0]
        fwd, up = np.array(gl.forward), np.array(gl.up)
        R = np.stack([np.cross(fwd, up), up, -fwd], 1)
        fy = self.h / 2 * gl.frustum_near / gl.frustum_top
        img, _ = world.composite_fires(rgb, depth, np.array(gl.pos), R, fy, fires, strengths, t, sprites)
        ih, iw = inset.shape[:2]
        img[8:12 + ih, self.w - 12 - iw:self.w - 8] = 255
        img[10:10 + ih, self.w - 10 - iw:self.w - 10] = inset
        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im)
        dr.rectangle([8, 8, 330, 16 + 22 * (len(text_lines) + 1)], fill=(255, 255, 255))
        dr.text((16, 12), f"{self.title}  t = {t:5.1f} s", fill=(0, 0, 0), font=self.font)
        for k, line in enumerate(text_lines):
            dr.text((16, 36 + 22 * k), line, fill=(0, 0, 0), font=self.small if k else self.font)
        dr.text((self.w - 10 - iw, 14 + ih), "robot camera (B0201, 640 x 480)", fill=(255, 255, 255), font=self.small)
        self.ff.stdin.write(np.asarray(im).tobytes())
        self.n += 1

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()
        self.r.close()


def run(spec, detector=BASELINE, policy=POLICY, video=None, sil=None, title="", crf=26, video_speed=1.0, keep_log=True, limit=6.0,
        snaps=None, aim=None):
    """One mission. Returns a JSON-able result dict. video: mp4 path. sil: None or a SilLink-like object factory.
    limit: leg torque limit, Nm (hw_risks case 7: 6 Nm). snaps: (directory, list of times): save the robot camera image
    with the detection boxes at these times. aim: aim/ ControllerConfig values to override, for example {"min_conf": 0.3}."""
    t_wall = time.time()
    detect = None if str(detector) == "oracle" else load_detector(detector)
    m = mujoco.MjModel.from_xml_string(world.build_xml(spec))
    m.opt.timestep = PHYSICS_DT
    m.actuator_forcerange[:12] = [-limit, limit]
    d = mujoco.MjData(m)
    legs = [m.joint(f"leg_{s}_{j}_joint").qposadr[0] for s in ("left", "right")
            for j in ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")]
    assert legs == list(range(7, 19)) and [m.actuator(i).name for i in range(12)] == [m.joint(q - 6).name for q in legs]
    jpan, jtilt = m.joint("pan").qposadr[0], m.joint("tilt").qposadr[0]
    vpan, vtilt = m.joint("pan").dofadr[0], m.joint("tilt").dofadr[0]
    base, tiltb, nozzle = m.body("base").id, m.body("tilt").id, m.site("nozzle").id
    cam_id = m.camera("rgb").id
    water_b = m.body("payload_water").id
    m_water, i_water = m.body_mass[water_b], m.body_inertia[water_b].copy()
    fire_lights = [m.light(f"fire{i}").id for i in range(len(spec["fires"]))]
    light0 = m.light_diffuse[fire_lights].copy() if fire_lights else None

    x0, y0, yaw0 = spec["start"]
    d.qpos[:3] = x0, y0, 0.0  # BHL start: upright, default joints, feet 3.9 cm above the floor
    d.qpos[3:7] = math.cos(yaw0 / 2), 0, 0, math.sin(yaw0 / 2)
    d.qpos[7:19] = DEFAULT
    mujoco.mj_forward(m, d)

    rng = np.random.default_rng([spec["seed"], 11])
    off = np.deg2rad(np.random.default_rng([spec["seed"], 1]).uniform(-1, 1, 12) * ZERO_DEG)  # leg zero errors
    pol = Policy(policy)
    ctl = Controller(CFG)
    # the jet of this design, as aim/ knows it: speed at the measured battery voltage, nominal drag, tank size
    ctl.cc.v0, ctl.cc.drag, ctl.cc.tank_time = world.jet_v0(spec), spec["drag_aim"], spec["tank"] / spec["flow"] * 60
    for k, v in (aim or {}).items():
        assert hasattr(ctl.cc, k), k
        setattr(ctl.cc, k, v)
    fires, strengths = spec["fires"], [1.0] * len(spec["fires"])
    t_out = [None] * len(fires)
    sprites = world.flame_sprites(spec.get("flames", "procedural"))
    water = world.Water(spec, world.obstacle_boxes(m), np.random.default_rng([spec["seed"], 3]))
    dpoints = world.distractor_points(spec)
    robot_view = mujoco.Renderer(m, world.H, world.W, max_geom=4000)
    ropt = mujoco.MjvOption()
    ropt.geomgroup[4] = 1  # the ceiling: robot camera only
    noise_bank = [np.random.default_rng([spec["seed"], 5, k]).normal(0, 2.5, (world.H, world.W, 3)).astype(np.float32) for k in range(4)]
    vid = Video(m, video, crf, video_speed, title, None if spec["outdoor"] else spec["room"]) if video else None
    link = sil(spec) if sil else None

    action = np.zeros(12, np.float32)
    cmd_vec = np.zeros(3, np.float32)
    pan_ref = tilt_ref = 0.0  # servo goal positions (true joint angles)
    pan_goal, tilt_goal = 0.0, ctl.cc.search_tilt
    pump = False
    pending = []  # detections of the last image: the controller gets them one camera period later (detector latency)
    next_aim = next_frame = 0.0
    z0 = d.site("imu").xpos[2]
    fall = abort = None
    damping = False
    t_done = None
    cmd = None
    # odometry seen by aim/: the pivot point pose, with a distance scale error and a gyro bias
    piv = d.xpos[base] + d.xmat[base].reshape(3, 3) @ np.array([0, 0, PIVOT_H])
    odo = np.array([piv[0], piv[1]])
    last_piv = piv[:2].copy()
    inset = np.zeros((192, 256, 3), np.uint8)
    log, det_samples, loc_samples, false_dets, sprays, counts = [], [], [], {}, [], dict(tp=0, fp=0, frames=0)
    path, on_fire = [], 0.0
    t_max = ctl.cc.mission_time + 5.0
    steps_per_policy = round(POLICY_DT / PHYSICS_DT)
    while d.time < t_max:
        R = d.xmat[base].reshape(3, 3)
        quat, gyro = d.sensor("imu_quat").data.copy(), d.sensor("imu_gyro").data.copy()
        obs = observation(cmd_vec, quat, gyro, d.qpos[7:19] + off, d.qvel[6:18], action)
        obs += (rng.uniform(-1, 1, 45) * NOISE).astype(np.float32)  # training noise, as hw_risks
        action = pol(obs)
        goal = DEFAULT + ACTION_SCALE * action
        if link:
            link.realtime(d.time)
            link.send_pose(odo_pose(odo, R, spec, d.time))
        for k in range(steps_per_policy // CHUNK):
            if k * CHUNK == CMD_DELAY and not damping:  # the drives get the new targets 10 ms after the policy step
                d.ctrl[:12] = goal - off
            if damping:  # fall or ABORT: the drives go to damping (K = 0), like the BHL runtime's idle mode
                d.ctrl[:12] = 0
                m.actuator_gainprm[:12, 0] = 0
                m.actuator_biasprm[:12, 1] = 0
            # ST3215: goal speed limit, then its position loop (MuJoCo position actuator)
            dt = CHUNK * PHYSICS_DT
            pan_ref += float(np.clip(pan_goal - pan_ref, -SERVO_SPEED * dt, SERVO_SPEED * dt))
            tilt_ref += float(np.clip(tilt_goal - tilt_ref, -SERVO_SPEED * dt, SERVO_SPEED * dt))
            d.ctrl[12], d.ctrl[13] = np.clip(pan_ref, *world.PAN_LIM), np.clip(tilt_ref, *world.TILT_LIM)
            # jet: recoil on the tilt body at the nozzle, droplets, fire model
            d.qfrc_applied[:] = 0
            jet_dir = d.xmat[tiltb].reshape(3, 3)[:, 0]
            on = pump and water.left > 0
            if on:
                mujoco.mj_applyFT(m, d, -spec["recoil"] * jet_dir, np.zeros(3), d.site_xpos[nozzle], tiltb, d.qfrc_applied)
            hits = water.step(d.time, on, d.site_xpos[nozzle].copy(), jet_dir, fires, strengths)
            on_fire += float(hits.sum())
            strengths = world.fire_step(fires, strengths, hits, dt, spec)
            for i, s in enumerate(strengths):
                if s <= 0 and t_out[i] is None:
                    t_out[i] = round(d.time, 2)
            if fire_lights:
                m.light_diffuse[fire_lights] = light0 * np.array(strengths)[:, None] * (0.85 + 0.3 * rng.random((len(fires), 1)))
            frac = water.left / (spec["tank"] / 1000)
            m.body_mass[water_b], m.body_inertia[water_b] = max(m_water * frac, 1e-4), np.maximum(i_water * frac, 1e-8)
            mujoco.mj_step(m, d, nstep=CHUNK)

            if d.time >= next_aim - 1e-9:  # 10 Hz: robot camera, detector, aim/
                next_aim += AIM_DT
                R = d.xmat[base].reshape(3, 3)
                piv = d.xpos[base] + R @ np.array([0, 0, PIVOT_H])
                yaw = yaw_of(R)
                pose_true = (piv[0], piv[1], yaw)
                roll, pitch = math.atan2(R[2, 1], R[2, 2]), math.asin(-np.clip(R[2, 0], -1, 1))
                # odometry: integrate the true pivot motion with the scale error, in the drifted heading
                dxy = piv[:2] - last_piv
                last_piv = piv[:2].copy()
                bias = spec["gyro_bias"] * d.time
                c, s_ = math.cos(bias), math.sin(bias)
                odo += spec["odo_scale"] * np.array([c * dxy[0] - s_ * dxy[1], s_ * dxy[0] + c * dxy[1]])
                pose_odo = (odo[0], odo[1], yaw + bias, roll, pitch)
                pan_m = d.qpos[jpan] + spec["pan_zero"]  # ST3215 encoder reading: true angle + calibration error
                tilt_m = d.qpos[jtilt] + spec["tilt_zero"]
                robot_view.update_scene(d, camera="rgb", scene_option=ropt)
                for q in water.p[::2]:
                    add_sphere(robot_view.scene, q, 0.0015, (0.85, 0.92, 1.0, 0.35))
                rgb, depth = world.render_rgbd(robot_view)
                rgb, info = world.composite_fires(rgb, depth, d.cam_xpos[cam_id].copy(), d.cam_xmat[cam_id].reshape(3, 3).copy(),
                                                  world.K.fy, fires, strengths, d.time, sprites)
                img = rgb.astype(np.float32) * spec["exposure"] + noise_bank[int(d.time * 10) % 4]
                rgb = np.clip(img, 0, 255).astype(np.uint8)
                t_img = link.now() if link else d.time
                dets = oracle(info, t_img) if detect is None else detect(rgb, t_img)
                labels = match(dets, info)
                counts["frames"] += 1
                for det, lab in zip(dets, labels):  # localisation error of each true detection (aim/ geometry, true pose)
                    if lab >= 0:
                        counts["tp"] += 1
                        p = localise(CFG, det, pan_m, tilt_m, 0.0, (roll, pitch))
                        if p is not None:
                            pw = robot_to_world(pose_true, p)
                            f = fires[lab]
                            loc_samples.append((round(math.hypot(f["x"] - piv[0], f["y"] - piv[1]), 3),
                                                round(math.hypot(pw[0] - f["x"], pw[1] - f["y"]), 3), lab))
                    else:
                        counts["fp"] += 1
                        name = false_label(det, d, cam_id, dpoints)
                        false_dets[name] = false_dets.get(name, 0) + 1
                for i, (n, box) in enumerate(info):  # detection range: was each visible fire detected in this image?
                    if n >= 10:
                        f = fires[i]
                        det_samples.append((round(math.hypot(f["x"] - piv[0], f["y"] - piv[1]), 2), n, int(i in labels), round(strengths[i], 2), i))
                if link:
                    link.send(dets, t_img)
                    cmd = link.command()
                    pan_goal, tilt_goal = cmd.pan - spec["pan_zero"], cmd.tilt - spec["tilt_zero"]
                else:
                    cmd = ctl.step(d.time, pending, pose_odo, pan_m, tilt_m, frames=1)
                    pan_goal, tilt_goal = cmd.pan - spec["pan_zero"], cmd.tilt - spec["tilt_zero"]
                pending = dets
                pump = cmd.pump_on and not damping
                cmd_vec[:] = (np.clip(cmd.vx, -0.5, 0.5), 0.0, np.clip(cmd.yaw_rate, -1.0, 1.0))
                if damping:
                    cmd_vec[:] = 0
                if cmd.state == "ABORT" and abort is None:
                    abort, damping = round(d.time, 2), True
                if cmd.state == "DONE" and t_done is None:
                    t_done = round(d.time, 2)
                tgt = None  # where aim/ aims, in the world (its target in the robot frame, placed with the true pose)
                if not link and ctl.filter.x is not None:
                    tr = world_to_robot(pose_odo, ctl.filter.x)
                    tgt = robot_to_world(pose_true, tr)
                if pump:
                    err = min((math.hypot(tgt[0] - f["x"], tgt[1] - f["y"]) for f in fires), default=math.inf) if tgt is not None else None
                    sprays.append((round(d.time, 1), None if err is None else round(err, 3)))
                path.append((round(piv[0], 3), round(piv[1], 3), cmd.state))
                if keep_log:
                    log.append(dict(t=round(d.time, 2), state=cmd.state, x=round(piv[0], 3), y=round(piv[1], 3), yaw=round(yaw, 3),
                                    odo_x=round(odo[0], 3), odo_y=round(odo[1], 3), roll=round(roll, 3), pitch=round(pitch, 3),
                                    vx_cmd=round(cmd.vx, 3), yaw_rate_cmd=round(cmd.yaw_rate, 3), pan=round(pan_m, 3), tilt=round(tilt_m, 3),
                                    pan_cmd=round(cmd.pan, 3), tilt_cmd=round(cmd.tilt, 3), pump=pump, dets=len(dets),
                                    tp=sum(lab >= 0 for lab in labels), water_l=round(water.used * 1000, 4),
                                    strength=[round(s, 3) for s in strengths]))
                snap = snaps and any(abs(d.time - ts) < AIM_DT / 2 for ts in snaps[1])
                if vid or snap:  # the inset: robot camera with detection boxes (green: on a fire, red: false)
                    pic = rgb.copy()
                    for det, lab in zip(dets, labels):
                        draw_box(pic, det.bbox, (0, 230, 0) if lab >= 0 else (255, 40, 40), 3)
                    inset = np.asarray(Image.fromarray(pic).resize((256, 192), Image.BILINEAR))
                    if snap:
                        Image.fromarray(pic).save(Path(snaps[0]) / f"cam_{spec['seed']}_{d.time:06.1f}.png")

            if vid and d.time >= next_frame - 1e-9:
                next_frame += vid.speed / FPS
                tgt_w = None
                if cmd is not None and not link and ctl.filter.x is not None and cmd.state not in ("SEARCH", "DONE"):
                    tgt_w = robot_to_world(pose_true, world_to_robot(pose_odo, ctl.filter.x))
                st = cmd.state if cmd else "START"
                lines = [f"state: {st}", f"water used {water.used * 1e6:4.0f} mL of {spec['tank'] * 1e3:.0f} mL, pump {'ON' if pump else 'off'}"]
                lines += [f"fire {i + 1}: {'OUT at %.1f s' % t_out[i] if t_out[i] is not None else 'strength %3.0f %%' % (100 * s)}"
                          for i, s in enumerate(strengths)]
                vid.frame(m, d, base, tgt_w, water.p, fires, strengths, d.time, sprites, inset, st, lines)

        g = quat_rotate_inverse(d.sensor("imu_quat").data, np.array([0, 0, -1.0]))
        tilt_now = math.acos(np.clip(-g[2], -1, 1))
        if fall is None and (tilt_now > TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0):
            fall, damping = round(d.time, 2), True
        if (fall is not None and d.time > fall + 2.0) or (t_done is not None and d.time > t_done + 1.0) or \
                (abort is not None and d.time > abort + 3.0):
            break

    robot_view.close()
    if vid:
        vid.close()
    if link:
        link.close()
    used_l = water.used * 1000
    out_all = all(t is not None for t in t_out)
    false_spray = [s for s in sprays if s[1] is not None and s[1] > 0.5]
    ends = [t for t in t_out if t is not None]
    res = dict(seed=spec["seed"], outdoor=spec["outdoor"], n_fires=len(fires), fire_d=[round(f["d"], 3) for f in fires],
               fire_dist=[round(math.hypot(f["x"] - x0, f["y"] - y0), 2) for f in fires],
               success=bool(out_all and fall is None and abort is None), all_out=out_all, t_out=t_out,
               t_all_out=max(ends) if out_all else None, t_done=t_done, end_state=cmd.state if cmd else None,
               reason=ctl.reason if not link else "", fall=fall, ignored=_ignored(ctl, fires, pose_odo, pose_true) if not link else [], abort=abort, water_l=round(used_l, 4),
               water_on_fire_l=round(on_fire, 4), tank_empty=water.left <= 0, spray_s=round(0.1 * len(sprays), 1),
               false_spray_s=round(0.1 * len(false_spray), 1), false_spray_episodes=_episodes(false_spray),
               false_dets=false_dets, tp=counts["tp"], fp=counts["fp"], frames=counts["frames"], sim_time=round(d.time, 2),
               wall_s=round(time.time() - t_wall, 1), det_samples=det_samples, loc_samples=loc_samples, path=path[::5],
               spec=spec, detector=Path(str(detector)).name, policy=Path(policy).name, limit=limit, aim=aim or {})
    if keep_log:
        res["log"] = log
    if link:
        res["sil"] = link.stats
    return res


def _ignored(ctl, fires, pose_odo, pose_true):
    """For each target that aim/ gave up on (no response to water): distance from it to the nearest fire, m."""
    out = []
    for q in ctl.ignored:
        w = robot_to_world(pose_true, world_to_robot(pose_odo, q))
        out.append(round(min((math.hypot(w[0] - f["x"], w[1] - f["y"]) for f in fires), default=math.inf), 2))
    return out


def oracle(info, t):
    """Perfect detector: the true visible flame boxes (10 pixels or more), with the bottom centre as the fire point."""
    from detection import FireDetection
    return [FireDetection(t, "rgb", (b[0] + b[2]) / 2, b[3], tuple(float(x) for x in b), 0.95) for n, b in info if n >= 10]


def _episodes(samples, gap=0.35):
    """Number of separate stretches in a list of (t, ...) samples taken every 0.1 s."""
    ts = [s[0] for s in samples]
    return int(sum(1 for a, b in zip([-math.inf] + ts, ts) if b - a > gap))


def odo_pose(odo, R, spec, t):
    bias = spec["gyro_bias"] * t
    return (float(odo[0]), float(odo[1]), yaw_of(R) + bias, math.atan2(R[2, 1], R[2, 2]), math.asin(-np.clip(R[2, 0], -1, 1)))


# ---------------- software in the loop ----------------
class SilLink:
    """aim/run.py F2 --feedback and actuators/actuator_node.py --mock as separate processes, over the robot's UDP ports
    (robot/config.toml). The simulator replaces the camera, the legs and the servos, and runs
    in real time because the programs use time.monotonic(). The detector runs here, on each rendered image."""

    DET, WALK, FEEDBACK, POSE = 5600, 5602, 5604, 5605

    def __init__(self, spec, stem="sil"):
        import socket
        import threading
        from types import SimpleNamespace
        sys.path.insert(0, str(ROOT / "actuators"))
        from walk_bridge import AimGamepad
        self.ns, self.socket = SimpleNamespace, socket
        self.walk = AimGamepad(port=self.WALK)
        self.walk.start_udp()
        self.fb_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.fb_rx.bind(("127.0.0.1", self.FEEDBACK))
        self.fb_rx.setblocking(False)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        RUNS.mkdir(parents=True, exist_ok=True)
        # robot/config.toml for this test: concept F2, the B0201 field of view, the ST3215 speed (270 deg/s), and the jet speed
        # at this mission's battery voltage (the mock servos report no voltage, the real ones do: aim/run.py reads it)
        import re
        text = (ROOT / "robot" / "config.toml").read_text()
        text = re.sub(r'(?m)^concept = "\w+"', f'concept = "{spec.get("robot", "F2")}"', text)
        text = re.sub(r"(?m)^hfov_deg = [0-9.]+", f"hfov_deg = {math.degrees(2 * math.atan(world.W / 2 / world.K.fx)):.3f}", text)
        text = re.sub(r"(?m)^max_speed_dps = [0-9.]+", "max_speed_dps = 270", text)
        text = re.sub(r"(?m)^\[aim\.controller\].*$", f"[aim.controller]\nv0 = {world.jet_v0(spec):.4f}", text)
        conf = RUNS / f"{stem}_config.toml"
        conf.write_text(text)
        env = dict(os.environ, ROBOT_CONFIG=str(conf))
        self.logs = [open(RUNS / f"{stem}_node.log", "w"), open(RUNS / f"{stem}_aim_stderr.log", "w")]
        self.node = subprocess.Popen([sys.executable, "-u", "actuators/actuator_node.py", "--mock"],
                                     cwd=ROOT, stdout=subprocess.DEVNULL, stderr=self.logs[0], env=env)
        time.sleep(0.5)
        self.aim = subprocess.Popen([sys.executable, "-u", "aim/run.py", spec.get("robot", "F2"), "--feedback"], cwd=ROOT, stdout=subprocess.PIPE,
                                    stderr=self.logs[1], text=True, env=env)
        self.cmd, self.cmd_times, self.fb, self.sent, self.lag = {}, [], None, 0, 0.0
        threading.Thread(target=self._read, daemon=True).start()
        time.sleep(0.5)
        self.wall0 = time.monotonic()

    def _read(self):
        for line in self.aim.stdout:
            try:
                self.cmd = json.loads(line)
                self.cmd_times.append(time.monotonic())
            except ValueError:
                pass

    def now(self):
        return time.monotonic()

    def realtime(self, t_sim):
        lag = time.monotonic() - (self.wall0 + t_sim)
        if lag < 0:
            time.sleep(-lag)
        self.lag = max(self.lag, lag)

    def send(self, dets, t_img):
        msg = dict(timestamp=t_img, camera="rgb", frame=True, detections=[asdict(x) for x in dets])
        self.tx.sendto(json.dumps(msg, default=float).encode(), ("127.0.0.1", self.DET))
        self.sent += 1

    def send_pose(self, pose):
        x, y, yaw, roll, pitch = pose
        msg = dict(timestamp=time.monotonic(), x=x, y=y, yaw=yaw, roll=roll, pitch=pitch)
        self.tx.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.POSE))

    def command(self):
        while True:
            try:
                self.fb = json.loads(self.fb_rx.recv(65536))
            except BlockingIOError:
                break
        fb = self.fb or dict(pan=0.0, tilt=0.0, water=False)
        c = self.walk.commands
        return self.ns(state=self.cmd.get("state", "START"), pump_on=bool(fb["water"]), vx=c["velocity_x"], yaw_rate=c["velocity_yaw"],
                       pan=fb["pan"], tilt=fb["tilt"])

    def close(self):
        for p in (self.aim, self.node):
            if p.poll() is None:
                p.terminate()
            p.wait()
        self.walk.stop()
        time.sleep(0.1)
        self.walk.sock.close()
        self.fb_rx.close()
        for f in self.logs:
            f.close()
        gaps = np.diff(self.cmd_times) if len(self.cmd_times) > 1 else np.zeros(1)
        err = Path(self.logs[1].name).read_text()
        self.stats = dict(frames_sent=self.sent, commands=len(self.cmd_times), cmd_gap_max=round(float(gaps.max()), 3),
                          rejected=err.count("bad detection") + err.count("bad pose"), dead_reckoning=err.count("dead reckoning"),
                          sim_lag_max=round(self.lag, 3), last_state=self.cmd.get("state"))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--detector", default=str(BASELINE))
    ap.add_argument("--policy", default=str(POLICY))
    ap.add_argument("--video", help="mp4 path")
    ap.add_argument("--speed", type=float, default=1.0, help="video playback speed")
    ap.add_argument("--sil", action="store_true", help="aim/run.py and the actuator node as separate processes over UDP")
    ap.add_argument("--outdoor", action="store_true")
    ap.add_argument("--indoor", action="store_true")
    ap.add_argument("--json", help="write the result here")
    ap.add_argument("--design", default="f2", choices=list(world.DESIGNS), help="water system and fire model")
    ap.add_argument("--flames", default="procedural", choices=["procedural", "real"], help="flame sprites")
    ap.add_argument("--min-conf", type=float, help="aim/ min_conf override (default: aim/geometry.py F2, 0.3)")
    ap.add_argument("--limit", type=float, default=6.0, help="leg torque limit, Nm")
    ap.add_argument("--snaps", nargs="+", type=float, help="save the robot camera image at these times (s) in --snapdir")
    ap.add_argument("--snapdir", default=str(RUNS / "snaps"))
    a = ap.parse_args()
    spec = world.random_spec(a.seed, True if a.outdoor else False if a.indoor else None, a.design, a.flames)
    if a.snaps:
        Path(a.snapdir).mkdir(parents=True, exist_ok=True)
    r = run(spec, a.detector, a.policy, a.video, SilLink if a.sil else None, title=f"mission seed {a.seed}", video_speed=a.speed, limit=a.limit,
            snaps=(a.snapdir, a.snaps) if a.snaps else None, aim=None if a.min_conf is None else {"min_conf": a.min_conf})
    short = {k: v for k, v in r.items() if k not in ("log", "det_samples", "loc_samples", "path", "spec")}
    print(json.dumps(short, indent=1, default=str))
    if a.json:
        Path(a.json).write_text(json.dumps(r, default=str))


if __name__ == "__main__":
    main()
