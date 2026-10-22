"""Camera model, pan-tilt kinematics and fire localisation.

Robot frame: x forward, y left, z up, metres, origin = floor point under the BHL base link origin (pelvis centre).
Pan: rotation about z, positive = left. Tilt: elevation, positive = up.
Camera optical frame (OpenCV convention): z forward, x right, y down.
"""
import math
from dataclasses import dataclass

import numpy as np

T = 0.791  # m, top of the Berkeley Humanoid Lite torso frame, in the CAD robot frame


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


def intrinsics_from_fov(width, height, hfov_deg):
    """Pinhole intrinsics from the image size and the horizontal field of view. Square pixels, centred."""
    f = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    return Intrinsics(f, f, width / 2, height / 2, width, height)


# Geometry from the CAD unless noted (cad/final for F2). Positions in metres. "pivot" is the tilt pivot, on the pan axis.
# Camera "mount": "tilt" = moves with pan and tilt (pos in the tilting frame), "body" = fixed (pos in the robot frame).
# Camera "pitch"/"yaw": extra rotation of the camera in its mount, radians. Bore-sight calibration sets them.
# Intrinsics are placeholders (RGB 640 x 480, 90 deg; thermal 160 x 120, 56 deg): measure the real cameras.
# Camera poses of the earlier designs A, B and E come from their CAD studies (cad/parts and older studies).
CONCEPTS = {
    "A": dict(  # water gun on a 2-servo pan-tilt. Source: cad/parts/out/assembly.json and cad/parts/README.md (DS3218 version)
        pivot=(-0.038, 0.0, T + 0.0723),  # pan axis x = -38 mm, tilt axis 72.3 mm above the torso top (63.0 mm with STS3215)
        nozzle=(0.248, 0.0, 0.084),  # ESTIMATE: CAD gun box (centre 22 mm behind the axis, 78.5 mm above it) + the CAD barrel
        pan_limits=(-math.radians(90), math.radians(90)),
        tilt_limits=(-math.radians(40), math.radians(45)),
        # USB camera 57 mm right of the barrel line (CAM_Y), 59 mm above the axis, 16 deg below the barrel (lens about x 0.07, z 0.91 at -6 deg)
        cameras={"rgb": dict(mount="tilt", pos=(0.103, -0.057, 0.059), pitch=-math.radians(16), yaw=0.0, K=intrinsics_from_fov(640, 480, 90))},
    ),
    "B": dict(  # head with cameras and nozzle on a pan-tilt
        pivot=(0.0, 0.0, T + 0.075),
        nozzle=(0.145, 0.0, -0.050),  # centred, 62 mm under the cameras (cad/parts/README.md, head_b)
        pan_limits=(-math.radians(90), math.radians(90)),
        tilt_limits=(-math.radians(40), math.radians(45)),
        cameras={
            # head_b: RGB 21 mm right (-y), thermal 21 mm left (+y), both 15 deg down (cad/parts/README.md)
            "rgb": dict(mount="tilt", pos=(0.059, -0.021, 0.012), pitch=-math.radians(15), yaw=0.0, K=intrinsics_from_fov(640, 480, 90)),
            "thermal": dict(mount="tilt", pos=(0.059, 0.021, 0.012), pitch=-math.radians(15), yaw=0.0, K=intrinsics_from_fov(160, 120, 56)),
        },
    ),
    "F1": dict(  # final design F1 (cad/final/README.md): nozzle and Arducam B0201 camera on an ST3215 pan-tilt (cad/parts/out_sts3215)
        pivot=(-0.038, 0.0, T + 0.0630),  # assembly.json: pan axis x = -38 mm, tilt axis 63.0 mm above the torso top
        nozzle=(0.100, 0.0, 0.040),  # ESTIMATE: nozzle tip at the front of the tilt plate (plate top 30 mm above the axis), 10 mm above it
        pan_limits=(-math.radians(90), math.radians(90)),
        tilt_limits=(-math.radians(40), math.radians(45)),
        # ESTIMATE: B0201 lens 35 mm right of the nozzle line, 26 mm above the plate (CAM_RISE + 3 mm for the 38 mm board), 16 deg down,
        # 12 mm in front of the nozzle tip, so the nozzle and the plate stay out of the image.
        # 120 deg diagonal over the 16:9 IMX291 sensor. 640 x 480 is a centre 4:3 crop of it -> 97.1 deg horizontal, f = 283 px
        # (vision/README.md). Pinhole model: the real M12 lens has barrel distortion. Measure it with vision/calibrate.py.
        cameras={"rgb": dict(mount="tilt", pos=(0.112, -0.035, 0.056), pitch=-math.radians(16), yaw=0.0,
                             K=intrinsics_from_fov(640, 480, 97.1))},
        # Controller defaults for F1 (aim/controller.py ControllerConfig). Jet: gear pump on the 3S pack, 1.0 mm nozzle:
        # 0.40 L/min, v0 = 9.5 m/s at 11.1 V, drag 0.11 1/m (model, not measured). v0 follows the 3S voltage:
        # v0 = 9.5 (V / 11.1)^0.73 m/s. aim/run.py sets cc.v0 from the servo voltage reading (F1 only).
        # Tank: 0.571 L usable / 0.40 L/min = 86 s of pump time. max_spray 11 s: the pump node stops the pump after 12 s on
        # (robot/config.toml max_spray_s), so aim/ ends each spray first and aims again (about 0.2 s without water).
        # min_conf 0.3: the trained detector's threshold is 0.28. With real-fire flames it doubles the fires put out against 0.5
        # (12 -> 25 of 102, 1 false-spray mission of 50, sim/mission/README.md). The 3-frame confirmation filters single frames.
        controller=dict(v0=9.5, drag=0.11, min_conf=0.3, walk_speed=0.3, turn_vx=0.3, scan_pan=1.4, scan_period=7.0, scan_still=7.0,
                        verify_time=5.0, overspray=4.0, gone_frames=50, sweep=0.08, max_spray=11.0, max_sprays=30,
                        tank_time=0.571 / (0.40 / 60), search_time=30.0, mission_time=240.0, respond_time=15.0),
    ),
}
# Final design F2 (cad/final/README.md): F1 with two R2 tanks on the torso posts. Same pan-tilt, camera and nozzle.
# Pump: Kamoer KLP02 24 V with the DMC-101 controller, on the motor bus. Jet model (docs/hardware/water.md): 1.0 mm nozzle,
# 0.42 L/min, v0 = 10.0 m/s at 22.2 V (10.6 m/s at 24 V), drag 0.11 1/m. Not measured: the jet test (robot/bench.py pump)
# gives the real v0 for robot/config.toml [aim.controller]. Tanks: 0.749 L usable of 0.781 L / 0.42 L/min = 107 s of pump time.
CONCEPTS["F2"] = dict(CONCEPTS["F1"], controller=dict(CONCEPTS["F1"]["controller"], v0=10.0, tank_time=0.749 / (0.42 / 60)))
CONCEPTS |= {
    "E": dict(  # nozzle and valve on a pan-tilt, cameras on a fixed mast (pod 6 deg down, RGB 14 deg more)
        pivot=(0.0, 0.0, T + 0.095),
        nozzle=(0.165, 0.0, 0.0),
        pan_limits=(-math.radians(90), math.radians(90)),
        tilt_limits=(-math.radians(40), math.radians(45)),
        cameras={
            "rgb": dict(mount="body", pos=(-0.015, 0.128, 0.962), pitch=-math.radians(20), yaw=0.0, K=intrinsics_from_fov(640, 480, 90)),
            "thermal": dict(mount="body", pos=(-0.019, 0.052, T + 0.172), pitch=-math.radians(6), yaw=0.0, K=intrinsics_from_fov(160, 120, 56)),
        },
    ),
}

# Columns: optical x (right), y (down), z (forward) expressed in a frame with x forward, y left, z up.
OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def rot_pan_tilt(pan, tilt):
    """Rotation that turns +x into the direction (pan, tilt)."""
    cp, sp, ct, st = math.cos(pan), math.sin(pan), math.cos(tilt), math.sin(tilt)
    rz = np.array([[cp, -sp, 0], [sp, cp, 0], [0, 0, 1]])
    ry = np.array([[ct, 0, -st], [0, 1, 0], [st, 0, ct]])  # rotation about y by -tilt
    return rz @ ry


BASE_Z = 0.64  # m, approximate height of the BHL base link (pelvis) when standing: the torso tilts about it

LEVEL = (0.0, 0.0)


def tilted(att, r, t):
    """Apply the torso attitude att = (roll, pitch) to a rotation r and a point t of the robot frame.

    Base rotation = Rz(yaw) Ry(pitch) Rx(roll), so here Ry(pitch) Rx(roll) about the base link (0, 0, BASE_Z).
    The result is in the level robot frame: x forward along the heading, z up.
    """
    roll, pitch = att
    if roll == 0 and pitch == 0:
        return r, t
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    a = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    b = np.array([0.0, 0.0, BASE_Z])
    return a @ r, b + a @ (t - b)


def nozzle_exit(cfg, pan, tilt, att=LEVEL):
    """Nozzle exit point and unit jet direction in the (level) robot frame."""
    r, e = tilted(att, rot_pan_tilt(pan, tilt), np.array(cfg["pivot"]) + rot_pan_tilt(pan, tilt) @ np.array(cfg["nozzle"]))
    return e, r[:, 0]


def camera_pose(cfg, cam, pan, tilt, att=LEVEL):
    """(R, t): R maps optical-frame vectors to the robot frame, t is the camera centre in the robot frame."""
    c = cfg["cameras"][cam]
    local = rot_pan_tilt(c["yaw"], c["pitch"]) @ OPTICAL
    if c["mount"] == "tilt":
        r = rot_pan_tilt(pan, tilt)
        return tilted(att, r @ local, np.array(cfg["pivot"]) + r @ np.array(c["pos"]))
    return tilted(att, local, np.array(c["pos"], dtype=float))


def project(cfg, cam, pan, tilt, p, att=LEVEL):
    """Robot-frame point -> pixel (u, v), or None when the point is behind the camera."""
    r, t = camera_pose(cfg, cam, pan, tilt, att)
    x, y, z = r.T @ (np.asarray(p, dtype=float) - t)
    if z <= 1e-6:
        return None
    k = cfg["cameras"][cam]["K"]
    return k.fx * x / z + k.cx, k.fy * y / z + k.cy


def pixel_ray(cfg, cam, pan, tilt, u, v, att=LEVEL):
    """Camera centre and ray direction (robot frame) through pixel (u, v). The ray has optical z = 1."""
    r, t = camera_pose(cfg, cam, pan, tilt, att)
    k = cfg["cameras"][cam]["K"]
    return t, r @ np.array([(u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0])


def localise(cfg, det, pan, tilt, fire_height=0.0, att=LEVEL):
    """Fire position in the robot frame from one detection, or None.

    Uses det.depth_m when given. Otherwise intersects the pixel ray with the plane z = fire_height.
    pan, tilt: actuator angles when the image was taken. att: torso (roll, pitch) at that time.
    """
    o, d = pixel_ray(cfg, det.camera, pan, tilt, det.u, det.v, att)
    if det.depth_m is not None:
        return o + det.depth_m * d
    if d[2] >= -1e-6:  # ray parallel to the floor or pointing up
        return None
    s = (fire_height - o[2]) / d[2]
    return o + s * d if s > 0 else None


def robot_to_world(pose, p):
    """pose = (x, y, yaw, ...) of the robot in the odometry frame."""
    x, y, yaw = pose[:3]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x + c * p[0] - s * p[1], y + s * p[0] + c * p[1], p[2]])


def world_to_robot(pose, p):
    x, y, yaw = pose[:3]
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - x, p[1] - y
    return np.array([c * dx + s * dy, -s * dx + c * dy, p[2]])


@dataclass
class TargetFilter:
    """Constant-position Kalman filter, same variance on each axis, with outlier rejection.

    Limit: one scalar variance for all axes; use a range-dependent 3x3 covariance if far fires jitter too much.
    """
    q: float = 0.02 ** 2  # m^2, process noise added at each update (fire drifts, robot odometry drifts)
    r: float = 0.15 ** 2  # m^2, measurement noise
    gate: float = 3.0  # reject a measurement farther than gate * sigma from the estimate
    reset_after: int = 5  # accept the new position after this many rejections in a row
    x: np.ndarray | None = None
    p: float = 0.0
    rejected: int = 0

    def update(self, z):
        """Returns True when the measurement was used."""
        z = np.asarray(z, dtype=float)
        if self.x is None or self.rejected >= self.reset_after:
            self.x, self.p, self.rejected = z, self.r, 0
            return True
        self.p += self.q
        if np.linalg.norm(z - self.x) > self.gate * math.sqrt(self.p + self.r):
            self.rejected += 1
            return False
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (z - self.x)
        self.p *= 1 - k
        self.rejected = 0
        return True
