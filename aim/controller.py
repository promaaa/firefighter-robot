"""SEARCH -> APPROACH -> AIM -> SPRAY -> CHECK -> (SEARCH or DONE).

Mission mode (ControllerConfig.verify_time > 0, the default of the F1 and F2 robot configs): SPRAY -> VERIFY instead of CHECK.
VERIFY watches the sprayed fire. It goes back to AIM when the fire shows again, and to SEARCH (next fire) when the fire
stays gone for verify_time. Any state goes to ABORT on a fall-size torso tilt, and to DONE at the mission time limit
or when the tank is empty.

Call Controller.step() once per camera frame. It returns a plain Command for the pan-tilt, the pump and the walking controller.
"""
import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from ballistics import solve_aim
from geometry import TargetFilter, localise, robot_to_world, world_to_robot

# Spray interlock on the torso attitude. The geometry compensates a steady tilt (SIL: about 5 deg nose-up while
# spraying), so only a large tilt or a fast change (a stumble or a fall) stops the spray, for SETTLE_TIME after it.
MAX_BODY_TILT = math.radians(15)  # rad, roll or pitch
MAX_TILT_RATE = math.radians(30)  # rad/s, from successive poses
SETTLE_TIME = 2.0  # s, also after start



@dataclass
class Command:
    pan: float  # rad, positive = left
    tilt: float  # rad, positive = up
    pump_on: bool
    vx: float  # m/s, forward walking speed
    yaw_rate: float  # rad/s, positive = turn left
    state: str


@dataclass
class ControllerConfig:
    v0: float = 8.0  # m/s, jet exit speed. Placeholder: measure it (aim/README.md, "Jet speed v0")
    drag: float = 0.0  # 1/m, quadratic drag coefficient, 0 = no drag
    fire_height: float = 0.0  # m, height of the fire point above the floor
    min_conf: float = 0.5  # detections below this confidence are ignored
    confirm_frames: int = 3  # valid detections (camera frames) within confirm_window before SEARCH -> APPROACH
    confirm_window: float = 0.6  # s, so late or missing frames do not reset the count
    gone_frames: int = 10  # frames in a row without a valid detection that end a spray
    max_age: float = 0.5  # s, no spray when the last valid detection is older
    lost_time: float = 3.0  # s, APPROACH and AIM go back to SEARCH after this time without a detection
    min_range: float = 0.7  # m, no spray closer than this (horizontal, from the robot origin)
    max_range: float = 4.0  # m, no spray farther than this
    standoff: float = 2.0  # m, APPROACH stops at this distance when the jet reaches the fire
    range_margin: float = 0.2  # m, hysteresis: after a step back, AIM only from min_range + this
    tilt_margin: float = math.radians(5)  # rad, hysteresis: AIM only when the aim tilt is this far above the lower limit
    aim_tol: float = math.radians(2)  # rad, pan and tilt error allowed to start and keep the spray
    heading_tol: float = math.radians(10)  # rad, walk at walk_speed within this heading error, at turn_vx outside it
    max_spray: float = 10.0  # s, length of one spray
    check_time: float = 3.0  # s, wait with the pump off after a spray, then search again
    max_sprays: int = 3  # sprays before DONE (tank size)
    search_time: float = 20.0  # s, DONE after a search this long finds nothing (a full circle takes about 16 s)
    max_body_tilt: float = MAX_BODY_TILT  # rad, no spray while the torso roll or pitch is larger
    max_tilt_rate: float = MAX_TILT_RATE  # rad/s, no spray while the roll or pitch changes faster
    settle_time: float = SETTLE_TIME  # s, no spray in the first seconds after start and after a tilt event
    search_tilt: float = math.radians(-6)  # rad, tilt while searching: with its pitch on the tilt plate, the camera looks down at the floor ahead
    search_yaw_rate: float = 0.4  # rad/s, turn while searching. _walk() adds turn_vx: the robot walks a small circle
    walk_speed: float = 0.25  # m/s. In our simulations, the BHL policy walked reliably only from 0.25 m/s
    turn_vx: float = 0.25  # m/s, smallest forward speed while turning: the BHL policy does not turn on the spot, and stands still below 0.25 m/s
    turn_eps: float = 0.05  # rad/s, a yaw rate below this is not a turn
    back_speed: float = 0.25  # m/s, step back without turning when the fire is too close. Same size as walk_speed: a slower command may not move the robot. Backward walking is not tested
    turn_gain: float = 1.5  # 1/s, yaw rate per radian of heading error
    max_yaw_rate: float = 0.6  # rad/s
    abort_tilt: float = 0.6  # rad, torso roll or pitch: the robot is falling. ABORT: pump off, stop, hold (the walking side damps)
    # Mission mode. The defaults switch it off, so concepts A, B and E behave as before.
    scan_pan: float = 0.0  # rad, pan amplitude of the search scan (sine), 0 = no scan
    scan_period: float = 7.0  # s, one scan cycle
    scan_still: float = 0.0  # s, stand still at the start of each search and scan, then walk the search circle
    verify_time: float = 0.0  # s, > 0: VERIFY instead of CHECK. The fire must stay out of sight this long
    overspray: float = 0.0  # s, the pump stays on this long after the last detection of a confirmed fire in SPRAY
    sweep: float = 0.0  # m, the spray sweeps the aim point this far before and behind the fire (along the line of sight)
    sweep_period: float = 2.0  # s
    tank_time: float = math.inf  # s of pump time in a full tank: DONE when it is used up
    mission_time: float = math.inf  # s, DONE after this time from the first step
    respond_time: float = math.inf  # s of pump time on one target, then its detection box must have shrunk (else: ignore it)
    respond_ratio: float = 0.9  # largest box area ratio (last 3 s / first 3 s of spraying) that counts as a response
    ignore_radius: float = 0.6  # m, detections this close to an ignored target are dropped


def _clip(x, lim):
    return min(max(x, lim[0]), lim[1])


@dataclass
class Controller:
    robot: dict  # one entry of geometry.CONCEPTS
    cc: ControllerConfig = None  # None: ControllerConfig with the robot's "controller" values (geometry.CONCEPTS)
    state: str = "SEARCH"
    filter: TargetFilter = field(default_factory=TargetFilter)
    seen: deque = field(default_factory=lambda: deque(maxlen=20))  # image times of the accepted detections
    missed: int = 0  # frames (or ticks, with the old detector format) in a row without a valid detection
    last_seen: float = -math.inf
    t_state: float | None = None  # time of the last state change, set at the first step
    spray_start: float = 0.0
    sprays: int = 0
    aim: object = None  # last ballistics.Aim, for logging
    history: deque = field(default_factory=lambda: deque(maxlen=50))  # (t, x, y, yaw, roll, pitch, pan, tilt)
    t_start: float = None  # time of the first step
    t_unsteady: float = -math.inf  # last time of a tilt event (too large or too fast)
    last_att: tuple = None  # (t, roll, pitch) of the previous step
    pump_time: float = 0.0  # s, total pump-on time (tank use)
    out: int = 0  # fires that stayed out for verify_time
    reason: str = ""  # why the mission ended (DONE or ABORT)
    last_t: float = None  # time and pump state of the previous step
    last_pump: bool = False
    ignored: list = field(default_factory=list)  # odometry-frame positions of targets that did not respond to water
    areas: list = field(default_factory=list)  # (image time, box area) of the tracked target while spraying
    target_pump: float = 0.0  # s of pump time on the current target
    checked: bool = False  # the response check of the current target is done

    def __post_init__(self):
        if self.cc is None:
            self.cc = ControllerConfig(**self.robot.get("controller", {}))
        else:  # a config from the caller (aim/run.py: robot/config.toml overrides). The robot's values fill the fields it left
            base = ControllerConfig()  # at the default. Limit: an override equal to the default is not seen as an override
            for k, v in self.robot.get("controller", {}).items():
                if getattr(self.cc, k) == getattr(base, k):
                    setattr(self.cc, k, v)

    def _at(self, ts):
        """Pose and pan-tilt angles at time ts, interpolated in the history (clamped at its ends)."""
        h = np.array(self.history)
        h[:, 3] = np.unwrap(h[:, 3])
        return [float(np.interp(ts, h[:, 0], h[:, k])) for k in range(1, 8)]

    def _walk(self, vx, yaw_rate):
        """Walk command the BHL policy can follow: no turn on the spot, no turn while stepping back."""
        if vx < 0:
            return vx, 0.0
        if abs(yaw_rate) > self.cc.turn_eps:
            return max(vx, self.cc.turn_vx), yaw_rate
        return vx, 0.0 if vx == 0 else yaw_rate

    hold: tuple = None  # last commanded (pan, tilt): CHECK and DONE hold it, never the measured angles

    def _go(self, state, now):
        self.state, self.t_state = state, now
        if state == "SEARCH":
            self.filter = TargetFilter()
            self.seen.clear()
        # from AIM to CHECK (VERIFY) the filter keeps its fire: another fire cannot take over the target
        self.filter.reset_after = math.inf if state in ("AIM", "SPRAY", "CHECK", "VERIFY") else TargetFilter.reset_after

    def confirmed(self, now):
        return sum(now - t <= self.cc.confirm_window for t in self.seen) >= self.cc.confirm_frames

    def step(self, now, detections, pose, pan, tilt, t_angles=None, frames=None):
        if self.t_state is None:
            self.t_state, self.t_start, self.hold = now, now, (0.0, self.cc.search_tilt)
        if self.last_pump and self.last_t is not None:
            self.pump_time += now - self.last_t
            self.target_pump += now - self.last_t
        cmd = self._step(now, detections, pose, pan, tilt, t_angles, frames)
        self.hold = (cmd.pan, cmd.tilt)
        self.last_t, self.last_pump = now, cmd.pump_on
        return cmd

    def _end(self, now, reason):
        self.reason = reason
        self._go("DONE", now)

    def _step(self, now, detections, pose, pan, tilt, t_angles, frames):
        """frames: camera frames received since the last step (interface v2), or None when unknown (old format)."""
        """now: s. detections: list of FireDetection for this frame (can be empty).
        pose: (x, y, yaw) from odometry, or (0, 0, 0) without odometry. pan, tilt: actuator angles, rad.
        t_angles: time when pan and tilt were measured (default now)."""
        cc, cfg = self.cc, self.robot
        t = now if t_angles is None else min(t_angles, now)
        if not self.history or t > self.history[-1][0]:  # the interpolation needs increasing times
            self.history.append((t, *pose[:3], *(pose[3:5] if len(pose) >= 5 else (0.0, 0.0)), pan, tilt))
        cands = []  # (detection, fire position in the odometry frame)
        for d in detections:
            if d.confidence >= cc.min_conf and 0 <= now - d.timestamp <= cc.lost_time:
                x, y, yaw, roll, pitch, pan_img, tilt_img = self._at(d.timestamp)  # pose and angles at the image time
                p = localise(cfg, d, pan_img, tilt_img, cc.fire_height, (roll, pitch))
                if p is not None:
                    w = robot_to_world((x, y, yaw), p)
                    if all(np.linalg.norm(w[:2] - q[:2]) > cc.ignore_radius for q in self.ignored):
                        cands.append((d, w))
        det = w = None
        if cands and self.filter.x is not None:  # several fires: keep the one we track
            det, w = min(cands, key=lambda c: np.linalg.norm(c[1] - self.filter.x))
        elif cands:
            det, w = max(cands, key=lambda c: c[0].confidence)
        if det is not None and self.filter.update(w):
            self.seen.append(det.timestamp)
            self.missed, self.last_seen = 0, det.timestamp
            if self.state == "SPRAY":
                self.areas.append((det.timestamp, (det.bbox[2] - det.bbox[0]) * (det.bbox[3] - det.bbox[1])))
        else:  # with frame messages, a late frame is not a miss. Without them, each step without a fire is one
            self.missed += 1 if frames is None else frames

        age = now - self.last_seen
        target = world_to_robot(pose, self.filter.x) if self.filter.x is not None else None
        att = tuple(pose[3:5]) if len(pose) >= 5 else (0.0, 0.0)  # torso roll and pitch now
        if self.state == "ABORT" or max(abs(att[0]), abs(att[1])) > cc.abort_tilt:
            if self.state != "ABORT":
                self.reason = "torso tilt %.0f deg" % math.degrees(max(abs(att[0]), abs(att[1])))
                self._go("ABORT", now)
            return Command(*self.hold, False, 0.0, 0.0, "ABORT")
        if self.state != "DONE" and now - self.t_start >= cc.mission_time:
            self._end(now, "time limit")
        if self.state != "DONE" and self.pump_time >= cc.tank_time:
            self._end(now, "tank empty")
        rate = 0.0
        if self.last_att and now > self.last_att[0]:
            rate = max(abs(att[0] - self.last_att[1]), abs(att[1] - self.last_att[2])) / (now - self.last_att[0])
        self.last_att = (now, *att)
        if max(abs(att[0]), abs(att[1])) > cc.max_body_tilt or rate > cc.max_tilt_rate:
            self.t_unsteady = now
        steady = now - self.t_start >= cc.settle_time and now - self.t_unsteady >= cc.settle_time
        dist = math.hypot(target[0], target[1]) if target is not None else math.inf
        if target is not None and cc.sweep and self.state == "SPRAY":  # sweep the base of the fire along the line of sight
            k = 1 + cc.sweep / max(dist, 0.1) * math.sin(2 * math.pi * (now - self.spray_start) / cc.sweep_period)
            target = np.array([target[0] * k, target[1] * k, target[2]])
        aim = self.aim = solve_aim(cfg, target, cc.v0, cc.drag, att=att) if target is not None else None
        heading = math.atan2(target[1], target[0]) if target is not None else 0.0
        in_range = aim is not None and aim.in_range and cc.min_range <= dist <= cc.max_range
        on_target = aim is not None and abs(pan - aim.pan) < cc.aim_tol and abs(tilt - aim.tilt) < cc.aim_tol
        fresh = age <= cc.max_age + (cc.overspray if self.state == "SPRAY" else 0.0)
        safe = in_range and on_target and fresh and steady

        s = self.state
        if s == "SEARCH":
            if self.confirmed(now):  # a new target
                self.areas, self.target_pump, self.checked = [], 0.0, False
                self._go("APPROACH", now)
            elif now - self.t_state >= cc.search_time:
                self._end(now, "search found no fire")
            ts = now - self.t_state
            pan = cc.scan_pan * math.sin(2 * math.pi * ts / cc.scan_period) if cc.scan_pan else 0.0
            walk = self._walk(0.0, cc.search_yaw_rate) if ts >= cc.scan_still else (0.0, 0.0)
            return Command(pan, cc.search_tilt, False, *walk, s)

        if s in ("APPROACH", "AIM") and age > cc.lost_time:
            self._go("SEARCH", now)
            return Command(0.0, cc.search_tilt, False, 0.0, 0.0, s)

        if s == "APPROACH":
            # hysteresis: step back past the limits by a margin, so AIM and SPRAY do not stop for small range changes
            close = dist < cc.min_range + cc.range_margin or aim.reason == "too close" or \
                (aim.in_range and aim.tilt < cfg["tilt_limits"][0] + cc.tilt_margin)
            if close:
                vx, yaw_rate = -cc.back_speed, 0.0
            elif in_range and dist <= cc.standoff:  # off-centre is fine: the pan-tilt turns, not the body
                vx, yaw_rate = 0.0, 0.0
                self._go("AIM", now)
            else:  # too far, or outside the pan limits: walk an arc toward the fire
                yaw_rate = _clip(cc.turn_gain * heading, (-cc.max_yaw_rate, cc.max_yaw_rate))
                vx = cc.walk_speed if abs(heading) <= cc.heading_tol else cc.turn_vx
            vx, yaw_rate = self._walk(vx, yaw_rate)
            # look at the fire so a camera on the pan-tilt keeps it in view, centred when it has a pitch on the part
            look = math.atan2(target[2] - cfg["pivot"][2], dist)
            cam = next(iter(cfg["cameras"].values()))
            if cam["mount"] == "tilt":
                look -= cam["pitch"]
            return Command(_clip(heading, cfg["pan_limits"]), _clip(look, cfg["tilt_limits"]), False, vx, yaw_rate, s)

        if s == "AIM":
            if not in_range:
                self._go("APPROACH", now)
            elif safe:
                self._go("SPRAY", now)
                self.spray_start, self.sprays = now, self.sprays + 1
            return Command(aim.pan, aim.tilt, False, 0.0, 0.0, s)

        if s == "SPRAY" and not self.checked and self.target_pump >= cc.respond_time:
            # a fire shrinks under water; a red object or a lamp does not. Compare the box area at the start and now.
            self.checked = True
            first = [a for t_, a in self.areas if t_ <= self.areas[0][0] + 3.0] if self.areas else []
            last = [a for t_, a in self.areas if t_ >= now - 3.0]
            if len(first) >= 5 and len(last) >= 5 and np.median(last) > cc.respond_ratio * np.median(first):
                self.ignored.append(self.filter.x.copy())
                self._go("SEARCH", now)
                return Command(*self.hold, False, 0.0, 0.0, s)

        if s == "SPRAY":
            if self.missed >= cc.gone_frames or now - self.spray_start >= cc.max_spray:
                self._go("VERIFY" if cc.verify_time else "CHECK", now)
                return Command(aim.pan, aim.tilt, False, 0.0, 0.0, s)
            if not in_range:
                self._go("APPROACH", now)
            return Command(aim.pan, aim.tilt, safe and self.state == "SPRAY", 0.0, 0.0, s)

        if s == "VERIFY":  # watch the sprayed fire: spray again when it shows, next fire when it stays out
            if self.confirmed(now):
                self._go("AIM", now)
            elif now - max(self.last_seen, self.t_state) >= cc.verify_time:
                self.out += 1
                self._go("DONE" if self.sprays >= cc.max_sprays else "SEARCH", now)
                self.reason = "max sprays" if self.state == "DONE" else ""
            return Command(*self.hold, False, 0.0, 0.0, s)

        if s == "CHECK":  # the same fire again, or gone: search again (other fires), unless the tank is empty
            if self.confirmed(now) or now - self.t_state >= cc.check_time:
                self._go("DONE" if self.sprays >= cc.max_sprays else "SEARCH", now)
            return Command(*self.hold, False, 0.0, 0.0, s)

        return Command(*self.hold, False, 0.0, 0.0, s)  # DONE
