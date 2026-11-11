"""The 12 leg joints on XDrive MINI drives: joint map, units, gains, zeroing and the drive modes.

Units: joint angles in rad at the joint. Drive positions in motor turns. N = 15 motor turns per joint turn.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

import odrive_can as oc

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT))
sys.path.append(str(ROOT / "robot"))
from source.robot_models.robots import asmr2026_policy as policy_model  # noqa: E402
from robot_config import CONF  # noqa: E402  robot/config.toml

N = 15.0                         # ASMR course gear ratio, defined positive at the joint-side model
TURNS_PER_RAD = N / (2 * np.pi)  # motor turns per joint radian
KP, KD = policy_model.KP, policy_model.KD  # per-joint gains from the ASMR MJCF position actuators
TORQUE_LIMIT = 6.0               # Nm at the joint: conservative bring-up limit; raise only after hardware checks
TORQUE_CONSTANT = {"6512": 0.067, "5010": 0.0836}  # Nm/A at the motor, course xdrive_config.txt
LIMIT_MARGIN = 0.2               # rad. A joint this far beyond a limit has a wrong zero (a wrong zero is 0.42 rad).
RATE = 25.0                      # Hz, the policy rate. All host loops use it.
CHANNELS = tuple(CONF["can"]["channels"])  # robot/config.toml. robot/setup names the adapters by USB serial number.
CALIB = Path(__file__).with_name("calibration.json")

# Model-coordinate conversion. This is separate from `sign`, which maps encoder turns
# to the existing low-level joint coordinates. The policy sees ASMR coordinates.
BHL_TO_ASMR = policy_model.BHL_TO_ASMR.astype(float)


def to_asmr(q):
    """Convert joint positions or velocities from the low-level/BHL convention to ASMR coordinates."""
    return policy_model.to_asmr(q)


def to_hardware_coordinates(q):
    """Convert ASMR joint positions or velocities to the convention used by the drive map."""
    return policy_model.to_drive_coordinates(q)


# The encoder signs remain the existing low-level map. The ASMR/BHL model-axis
# conversion is applied at the policy boundary, not folded into motor calibration.
_DRIVES = [
    # joint name        node motor  encoder sign  calibration stop
    ("left_hip_roll",     1, "6512", +1, "lo"),
    ("left_hip_yaw",      3, "6512", -1, "hi"),
    ("left_hip_pitch",    5, "6512", +1, "hi"),
    ("left_knee_pitch",   7, "6512", +1, "lo"),
    ("left_ankle_pitch", 11, "5010", +1, "lo"),
    ("left_ankle_roll",  13, "5010", -1, "lo"),
    ("right_hip_roll",    2, "6512", +1, "hi"),
    ("right_hip_yaw",     4, "6512", -1, "lo"),
    ("right_hip_pitch",   6, "6512", -1, "hi"),
    ("right_knee_pitch",  8, "6512", -1, "lo"),
    ("right_ankle_pitch", 12, "5010", -1, "lo"),
    ("right_ankle_roll", 14, "5010", -1, "hi"),
]

_HARDWARE_LIMITS = policy_model.JOINT_LIMITS.copy()
_flipped_axes = BHL_TO_ASMR < 0
_HARDWARE_LIMITS[_flipped_axes] = -policy_model.JOINT_LIMITS[_flipped_axes, ::-1]
JOINTS = [
    (name, node, motor, sign, *_HARDWARE_LIMITS[i], stop)
    for i, (name, node, motor, sign, stop) in enumerate(_DRIVES)
]
NAMES = [j[0] for j in JOINTS]
assert [f"leg_{n}_joint" for n in NAMES] == list(policy_model.LEG_JOINTS), "JOINTS must be in the policy order"
LO, HI = np.array([j[4] for j in JOINTS]), np.array([j[5] for j in JOINTS])
STOP = np.where([j[6] == "lo" for j in JOINTS], LO, HI)  # calibration stop angles (rad)
POLICY_DEFAULT = policy_model.DEFAULT.astype(float)
DEFAULT = to_hardware_coordinates(POLICY_DEFAULT)  # stand pose in the drive-map convention
ZERO_POSE = np.zeros(12)  # zeroing pose: legs hang straight down, knees straight, soles flat, toes forward


def joint_gains(pos_gain, vel_gain):
    """Joint stiffness K (Nm/rad) and damping D (Nm*s/rad) of the drive cascade (controller.cpp:245,304):
    vel_des = vel_ff + pos_gain*(pos_set - pos), torque = torque_ff + vel_gain*(vel_des - vel),
    in motor turns and Nm at the motor. At the joint this gives K = N^2/(2 pi)*pos_gain*vel_gain, D = N^2/(2 pi)*vel_gain."""
    D = N * TURNS_PER_RAD * vel_gain
    return D * pos_gain, D


def drive_gains(K, D):
    """pos_gain ((turn/s)/turn) and vel_gain (Nm/(turn/s)) that give the joint gains K and D."""
    return K / D, D / (N * TURNS_PER_RAD)


def joint_torque_limit(current_lim, torque_constant):
    return N * current_lim * torque_constant


def current_limit(joint_torque, torque_constant):
    return joint_torque / (N * torque_constant)


def drive_settings(motor, torque_limit=TORQUE_LIMIT):
    """Drive settings using the ASMR gains for this motor group and the requested joint torque limit."""
    i = next(i for i, joint in enumerate(JOINTS) if joint[2] == motor)
    pos_gain, vel_gain = drive_gains(KP[i], KD[i])
    return dict(pos_gain=pos_gain, vel_gain=vel_gain, current_lim=current_limit(torque_limit, TORQUE_CONSTANT[motor]))


def zero_offset(p, enc_ref, ref_pose, sign):
    """Motor turns at joint angle 0, from readings p taken with each joint within +-12 deg of ref_pose.
    enc_ref is the encoder angle (turns, mod 1) at ref_pose. The encoder knows the angle in one motor turn
    only, which is 24 deg at the joint. So the code takes the motor turn that puts the joint closest to ref_pose."""
    d = (p - enc_ref + 0.5) % 1.0 - 0.5  # motor turns away from ref_pose, in [-0.5, 0.5)
    return p - d - sign * ref_pose * TURNS_PER_RAD


def enc_at_zero_pose(p, q_known, sign):
    """Encoder angle (turns, mod 1) at ZERO_POSE, from readings p taken at known joint angles q_known."""
    return (p + sign * (ZERO_POSE - q_known) * TURNS_PER_RAD) % 1.0


def wrong_zero(q_at_stop):
    """True for each joint that reads more than LIMIT_MARGIN away from its calibration stop, at that stop.
    A wrong zero is off by one motor turn: 24 deg = 0.42 rad."""
    return np.abs(q_at_stop - STOP) > LIMIT_MARGIN


def ticks(hz):
    """Yield 1, 2, 3, ... at a fixed rate."""
    t, k = time.monotonic(), 0
    while True:
        k += 1
        t += 1.0 / hz
        time.sleep(max(0.0, t - time.monotonic()))
        yield k


def open_buses(fake=False):
    """The two CAN buses, real (SocketCAN) or fake. Returns (buses, FakeRobot or None)."""
    if not fake:
        return [oc.Bus(c) for c in CHANNELS], None
    from fake_bus import FakeRobot
    sim = FakeRobot()
    return sim.buses(), sim


class Robot:
    def __init__(self, buses, calib=CALIB):
        self.calib = calib
        self.node = [j[1] for j in JOINTS]
        self.sign = np.array([j[3] for j in JOINTS], float)
        self.lo, self.hi = LO.copy(), HI.copy()
        self.p, self.v = np.zeros(12), np.zeros(12)  # last motor turns and turns/s
        self.miss = np.zeros(12, int)               # encoder requests without a reply, in a row
        self.enabled = np.zeros(12, bool)
        self.offset = None                          # motor turns at joint angle 0, from zero()
        self.enc_ref, self.ref_pose = np.full(12, np.nan), ZERO_POSE.copy()
        if calib and Path(calib).exists():
            c = json.loads(Path(calib).read_text())
            self.enc_ref = np.array([c[n]["enc_ref"] if n in c else np.nan for n in NAMES])
            self.ref_pose = np.array([c[n]["ref_pose"] if n in c else 0.0 for n in NAMES])
        for b in buses:  # find each node on its bus by its heartbeat
            b.poll(0.3)
        self.bus = [next((b for b in buses if (n, oc.HEARTBEAT) in b.last), None) for n in self.node]
        self.idx = [i for i in range(12) if self.bus[i] is not None]  # present joints
        self.groups = [(b, [i for i in self.idx if self.bus[i] is b]) for b in buses]
        self.groups = [(b, idx) for b, idx in self.groups if idx]
        if not self.idx:
            raise RuntimeError("No drive answers. Run: bringup.py scan")

    def missing(self):
        return [f"{NAMES[i]} (node {self.node[i]})" for i in range(12) if i not in self.idx]

    def read_turns(self):
        """Ask all present drives for their encoder estimates. Keep the last value when a reply is missing."""
        for b, idx in self.groups:
            b.poll()
            got = b.ask([self.node[i] for i in idx], oc.GET_ENCODER_ESTIMATES)
            for i in idx:
                if self.node[i] in got:
                    (self.p[i], self.v[i]), self.miss[i] = got[self.node[i]], 0
                else:
                    self.miss[i] += 1
        return self.p.copy()

    def turns(self):
        """Motor turns of the present joints. Raises an error when a drive does not reply 3 times."""
        for _ in range(3):
            p = self.read_turns()
            if not self.miss[self.idx].any():
                return p
        raise RuntimeError("No encoder reply from " + ", ".join(NAMES[i] for i in self.idx if self.miss[i]))

    def to_joint(self, p):
        return self.sign * (p - self.offset) / TURNS_PER_RAD

    def read(self):
        """Joint angles (rad) and velocities (rad/s). Needs zero() first."""
        self.read_turns()
        return self.to_joint(self.p), self.sign * self.v / TURNS_PER_RAD

    def calibrate(self, p, q_known):
        """Save the encoder angle at ZERO_POSE, from readings p taken at known joint angles q_known."""
        enc = enc_at_zero_pose(p, q_known, self.sign)
        self.enc_ref[self.idx], self.ref_pose[self.idx] = enc[self.idx], ZERO_POSE[self.idx]
        if self.calib:
            data = {n: {"enc_ref": round(float(e), 6), "ref_pose": float(r)}
                    for n, e, r in zip(NAMES, self.enc_ref, self.ref_pose) if not np.isnan(e)}
            Path(self.calib).write_text(json.dumps(data, indent=1) + "\n")

    def zero(self):
        """Set the joint zeros. Each joint must be within +-12 deg of its calibration pose (legs hanging)."""
        if np.isnan(self.enc_ref[self.idx]).any():
            raise RuntimeError(f"No calibration in {self.calib} for some joints. Run: bringup.py calibrate")
        self.offset = zero_offset(self.turns(), self.enc_ref, self.ref_pose, self.sign)

    def _send(self, idx, make):
        for i in idx:
            for msg in make(self.node[i], i):
                self.bus[i].send(msg)

    def heartbeat(self, i):
        """(time, axis error, axis state) of the last heartbeat of joint i."""
        t, (err, state) = self.bus[i].last.get((self.node[i], oc.HEARTBEAT), (0.0, (0, 0)))
        return t, err, state

    def enable(self, idx=None):
        """Clear old errors, then closed loop in position control with PASSTHROUGH input. Holds the current pose."""
        idx = list(self.idx if idx is None else idx)
        for i in idx:
            if err := self.heartbeat(i)[1] & ~(1 << 11):  # a watchdog error is normal on an idle drive
                print(f"{NAMES[i]}: old drive error {oc.error_names(err)}, cleared")
        self.read_turns()
        todo = idx
        for _ in range(3):  # a drive can lose a frame in a burst: send again to the drives that did not switch
            t0 = time.monotonic()
            self._send(todo, lambda n, i: [oc.clear_errors(n),
                                           oc.set_controller_modes(n, oc.POSITION_CONTROL, oc.PASSTHROUGH),
                                           oc.set_input_pos(n, self.p[i]),
                                           oc.set_axis_state(n, oc.CLOSED_LOOP)])
            while todo and time.monotonic() < t0 + 0.3:
                for b, _ in self.groups:
                    b.poll(0.01)
                todo = [i for i in todo if not (self.heartbeat(i)[0] > t0 and self.heartbeat(i)[2] == oc.CLOSED_LOOP)]
            if not todo:
                self.enabled[idx] = True
                return
        self.idle()
        raise RuntimeError("Closed loop failed: " + ", ".join(
            f"{NAMES[i]} (state {self.heartbeat(i)[2]}, error {oc.error_names(self.heartbeat(i)[1])})" for i in todo))

    def command(self, q, idx=None):
        """Send joint position targets (rad), clipped to the joint limits, to the enabled joints."""
        turns = self.offset + self.sign * np.clip(q, self.lo, self.hi) * TURNS_PER_RAD
        self._send(np.flatnonzero(self.enabled) if idx is None else idx,
                   lambda n, i: [oc.set_input_pos(n, turns[i])])

    def damping(self):
        """K = 0: velocity control with target 0. Only the damping D = 2 Nm*s/rad stays."""
        self._send(np.flatnonzero(self.enabled), lambda n, i: [
            oc.set_controller_modes(n, oc.VELOCITY_CONTROL, oc.PASSTHROUGH), oc.set_input_vel(n, 0.0)])

    def idle(self):
        """Motors off."""
        self._send(self.idx, lambda n, i: [oc.set_axis_state(n, oc.IDLE)])
        self.enabled[:] = False

    def relax(self, seconds=3.0):
        """Damping for some seconds, so that the legs fold slowly. Then idle. Ctrl+C goes to idle at once."""
        try:
            for k in ticks(RATE):
                self.damping()  # sent again in each tick: this feeds the drive watchdog
                if k >= seconds * RATE:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.idle()

    def vbus(self):
        """Bus voltage (V) of each present drive."""
        out = {}
        for b, idx in self.groups:
            got = b.ask([self.node[i] for i in idx], oc.GET_VBUS_VOLTAGE)
            out.update({NAMES[i]: got[self.node[i]][0] for i in idx if self.node[i] in got})
        return out

    def faults(self, q=None):
        """Reasons to stop: an error or a stop of an enabled drive, no heartbeat, no encoder reply,
        or a joint beyond its limits (then the zero is wrong)."""
        out, now = [], time.monotonic()
        for i in self.idx:
            t, err, state = self.heartbeat(i)
            if self.enabled[i] and err:
                out.append(f"{NAMES[i]}: drive error {oc.error_names(err)}")
            elif self.enabled[i] and state != oc.CLOSED_LOOP:
                out.append(f"{NAMES[i]}: drive is not in closed loop (state {state})")
            if now - t > 0.5:
                out.append(f"{NAMES[i]}: no heartbeat for 0.5 s")
            if self.miss[i] >= 3:
                out.append(f"{NAMES[i]}: no encoder reply 3 times")
            if q is not None and not self.lo[i] - LIMIT_MARGIN <= q[i] <= self.hi[i] + LIMIT_MARGIN:
                out.append(f"{NAMES[i]}: {q[i]:+.2f} rad is beyond its limits. Wrong zero?")
        return out

    def move_to(self, target, seconds=2.0, until=lambda: True):
        """Ramp from the current pose to target in `seconds`, then hold until until() is True.
        Return the list of faults, or [] when until() is True."""
        q0, _ = self.read()
        n = max(1, round(seconds * RATE))
        for k in ticks(RATE):
            q, _ = self.read()
            if f := self.faults(q):
                return f
            self.command(q0 + min(1.0, k / n) * (target - q0))
            if k >= n and until():
                return []
