"""Run the ASMR 2026 baseline walking policy at 25 Hz on the 12 leg drives over CAN, or on fake drives.

    uv run python legs/run_policy.py --fake                 fake drives on the ASMR MuJoCo model, no hardware
    uv run python legs/run_policy.py                        the robot: CAN buses and IMU of robot/config.toml
    uv run python legs/run_policy.py --vx 0.2 --knee-max 1.4 --seconds 30
    uv run python legs/run_policy.py --udp                  velocity from aim/run.py on the walk port (robot/launch.py)

Sequence: the robot hangs from the gantry, legs straight down. The code zeroes the joints, ramps them to the
default pose in 2 s and holds it. Lower the robot until it stands, then press Enter: the policy starts.
With --udp, the velocity comes from aim/run.py through actuators/walk_bridge.py: clipped, and 0 after 0.5 s without a message.
While the policy runs, the program sends a pose estimate to aim/run.py (pose port) and robot/launch.py (supervisor port):
yaw, roll and pitch from the IMU, x and y from the commanded vx along that yaw. With --fake, it also sends the true
MuJoCo torso pose to the sim_truth port (the fake camera of robot/launch.py --sim).
Ctrl+C, SIGTERM or any fault (drive error, lost drive, joint beyond its limits, torso tilt > 0.78 rad):
damping for 3 s, then the motors go idle.
"""
import argparse
import json
import math
import select
import signal
import socket
import sys
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation

import robot as rb
from source.robot_models.robots import asmr2026_policy as policy_model
from robot_config import CONF, HOST, path  # noqa: E402  robot/ is on sys.path after `import robot`

POLICY = path(CONF["legs"]["policy"])
PORTS = CONF["ports"]


def enter_pressed():
    return bool(select.select([sys.stdin], [], [], 0)[0]) and sys.stdin.readline() is not None


def check_policy():
    """Check the configured ONNX file and observation/action shapes without opening CAN buses."""
    if not POLICY.is_file():
        raise FileNotFoundError(f"walking network not found: {POLICY}")
    session = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
    inputs, outputs = session.get_inputs(), session.get_outputs()
    if len(inputs) != 1 or inputs[0].name != "obs":
        raise RuntimeError(f"expected one ONNX input named 'obs', got {[x.name for x in inputs]}")
    if len(outputs) != 1:
        raise RuntimeError(f"expected one ONNX output, got {[x.name for x in outputs]}")
    if inputs[0].shape[-1] != policy_model.OBSERVATION_SIZE:
        raise RuntimeError(
            f"expected {policy_model.OBSERVATION_SIZE} policy observations, got {inputs[0].shape}"
        )
    obs = policy_model.observation(
        np.zeros(3, np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], np.float32),
        np.zeros(3, np.float32),
        policy_model.DEFAULT,
        np.zeros(12, np.float32),
        np.zeros(12, np.float32),
    )
    action = session.run(None, {inputs[0].name: obs})[0]
    expected_action_shape = (1, policy_model.ACTION_SIZE)
    if action.shape != expected_action_shape or not np.isfinite(action).all():
        raise RuntimeError(f"expected finite policy output with shape {expected_action_shape}, got {action.shape}")
    print(f"Policy check passed: {POLICY}")
    print(f"  input {inputs[0].name}: {obs.shape}; output {outputs[0].name}: {action.shape}")
    print(f"  ASMR joints: {len(policy_model.LEG_JOINTS)}; simulation modules were not loaded")


def zyx(quat):
    """yaw (left +), pitch (nose down +), roll (left side up +) of a quaternion [w, x, y, z], as aim/run.py wants them."""
    r = Rotation.from_quat(quat, scalar_first=True).as_matrix()
    return math.atan2(r[1, 0], r[0, 0]), math.asin(-np.clip(r[2, 0], -1, 1)), math.atan2(r[2, 1], r[2, 2])


class PoseOut:
    """Pose estimate for aim/run.py: IMU yaw (0 at the policy start), roll and pitch; x and y from the commanded vx.

    Limit: odometry from the command, not from the legs. Add leg odometry when the drift matters on the floor."""

    def __init__(self, truth=None):
        self.tx, self.truth = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), truth
        self.yaw0, self.xy, self.t = None, np.zeros(2), None

    def _send(self, msg, ports):
        for port in ports:
            try:
                self.tx.sendto(json.dumps(msg).encode(), (HOST, port))
            except OSError:
                pass

    def send(self, quat, vx):
        now = time.monotonic()
        yaw, pitch, roll = zyx(quat)
        self.yaw0 = yaw if self.yaw0 is None else self.yaw0
        yaw = math.remainder(yaw - self.yaw0, 2 * math.pi)
        if self.t is not None:
            self.xy += vx * (now - self.t) * np.array([math.cos(yaw), math.sin(yaw)])
        self.t = now
        r = lambda v: round(float(v), 4)
        self._send(dict(timestamp=now, x=r(self.xy[0]), y=r(self.xy[1]), yaw=r(yaw), roll=r(roll), pitch=r(pitch), vx=r(vx)),
                   (PORTS["pose"], PORTS["supervisor"]))
        if self.truth:
            pos, q, sim_t = self.truth()
            yaw, pitch, roll = zyx(q)
            self._send(dict(timestamp=now, truth=True, x=r(pos[0]), y=r(pos[1]), z=r(pos[2]), yaw=r(yaw), roll=r(roll), pitch=r(pitch),
                            sim_t=r(sim_t)),
                       (PORTS["sim_truth"], PORTS["supervisor"]))


def run(robot, read_imu, ready, seconds, cmd, log=print, pose_out=None):
    """Ramp to the default pose, hold it until ready() is True, then run the policy. Return why it stopped.
    cmd: (vx, vy, yaw rate), or a function that gives them at each tick (the walk bridge)."""
    policy = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
    robot.enable()
    log("Ramp to the default pose in 2 s. Then lower the robot until it stands, and press Enter.")
    if f := robot.move_to(rb.DEFAULT, 2.0, until=ready):
        return "; ".join(f)
    log(f"Policy runs for {seconds:.0f} s. Ctrl+C stops it.")
    get_cmd = cmd if callable(cmd) else (lambda: cmd)
    action, period, body = np.zeros(12, np.float32), policy_model.POLICY_DT, []
    t_end = time.monotonic() + seconds
    try:
        for _ in rb.ticks(1.0 / period):
            t0 = time.monotonic()
            q, qd = robot.read()
            quat, gyro = read_imu()
            tilt = np.arccos(np.clip(-policy_model.quat_rotate_inverse(quat, np.array([0, 0, -1.0]))[2], -1, 1))
            if f := robot.faults(q) + ([f"torso tilt {tilt:.2f} rad"] if tilt > policy_model.TILT_LIMIT else []):
                return "; ".join(f)
            c = np.asarray(get_cmd(), np.float32)
            q_policy, qd_policy = rb.to_asmr(q), rb.to_asmr(qd)
            action = policy.run(None, {"obs": policy_model.observation(c, quat, gyro, q_policy, qd_policy, action)})[0][0]
            target_policy = policy_model.DEFAULT + policy_model.ACTION_SCALE * action
            robot.command(rb.to_hardware_coordinates(target_policy))
            if pose_out:
                pose_out.send(quat, float(c[0]))
            body.append(time.monotonic() - t0)
            if time.monotonic() > t_end:
                return "time"
    finally:  # the loop timing on this computer: the 25 Hz loop must stay below 40 ms
        if body:
            b = np.array(body) * 1000
            log(f"Policy loop: {len(b)} ticks, time per tick mean {b.mean():.1f} ms, max {b.max():.1f} ms, "
                f"{int((b > period * 1000).sum())} ticks longer than {period * 1000:.0f} ms")


def _stop(*_):
    raise KeyboardInterrupt  # SIGTERM (robot/launch.py, systemd) stops like Ctrl+C: damping, then idle


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fake", action="store_true", help="fake drives on a MuJoCo biped")
    ap.add_argument("--check", action="store_true", help="check the configured network and shapes; do not open CAN")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--vx", type=float, default=0.0, help="m/s, training range -0.5 to 0.5")
    ap.add_argument("--vy", type=float, default=0.0, help="m/s, training range -0.25 to 0.25")
    ap.add_argument("--yaw", type=float, default=0.0, help="rad/s, training range -1 to 1")
    ap.add_argument("--knee-max", type=float, help="knee target limit (rad). F2 needs 1.4 (robot/config.toml [legs] knee_max).")
    ap.add_argument("--udp", action="store_true", help="velocity from aim/run.py on the walk port, not from --vx, --vy, --yaw")
    a = ap.parse_args()
    signal.signal(signal.SIGTERM, _stop)

    if a.check:
        check_policy()
        return

    buses, sim = rb.open_buses(a.fake)
    robot = rb.Robot(buses, calib=None if sim else rb.CALIB)
    if missing := robot.missing():
        sys.exit("Drives not found: " + ", ".join(missing))
    if sim:
        robot.calibrate(robot.turns(), rb.ZERO_POSE)  # the fake starts exactly in ZERO_POSE, like a jig
        read_imu, ready = sim.imu, sim.ready
    else:
        from imu import Im10a
        imu = Im10a()
        time.sleep(0.3)
        read_imu, ready = imu.read, enter_pressed
    read_imu()  # fails now, before a motor moves, when the IMU sends nothing
    if a.knee_max is not None:
        knees = [i for i, n in enumerate(rb.NAMES) if "knee" in n]
        robot.lo[knees] = np.maximum(robot.lo[knees], -a.knee_max)
        robot.hi[knees] = np.minimum(robot.hi[knees], a.knee_max)
    v = min(robot.vbus().values())
    print(f"Lowest bus voltage: {v:.1f} V" + ("  (low: charge the battery, the drives trip at 21.0 V)" if v < 22.0 else ""))
    robot.zero()
    print("Joint angles after zeroing (deg):", np.round(np.degrees(robot.read()[0]), 1))

    cmd, bridge = (a.vx, a.vy, a.yaw), None
    if a.udp:
        sys.path.append(str(Path(__file__).resolve().parents[1] / "actuators"))
        from walk_bridge import AimGamepad
        bridge = AimGamepad()
        bridge.start_udp()
        cmd = lambda: (bridge.commands["velocity_x"], 0.0, bridge.commands["velocity_yaw"])  # noqa: E731
        print(f"Walk command from UDP port {PORTS['walk_cmd']}")
    reason = "error"
    try:
        reason = run(robot, read_imu, ready, a.seconds, cmd, pose_out=PoseOut(sim.truth if sim else None))
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    except Exception as e:  # noqa: BLE001  stop the motors safely on any error, then report it
        reason = f"error: {e!r}"
    finally:
        print(f"Stop: {reason}. Damping for 3 s, then idle.")
        if sim:
            print(f"Fake robot: torso height {sim.height():.2f} m, lowest since the hoist let go {sim.min_height:.2f} m")
        robot.relax()
        if bridge:
            bridge.stop()
        for b in buses:
            b.close()
        if sim:
            sim.close()


if __name__ == "__main__":
    main()
