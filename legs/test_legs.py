"""Tests for legs/: CAN messages, IMU frames, gain numbers, zeroing math, and a policy run on the fake drives.

    uv run pytest legs -q
"""
import can
import numpy as np
from pytest import approx

import imu
import odrive_can as oc
import robot as rb


def raw(m):
    return m.arbitration_id, bytes(m.data), m.is_remote_frame


def test_messages_against_hand_computed_bytes():
    # ID = node << 5 | cmd. Floats little-endian: 1.5 = 3fc00000, 0.5 = 3f000000, 50.0 = 42480000.
    m = oc.set_input_pos(3, 1.5, vel_ff=0.25, torque_ff=-0.1)  # int16 x 0.001: 250 = 00fa, -100 = ff9c
    assert raw(m) == (0x6C, bytes.fromhex("0000c03f fa00 9cff"), False)
    assert oc.decode(m) == (3, oc.SET_INPUT_POS, (1.5, 0.25, -0.1))
    assert raw(oc.set_axis_state(1, oc.CLOSED_LOOP)) == (0x27, bytes.fromhex("08000000"), False)
    assert raw(oc.set_controller_modes(2, oc.POSITION_CONTROL, oc.PASSTHROUGH)) == (0x4B, bytes.fromhex("03000000 01000000"), False)
    assert raw(oc.set_input_vel(4, 0.0)) == (0x8D, bytes(8), False)
    assert raw(oc.set_input_torque(5, 0.5)) == (0xAE, bytes.fromhex("0000003f"), False)
    assert raw(oc.set_vel_limit(1, 50.0)) == (0x2F, bytes.fromhex("00004842"), False)
    assert [raw(f(6))[0] for f in (oc.estop, oc.clear_errors, oc.reboot)] == [0xC2, 0xD8, 0xD6]
    r = oc.request(4, oc.GET_ENCODER_ESTIMATES)
    assert raw(r) == (0x89, b"", True) and r.dlc == 8
    # replies from a drive
    hb = can.Message(arbitration_id=0xA1, data=bytes.fromhex("00010000 08000000"), is_extended_id=False)
    assert oc.decode(hb) == (5, oc.HEARTBEAT, (0x100, 8)) and oc.error_names(0x100) == "ENCODER_FAILED"
    enc = can.Message(arbitration_id=0x89, data=bytes.fromhex("00001040 000080bf"), is_extended_id=False)
    assert oc.decode(enc) == (4, oc.GET_ENCODER_ESTIMATES, (2.25, -1.0))
    vbus = can.Message(arbitration_id=0x37, data=bytes.fromhex("0000c041 00000000"), is_extended_id=False)
    assert oc.decode(vbus) == (1, oc.GET_VBUS_VOLTAGE, (24.0,))


def test_imu_frames():
    gyro = bytes.fromhex("55 52 0040 0000 00c0 0000 a7")  # 16384, 0, -16384: +-1000 deg/s. Sum 0x1a7.
    quat = bytes.fromhex("55 59 ff7f 0000 0000 0000 2c")  # w = 32767/32768. Sum 0x22c.
    bad = bytes.fromhex("55 52 0100 0000 0000 0000 00")   # wrong checksum: skipped
    vals, rest = imu.parse(b"\x00\x13" + bad + gyro + quat + gyro[:5])
    assert vals == {0x52: (16384, 0, -16384, 0), 0x59: (32767, 0, 0, 0)} and rest == gyro[:5]
    q, w = imu.to_body(vals[0x59], vals[0x52])
    assert q == approx([1, 0, 0, 0], abs=1e-4) and w == approx([np.radians(1000), 0, -np.radians(1000)])
    turned = np.diag([-1.0, -1.0, 1.0])  # IMU mounted with its X mark backward
    assert imu.to_body(vals[0x59], vals[0x52], turned)[1] == approx([-np.radians(1000), 0, -np.radians(1000)])


def test_gains():
    assert rb.joint_gains(12.6, 0.223) == approx((100.6, 7.99), abs=0.05)  # course config, 6512
    assert rb.joint_gains(20.0, 0.1) == approx((71.6, 3.58), abs=0.05)     # course config, 5010
    assert rb.drive_gains(20.0, 2.0) == approx((10.0, 0.05585), abs=1e-5)  # policy K = 20, D = 2
    assert rb.joint_torque_limit(18, 0.067) == approx(18.09, abs=0.01)
    assert rb.joint_torque_limit(18, 0.0836) == approx(22.57, abs=0.01)
    assert rb.current_limit(6.0, 0.067) == approx(5.97, abs=0.01)
    assert rb.current_limit(2.0, 0.0836) == approx(1.59, abs=0.01)


def test_zeroing_11_and_13_degrees():
    rng = np.random.default_rng(1)
    sign, enc_ref = np.array([j[3] for j in rb.JOINTS], float), rng.random(12)  # enc_ref: encoder at ZERO_POSE
    whole_turns = rng.integers(-3, 4, 12)  # unknown after each power-on

    def drive_reading(q):  # motor turns that the drive reports at joint angles q
        return enc_ref + sign * q * rb.TURNS_PER_RAD + whole_turns

    # one-time calibration at the stops gives back the encoder angle at ZERO_POSE
    assert rb.enc_at_zero_pose(drive_reading(rb.STOP), rb.STOP, sign) == approx(enc_ref)
    # zeroing with the legs 11 or 13 deg away from ZERO_POSE
    start = np.radians([11, -11, 13, -13] * 3)
    offset = rb.zero_offset(drive_reading(start), enc_ref, rb.ZERO_POSE, sign)

    def joint_reading(q):
        return sign * (drive_reading(q) - offset) / rb.TURNS_PER_RAD

    assert np.degrees(joint_reading(start) - start) == approx([0, 0, -24, 24] * 3, abs=1e-6)
    # bringup.py check: at the calibration stops, only the joints that were 13 deg off read wrong
    assert rb.wrong_zero(joint_reading(rb.STOP)).tolist() == [False, False, True, True] * 3


def test_fake_policy_run_stays_up():
    import run_policy
    from fake_bus import FakeRobot
    sim = FakeRobot()
    buses = sim.buses()
    try:
        robot = rb.Robot(buses, calib=None)
        assert not robot.missing()
        robot.calibrate(robot.turns(), rb.ZERO_POSE)
        robot.zero()
        assert robot.read()[0] == approx(np.zeros(12), abs=1e-4)
        reason = run_policy.run(robot, sim.imu, sim.ready, seconds=6.0, cmd=(0.0, 0.0, 0.0), log=lambda *a: None)
        assert reason == "time"
        assert sim.min_height > 0.5, f"torso (IMU) height fell to {sim.min_height:.2f} m"
        robot.idle()
    finally:
        for b in buses:
            b.close()
        sim.close()


def test_pose_out_estimate():
    """run_policy.PoseOut: yaw from the IMU, relative to the start; x and y from the commanded vx along that yaw."""
    import json
    import socket
    import time
    import run_policy
    from scipy.spatial.transform import Rotation
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(1.0)
    run_policy.PORTS = dict(run_policy.PORTS, pose=rx.getsockname()[1], supervisor=rx.getsockname()[1])
    q = lambda yaw, pitch=0.0: Rotation.from_euler("ZYX", [yaw, pitch, 0.0]).as_quat(scalar_first=True)  # noqa: E731
    out = run_policy.PoseOut()
    out.send(q(1.0), 0.5)  # the start heading becomes yaw 0
    time.sleep(0.2)
    out.send(q(1.0 + np.pi / 2, 0.1), 0.5)  # turned 90 deg left, nose down 0.1 rad
    msgs = [json.loads(rx.recv(4096)) for _ in range(4)]
    m = msgs[-1]
    assert m["yaw"] == approx(np.pi / 2, abs=1e-3) and m["pitch"] == approx(0.1, abs=1e-3) and m["roll"] == approx(0.0, abs=1e-3)
    assert m["x"] == approx(0.0, abs=0.02) and m["y"] == approx(0.5 * 0.2, abs=0.03)  # 0.2 s at 0.5 m/s along +y
