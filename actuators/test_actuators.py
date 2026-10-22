import pytest

import st3215
from st3215 import Bus, ServoError, packet, parse_status


class FakePort:
    """Stands in for pyserial: records what we write, returns queued reply bytes."""

    def __init__(self, replies=b""):
        self.written, self.rx = b"", bytearray(replies)

    def write(self, data):
        self.written += data

    def read(self, n):
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def reset_input_buffer(self):
        pass


def status(sid, params=b"", err=0):
    body = bytes([sid, len(params) + 2, err]) + params
    return b"\xff\xff" + body + bytes([st3215.checksum(body)])


def test_packets():
    assert packet(1, st3215.PING) == bytes.fromhex("ff ff 01 02 01 fb")
    # examples from the Feetech protocol manual (actuators/README.md, source U2)
    assert packet(1, st3215.READ, [0x38, 2]) == bytes.fromhex("ff ff 01 04 02 38 02 be")  # read present position
    assert packet(st3215.BROADCAST, st3215.WRITE, [st3215.ID, 1]) == bytes.fromhex("ff ff fe 04 03 05 01 f4")  # set id 1
    # goal position 2048, time 0, speed 1000 (little endian)
    assert packet(1, st3215.WRITE, bytes.fromhex("2a 00 08 00 00 e8 03")) == bytes.fromhex("ff ff 01 09 03 2a 00 08 00 00 e8 03 d5")


def test_sync_write_two_servos():
    port = FakePort()
    Bus(port).sync_write_goal({1: (2048, 1000, 50), 2: (1024, 1000, 50)})
    assert port.written == bytes.fromhex("ff ff fe 14 83 29 07 01 32 00 08 00 00 e8 03 02 32 00 04 00 00 e8 03 f1")


def test_negative_position_and_signs():
    assert st3215.from_signed(-100) == 0x8064
    assert st3215.to_signed(0x8064, 15) == -100
    assert st3215.to_signed(0x0464, 10) == -100


def test_parse_status_and_state():
    pid, err, params = parse_status(bytes.fromhex("ff ff 01 02 00 fc"))  # ping reply, manual
    assert (pid, err, params) == (1, 0, b"")
    assert parse_status(bytes.fromhex("ff ff 01 04 00 18 05 dd"))[2] == bytes.fromhex("18 05")  # position 1304, manual
    with pytest.raises(ServoError):
        parse_status(bytes.fromhex("ff ff 01 02 00 fd"))
    # position 2048, speed -16, load -10.0 %, 12.0 V, 35 deg C
    port = FakePort(b"\x00" + status(1, bytes.fromhex("00 08 10 80 64 04 78 23")))
    s = Bus(port).state(1)
    assert port.written == packet(1, st3215.READ, [56, 8])
    assert s == dict(position=2048, speed=-16, load=-100, voltage=12.0, temperature=35)


def test_ping_and_missing_servo():
    assert Bus(FakePort(status(3))).ping(3)
    assert not Bus(FakePort(b"")).ping(3)
    assert not Bus(FakePort(status(4))).ping(3)  # wrong id answers


def test_pico_logic():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("pico_main", Path(__file__).parent / "pico_fw" / "main.py")
    fw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fw)  # main() does not run: __name__ is not "__main__"
    st = {"P": 0, "T": 0, "E": 0}
    assert fw.handle(st, "P1\n") == "OK P1 T0 E0"
    assert fw.handle(st, "t1") == "OK P1 T1 E0"
    assert fw.handle(st, "X").startswith("ERR")
    assert fw.safety(st, estop=False, ms_since_line=100) is None and st["P"] == 1
    assert fw.safety(st, estop=False, ms_since_line=600) == "TIMEOUT" and st == {"P": 0, "T": 0, "E": 0}
    fw.handle(st, "P1")
    assert fw.safety(st, estop=True, ms_since_line=0) == "ESTOP" and st["P"] == 0
    assert fw.handle(st, "P1") == "ERR ESTOP" and st["P"] == 0
    assert fw.handle(st, "S") == "OK P0 T0 E1"


def test_angle_mapping_and_mock_servos():
    import math
    from actuator_node import DEFAULT_CALIB, MockServos, to_rad, to_ticks
    c = dict(DEFAULT_CALIB["tilt"], zero=1900, direction=-1)
    assert to_ticks(c, 0.0) == 1900
    assert abs(to_rad(c, to_ticks(c, 0.3)) - 0.3) < 2 * math.pi / 4096
    assert to_ticks(c, 3.0) == to_ticks(c, math.radians(90))  # clipped to the limit
    s = MockServos(DEFAULT_CALIB)
    s.move({"pan": 1.0, "tilt": 0.0}, 0.1)
    assert abs(s.read()[0] - math.radians(18)) < 1e-9  # 180 deg/s for 0.1 s


def test_walk_bridge_clips_and_times_out():
    import json
    import socket
    import time
    from walk_bridge import AimGamepad
    g = AimGamepad(port=0)  # any free port
    g.start_udp()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = g.sock.getsockname()
    tx.sendto(b"not json", addr)
    tx.sendto(json.dumps({"vx": 1.0, "yaw_rate": -2.0, "pan": 0.1}).encode(), addr)
    time.sleep(0.2)
    assert g.commands["velocity_x"] == 0.5 and g.commands["velocity_yaw"] == -1.0 and g.commands["velocity_y"] == 0.0
    time.sleep(0.6)  # no message for more than 0.5 s
    assert g.commands["velocity_x"] == 0.0 and g.commands["velocity_yaw"] == 0.0
    g.stop()


def test_selftest_mock_passes(capsys, monkeypatch):
    import selftest
    monkeypatch.setattr(selftest, "UDP_PORTS", [0, 0])  # any free port, so a running pipe does not disturb the test
    assert selftest.main(["--mock", "--yes"]) == 0
    assert capsys.readouterr().out.count(" PASS: ") == 7


def test_selftest_stops_on_bad_voltage(capsys, monkeypatch):
    import selftest
    monkeypatch.setattr(selftest, "UDP_PORTS", [0])
    monkeypatch.setattr(selftest.MockBus, "state", lambda self, sid: dict(position=2048, speed=0, load=0, voltage=7.4, temperature=30))
    assert selftest.main(["--mock", "--yes"]) == 1
    out = capsys.readouterr().out
    assert "Step 2 FAIL" in out and "Step 3 SKIP" in out and "Step 7 PASS" in out


def test_walk_bridge_start_kick():
    """From a stand, a forward command gets at least KICK_VX for KICK_S, so the policy starts to step."""
    import json
    import socket
    import time
    import walk_bridge as wb
    g = wb.AimGamepad(port=0)
    g.start_udp()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send = lambda vx: tx.sendto(json.dumps({"vx": vx, "yaw_rate": 0.4}).encode(), g.sock.getsockname())  # noqa: E731
    send(0.25)
    time.sleep(0.1)
    assert g.commands["velocity_x"] == max(0.25, wb.KICK_VX) and g.commands["velocity_yaw"] == 0.4
    for _ in range(int(wb.KICK_S / 0.1) + 2):
        send(0.25)
        time.sleep(0.1)
    assert g.commands["velocity_x"] == 0.25  # after the kick
    send(0.0)
    time.sleep(0.1)
    send(-0.25)  # backward: no kick
    time.sleep(0.1)
    assert g.commands["velocity_x"] == -0.25
    g.stop()
