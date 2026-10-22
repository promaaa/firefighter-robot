"""Hiwonder IM10A IMU (WitMotion JY901B core) on a serial port (header UART or USB), WitMotion standard protocol.

    uv run python legs/imu.py --setup   one time: gyro + quaternion output, 200 Hz, 230400 baud, saved in the IMU
    uv run python legs/imu.py           live gravity direction and gyro in the torso frame, to check IMU_TO_BODY

Frame: 0x55, type, 4 x int16 little-endian, checksum = low byte of the sum of the first 10 bytes.
Type 0x52: angular velocity = value / 32768 * 2000 deg/s. Type 0x59: quaternion (w, x, y, z) = value / 32768.
Config command: 0xFF 0xAA, register, value (uint16 little-endian). Unlock first, save last, within 10 s.
Sources: wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol,
and BHL Lowlevel robot/imu.py (BHL uses the same IM10A).
"""
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.append(str(Path(__file__).resolve().parents[1] / "robot"))
from robot_config import CONF  # noqa: E402

PORT, BAUD = CONF["imu"]["port"], CONF["imu"]["baud"]  # robot/config.toml: header UART. The doc allows 460800 only for other WitMotion models.
GYRO, QUAT = 0x52, 0x59
# IMU -> torso rotation. Column k = the direction of IMU axis k (the X, Y, Z marks on the IMU) in the torso frame
# (x forward, y left, z up). The identity means: the IMU lies flat, X mark forward. CHECK on the robot (README).
IMU_TO_BODY = np.eye(3)


def parse(buf):
    """Decode all complete frames in buf. Return ({type: 4 int16}, the bytes that are left)."""
    out, i = {}, 0
    while len(buf) - i >= 11:
        f = buf[i:i + 11]
        if f[0] != 0x55 or sum(f[:10]) & 0xFF != f[10]:
            i += 1  # not at a frame start, or a bad checksum: move on by one byte
            continue
        out[f[1]] = struct.unpack_from("<4h", f, 2)
        i += 11
    return out, buf[i:]


def to_body(quat_raw, gyro_raw, rot=IMU_TO_BODY):
    """Torso quaternion [w, x, y, z] and torso angular velocity (rad/s) from the raw int16 values."""
    imu_to_body = Rotation.from_matrix(rot)
    q = Rotation.from_quat(np.array(quat_raw) / 32768.0, scalar_first=True) * imu_to_body.inv()
    return q.as_quat(scalar_first=True), imu_to_body.apply(np.radians(np.array(gyro_raw[:3]) * 2000.0 / 32768.0))


class Im10a:
    def __init__(self, port=PORT, baud=BAUD):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.last, self.t = {}, {}
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        buf = b""
        while True:
            vals, buf = parse(buf + self.ser.read(max(1, self.ser.in_waiting)))
            now = time.monotonic()
            for k, v in vals.items():
                self.last[k], self.t[k] = v, now

    def read(self):
        """Torso quaternion [w, x, y, z] and angular velocity (rad/s). Raises an error after 0.1 s without data."""
        now = time.monotonic()
        if now - min(self.t.get(GYRO, 0.0), self.t.get(QUAT, 0.0)) > 0.1:
            raise RuntimeError(f"IMU: no gyro or quaternion frame for 0.1 s on {self.ser.port}")
        return to_body(self.last[QUAT], self.last[GYRO])


def setup(port=PORT, old_baud=9600):
    """One time: gyro + quaternion output (register 0x02), 200 Hz (0x03), 230400 baud (0x04), then save (0x00)."""
    import serial

    def write(ser, reg, value):
        ser.write(struct.pack("<BBBH", 0xFF, 0xAA, reg, value))
        time.sleep(0.1)

    ser = serial.Serial(port, old_baud, timeout=0.1)
    write(ser, 0x69, 0xB588)                  # unlock: FF AA 69 88 B5
    write(ser, 0x02, 1 << 2 | 1 << 9)         # output content: angular velocity (bit 2), quaternion (bit 9)
    write(ser, 0x03, 0x0B)                    # output rate 200 Hz
    write(ser, 0x00, 0x0000)                  # save
    write(ser, 0x69, 0xB588)
    write(ser, 0x04, 0x07)                    # baud 230400. The IMU switches now.
    ser.close()
    ser = serial.Serial(port, BAUD, timeout=0.1)
    write(ser, 0x00, 0x0000)                  # save, at the new baud (as BHL robot/imu.py does)
    ser.close()


if __name__ == "__main__":
    if "--setup" in sys.argv:
        setup()
        print(f"IMU set to 200 Hz, {BAUD} baud, gyro + quaternion. Now run: uv run python legs/imu.py")
        sys.exit()
    imu = Im10a()
    time.sleep(0.5)
    print("Upright: gravity = (0, 0, -1). Tilt forward: gravity x > 0. Tilt to the left: gravity y > 0. Ctrl+C stops.")
    while True:
        q, w = imu.read()
        g = Rotation.from_quat(q, scalar_first=True).inv().apply([0, 0, -1.0])
        print(f"gravity {g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f}   gyro (rad/s) {w[0]:+.2f} {w[1]:+.2f} {w[2]:+.2f}")
        time.sleep(0.2)
