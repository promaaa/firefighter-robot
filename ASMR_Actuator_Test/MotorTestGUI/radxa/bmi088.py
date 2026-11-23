"""Bosch BMI088 IMU (Seeed Grove 6-axis board, course BOM) over I2C, with a Mahony orientation filter.

The BMI088 gives acceleration and angular rate only, no orientation. Bmi088Imu reads both in a background thread,
rotates them into the robot base frame, removes the gyro bias measured at startup, and estimates the base orientation
with a Mahony filter (gyro integration corrected towards the accelerometer's gravity direction). It offers the
attributes of BHL's SerialImu that Humanoid.get_observations reads, so it can replace it:
    quaternion        (w, x, y, z) base orientation; only roll and pitch are meaningful (no magnetometer), which is
                      all the policy uses (projected gravity)
    angular_velocity  (x, y, z) deg/s in the base frame, bias removed
    acceleration      (x, y, z) m/s^2 in the base frame (specific force: +9.81 up at rest)

Register values are from the BMI088 data sheet (BST-BMI088-DS001, rev 1.3). That revision's accelerometer conversion
lacks the factor 1.5 that makes the ranges +-3/6/12/24 g (Bosch's BMI08x API has it); test_bmi088.py prints |a|,
which must read 9.81 m/s^2 at rest.

Only numpy and smbus2 (pip install smbus2) are needed; smbus2 is imported when the sensor is opened, so the filter can
be used offline.
"""

import threading
import time

import numpy as np

GRAVITY = 9.80665

# accelerometer registers (I2C address 0x18 with SDO1 low, 0x19 with SDO1 high)
ACC_CHIP_ID, ACC_CHIP_ID_VALUE = 0x00, 0x1E
ACC_DATA = 0x12  # X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
ACC_CONF = 0x40  # bit 7 must be 1, [6:4] bandwidth (0 OSR4, 1 OSR2, 2 normal), [3:0] ODR (0x0A 400 Hz)
ACC_RANGE = 0x41  # 0 +-3 g, 1 +-6 g, 2 +-12 g, 3 +-24 g
ACC_PWR_CONF = 0x7C  # 0x00 active, 0x03 suspend
ACC_PWR_CTRL = 0x7D  # 0x04 on, 0x00 off
ACC_SOFTRESET = 0x7E
# gyroscope registers (I2C address 0x68 with SDO2 low, 0x69 with SDO2 high)
GYRO_CHIP_ID, GYRO_CHIP_ID_VALUE = 0x00, 0x0F
GYRO_DATA = 0x02  # X LSB ... Z MSB
GYRO_RANGE = 0x0F  # 0 +-2000, 1 +-1000, 2 +-500, 3 +-250, 4 +-125 deg/s
GYRO_BANDWIDTH = 0x10  # 0x02: ODR 1000 Hz, filter 116 Hz; 0x03: 400 Hz, 47 Hz
GYRO_LPM1 = 0x11  # 0x00 normal
GYRO_SOFTRESET = 0x14
SOFTRESET_CMD = 0xB6

ACC_RANGE_G = {0: 3.0, 1: 6.0, 2: 12.0, 3: 24.0}
GYRO_RANGE_DPS = {0: 2000.0, 1: 1000.0, 2: 500.0, 3: 250.0, 4: 125.0}


def axes_to_matrix(axes: tuple[str, str, str]) -> np.ndarray:
    """Rotation from the IMU frame to the base frame from signed axis names: axes[i] is the IMU axis along base axis
    i, e.g. ("+y", "-x", "+z") for a board turned 90 deg about z (base x = IMU +y, base y = IMU -x)."""
    matrix = np.zeros((3, 3))
    for row, name in enumerate(axes):
        sign = -1.0 if name.startswith("-") else 1.0
        matrix[row, "xyz".index(name.lstrip("+-"))] = sign
    if abs(np.linalg.det(matrix) - 1.0) > 1e-6:
        raise ValueError(f"axes {axes} are not a right-handed rotation")
    return matrix


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def up_in_body(q: np.ndarray) -> np.ndarray:
    """World up (0, 0, 1) expressed in the body frame of orientation q (body to world)."""
    w, x, y, z = q
    return np.array([2.0 * (x * z - w * y), 2.0 * (w * x + y * z), w * w - x * x - y * y + z * z])


class MahonyFilter:
    """Mahony orientation filter: integrates the gyro and pulls the estimated up direction towards the measured one.

    kp (rad/s) sets how fast the tilt follows the accelerometer (time constant about 1 / kp), ki the integral that
    tracks the remaining gyro bias. Accelerometer samples whose magnitude is off 1 g by more than accel_gate (fraction)
    are skipped, so foot impacts and pushes do not tilt the estimate.

    Defaults from MuJoCo walking data (course robot, BMI088-like noise at 400 Hz, 0.3 deg/s bias left; 2026-11-23):
    tilt error mean 0.85, p95 2.0, max 3.8 deg; kp 0.5-3, ki 0-0.05 and gates 0.15-0.5 all gave mean 0.8-1.3 and
    p95 2.0-3.1 deg. The policy trains with +- 2 deg IMU tilt and +- 0.05 projected gravity noise.
    """

    def __init__(self, kp: float = 2.0, ki: float = 0.02, accel_gate: float = 0.3):
        self.kp, self.ki, self.accel_gate = kp, ki, accel_gate
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.integral = np.zeros(3)

    def initialize(self, accel: np.ndarray):
        """Start level with the measured gravity (roll and pitch from the accelerometer, yaw 0)."""
        ax, ay, az = accel / np.linalg.norm(accel)
        roll, pitch = np.arctan2(ay, az), np.arctan2(-ax, np.hypot(ay, az))
        quat_roll = np.array([np.cos(roll / 2), np.sin(roll / 2), 0.0, 0.0])
        quat_pitch = np.array([np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0])
        self.q = quat_mul(quat_pitch, quat_roll)
        self.integral[:] = 0.0

    def update(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> np.ndarray:
        """Advance by dt with the gyro (rad/s) and accelerometer (m/s^2) in the body frame; returns the corrected
        angular velocity (gyro plus the integral bias correction)."""
        omega = gyro.copy()
        norm = np.linalg.norm(accel)
        if norm > 0.0 and abs(norm - GRAVITY) < self.accel_gate * GRAVITY:
            error = np.cross(accel / norm, up_in_body(self.q))
            self.integral += self.ki * error * dt
            omega_corrected = omega + self.integral + self.kp * error
        else:
            omega_corrected = omega + self.integral
        self.q = self.q + 0.5 * quat_mul(self.q, np.array([0.0, *omega_corrected])) * dt
        self.q /= np.linalg.norm(self.q)
        return omega + self.integral


class Bmi088:
    """BMI088 accelerometer and gyroscope on one I2C bus; read() returns (accel m/s^2, gyro rad/s) in the IMU frame.

    Defaults: accelerometer +-12 g (foot impacts reached 7 g at the torso in MuJoCo), 400 Hz, OSR2 (75 Hz filter);
    gyroscope +-1000 deg/s, 1000 Hz with the 116 Hz filter.
    """

    def __init__(self, bus: int, acc_address: int | None = None, gyro_address: int | None = None,
                 acc_range: int = 2, gyro_range: int = 1, gyro_bandwidth: int = 0x02,
                 acc_conf: int = 0x80 | (0x01 << 4) | 0x0A):
        from smbus2 import SMBus, i2c_msg

        self._i2c_msg = i2c_msg
        self.bus = SMBus(bus)
        self.acc_address = acc_address or self._probe((0x19, 0x18), ACC_CHIP_ID_VALUE, "accelerometer")
        self.gyro_address = gyro_address or self._probe((0x69, 0x68), GYRO_CHIP_ID_VALUE, "gyroscope")
        # accelerometer: reset, switch on (it starts suspended), then configure
        self._soft_reset(self.acc_address, ACC_SOFTRESET)
        time.sleep(0.002)
        self.bus.read_byte_data(self.acc_address, ACC_CHIP_ID)
        self.bus.write_byte_data(self.acc_address, ACC_PWR_CONF, 0x00)
        time.sleep(0.001)
        self.bus.write_byte_data(self.acc_address, ACC_PWR_CTRL, 0x04)
        time.sleep(0.05)
        self.bus.write_byte_data(self.acc_address, ACC_CONF, acc_conf)
        self.bus.write_byte_data(self.acc_address, ACC_RANGE, acc_range)
        # gyroscope
        self._soft_reset(self.gyro_address, GYRO_SOFTRESET)
        time.sleep(0.03)
        self.bus.write_byte_data(self.gyro_address, GYRO_LPM1, 0x00)
        self.bus.write_byte_data(self.gyro_address, GYRO_RANGE, gyro_range)
        self.bus.write_byte_data(self.gyro_address, GYRO_BANDWIDTH, gyro_bandwidth)
        time.sleep(0.01)
        self.acc_scale = ACC_RANGE_G[acc_range] * GRAVITY / 32768.0
        self.gyro_scale = np.deg2rad(GYRO_RANGE_DPS[gyro_range]) / 32768.0

    def _soft_reset(self, address: int, register: int):
        """The chip resets during this write and does not acknowledge it, so Linux returns ENXIO (errno 6).
        Seen on the robot on 2026-11-23 (ROCK 3C /dev/i2c-3, Grove BMI088, both chips); the chips respond again
        1 ms later. Ignore only that error. (MotorTestGUI fix; the SOBANG_Locomotion copy needs the same change.)"""
        try:
            self.bus.write_byte_data(address, register, SOFTRESET_CMD)
        except OSError as e:
            if e.errno != 6:
                raise

    def _probe(self, addresses, chip_id, name) -> int:
        for address in addresses:
            try:
                if self.bus.read_byte_data(address, 0x00) == chip_id:
                    return address
            except OSError:
                pass
        raise RuntimeError(f"BMI088 {name} not found at {[hex(a) for a in addresses]} (check the bus with i2cdetect)")

    def _read_vector(self, address: int, register: int) -> np.ndarray:
        write, read = self._i2c_msg.write(address, [register]), self._i2c_msg.read(address, 6)
        self.bus.i2c_rdwr(write, read)
        return np.frombuffer(bytes(read), dtype="<i2").astype(np.float64)

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        accel = self._read_vector(self.acc_address, ACC_DATA) * self.acc_scale
        gyro = self._read_vector(self.gyro_address, GYRO_DATA) * self.gyro_scale
        return accel, gyro

    def close(self):
        self.bus.close()


class Bmi088Imu:
    """Threaded BMI088 + Mahony filter with SerialImu's interface (quaternion, angular_velocity in deg/s,
    acceleration, run_forever, stop), in the robot base frame.

    Keep the robot still for calibration_s after run_forever(): the gyro bias is the mean rate over that time and the
    filter starts level with the mean acceleration.
    """

    def __init__(self, bus: int = 3, axes: tuple[str, str, str] = ("+x", "+y", "+z"), rate_hz: float = 400.0,
                 calibration_s: float = 2.0, kp: float = 2.0, ki: float = 0.02, accel_gate: float = 0.3,
                 **sensor_kwargs):
        self.bus_number, self.sensor_kwargs = bus, sensor_kwargs
        self.rotation = axes_to_matrix(axes)
        self.period = 1.0 / rate_hz
        self.calibration_s = calibration_s
        self.filter = MahonyFilter(kp, ki, accel_gate)
        self.gyro_bias = np.zeros(3)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.angular_velocity = np.zeros(3, dtype=np.float32)
        self.acceleration = np.zeros(3, dtype=np.float32)
        self.loop_rate = 0.0
        self.is_stopped = threading.Event()
        self.is_calibrated = threading.Event()
        self.thread = None

    def _read_base(self, sensor: Bmi088) -> tuple[np.ndarray, np.ndarray]:
        accel, gyro = sensor.read()
        return self.rotation @ accel, self.rotation @ gyro

    def _run(self):
        sensor = Bmi088(self.bus_number, **self.sensor_kwargs)
        try:
            samples = []
            end = time.perf_counter() + self.calibration_s
            while time.perf_counter() < end:
                samples.append(self._read_base(sensor))
                time.sleep(self.period)
            accel_mean = np.mean([s[0] for s in samples], axis=0)
            self.gyro_bias = np.mean([s[1] for s in samples], axis=0)
            self.filter.initialize(accel_mean)
            self.is_calibrated.set()
            last = time.perf_counter()
            next_time = last
            count, count_start = 0, last
            while not self.is_stopped.is_set():
                accel, gyro = self._read_base(sensor)
                now = time.perf_counter()
                omega = self.filter.update(gyro - self.gyro_bias, accel, min(now - last, 0.02))
                last = now
                self.quaternion[:] = self.filter.q
                self.angular_velocity[:] = np.rad2deg(omega)
                self.acceleration[:] = accel
                count += 1
                if now - count_start >= 1.0:
                    self.loop_rate, count, count_start = count / (now - count_start), 0, now
                next_time += self.period
                time.sleep(max(next_time - time.perf_counter(), 0.0))
        finally:
            sensor.close()

    def run_forever(self):
        """Start the reading thread (returns immediately; wait on is_calibrated before using the outputs)."""
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.is_stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
