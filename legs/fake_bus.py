"""Fake XDrive MINI drives, for tests without hardware.

12 virtual drives answer on python-can 'virtual' buses (in-process: no sudo, no vcan) with the same CANSimple
messages as the real drives. They move the MuJoCo BHL biped of sim/sim2sim in real time. Each drive runs the
ODrive cascade of controller.cpp (position -> velocity -> torque, then the torque limit) at the physics rate (2 kHz).

The robot starts in ZERO_POSE and hangs from a hoist (a weld of the torso to the world).
A jig holds the joints still until the first drive enters closed loop. ready() lets go of the hoist.
"""
import struct
import threading
import time

import can
import mujoco
import numpy as np

import odrive_can as oc
from robot import JOINTS, N, NAMES, TORQUE_CONSTANT, TORQUE_LIMIT, TURNS_PER_RAD, ZERO_POSE, drive_settings, sim2sim

VBUS = 24.0         # V
VEL_LIMIT = 50.0    # turns/s, course xdrive_config.txt
HOIST_Z = 0.02      # m, torso height on the hoist above the standing height: the feet are off the floor
WATCHDOG = 1.0      # s, axis0.config.watchdog_timeout of bringup.py settings


class FakeRobot:
    def __init__(self, torque_limit=TORQUE_LIMIT, seed=0):
        spec = mujoco.MjSpec.from_file(str(sim2sim.MODELS / "bhl_biped.xml"))
        hoist = spec.add_equality()
        hoist.type, hoist.objtype, hoist.name1 = mujoco.mjtEq.mjEQ_WELD, mujoco.mjtObj.mjOBJ_BODY, "base"
        hoist.data = [0, 0, 0, 0, 0, -HOIST_Z, 1, 0, 0, 0, 1]  # pose of the world in the torso frame
        m = self.m = spec.compile()
        m.opt.timestep = sim2sim.PHYSICS_DT
        d = self.d = mujoco.MjData(m)
        assert [m.joint(f"leg_{n}_joint").qposadr[0] for n in NAMES] == list(range(7, 19))
        assert np.allclose(m.jnt_range[1:13], [j[4:6] for j in JOINTS], atol=1e-4), "JOINTS limits differ from the model"
        d.qpos[2], d.qpos[7:19] = HOIST_Z, ZERO_POSE
        mujoco.mj_forward(m, d)

        self.sign = np.array([j[3] for j in JOINTS], float)
        s = [drive_settings(j[2], torque_limit) for j in JOINTS]
        self.pos_gain, self.vel_gain = np.array([x["pos_gain"] for x in s]), np.array([x["vel_gain"] for x in s])
        self.tmax = np.array([x["current_lim"] * TORQUE_CONSTANT[j[2]] for x, j in zip(s, JOINTS)])  # Nm at the motor
        self.enc0 = np.random.default_rng(seed).random(12)  # hidden: encoder angle (turns) at joint angle 0
        self.turn0 = np.floor(self._motor_turns())          # power-on: the drive knows the angle in [0, 1) only
        self.state, self.err = np.full(12, oc.IDLE), np.zeros(12, int)
        self.cmode, self.vel_limit = np.full(12, oc.POSITION_CONTROL), np.full(12, VEL_LIMIT)
        self.inp = np.zeros((3, 12))  # input_pos, input_vel, input_torque
        self.fed = np.zeros(12)       # time of the last frame to each drive: any frame feeds the watchdog
        self.where = {(0 if i < 6 else 1, j[1]): i for i, j in enumerate(JOINTS)}  # (bus, node) -> joint
        self.jig, self.hoisted, self.t_release, self.min_height = True, True, None, np.inf

        self.channels = [f"fake{id(self)}-{c}" for c in ("can0", "can1")]
        self.drive_side = [can.Bus(interface="virtual", channel=c) for c in self.channels]
        self.lock, self.running = threading.Lock(), True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    # ---- what the host side uses
    def buses(self):
        return [oc.Bus(c, "virtual") for c in self.channels]

    def imu(self):
        """Quaternion [w, x, y, z] and angular velocity (rad/s) of the torso, like the real IMU."""
        with self.lock:
            return self.d.sensor("imu_quat").data.copy(), self.d.sensor("imu_gyro").data.copy()

    def truth(self):
        """True torso pose for the fake camera: position (x, y, z), quaternion [w, x, y, z] in the world, and the
        simulated time (s). The simulated time lags the wall clock when the computer is too slow."""
        with self.lock:
            return self.d.qpos[:3].copy(), self.d.qpos[3:7].copy(), float(self.d.time)

    def height(self):
        with self.lock:
            return float(self.d.site("imu").xpos[2])

    def ready(self):
        """Plays the operator: lower the robot (let go of the hoist), say "on the ground" 1 s later."""
        if self.hoisted:
            self.hoisted, self.t_release = False, time.monotonic()
            self.d.eq_active[0] = 0
        return time.monotonic() - self.t_release > 1.0

    def close(self):
        self.running = False
        self.thread.join()
        for b in self.drive_side:
            b.shutdown()

    # ---- the drives
    def _motor_turns(self):
        return self.enc0 + self.sign * self.d.qpos[7:19] * TURNS_PER_RAD

    def _pos(self):  # pos_estimate: continuous from the power-on angle
        return self._motor_turns() - self.turn0

    def _vel(self):
        return self.sign * self.d.qvel[6:18] * TURNS_PER_RAD

    def _handle(self, k, msg):
        node, cmd, vals = oc.decode(msg)
        i = self.where.get((k, node))
        if i is None:
            return
        self.fed[i] = self.now
        if msg.is_remote_frame:
            data = {oc.GET_ENCODER_ESTIMATES: lambda: struct.pack("<ff", self._pos()[i], self._vel()[i]),
                    oc.GET_VBUS_VOLTAGE: lambda: struct.pack("<ff", VBUS, 0.0)}.get(cmd)
            if data:
                self.drive_side[k].send(oc.frame(node, cmd, data()))
        elif cmd == oc.SET_AXIS_STATE:
            if vals[0] == oc.CLOSED_LOOP and not self.err[i]:
                self.state[i], self.inp[0, i], self.jig = oc.CLOSED_LOOP, self._pos()[i], False  # axis.cpp:335
            elif vals[0] == oc.IDLE:
                self.state[i] = oc.IDLE
        elif cmd == oc.SET_CONTROLLER_MODES:
            self.cmode[i] = vals[0]  # Limit: the input mode acts as PASSTHROUGH always
        elif cmd == oc.SET_INPUT_POS:
            self.inp[:, i] = vals
        elif cmd == oc.SET_INPUT_VEL:
            self.inp[1:, i] = vals
        elif cmd == oc.SET_INPUT_TORQUE:
            self.inp[2, i] = vals[0]
        elif cmd == oc.SET_VEL_LIMIT:
            self.vel_limit[i] = vals[0]
        elif cmd == oc.ESTOP:
            self.err[i] |= 1 << 14
            self.state[i] = oc.IDLE
        elif cmd == oc.CLEAR_ERRORS:
            self.err[i] = 0
        elif cmd == oc.REBOOT:
            self.turn0[i], self.state[i], self.err[i] = np.floor(self._motor_turns()[i]), oc.IDLE, 0

    def _step(self):
        """One physics step with the drive cascade (controller.cpp:245-329)."""
        on = self.state == oc.CLOSED_LOOP
        p, v = self._pos(), self._vel()
        vel_des = self.inp[1] + np.where(self.cmode == oc.POSITION_CONTROL, self.pos_gain * (self.inp[0] - p), 0.0)
        vel_des = np.clip(vel_des, -self.vel_limit, self.vel_limit)
        torque = self.inp[2] + np.where(self.cmode >= oc.VELOCITY_CONTROL, self.vel_gain * (vel_des - v), 0.0)
        torque = np.clip(torque, -self.tmax, self.tmax)
        over = on & (np.abs(v) > 1.2 * self.vel_limit)  # overspeed: CONTROLLER_FAILED, the drive goes idle
        self.err[over] |= 1 << 9
        self.state[over] = oc.IDLE
        with self.lock:
            self.d.ctrl[:] = np.where(on & ~over, self.sign * N * torque, 0.0)
            mujoco.mj_step(self.m, self.d)
        if not self.hoisted:
            self.min_height = min(self.min_height, self.d.site("imu").xpos[2])

    def _run(self):
        t0, t, beat = time.monotonic(), 0.0, 0.0
        while self.running:
            now = self.now = time.monotonic() - t0
            for k, b in enumerate(self.drive_side):
                while (msg := b.recv(0)) is not None:
                    self._handle(k, msg)
            late = now - self.fed > WATCHDOG  # axis.cpp:218-228. Also in idle, as in the firmware.
            self.err[late] |= 1 << 11
            self.state[late] = oc.IDLE
            for _ in range(20):  # at most 10 ms of physics between two reads of the buses
                if t >= now:
                    break
                if not self.jig:
                    self._step()
                t += self.m.opt.timestep
            if now >= beat:
                beat += 0.1  # heartbeat every 100 ms
                for (k, node), i in self.where.items():
                    self.drive_side[k].send(oc.frame(node, oc.HEARTBEAT, struct.pack("<II", self.err[i], self.state[i])))
            time.sleep(0.0005)
