"""장치 없이 GUI를 시험하기 위한 가상 로봇 (python-can virtual 버스 위의 ODrive 0.5.1 노드 여러 개).

물리는 같은 MJCF를 MuJoCo로 돌린다(몸통 고정, 중력 있음). 각 관절은 드라이버처럼 모터축 위치·속도 PD
(코스 xdrive_config 게인, 전류 한계 18 A)로 움직인다. 실물을 흉내 내려고 감춘 값을 둔다:
  - 엔코더 원점: 코스 펌웨어처럼 부팅 때 pos_estimate가 0~1 turn 사이 아무 값 (관절 기준 24도마다 반복)
  - 방향: flips에 넣은 관절은 모터 +가 시뮬 관절 -  (GUI의 방향 확인 연습용)
  - Iq 부호: 설정 파일의 torque_sign과 같게 (= 백업의 motor.config.direction)
  - CAN 버스: 채널을 두 개 주면 실물처럼 왼다리는 버스 1, 오른다리는 버스 2에만 있다
    (자기 버스로 온 요청에만 답하고, heartbeat도 자기 버스로만 보냄)
실측을 대신하지 않는다.
"""
import math
import random
import struct
import threading
import time

import can
import mujoco
import numpy as np

from bench import can_simple as cs
from bench import params

from . import protocol as P
from .sim import joint_infos, load_model


class _Axis:
    def __init__(self, cfg_joint, info, flip, rng):
        self.j = cfg_joint
        self.info = info
        m = params.MOTORS[cfg_joint.motor]
        self.dir = -1.0 if flip else 1.0
        self.torque_sign = float(cfg_joint.torque_sign)
        self.pos_gain, self.vel_gain = m["course_gains"]
        self.kt = m["kt"]
        self.current_lim = params.CURRENT_LIM
        self.vel_limit = params.VEL_LIMIT
        self.base = rng.random()           # q = 0일 때의 부팅 pos_estimate (0~1 turn)
        self.state = P.STATE_IDLE
        self.error = 0
        self.input_pos = 0.0
        self.vel_ff = 0.0
        self.torque_ff = 0.0
        self.control_mode = P.CONTROL_POSITION
        self.current = 0.0

    def motor_pos(self, q):
        return self.base + self.dir * q * params.GEAR_RATIO / (2 * math.pi)

    def motor_vel(self, qd):
        return self.dir * qd * params.GEAR_RATIO / (2 * math.pi)


class VirtualRobot(threading.Thread):
    def __init__(self, channels, cfg, mjcf_path, flips=(), nodes=None, seed=None, powered=()):
        """channels: python-can virtual 채널 이름 (문자열 하나 또는 목록 1~2개).
        powered: 처음부터 closed loop로 켜져 있는 node (다른 스크립트가 토크를 켜 둔 채 끝난 상황 흉내)."""
        super().__init__(name="virtual-robot", daemon=True)
        self.channels = [channels] if isinstance(channels, str) else list(channels)
        two = len(self.channels) >= 2
        # 버스 2개면 왼다리 = 버스 0, 오른다리 = 버스 1
        self.bus_of_node = {j.node: (1 if two and "_right_" in j.name else 0) for j in cfg.joints}
        self.model, _ = load_model(mjcf_path, visuals=False)
        self.model.opt.timestep = 0.001
        # 관절 범위 끝 = 기구 끝. MuJoCo 기본(시정수 0.02 s)은 손으로 밀면 5~9°나 파고들어서
        # 3D 프린팅 스톱처럼 단단하게 (시정수 0.004 s, dt의 4배라 안정)
        self.model.jnt_solref[:, 0] = 0.004
        self.model.jnt_solref[:, 1] = 1.0
        # MJCF actuator는 끄고 드라이버 PD를 qfrc_applied로 건다
        self.model.actuator_gainprm[:, 0] = 0.0
        self.model.actuator_biasprm[:, 1:3] = 0.0
        self.data = mujoco.MjData(self.model)
        rng = random.Random(seed)
        infos = joint_infos(self.model, cfg)
        flips = set(flips or ())
        self.axes = {}
        for j in cfg.joints:
            if infos.get(j.name) is None or (nodes is not None and j.node not in nodes):
                continue
            self.axes[j.node] = _Axis(j, infos[j.name], j.name in flips or j.short in flips, rng)
        # 부팅 자세: 약간 흔들린 0 자세 (+-3도)
        for ax in self.axes.values():
            self.data.qpos[ax.info.qadr] = math.radians(rng.uniform(-3, 3))
            ax.base -= math.floor(ax.motor_pos(self.data.qpos[ax.info.qadr]))   # 부팅 값은 0~1 turn으로 감김
        # 기구 끝은 MJCF 범위가 아니라 설정의 stop_lo/hi_deg (메쉬 충돌 검사 값).
        # 기구 끝이 없는 쪽은 실물에서 반대쪽 다리·배선에 막히는 정도로 MJCF 범위 끝 + 30°에서 막는다.
        extra = math.radians(30)
        for ax in self.axes.values():
            j, jid = ax.j, ax.info.jid
            mlo = ax.info.lo if ax.info.lo is not None else -math.pi
            mhi = ax.info.hi if ax.info.hi is not None else math.pi
            lo = mlo - extra if j.stop_lo_deg is None else math.radians(j.stop_lo_deg)
            hi = mhi + extra if j.stop_hi_deg is None else math.radians(j.stop_hi_deg)
            self.model.jnt_range[jid] = (lo, hi)
            self.model.jnt_limited[jid] = 1
        for n in powered or ():
            ax = self.axes.get(int(n))
            if ax is not None:
                ax.input_pos = ax.motor_pos(self.data.qpos[ax.info.qadr])
                ax.state = P.STATE_CLOSED_LOOP
        mujoco.mj_forward(self.model, self.data)
        self.hand_torque = {}        # node -> 관절 토크 [Nm]: 사람이 손으로 미는 것 흉내 (시험용)
        self._halt = threading.Event()
        self._lock = threading.Lock()

    def stop(self):
        self._halt.set()
        self.join(2.0)

    # ---- 물리
    def _step_physics(self):
        d = self.data
        for ax in self.axes.values():
            i = ax.info
            if ax.state == P.STATE_CLOSED_LOOP:
                p = ax.motor_pos(d.qpos[i.qadr])
                v = ax.motor_vel(d.qvel[i.dadr])
                vel_sp = ax.pos_gain * (ax.input_pos - p) + ax.vel_ff
                vel_sp = max(-ax.vel_limit, min(ax.vel_limit, vel_sp))
                torque = ax.vel_gain * (vel_sp - v) + ax.torque_ff
                cur = max(-ax.current_lim, min(ax.current_lim, torque / ax.kt))
                ax.current = cur
                d.qfrc_applied[i.dadr] = ax.dir * cur * ax.kt * params.GEAR_RATIO
            else:
                ax.current = 0.0
                d.qfrc_applied[i.dadr] = 0.0
            d.qfrc_applied[i.dadr] += self.hand_torque.get(ax.j.node, 0.0)
        mujoco.mj_step(self.model, d)

    # ---- CAN
    def _reply(self, bus, node, cmd, data):
        bus.send(can.Message(arbitration_id=cs.arb_id(node, cmd), data=data.ljust(8, b"\x00"),
                             is_extended_id=False))

    def _handle(self, bus, idx, msg):
        node, cmd = cs.split_id(msg.arbitration_id)
        ax = self.axes.get(node)
        if ax is None or self.bus_of_node.get(node, 0) != idx:   # 그 노드가 없는 버스로 온 프레임
            return
        d = self.data
        if msg.is_remote_frame:
            if cmd == cs.GET_ENCODER_ESTIMATES:
                p = ax.motor_pos(d.qpos[ax.info.qadr]) + random.gauss(0, 2e-5)
                v = ax.motor_vel(d.qvel[ax.info.dadr]) + random.gauss(0, 2e-3)
                self._reply(bus, node, cmd, struct.pack("<ff", p, v))
            elif cmd == cs.GET_IQ:
                iq = ax.current * ax.torque_sign
                self._reply(bus, node, cmd, struct.pack("<ff", iq, iq + random.gauss(0, 0.03)))
            elif cmd == cs.GET_VBUS:
                self._reply(bus, node, cmd, struct.pack("<f", 24.0 + random.gauss(0, 0.03)))
            elif cmd in (cs.GET_MOTOR_ERROR, cs.GET_ENCODER_ERROR):
                self._reply(bus, node, cmd, struct.pack("<Q", 0))
            return
        data = bytes(msg.data).ljust(8, b"\x00")
        if cmd == cs.SET_AXIS_STATE:
            state = struct.unpack_from("<I", data)[0]
            if state == P.STATE_CLOSED_LOOP and not ax.error:
                ax.input_pos = ax.motor_pos(d.qpos[ax.info.qadr])   # 펌웨어처럼 현재 위치에서 시작
                ax.state = P.STATE_CLOSED_LOOP
            elif state == P.STATE_IDLE:
                ax.state = P.STATE_IDLE
        elif cmd == cs.SET_CONTROLLER_MODES:
            ax.control_mode, _ = struct.unpack_from("<ii", data)
        elif cmd == cs.SET_INPUT_POS:
            p, v, t = struct.unpack_from("<fhh", data)
            ax.input_pos, ax.vel_ff, ax.torque_ff = p, v * 1e-3, t * 1e-3
        elif cmd == cs.CLEAR_ERRORS:
            ax.error = 0
        elif cmd == cs.ESTOP:
            ax.error |= 0x4000
            ax.state = P.STATE_IDLE

    def run(self):
        buses = [can.Bus(interface="virtual", channel=ch, receive_own_messages=False) for ch in self.channels]
        dt = self.model.opt.timestep
        t0 = time.perf_counter()
        steps = 0
        next_hb = 0.0
        try:
            while not self._halt.is_set():
                got = False
                for i, bus in enumerate(buses):
                    msg = bus.recv(0)
                    while msg is not None:
                        got = True
                        if not msg.is_extended_id:
                            self._handle(bus, i, msg)
                        msg = bus.recv(0)
                if not got:
                    msg = buses[0].recv(0.001) if len(buses) == 1 else None
                    if msg is not None and not msg.is_extended_id:
                        self._handle(buses[0], 0, msg)
                    elif len(buses) > 1:
                        time.sleep(0.0005)
                now = time.perf_counter() - t0
                n = int(now / dt) - steps
                if n > 200:
                    steps += n - 200
                    n = 200
                for _ in range(max(0, n)):
                    self._step_physics()
                steps += max(0, n)
                if now >= next_hb:
                    for node, ax in self.axes.items():
                        self._reply(buses[self.bus_of_node.get(node, 0)], node, cs.HEARTBEAT,
                                    struct.pack("<IB3x", ax.error, ax.state))
                    next_hb = now + 0.1
        finally:
            for bus in buses:
                bus.shutdown()
