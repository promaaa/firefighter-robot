"""장치 없이 스크립트 흐름을 리허설하기 위한 가짜 드라이버.

FakeODrive는 스크립트가 쓰는 odrv0 속성만 흉내 낸다. 물리는 model.Actuator (학습 모델 값)이고,
드라이버 설정은 코스 xdrive_config.txt 값으로 시작한다. 실측 대용이 아니다.
"""
import math
import random
import struct
import threading
import time
from types import SimpleNamespace as NS

from . import can_simple as cs
from . import model, params


class _Prop:
    """읽을 때마다 함수를 부르는 속성 (fibre의 원격 속성처럼)."""

    def __init__(self, get, set=None):
        self.get, self.set_ = get, set


class _Obj:
    def __init__(self, **kw):
        object.__setattr__(self, "_props", {})
        for k, v in kw.items():
            setattr(self, k, v)

    def __getattribute__(self, name):
        props = object.__getattribute__(self, "_props")
        if name in props:
            return props[name].get()
        return object.__getattribute__(self, name)

    def __setattr__(self, name, value):
        if isinstance(value, _Prop):
            self._props[name] = value
        elif name in self._props:
            if self._props[name].set_ is None:
                raise AttributeError(f"{name} is read-only")
            self._props[name].set_(value)
        else:
            object.__setattr__(self, name, value)


class FakeODrive(_Obj):
    def __init__(self, motor="6512"):
        super().__init__()
        m = params.MOTORS[motor]
        self._motor = motor
        self.ext_torque = 0.0  # 관절축 외력 [Nm]. 스크립트가 "추를 걸었다"는 시점에 바꾼다.
        self._act = model.Actuator(model.Plant.for_motor(motor),
                                   theta=params.turn_to_out_rad(random.random()))  # 코스 fw: 부팅 시 0~1 turn
        self._state = 1
        self._error = 0
        self._input_pos = self._act.pos_turn
        self._input_vel = 0.0
        self._input_torque = 0.0

        pg, vg = m["course_gains"]
        self.serial_number = 0x2061384D4E50
        self.fw_version_major, self.fw_version_minor, self.fw_version_revision = 0, 5, 1
        self.hw_version_major, self.hw_version_minor = 3, 6
        # 모터 보정 흉내용: 실제 상저항·인덕턴스, 보정 중 DC 부하 [W], 전원 경로 저항 [Ohm]
        self._R_true = m.get("phase_R_ref") or 0.15
        self._L_true = m.get("phase_L_ref") or 60e-6
        self._load_W = 0.0
        self._path_R = 0.04
        self.vbus_voltage = _Prop(lambda: 24.0 - self._load_W / 24.0 * self._path_R + random.gauss(0, 0.02))
        self.config = NS(brake_resistance=0.0, dc_max_negative_current=-3.0, dc_max_positive_current=12.0,
                         dc_bus_overvoltage_trip_level=26.0, dc_bus_undervoltage_trip_level=10.0,
                         max_regen_current=0.0, enable_brake_resistor=False)
        self.can = NS(config=NS(baud_rate=params.CAN_BITRATE, protocol=1))

        ccfg = NS(control_mode=3, input_mode=1, pos_gain=pg, vel_gain=vg, vel_integrator_gain=0.0,
                  vel_limit=50.0, enable_vel_limit=True, enable_current_mode_vel_limit=True,
                  enable_overspeed_error=True, vel_limit_tolerance=1.2, vel_ramp_rate=1.0)
        mcfg = NS(current_lim=18.0, torque_constant=m["kt"], torque_lim=math.inf, pre_calibrated=True,
                  pole_pairs=14, motor_type=0, calibration_current=5.0, requested_current_range=25.0,
                  resistance_calib_max_voltage=2.0, phase_resistance=self._R_true,
                  phase_inductance=self._L_true)
        self._ccfg, self._mcfg = ccfg, mcfg

        act = self._act
        encoder = _Obj(error=0, config=NS(mode=257, cpr=16384, bandwidth=3000.0, pre_calibrated=True,
                                          abs_spi_cs_gpio_pin=7, calib_range=10.0))
        encoder.pos_estimate = _Prop(lambda: act.pos_turn + random.gauss(0, 2e-5))
        encoder.vel_estimate = _Prop(lambda: act.vel_turn + random.gauss(0, 2e-3))
        motor_ = _Obj(error=0, config=mcfg, current_control=_Obj())
        motor_.current_control.Iq_measured = _Prop(lambda: act.iq + random.gauss(0, 0.03))
        motor_.current_control.Iq_setpoint = _Prop(lambda: act.iq)
        controller = _Obj(error=0, config=ccfg)
        controller.input_pos = _Prop(lambda: self._input_pos, lambda v: setattr(self, "_input_pos", float(v)))
        controller.input_vel = _Prop(lambda: self._input_vel, lambda v: setattr(self, "_input_vel", float(v)))
        controller.input_torque = _Prop(lambda: self._input_torque,
                                        lambda v: setattr(self, "_input_torque", float(v)))
        controller.pos_setpoint = _Prop(lambda: self._input_pos)
        axis = _Obj(motor=motor_, encoder=encoder, controller=controller,
                    config=NS(can_node_id=1, can_heartbeat_rate_ms=100, startup_closed_loop_control=False,
                              startup_encoder_offset_calibration=False, enable_watchdog=False,
                              watchdog_timeout=0.0))
        axis.error = _Prop(lambda: self._error, lambda v: setattr(self, "_error", int(v)))
        axis.current_state = _Prop(lambda: self._state)
        axis.requested_state = _Prop(lambda: 0, self._request_state)
        self.axis0 = axis
        self.axis1 = NS(config=NS(can_node_id=63, can_heartbeat_rate_ms=0))

        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _request_state(self, state):
        with self._lock:
            if state == 4:
                if self._state != 4:
                    self._state = 4
                    threading.Thread(target=self._motor_calibration, daemon=True).start()
                return
            if state == 8:
                self._input_pos = self._act.pos_turn  # 펌웨어처럼 현재 위치를 목표로
                self._act.integrator = 0.0
            self._state = 8 if state == 8 else 1

    def _motor_calibration(self):
        """AXIS_STATE_MOTOR_CALIBRATION 흉내 (motor.cpp): 저항 3 s, 인덕턴스 약 1 s, 그다음 IDLE."""
        mc = self._mcfg
        cur = mc.calibration_current
        r = self._R_true * (1 + random.gauss(0, 0.005))
        if cur * r > mc.resistance_calib_max_voltage:
            time.sleep(0.05)
            self.axis0.motor.error = 0x1  # MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE
            self._error = 0x40            # AXIS_ERROR_MOTOR_FAILED
            self._state = 1
            return
        self._load_W = 1.5 * cur ** 2 * r
        time.sleep(3.0)
        self._load_W = 0.0
        time.sleep(1.0)
        if self._state == 4:
            mc.phase_resistance = r
            mc.phase_inductance = self._L_true * (1 + random.gauss(0, 0.01))
            self._state = 1

    def save_configuration(self):
        raise RuntimeError("[SIM] 이 스크립트들은 save_configuration()을 부르지 않습니다.")

    def _cfg(self):
        c, m = self._ccfg, self._mcfg
        return model.DriveConfig(c.pos_gain, c.vel_gain, c.vel_integrator_gain, c.vel_limit, m.current_lim,
                                 m.torque_constant, m.torque_lim, c.control_mode, c.enable_vel_limit,
                                 c.enable_current_mode_vel_limit)

    def _run(self):
        last = time.perf_counter()
        while True:
            time.sleep(0.001)
            now = time.perf_counter()
            n = int((now - last) / model.DT)
            if n > 400:  # 너무 밀리면 따라잡지 않음
                n, last = 400, now
            else:
                last += n * model.DT
            with self._lock:
                cfg = self._cfg()
                for _ in range(n):
                    if self._state == 8:
                        self._act.step(cfg, self._input_pos, self._input_vel, self._input_torque, self.ext_torque)
                    else:
                        self._act.iq = 0.0
                        self._idle_step()

    def _idle_step(self):
        """IDLE: 모터 토크 없이 외력과 마찰만."""
        a, p = self._act, self._act.plant
        tau = self.ext_torque - p.viscous * a.omega
        if abs(a.omega) < 1e-6 and abs(tau) <= p.coulomb:
            a.omega = 0.0
            return
        tau -= math.copysign(p.coulomb, a.omega if a.omega else tau)
        a.omega += tau / p.inertia * model.DT
        a.theta += a.omega * model.DT


class _CanNode(threading.Thread):
    """가상 CAN 버스에서 FakeODrive 하나를 CAN Simple 노드로 응답시킨다."""

    def __init__(self, odrv, channel, node_id):
        super().__init__(daemon=True)
        import can
        self.odrv, self.node = odrv, node_id
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self._can = can

    def reply(self, cmd, data):
        self.bus.send(self._can.Message(arbitration_id=cs.arb_id(self.node, cmd), data=data.ljust(8, b"\x00"),
                                        is_extended_id=False))

    def run(self):
        o, ax = self.odrv, self.odrv.axis0
        next_hb = 0.0
        while True:
            now = time.perf_counter()
            if now >= next_hb:
                self.reply(cs.HEARTBEAT, struct.pack("<II", o._error, o._state))
                next_hb = now + 0.1
            msg = self.bus.recv(0.01)
            if msg is None:
                continue
            node, cmd = cs.split_id(msg.arbitration_id)
            if node != self.node:
                continue
            time.sleep(0.0003)  # 대략적인 버스·처리 시간
            if msg.is_remote_frame:
                if cmd == cs.GET_ENCODER_ESTIMATES:
                    self.reply(cmd, struct.pack("<ff", ax.encoder.pos_estimate, ax.encoder.vel_estimate))
                elif cmd == cs.GET_IQ:
                    self.reply(cmd, struct.pack("<ff", o._act.iq, ax.motor.current_control.Iq_measured))
                elif cmd == cs.GET_VBUS:
                    self.reply(cmd, struct.pack("<f", o.vbus_voltage))
                elif cmd in (cs.GET_MOTOR_ERROR, cs.GET_ENCODER_ERROR):
                    self.reply(cmd, struct.pack("<I", 0))
                continue
            d = bytes(msg.data).ljust(8, b"\x00")
            if cmd == cs.SET_AXIS_STATE:
                o._request_state(struct.unpack("<I", d[:4])[0])
            elif cmd == cs.SET_CONTROLLER_MODES:
                o._ccfg.control_mode, o._ccfg.input_mode = struct.unpack("<ii", d)
            elif cmd == cs.SET_INPUT_POS:
                p, v, t = struct.unpack("<fhh", d)
                o._input_pos, o._input_vel, o._input_torque = p, v * 1e-3, t * 1e-3
            elif cmd == cs.SET_INPUT_TORQUE:
                o._input_torque = struct.unpack("<f", d[:4])[0]
            elif cmd == cs.SET_VEL_LIMIT:
                o._ccfg.vel_limit = struct.unpack("<f", d[:4])[0]
            elif cmd == cs.CLEAR_ERRORS:
                o._error = 0
            elif cmd == cs.ESTOP:
                o._error |= 0x4000
                o._request_state(1)


def start_virtual_can(channel, node_id, motor):
    odrv = FakeODrive(motor)
    _CanNode(odrv, channel, node_id).start()
    return odrv
