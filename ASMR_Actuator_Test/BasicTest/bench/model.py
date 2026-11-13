"""ODrive 0.5.1 위치/토크 제어기 + 감속기 + 관절 관성·마찰 모델.

시뮬레이터(sim.py)와 스텝 응답 예측(05, plot_step)이 같은 코드를 쓴다.
제어기는 펌웨어 controller.cpp의 순서를 단순화해 따른다(엔코더 PLL·전류 루프 지연은 무시).
"""
import math
from dataclasses import dataclass

from . import params

CONTROL_TORQUE = 1
CONTROL_VELOCITY = 2
CONTROL_POSITION = 3
DT = 1.0 / 8000.0  # ODrive 제어 주기


@dataclass
class DriveConfig:
    pos_gain: float
    vel_gain: float
    vel_integrator_gain: float = 0.0
    vel_limit: float = params.VEL_LIMIT
    current_lim: float = params.CURRENT_LIM
    torque_constant: float = 0.067
    torque_lim: float = math.inf
    control_mode: int = CONTROL_POSITION
    enable_vel_limit: bool = True
    enable_current_mode_vel_limit: bool = True


@dataclass
class Plant:
    kt: float                 # 실제 토크 상수 (모터축)
    inertia: float            # 관절축 관성 = armature + 부하
    coulomb: float            # 관절축 쿨롱 마찰
    viscous: float            # 관절축 점성 마찰
    ratio: float = params.GEAR_RATIO

    @classmethod
    def for_motor(cls, motor, extra_inertia=0.0):
        m = params.MOTORS[motor]
        return cls(kt=m["kt"], inertia=m["armature"] + extra_inertia,
                   coulomb=m["coulomb"], viscous=m["viscous"])


def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Actuator:
    """관절 상태(theta [rad], omega [rad/s])와 제어기 적분기를 가진 모델."""

    def __init__(self, plant, theta=0.0):
        self.plant = plant
        self.theta = theta
        self.omega = 0.0
        self.integrator = 0.0
        self.iq = 0.0
        self.torque_cmd = 0.0

    # 모터축 단위로 본 상태
    @property
    def pos_turn(self):
        return params.out_rad_to_turn(self.theta)

    @property
    def vel_turn(self):
        return params.out_rad_to_turn(self.omega)

    def controller(self, cfg, input_pos, input_vel=0.0, input_torque=0.0, dt=DT):
        """모터축 토크 명령 [Nm]. 펌웨어 Controller::update() 순서."""
        pos, vel = self.pos_turn, self.vel_turn
        max_torque = min(cfg.current_lim * cfg.torque_constant, cfg.torque_lim)

        if cfg.control_mode >= CONTROL_POSITION:
            vel_des = input_vel + cfg.pos_gain * (input_pos - pos)
        else:
            vel_des = input_vel
        if cfg.enable_vel_limit:
            vel_des = _clamp(vel_des, -cfg.vel_limit, cfg.vel_limit)

        torque = input_torque
        v_err = 0.0
        if cfg.control_mode >= CONTROL_VELOCITY:
            v_err = vel_des - vel
            torque += cfg.vel_gain * v_err + self.integrator
        elif cfg.enable_current_mode_vel_limit:
            t_max = (cfg.vel_limit - vel) * cfg.vel_gain
            t_min = (-cfg.vel_limit - vel) * cfg.vel_gain
            torque = _clamp(torque, t_min, t_max)

        limited = abs(torque) > max_torque
        torque = _clamp(torque, -max_torque, max_torque)

        if cfg.control_mode < CONTROL_VELOCITY or cfg.vel_integrator_gain == 0.0:
            self.integrator = 0.0
        elif limited:
            self.integrator *= 0.99  # 펌웨어의 anti-windup
        else:
            self.integrator += cfg.vel_integrator_gain * dt * v_err
        return torque

    def step(self, cfg, input_pos, input_vel=0.0, input_torque=0.0, ext_torque=0.0, dt=DT):
        """dt 만큼 진행. ext_torque는 관절축 외력 토크 [Nm]."""
        p = self.plant
        self.torque_cmd = self.controller(cfg, input_pos, input_vel, input_torque, dt)
        self.iq = self.torque_cmd / cfg.torque_constant
        tau = p.ratio * self.iq * p.kt + ext_torque - p.viscous * self.omega

        if abs(self.omega) < 1e-6:
            if abs(tau) <= p.coulomb:
                self.omega = 0.0
                return
            tau -= math.copysign(p.coulomb, tau)
        else:
            tau -= math.copysign(p.coulomb, self.omega)

        new_omega = self.omega + tau / p.inertia * dt
        if self.omega != 0.0 and new_omega * self.omega < 0.0:
            new_omega = 0.0  # 마찰로 0을 지나치지 않게
        self.omega = new_omega
        self.theta += self.omega * dt


def simulate(cfg, plant, t, ref_turn, pos0_turn, dt=DT):
    """기준 입력 ref_turn(t)를 따라가는 응답. t는 증가하는 시각 배열. 반환: 같은 시각의 pos [turn]."""
    act = Actuator(plant, theta=params.turn_to_out_rad(pos0_turn))
    out = []
    sim_t = t[0]
    k = 0
    for i, ti in enumerate(t):
        while sim_t < ti:
            while k + 1 < len(t) and t[k + 1] <= sim_t:
                k += 1
            act.step(cfg, ref_turn[k], dt=dt)
            sim_t += dt
        out.append(act.pos_turn)
    return out
