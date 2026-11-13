"""관절 목표(유지 / 부드러운 이동 궤적). 실물 스트리밍(CAN 스레드)과 시뮬(메인 스레드)이 같은 목표를 읽는다.

한 관절의 목표는 Target 하나다. 딕셔너리 항목을 통째로 바꾸므로 스레드 사이에 잠금 없이 읽어도 된다.
"""
import math


class Target:
    """q0 -> q1 최소 저크 궤적 (T = 0이면 q1 유지). real=True면 실물에도 보내는 목표."""

    __slots__ = ("q0", "q1", "t0", "T", "real", "t_created")

    def __init__(self, q0, q1, t0, T, real, t_created=None):
        self.q0, self.q1, self.t0, self.T, self.real = float(q0), float(q1), float(t0), float(T), bool(real)
        self.t_created = t0 if t_created is None else t_created

    @property
    def t_end(self):
        return self.t0 + self.T

    def eval(self, t):
        if self.T <= 0.0 or t >= self.t_end:
            return self.q1
        if t <= self.t0:
            return self.q0
        s = (t - self.t0) / self.T
        h = s * s * s * (10.0 + s * (-15.0 + 6.0 * s))
        return self.q0 + (self.q1 - self.q0) * h

    def moving(self, t):
        return t < self.t_end


def move_duration(dq, max_speed):
    """최소 저크 궤적의 최고 속도가 max_speed [rad/s]가 되는 시간 (최소 0.25 s)."""
    return max(0.25, 1.875 * abs(dq) / max(max_speed, 1e-6))


class Commander:
    def __init__(self, names):
        self._t = {n: None for n in names}

    def get(self, name):
        return self._t.get(name)

    def active_names(self):
        return [n for n, t in self._t.items() if t is not None]

    def clear(self, name):
        self._t[name] = None

    def clear_all(self):
        for n in self._t:
            self._t[n] = None

    def hold(self, name, q, t, real):
        self._t[name] = Target(q, q, t, 0.0, real)
        return self._t[name]

    def move(self, name, q_to, t, max_speed):
        cur = self._t[name]
        q_from = cur.eval(t)
        T = move_duration(q_to - q_from, max_speed)
        self._t[name] = Target(q_from, q_to, t, T, cur.real, cur.t_created)
        return self._t[name]

    def target_q(self, name, t):
        tg = self._t.get(name)
        return None if tg is None else tg.eval(t)

    def final_q(self, name):
        tg = self._t.get(name)
        return None if tg is None else tg.q1


def deg(x):
    return math.degrees(x)


def rad(x):
    return math.radians(x)
