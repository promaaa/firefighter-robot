"""Water jet as a projectile: pan/tilt solution, range and time of flight.

Optional quadratic drag: dv/dt = -g z - k |v| v, with k in 1/m (fit k from a measured range).
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from geometry import LEVEL, nozzle_exit, tilted

G = 9.81  # m/s^2


def v0_from_flow(q_m3s, d_m):
    """Exit speed from the pump flow Q (m^3/s) and the nozzle diameter d (m). 1 L/min = 1/60000 m^3/s."""
    return q_m3s / (math.pi * d_m ** 2 / 4)


def v0_from_horizontal_shot(x, h):
    """Exit speed from a horizontal shot at height h (m) that lands at distance x (m). No drag."""
    return x * math.sqrt(G / (2 * h))


@dataclass
class Aim:
    pan: float  # rad
    tilt: float  # rad
    in_range: bool
    reason: str  # "ok", "too far", "too close" (needs a tilt below the lower limit)
    max_range: float  # m, horizontal reach from the nozzle exit at the target height
    tof: float  # s, time of flight (nan when out of range)


def jet_path(v0, tilt, drag=0.0, z_stop=-10.0, dt=1e-3, t_max=10.0):
    """Jet in its vertical plane from the nozzle exit: list of (r, z, t), r horizontal. Stops below z_stop."""
    r, z, t = 0.0, 0.0, 0.0
    vr, vz = v0 * math.cos(tilt), v0 * math.sin(tilt)
    path = [(r, z, t)]
    while t < t_max and not (z < z_stop and vz < 0):
        s = math.hypot(vr, vz)
        vr -= drag * s * vr * dt
        vz -= (G + drag * s * vz) * dt
        r, z, t = r + vr * dt, z + vz * dt, t + dt
        path.append((r, z, t))
    return path


def _height_at(path, dist):
    """(z, t) where the path crosses the horizontal distance dist, or None when it never gets there."""
    for (r0, z0, t0), (r1, z1, t1) in zip(path, path[1:]):
        if r1 >= dist:
            a = (dist - r0) / (r1 - r0) if r1 > r0 else 0.0
            return z0 + a * (z1 - z0), t0 + a * (t1 - t0)
    return None


def _tilt_vacuum(dist, dz, v0):
    """Low-arc elevation to reach (dist, dz) without drag, or None when out of reach."""
    v2 = v0 * v0
    disc = v2 * v2 - G * (G * dist * dist + 2 * dz * v2)
    if disc < 0:
        return None
    return math.atan2(v2 - math.sqrt(disc), G * dist)


TABLE_DZ = -3.0  # m, the drag table covers targets down to 3 m below the nozzle exit
TABLE_STEP = 0.01  # m, distance step of the drag table


@lru_cache(maxsize=8)
def _drag_table(v0, drag, lo, hi):
    """Jet height z[i, j] and time t[i, j] at tilt angles[i] and distance j * TABLE_STEP.

    Built once for each (v0, drag, tilt limits), in about 0.3 s. Lookups then take microseconds.
    """
    angles = np.linspace(lo, hi, round(math.degrees(hi - lo) * 2) + 1)  # 0.5 deg steps
    paths = [np.array(jet_path(v0, a, drag, z_stop=TABLE_DZ)) for a in angles]
    dist = np.arange(0.0, max(p[-1, 0] for p in paths) + TABLE_STEP, TABLE_STEP)
    z = np.array([np.interp(dist, p[:, 0], p[:, 1], right=np.nan) for p in paths])
    t = np.array([np.interp(dist, p[:, 0], p[:, 2], right=np.nan) for p in paths])
    return angles, dist, z, t


def _column(table, dist):
    """Table values at distance dist for every tilt angle, nan outside the table."""
    j = dist / TABLE_STEP
    j0 = int(j)
    if j0 + 1 >= table.shape[1]:
        return np.full(table.shape[0], np.nan)
    return table[:, j0] + (j - j0) * (table[:, j0 + 1] - table[:, j0])


def _tilt_drag(dist, dz, v0, drag, lo, hi):
    """Low-arc elevation with drag: the lowest tilt in the table where the jet is at or above the target."""
    if dz < TABLE_DZ:
        return None
    angles, _, z, _ = _drag_table(v0, drag, lo, hi)
    f = np.nan_to_num(_column(z, dist) - dz, nan=-np.inf)  # jet height above the target, -inf = never gets there
    if f[0] > 0:
        return -math.pi / 2  # over the target even at the lowest tilt: the caller flags "too close"
    above = np.flatnonzero(f >= 0)
    if len(above) == 0:
        return None
    i = max(above[0], 1)
    if not np.isfinite(f[i - 1]):
        return float(angles[i])
    return float(angles[i - 1] + (angles[i] - angles[i - 1]) * -f[i - 1] / (f[i] - f[i - 1]))


def max_range(v0, dz, drag=0.0, tilt_limits=(-math.radians(40), math.radians(45))):
    """Horizontal reach from the nozzle exit for a target dz below (negative) or above it."""
    if drag == 0:
        h = -dz
        opt = math.atan2(v0, math.sqrt(max(v0 * v0 + 2 * G * h, 0.0)))
        t = min(opt, tilt_limits[1])
        vr, vz = v0 * math.cos(t), v0 * math.sin(t)
        disc = vz * vz + 2 * G * h
        return 0.0 if disc < 0 else vr * (vz + math.sqrt(disc)) / G
    _, dist, z, _ = _drag_table(v0, drag, *tilt_limits)
    ok = z >= dz
    if dz < TABLE_DZ or not ok.any():
        return 0.0
    last = ok.shape[1] - 1 - np.argmax(ok[:, ::-1], axis=1)  # last distance where each jet is still above dz
    return float(dist[last[ok.any(axis=1)]].max())


def solve_aim(cfg, target, v0, drag=0.0, iters=20, att=LEVEL):
    """Pan and tilt (servo angles) that put the jet on target (level robot frame).

    att = torso (roll, pitch): the jet direction is solved in the level frame, then turned into servo angles.
    Iterates because the nozzle exit moves with pan and tilt.
    """
    target = np.asarray(target, dtype=float)
    lo, hi = cfg["tilt_limits"]
    body_to_level, _ = tilted(att, np.eye(3), np.zeros(3))
    pan, tilt, jet = math.atan2(target[1], target[0]), 0.0, 0.0  # servo pan and tilt, jet elevation (level)
    reason = "ok"
    for _ in range(iters):
        e, _ = nozzle_exit(cfg, pan, tilt, att)
        dist = math.hypot(target[0] - e[0], target[1] - e[1])
        dz = target[2] - e[2]
        new = _tilt_vacuum(dist, dz, v0) if drag == 0 else _tilt_drag(dist, dz, v0, drag, lo, hi)
        reason = "ok" if new is not None else "too far"
        new = min(hi, math.radians(30)) if new is None else new
        jpan = math.atan2(target[1] - e[1], target[0] - e[0])
        d = body_to_level.T @ np.array([math.cos(new) * math.cos(jpan), math.cos(new) * math.sin(jpan), math.sin(new)])
        new_pan, new_tilt = math.atan2(d[1], d[0]), math.asin(max(-1.0, min(1.0, d[2])))
        if reason == "ok" and new_tilt < lo:
            reason, new_tilt = "too close", lo
        elif reason == "ok" and new_tilt > hi:
            reason, new_tilt = "too far", hi
        done = abs(new_tilt - tilt) < 1e-7 and abs(new_pan - pan) < 1e-7
        pan, tilt, jet = new_pan, new_tilt, new
        if done:
            break
    in_range = reason == "ok" and cfg["pan_limits"][0] <= pan <= cfg["pan_limits"][1]
    if reason == "ok" and not in_range:
        reason = "pan limit"
    tof = math.nan
    if in_range and drag == 0:
        tof = dist / (v0 * math.cos(jet))
    elif in_range:
        angles, _, _, times = _drag_table(v0, drag, lo, hi)
        tof = float(np.interp(jet, angles, _column(times, dist)))
    return Aim(pan, tilt, in_range, reason, max_range(v0, dz, drag, (lo, hi)), tof)
