"""Command-tracking protocol shared by the Isaac Lab and MuJoCo sim2sim evaluations.

Each trial starts from the standing pose, stands with a zero command for SETTLE_S, then holds one velocity command
for HOLD_S. Both simulators record the same trace every policy step: the torso CoM position, the base yaw and tilt,
and the joint torques. The metrics come from the last MEASURE_S of the hold:
  * vx, vy: mean torso CoM velocity in the heading (yaw) frame, as in the linear tracking reward,
  * wz: mean yaw rate,
  * height: mean torso CoM height, torque_rms: RMS joint torque over all joints.
A trial is a fall when the base tilts past the training termination angle; a fallen trial has no metrics.
"""

import csv
import math

import numpy as np

SETTLE_S = 2.0
HOLD_S = 8.0
MEASURE_S = 5.0
FALL_TILT_RAD = 0.78  # termination of the training task (bad_orientation)

# (name, (vx m/s, vy m/s, wz rad/s)), within the training command ranges
COMMANDS = [
    ("stand", (0.0, 0.0, 0.0)),
    ("forward", (0.3, 0.0, 0.0)),
    ("forward_fast", (0.5, 0.0, 0.0)),
    ("backward", (-0.3, 0.0, 0.0)),
    ("left", (0.0, 0.2, 0.0)),
    ("right", (0.0, -0.2, 0.0)),
    ("turn_left", (0.0, 0.0, 0.8)),
    ("turn_right", (0.0, 0.0, -0.8)),
]

FIELDS = ["sim", "obs_noise", "seed", "command", "cmd_vx", "cmd_vy", "cmd_wz", "fell", "fall_time",
          "vx", "vy", "wz", "height", "torque_rms"]


def command_at(t, cmd):
    """Command at time t (s) since the trial start."""
    return (0.0, 0.0, 0.0) if t < SETTLE_S - 1e-9 else cmd


def num_steps(policy_dt):
    return int(round((SETTLE_S + HOLD_S) / policy_dt))


def yaw_of(quat_wxyz):
    w, x, y, z = quat_wxyz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def tilt_of(quat_wxyz):
    """Angle between the base z axis and the world z axis."""
    w, x, y, z = quat_wxyz
    return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))


def trial_metrics(pos, yaw, tilt, torque, policy_dt):
    """Metrics of one trial from its trace (one row per policy step, recorded after the step).

    pos: (T, 3) torso CoM position, yaw: (T,), tilt: (T,), torque: (T, J).
    """
    pos, yaw, tilt, torque = map(np.asarray, (pos, yaw, tilt, torque))
    fallen = np.nonzero(tilt > FALL_TILT_RAD)[0]
    if len(fallen):
        return {"fell": 1, "fall_time": (fallen[0] + 1) * policy_dt, "vx": math.nan, "vy": math.nan,
                "wz": math.nan, "height": math.nan, "torque_rms": math.nan}
    start = len(yaw) - int(round(MEASURE_S / policy_dt))
    yaw_unwrapped = np.unwrap(yaw)
    step = np.diff(pos[start - 1:], axis=0)
    mid_yaw = 0.5 * (yaw_unwrapped[start - 1:-1] + yaw_unwrapped[start:])
    cos, sin = np.cos(mid_yaw), np.sin(mid_yaw)
    vx = (cos * step[:, 0] + sin * step[:, 1]) / policy_dt
    vy = (-sin * step[:, 0] + cos * step[:, 1]) / policy_dt
    return {
        "fell": 0,
        "fall_time": math.nan,
        "vx": float(vx.mean()),
        "vy": float(vy.mean()),
        "wz": float((yaw_unwrapped[-1] - yaw_unwrapped[start - 1]) / (MEASURE_S)),
        "height": float(pos[start:, 2].mean()),
        "torque_rms": float(np.sqrt(np.mean(torque[start:] ** 2))),
    }


def write_rows(path, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows):
    """Mean over seeds per command: falls, achieved velocities against the command."""
    print(f"{'command':14s} {'falls':>6s} {'vx':>14s} {'vy':>14s} {'wz':>14s} {'height':>7s} {'tau_rms':>7s}")
    for name, cmd in COMMANDS:
        trials = [r for r in rows if r["command"] == name]
        ok = [r for r in trials if not r["fell"]]

        def mean(key):
            return np.mean([r[key] for r in ok]) if ok else math.nan

        print(f"{name:14s} {sum(r['fell'] for r in trials):>3d}/{len(trials):<2d} "
              f"{mean('vx'):+6.3f}/{cmd[0]:+5.2f} {mean('vy'):+6.3f}/{cmd[1]:+5.2f} {mean('wz'):+6.3f}/{cmd[2]:+5.2f} "
              f"{mean('height'):7.3f} {mean('torque_rms'):7.2f}")
