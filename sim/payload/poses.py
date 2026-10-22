"""Leg poses and placement on the floor with flat feet."""
import mujoco
import numpy as np

CROUCH = {"hip_pitch": -0.42, "knee_pitch": 1.0, "ankle_pitch": -0.68}  # same hip+knee+ankle sum as STAND


def leg_q(pose, sway=0.0):
    """Joint dict for both legs. sway < 0 moves the pelvis over the left foot, sway > 0 over the right foot
    (hip roll and ankle roll of both legs turn by the same angle, so the legs form a parallelogram)."""
    q = {f"leg_{s}_{k}_joint": v for s in ("left", "right") for k, v in pose.items()}
    for s in ("left", "right"):
        q[f"leg_{s}_hip_roll_joint"], q[f"leg_{s}_ankle_roll_joint"] = sway, -sway
    return q


def foot_geom(m, side):
    return next(i for i in range(m.ngeom) if m.geom_bodyid[i] == m.body(f"leg_{side}_ankle_roll").id
                and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX)


def sole(m, d, side):
    """Centre of the sole (bottom face of the foot collision box), world frame. Valid when the foot is flat."""
    g = foot_geom(m, side)
    R, c, h = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g], m.geom_size[g]
    up = np.argmax(np.abs(R[2]))  # box axis that points up
    return c - R[:, up] * h[up] * np.sign(R[2, up])


def sole_half(m, side):
    """Half length (x) and half width (y) of the sole when the foot is flat (box axes: 0.036, 0.11, 0.02)."""
    h = m.geom_size[foot_geom(m, side)]
    return h[1], h[0]


def place(m, q, stance="left", gap=0.0):
    """qpos with joints q, base turned so the stance foot is flat, and base height so that the stance sole is at z = gap."""
    d = mujoco.MjData(m)
    d.qpos[3] = 1
    mujoco.mj_kinematics(m, d)
    b = d.body(f"leg_{stance}_ankle_roll")
    flat = b.xmat.reshape(3, 3).copy()  # foot orientation at zero joints: flat on the floor
    for j, v in q.items():
        d.joint(j).qpos = v
    mujoco.mj_kinematics(m, d)
    R = flat @ b.xmat.reshape(3, 3).T  # turns the current foot orientation back to flat
    mujoco.mju_mat2Quat(d.qpos[3:7], R.flatten())
    mujoco.mj_kinematics(m, d)
    d.qpos[2] += gap - sole(m, d, stance)[2]
    return d.qpos.copy()


def stand_qpos(m, pose, gap=0.001):
    return place(m, leg_q(pose), gap=gap)
