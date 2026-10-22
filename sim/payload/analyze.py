"""Static joint torques and centre of mass in the stand, crouch and one-leg poses (static_report).
final_check.py uses it for the statics table.

    uv run python sim/payload/analyze.py      # self-check (fast)
"""
import mujoco
import numpy as np

import models
from poses import CROUCH, leg_q, place, sole, sole_half

LEG = [f"leg_{s}_{j}_joint" for s in ("left", "right")
       for j in ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")]


# ---------- statics ----------
def statics(m, qpos, feet):
    """Joint torques that hold qpos with zero velocity, the given feet in contact (6D wrench at each sole
    centre). With two feet the split is not unique: take the split with the smallest sum of squared torques."""
    d = mujoco.MjData(m)
    d.qpos[:] = qpos
    mujoco.mj_forward(m, d)
    rhs = d.qfrc_bias.copy()  # gravity (velocity is zero)
    J = []
    for s in feet:
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jp, jr, sole(m, d, s), m.body(f"leg_{s}_ankle_roll").id)
        J += [jp, jr]
    J = np.vstack(J)  # (6 * feet) x nv
    A, b = J[:, :6].T, rhs[:6]  # base rows: contact wrenches carry the whole robot
    f = np.linalg.lstsq(A, b, rcond=None)[0]
    if len(feet) == 2:
        N = np.linalg.svd(A)[2][6:].T  # null space of A: load shift and internal forces between the feet
        r = rhs[6:] - J[:, 6:].T @ f
        f = f + N @ np.linalg.lstsq(J[:, 6:].T @ N, r, rcond=None)[0]
    assert np.allclose(A @ f, b, atol=1e-6)
    tau = rhs[6:] - J[:, 6:].T @ f
    names = [m.joint(m.dof_jntid[i]).name for i in range(6, m.nv)]
    cop_ok = True
    for k, s in enumerate(feet):
        F, M = f[6 * k:6 * k + 3], f[6 * k + 3:6 * k + 6]
        hx, hy = sole_half(m, s)
        cop_ok &= F[2] > 0 and abs(-M[1] / F[2]) <= hx and abs(M[0] / F[2]) <= hy
    return dict(zip(names, tau)), bool(cop_ok), d


def com_and_support(m, d, feet=("left", "right")):
    """Whole-body CoM, support centre (mean of the sole centres) and the sole x range, world frame."""
    c = d.subtree_com[1].copy()
    p = np.array([sole(m, d, s) for s in feet])
    hx = sole_half(m, feet[0])[0]
    return c, p.mean(0), (p[:, 0].min() - hx, p[:, 0].max() + hx)


def single_leg(m, side):
    """Sway the pelvis over the stance foot (bisection on the sway angle) and hold all weight on it."""
    sign = -1 if side == "left" else 1
    lim = min(m.joint("leg_left_hip_roll_joint").range[1], -m.joint("leg_left_hip_roll_joint").range[0])
    lo, hi = 0.0, lim
    for _ in range(30):
        a = (lo + hi) / 2
        q = place(m, leg_q(models.STAND, sign * a), stance=side)
        d = mujoco.MjData(m)
        d.qpos[:] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        gap = (d.subtree_com[1][1] - sole(m, d, side)[1]) * sign  # > 0: CoM still on the inner side
        lo, hi = (a, hi) if gap > 0 else (lo, a)
    tau, ok, d = statics(m, q, [side])
    return tau, ok, a, d


def static_report(variant):
    m = models.load(variant) if isinstance(variant, str) else variant
    out = {"mass": float(m.body_subtreemass[1])}
    q = place(m, leg_q(models.STAND))
    tau, ok, d = statics(m, q, ["left", "right"])
    c, centre, (x0, x1) = com_and_support(m, d)
    out.update(com=c.tolist(), com_dx=float(c[0] - centre[0]), com_dy=float(c[1] - centre[1]),
               support_x=[float(x0 - centre[0]), float(x1 - centre[0])], com_h=float(c[2]))
    out["stand"] = {"tau": tau, "cop_ok": ok}
    tau, ok, d = statics(m, place(m, leg_q(CROUCH)), ["left", "right"])
    out["crouch"] = {"tau": tau, "cop_ok": ok, "com_dx": float(com_and_support(m, d)[0][0] - com_and_support(m, d)[1][0])}
    for side in ("left", "right"):
        tau, ok, a, d = single_leg(m, side)
        out[f"single_{side}"] = {"tau": tau, "cop_ok": ok, "sway": a}
    for k in ("stand", "crouch", "single_left", "single_right"):
        t = out[k]["tau"]
        j = max(LEG, key=lambda n: abs(t[n]))
        out[k].update(peak=abs(t[j]), peak_joint=j)
    out["single"] = max((out["single_left"], out["single_right"]), key=lambda r: r["peak"])
    return out


# ---------- check ----------
def self_check():
    models.self_check()
    for v in ("biped",):  # symmetric robot
        r = static_report(v)
        t = r["stand"]["tau"]
        for j in LEG:  # symmetric pose, symmetric robot: left and right torques equal in size
            o = j.replace("left", "right") if "left" in j else j.replace("right", "left")
            assert abs(abs(t[j]) - abs(t[o])) < 0.05, (v, j, t[j], t[o])
        # the feet carry the whole weight: total vertical contact force = m g (checked inside statics via A f = b)
        assert r["stand"]["cop_ok"] and abs(r["com_dy"]) < 0.005, (v, r["com_dy"])
    print("self-check statics: left/right torques symmetric in the stand pose, CoP inside the feet")


if __name__ == "__main__":
    self_check()
