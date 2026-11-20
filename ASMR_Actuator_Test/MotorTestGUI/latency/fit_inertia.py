"""계단 시험 원자료로 관절의 실효 관성 J와 armature를 추정한다 (오프라인, 로봇 불필요).

모델: 명령 지연 TD(0.55 ms, 측정) 뒤 목표가 바뀌고, PD 토크 Kp(q_tgt - q) - Kd q' 가 전류 1차 지연 ti를 거쳐
관성 J에 걸린다. 쿨롱 마찰 Fc = Kp x (목표 대비 끝 오차의 두 방향 평균). 앞 60 ms에 J와 ti를 격자 탐색으로 맞춘다.
링크 관성은 강의 MJCF에서 그 자세로 계산한다 (몸통 고정, 다른 관절 자유, armature 제외).
armature 추정 = J - 링크 관성.

  python latency/fit_inertia.py <logs/latency 안의 계단 폴더 이름> [...]

결과: 화면 표 + logs/latency/model_fit.json
"""
import csv
import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import latency as L  # noqa: E402

BASE = os.path.join(HERE, "..", "logs", "latency")
MJ = os.path.join(HERE, "..", "assets", "asmr2026-humanoid-v1", "mjcf", "asmr2026_humanoid.xml")
OUT = os.path.join(BASE, "model_fit.json")
D2R = np.pi / 180
TD = 0.00055   # 명령이 드라이버에 닿는 시간 (측정: 명령→Iq 명령값 변화 중앙값)

model = mujoco.MjModel.from_xml_path(MJ)
data = mujoco.MjData(model)


def mj_terms(pose, jname):
    """그 자세에서 (링크 관성, MJCF armature, MJCF frictionloss, 중력 강성 Kg)."""
    def setpose(extra=0.0):
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        for n, deg in pose.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
            if jid >= 0:
                data.qpos[model.jnt_qposadr[jid]] = deg * D2R
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        data.qpos[model.jnt_qposadr[jid]] += extra
        mujoco.mj_forward(model, data)
        return model.jnt_dofadr[jid]
    dof = setpose()
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, M)
    leg = list(range(6, model.nv))          # 자유 관절(몸통) 6개를 뺀 다리 관절
    k_ = leg.index(dof)
    link = 1.0 / np.linalg.inv(M[np.ix_(leg, leg)])[k_, k_] - model.dof_armature[dof]
    h = 1e-3
    setpose(+h)
    gp = data.qfrc_bias[dof]
    setpose(-h)
    gm = data.qfrc_bias[dof]
    kg = (gp - gm) / (2 * h)                # 중력 강성 (참고용, 맞춤에는 안 씀)
    return link, model.dof_armature[dof], model.dof_frictionloss[dof], kg


def load(name):
    d = os.path.join(BASE, name)
    meta = json.load(open(os.path.join(d, "meta.json"), encoding="utf-8"))
    rows = [(r["kind"], int(r["node"]), int(r["t_tx_ns"]), int(r["t_rx_ns"]), float(r["v1"]), float(r["v2"]))
            for r in csv.DictReader(open(os.path.join(d, "samples.csv")))]
    cmds, enc, iq = L.joint_series(meta, rows)
    et = np.array([e[0] for e in enc])
    eq = np.array([e[1] for e in enc])
    out = {"+": [], "-": []}
    for (tc, qt), (tp, qp) in zip(cmds[1:], cmds[:-1]):
        dq = qt - qp
        b = (et > tc - 100e6) & (et < tc)
        qb = eq[b].mean()
        w = (et >= tc) & (et < tc + 300e6)
        h = (et >= tc + 750e6) & (et < tc + 950e6)
        s = np.sign(dq)
        # (명령 후 시간 s, 출발점 기준 변위 rad, 출발점에서 목표까지 rad, 목표 대비 끝 오차 rad) — 모두 계단 방향이 +
        out["+" if dq > 0 else "-"].append(((et[w] - tc) / 1e9, (eq[w] - qb) * D2R * s, (qt - qb) * D2R * s,
                                             (eq[h].mean() - qt) * D2R * s))
    return meta, out


def simulate(t_obs, A, Kp, Kd, Fc, J, ti, T):
    """J, ti: 같은 모양 배열(격자). 관측 시각의 각도 [rad]를 돌려준다."""
    dt = 2e-5
    n = int(T / dt) + 1
    q = np.zeros_like(J)
    v = np.zeros_like(J)
    tau = np.zeros_like(J)
    idx = np.clip((t_obs / dt).astype(int), 0, n - 1)
    order = np.argsort(idx)
    idx_s = idx[order]
    k = 0
    out = np.zeros(J.shape + (len(t_obs),))
    eps = 1e-3
    for i in range(n):
        tgt = A if i * dt >= TD else 0.0
        tc = Kp * (tgt - q) - Kd * v
        tau += (tc - tau) * (dt / ti)
        fr = np.where(np.abs(v) > eps, Fc * np.sign(v), np.clip(tau, -Fc, Fc))
        v_new = v + (tau - fr) / J * dt
        v = np.where((np.sign(v_new) != np.sign(v)) & (np.abs(v) > eps) & (np.abs(tau) < Fc), 0.0, v_new)
        q = q + v * dt
        while k < len(idx_s) and idx_s[k] == i:
            out[..., order[k]] = q
            k += 1
    return out


def main(names):
    summary = []
    for name in names:
        meta, steps = load(name)
        j = meta["joint"]
        Kp, Kd = j["driver_kp"], j["driver_kd"]
        pose = dict(meta.get("pose_deg", {}))
        pose[j["name"]] = meta["q0_deg"] + meta["step_deg"] / 2
        link, arm_mj, fl_mj, Kg = mj_terms(pose, j["name"])
        e_p = float(np.mean([s[3] for s in steps["+"]]))
        e_m = float(np.mean([s[3] for s in steps["-"]]))
        Fc = Kp * (-(e_p + e_m) / 2)
        for dn in ("+", "-"):
            ss = steps[dn]
            A = float(np.mean([s[2] for s in ss]))
            t_all = np.concatenate([s[0] for s in ss])
            q_all = np.concatenate([s[1] for s in ss])
            Js = np.geomspace(0.004, 0.08, 40)
            tis = np.geomspace(0.0003, 0.004, 16)
            JJ, TT = np.meshgrid(Js, tis, indexing="ij")
            sel = t_all < 0.06
            sim = simulate(t_all[sel], A, Kp, Kd, Fc, JJ, TT, 0.061)
            rms = np.sqrt(np.mean((sim - q_all[sel]) ** 2, axis=-1))
            i, k = np.unravel_index(np.argmin(rms), rms.shape)
            Jb, tib = float(Js[i]), float(tis[k])
            full = simulate(t_all, A, Kp, Kd, Fc, np.array([Jb]), np.array([tib]), 0.3)[0]
            r_full = float(np.sqrt(np.mean((full - q_all) ** 2)) / A)
            print(f"{j['short']:14s} {dn} {A / D2R:3.0f}°  J {Jb * 1e3:5.1f}e-3  링크 {link * 1e3:5.1f}e-3  "
                  f"-> armature {(Jb - link) * 1e3:5.1f}e-3 (MJCF {arm_mj * 1e3:.1f})  ti {tib * 1e3:.2f} ms  "
                  f"Fc {Fc:.2f} Nm (MJCF {fl_mj:.3f})  rms 60ms {rms[i, k] / A * 100:.2f}% 300ms {r_full * 100:.2f}%")
            summary.append({"run": name, "joint": j["short"], "dir": dn, "amp_deg": A / D2R, "Kp": Kp, "Kd": Kd,
                            "J": Jb, "link_J": float(link), "armature_est": Jb - float(link),
                            "armature_mjcf": float(arm_mj), "ti_ms": tib * 1e3, "gravity_stiffness_Nm_rad": float(Kg),
                            "Fc_Nm": float(Fc), "frictionloss_mjcf": float(fl_mj),
                            "rms_60ms_pct": float(rms[i, k] / A * 100), "rms_300ms_pct": r_full * 100})
    json.dump(summary, open(OUT, "w", encoding="utf-8"), indent=1, ensure_ascii=False)
    print("저장:", os.path.normpath(OUT))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    main(sys.argv[1:])
