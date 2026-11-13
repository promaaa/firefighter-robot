"""MJCF 불러오기(몸통 고정) + 시뮬레이션 로봇.

로봇은 받침대에 매달린 상태로 비교하므로 free joint를 지워 몸통(base)을 고정한다.
시뮬 관절은 MJCF의 position actuator(kp, kv, forcerange)로 움직인다. 관절에 actuator가 없으면
(mjlab용 XML처럼) bench/params의 학습 게인으로 같은 PD를 qfrc_applied로 건다.
"""
import math
import os
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass

import mujoco
import numpy as np

from bench import params

FLOOR_NAME = "gui_floor"
SIM_HIST_LEN = 4000


@dataclass
class JointInfo:
    name: str
    jid: int
    qadr: int
    dadr: int
    body: int
    lo: float | None
    hi: float | None
    act: int            # actuator id, 없으면 -1
    kp: float
    kd: float
    tau_lim: float      # inf면 제한 없음
    gain_src: str

    def clamp(self, q, margin=0.0):
        lo = -math.inf if self.lo is None else self.lo + margin
        hi = math.inf if self.hi is None else self.hi - margin
        return min(max(q, lo), hi)

    def in_range(self, q, tol=0.0):
        return (self.lo is None or q >= self.lo - tol) and (self.hi is None or q <= self.hi + tol)


def _abs_dirs(root, xml_dir):
    comp = root.find("compiler")
    if comp is None:
        comp = ET.SubElement(root, "compiler")
        root.remove(comp)
        root.insert(0, comp)
    asset_dir = comp.get("assetdir")
    if asset_dir is not None and not os.path.isabs(asset_dir):
        comp.set("assetdir", os.path.normpath(os.path.join(xml_dir, asset_dir)))
    for key in ("meshdir", "texturedir"):
        v = comp.get(key)
        if v is not None:
            if not os.path.isabs(v):
                comp.set(key, os.path.normpath(os.path.join(xml_dir, v)))
        elif comp.get("assetdir") is None:
            comp.set(key, xml_dir)
    for inc in root.iter("include"):
        f = inc.get("file")
        if f and not os.path.isabs(f):
            inc.set("file", os.path.normpath(os.path.join(xml_dir, f)))


def _remove_free_joints(root):
    removed = 0
    for body in root.iter("body"):
        for child in list(body):
            if child.tag == "freejoint" or (child.tag == "joint" and child.get("type") == "free"):
                body.remove(child)
                removed += 1
    return removed


def _add_visuals(root):
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="gui_sky", type="skybox", builtin="gradient",
                  rgb1="0.17 0.18 0.20", rgb2="0.04 0.04 0.05", width="256", height="1536")
    ET.SubElement(asset, "texture", name="gui_grid_tex", type="2d", builtin="checker", width="512", height="512",
                  rgb1="0.19 0.20 0.22", rgb2="0.14 0.15 0.16", mark="edge", markrgb="0.30 0.31 0.33")
    ET.SubElement(asset, "material", name="gui_grid", texture="gui_grid_tex", texrepeat="10 10",
                  reflectance="0.05")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name=FLOOR_NAME, type="plane", size="1.5 1.5 0.05", pos="0 0 0",
                  material="gui_grid", contype="0", conaffinity="0", group="0")
    ET.SubElement(world, "light", name="gui_key", directional="true", pos="1 1 3", dir="-0.4 -0.4 -1",
                  diffuse="0.55 0.55 0.55", specular="0.15 0.15 0.15", castshadow="false")
    ET.SubElement(world, "light", name="gui_fill", directional="true", pos="-1 -1 2", dir="0.5 0.6 -0.6",
                  diffuse="0.25 0.25 0.27", specular="0 0 0", castshadow="false")


def load_model(path, visuals=True):
    """MJCF -> (MjModel, 메모). 몸통 고정, 화면용 바닥·조명 추가."""
    path = os.path.abspath(str(path))
    tree = ET.parse(path)
    root = tree.getroot()
    _abs_dirs(root, os.path.dirname(path))
    removed = _remove_free_joints(root)
    if visuals:
        _add_visuals(root)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    note = f"free joint {removed}개 제거(몸통 고정)" if removed else "free joint 없음"
    if visuals:
        _place_floor(model)
        model.vis.headlight.ambient[:] = [0.35, 0.35, 0.37]
        model.vis.headlight.diffuse[:] = [0.45, 0.45, 0.45]
        model.vis.headlight.specular[:] = [0.1, 0.1, 0.1]
        model.vis.map.znear = 0.01
        model.vis.scale.framelength = 0.6
        model.vis.scale.framewidth = 0.03
    return model, note


def _place_floor(model):
    """q = 0 자세에서 가장 낮은 메쉬 꼭짓점 바로 아래에 바닥을 둔다 (화면용, 충돌 없음)."""
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, FLOOR_NAME)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    zmin = math.inf
    for g in range(model.ngeom):
        if g == fid:
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and model.geom_dataid[g] >= 0:
            mid = model.geom_dataid[g]
            a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            v = model.mesh_vert[a:a + n]
            z = data.geom_xpos[g][2] + v @ data.geom_xmat[g].reshape(3, 3)[2]
            zmin = min(zmin, float(z.min()))
        elif model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE:
            zmin = min(zmin, float(data.geom_xpos[g][2] - model.geom_rbound[g]))
    if math.isfinite(zmin) and fid >= 0:
        model.geom_pos[fid][2] = zmin - 0.002


def joint_infos(model, cfg):
    """설정의 관절 이름 -> JointInfo (모델에 없으면 None)."""
    out = {}
    for j in cfg.joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j.name)
        if jid < 0:
            out[j.name] = None
            continue
        limited = bool(model.jnt_limited[jid])
        lo, hi = (float(model.jnt_range[jid][0]), float(model.jnt_range[jid][1])) if limited else (None, None)
        act = -1
        for a in range(model.nu):
            if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[a][0] == jid:
                act = a
                break
        m = params.MOTORS[j.motor]
        if act >= 0 and model.actuator_biasprm[act][1] < 0:     # position actuator: bias = [0, -kp, -kv]
            kp = float(model.actuator_gainprm[act][0])
            kd = float(-model.actuator_biasprm[act][2])
            tau = (float(max(abs(model.actuator_forcerange[act][0]), abs(model.actuator_forcerange[act][1])))
                   if model.actuator_forcelimited[act] else math.inf)
            src = f"MJCF actuator '{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act)}'"
        else:
            act = -1
            kp, kd, tau = m["kp"], m["kd"], m["torque_limit"]
            src = "MJCF에 position actuator 없음 -> bench/params 학습 게인으로 PD"
        out[j.name] = JointInfo(j.name, jid, int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]),
                                int(model.jnt_bodyid[jid]), lo, hi, act, kp, kd, tau, src)
    return out


class SimRobot:
    """벽시계에 맞춰 MuJoCo를 진행. 관절마다 목표 각도(PD) 또는 None(힘 풀림)."""

    def __init__(self, model, infos):
        self.m = model
        self.d = mujoco.MjData(model)
        self.infos = {n: i for n, i in infos.items() if i is not None}
        self._gain0 = model.actuator_gainprm.copy()
        self._bias0 = model.actuator_biasprm.copy()
        self._off = set()
        self.hist = {n: deque(maxlen=SIM_HIST_LEN) for n in self.infos}   # (t, q, qd, tau)
        self.tau = {n: 0.0 for n in self.infos}
        self._wall0 = None
        self._steps = 0
        mujoco.mj_forward(model, self.d)

    @property
    def dt(self):
        return float(self.m.opt.timestep)

    def q(self, name):
        i = self.infos.get(name)
        return None if i is None else float(self.d.qpos[i.qadr])

    def qd(self, name):
        i = self.infos.get(name)
        return None if i is None else float(self.d.qvel[i.dadr])

    def reset(self, pose=None):
        mujoco.mj_resetData(self.m, self.d)
        if pose:
            self.set_pose(pose)
        mujoco.mj_forward(self.m, self.d)

    def set_pose(self, pose):
        for name, q in pose.items():
            i = self.infos.get(name)
            if i is not None and q is not None:
                self.d.qpos[i.qadr] = q
                self.d.qvel[i.dadr] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def _actuator_on(self, a, on):
        if on and a in self._off:
            self.m.actuator_gainprm[a] = self._gain0[a]
            self.m.actuator_biasprm[a] = self._bias0[a]
            self._off.discard(a)
        elif not on and a not in self._off:
            self.m.actuator_gainprm[a][0] = 0.0
            self.m.actuator_biasprm[a][1] = 0.0
            self.m.actuator_biasprm[a][2] = 0.0
            self._off.add(a)

    def _apply(self, targets):
        d = self.d
        for name, i in self.infos.items():
            qt = targets.get(name)
            if i.act >= 0:
                self._actuator_on(i.act, qt is not None)
                if qt is not None:
                    d.ctrl[i.act] = qt
                self.tau[name] = float(d.actuator_force[i.act])
            else:
                if qt is None:
                    tau = 0.0
                else:
                    tau = i.kp * (qt - d.qpos[i.qadr]) - i.kd * d.qvel[i.dadr]
                    tau = max(-i.tau_lim, min(i.tau_lim, tau))
                d.qfrc_applied[i.dadr] = tau
                self.tau[name] = float(tau)

    def advance(self, wall_t, target_fn, max_steps=60):
        """target_fn(t) -> {관절: 목표 q 또는 None}. 벽시계 wall_t까지 진행."""
        if self._wall0 is None:
            self._wall0, self._steps = wall_t, 0
        dt = self.dt
        n = int((wall_t - self._wall0) / dt) - self._steps
        if n > max_steps:      # 너무 밀렸으면 따라잡지 않고 시간을 당김
            self._wall0 = wall_t - (self._steps + max_steps) * dt
            n = max_steps
        for _ in range(max(0, n)):
            self._steps += 1
            self._apply(target_fn(self._wall0 + self._steps * dt))
            mujoco.mj_step(self.m, self.d)
        for name, i in self.infos.items():
            if i.act >= 0:
                self.tau[name] = float(self.d.actuator_force[i.act])
            self.hist[name].append((wall_t, float(self.d.qpos[i.qadr]), float(self.d.qvel[i.dadr]),
                                    self.tau[name]))
