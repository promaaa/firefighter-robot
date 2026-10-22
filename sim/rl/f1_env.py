"""Walking task for MJX: the official BHL policy interface, the BHL Isaac Lab rewards, the final robot F2.

Policy input: the 45 values of the official BHL biped policy (BHL low-level code, policy/rl_controller.py), with the
BHL training noise. Policy output: 12 actions, joint target = default + 0.25 * action, 25 Hz.

What is the same as BHL (biped/env_cfg.py at commit 984741a):
- rewards and weights, std 0.25 tracking kernels, termination at 0.78 rad tilt, rewards times the policy dt;
- 200 Hz physics, PD kp 20 / kd 2 in the actuator (damping inside the torque clip, as the drive);
- friction 0.4-1.2, kp and kd x0.8-1.2, joint zero offset +-0.05 rad, reset pose and velocity ranges,
  a constant external force and torque on the torso for each episode (BHL +-2 N, +-2 Nm, here +-1 by default).
What is new for our robot and hardware:
- model sim/payload/models/bhl_F2.xml (final design F2, 15.99 kg with full tanks; FF_DESIGN=<name> loads
  bhl_<name>.xml instead);
  water 0 to full, payload masses x0.8-1.2, payload CoM +-3 cm;
- knee range 0 to 1.4 rad, and the knee target is clipped at 1.4 rad (legs/run_policy.py --knee-max 1.4);
- torque limit 6-9 Nm; command latency 0-20 ms and observation latency 0-20 ms (5 ms steps);
- the feet air time reward also counts for turn-on-the-spot commands (BHL: only |v_xy| > 0.1 m/s);
- a penalty when both feet stay on the ground for 0.5 s on a walk or turn command ("no_step");
- a command mix with stand, turn on the spot and slow walking; command changes every 3-8 s;
- a recoil force at the nozzle (0.9 m), on and off, and velocity pushes;
- standing: a penalty for each foot in the air when the command is zero.

    uv run python f1_env.py      self-check on the CPU: model, observation layout against sim/sim2sim/run.py
"""

import os
from pathlib import Path

import jax
import jax.numpy as jp
import mujoco
import numpy as np
from ml_collections import config_dict
from mujoco import mjx
from mujoco_playground._src import mjx_env

HERE = Path(__file__).resolve().parent
# Final design F2 (two R2 tanks, 15.99 kg). FF_DESIGN=<name> trains on bhl_<name>.xml instead.
F1_XML = HERE.parent / "payload" / "models" / f"bhl_{os.environ.get('FF_DESIGN', 'F2')}.xml"
JOINTS = [f"leg_{s}_{j}_joint" for s in ("left", "right")
          for j in ("hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll")]
DEFAULT = np.array([0, 0, -0.2, 0.4, -0.3, 0] * 2, dtype=np.float32)
KP, KD, ACTION_SCALE = 20.0, 2.0, 0.25
KNEE_MAX = 1.4  # rad, software knee limit: at full knee range the foot touches a tank
NOZZLE = np.array([0.03, 0.0, 0.944])  # m, base frame: recoil point at the nozzle, about 0.9 m above the floor
# BHL uniform observation noise (biped/env_cfg.py:55-72): command, gyro, gravity, joint pos, joint vel, last action.
OBS_NOISE = np.r_[np.zeros(3), np.full(3, 0.3), np.full(3, 0.05), np.full(12, 0.05), np.full(12, 2.0), np.zeros(12)]
TILT_LIMIT = 0.78


def default_config():
  return config_dict.create(
      ctrl_dt=0.04, sim_dt=0.005, episode_length=500,  # 25 Hz policy, 200 Hz physics, 20 s (BHL)
      impl="warp", naconmax=16 * 8192, njmax=96,  # naconmax: all envs together
      noise=1.0,  # x OBS_NOISE
      torque_range=[6.0, 9.0],  # Nm
      act_delay_steps=4, obs_delay_steps=4,  # max command and observation latency, in 5 ms physics steps (20 ms)
      zero_offset=0.05,  # rad, BHL default joint position offset
      push=config_dict.create(enable=True, interval=[6.0, 12.0], magnitude=[0.1, 0.5]),  # m/s velocity kick
      recoil=config_dict.create(force=[0.0, 1.5], on=[0.5, 3.0], off=[0.5, 4.0]),  # N, s
      reset_wrench=[1.0, 1.0],  # N, Nm (BHL base_external_force_torque uses 2, 2)
      cmd_time=[3.0, 8.0],  # s between command changes
      cmd_mix=[0.2, 0.25, 0.35, 0.2],  # stand, turn on the spot, walk forward, BHL general range
      reward=config_dict.create(
          # BHL biped/env_cfg.py RewardsCfg
          track_lin_vel=2.0, track_ang_vel=1.0, termination=-10.0, lin_vel_z=-0.1, ang_vel_xy=-0.05,
          flat_orientation=-2.0, action_rate=-0.01, torques=-2.0e-3, dof_acc=-1.0e-6, dof_pos_limits=-1.0,
          feet_air_time=1.0, feet_slide=-0.1, joint_deviation_hip=-0.2, joint_deviation_ankle_roll=-0.2,
          # new
          stand_feet_air=-0.5, no_step=-1.0,
      ),
      air_time_threshold=0.4, soft_limit_factor=0.9, tracking_std=0.25,
  )


def build_model():
  """Training copy of the model: no meshes, only the feet collide (with the floor and each other), PD actuators,
  knee <= 1.4."""
  s = mujoco.MjSpec.from_file(str(F1_XML))
  s.modelname = "f1_train"
  for g in list(s.geoms):
    if g.type == mujoco.mjtGeom.mjGEOM_MESH:
      s.delete(g)
  for m in list(s.meshes):
    s.delete(m)
  for se in list(s.sensors):
    s.delete(se)
  for g in s.geoms:
    if g.name != "floor":
      g.contype, g.conaffinity = 0, 0
  for side in ("left", "right"):
    b = s.body(f"leg_{side}_ankle_roll")
    foot = [g for g in s.geoms if g.parent.name == b.name and g.type == mujoco.mjtGeom.mjGEOM_BOX][0]
    foot.name, foot.contype, foot.conaffinity, foot.condim = f"{side}_foot", 1, 1, 3
    foot.friction = [0.4, 0.005, 0.0001]  # MuJoCo takes the larger of two frictions: the floor friction decides
    b.add_site(name=f"{side}_foot", pos=foot.pos)
  for j in ("leg_left_knee_pitch_joint", "leg_right_knee_pitch_joint"):
    s.joint(j).range = [0.0, KNEE_MAX]
  for a in s.actuators:  # BHL PD in the drive: tau = clip(kp (q* - q) - kd qd, +-limit)
    a.gaintype, a.biastype = mujoco.mjtGain.mjGAIN_FIXED, mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = KP
    a.biasprm[:3] = [0.0, -KP, -KD]
    a.forcerange, a.forcelimited = [-6.0, 6.0], mujoco.mjtLimited.mjLIMITED_TRUE
    a.ctrllimited = mujoco.mjtLimited.mjLIMITED_FALSE
  s.option.timestep = 0.005
  s.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # implicit kd, as the PhysX drive of BHL
  s.option.iterations, s.option.ls_iterations = 3, 5
  xml = s.to_xml()
  sensors = "".join(f'<contact name="{f}_contact" geom1="{f}_foot" geom2="floor" num="1" data="found"/>'
                    f'<framelinvel name="{f}_vel" objtype="site" objname="{f}_foot"/>' for f in ("left", "right"))
  xml = xml.replace("</mujoco>", f"<sensor>{sensors}</sensor></mujoco>")
  m = mujoco.MjModel.from_xml_string(xml)
  assert [m.joint(j).qposadr[0] for j in JOINTS] == list(range(7, 19))
  assert [m.actuator(i).name for i in range(m.nu)] == JOINTS
  return m


def quat_rotate_inverse(q, v):
  """BHL RlController.quat_rotate_inverse, q = [w, x, y, z]."""
  w, u = q[0], q[1:4]
  return v * (2.0 * w * w - 1.0) - jp.cross(u, v) * w * 2.0 + u * jp.dot(u, v) * 2.0


def quat_rotate(q, v):
  w, u = q[0], q[1:4]
  return v * (2.0 * w * w - 1.0) + jp.cross(u, v) * w * 2.0 + u * jp.dot(u, v) * 2.0


def yaw_of(q):
  w, x, y, z = q
  return jp.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class F1Joystick(mjx_env.MjxEnv):

  def __init__(self, config=None, config_overrides=None):
    super().__init__(config or default_config(), config_overrides)
    self._mj_model = build_model()
    self._mjx_model = mjx.put_model(self._mj_model, impl=self._config.impl)
    if self._config.impl == "warp":  # silence the per-step "solver iterations limit" prints
      o = self._mjx_model.opt
      self._mjx_model = self._mjx_model.replace(opt=o.replace(_impl=o._impl.replace(warn_overflow=0)))
    m = self._mj_model
    self._lo, self._hi = m.jnt_range[1:].T.astype(np.float32)  # target clip, as legs/robot.py command()
    c, r = (self._lo + self._hi) / 2, (self._hi - self._lo) / 2
    f = self._config.soft_limit_factor
    self._soft_lo, self._soft_hi = c - f * r, c + f * r
    self._hip = np.array([i for i, n in enumerate(JOINTS) if "hip_yaw" in n or "hip_roll" in n])
    self._ankle_roll = np.array([i for i, n in enumerate(JOINTS) if "ankle_roll" in n])
    self._base = m.body("base").id
    self._contact_adr = np.array([m.sensor_adr[m.sensor(f"{f}_contact").id] for f in ("left", "right")])
    self._vel_adr = np.array([m.sensor_adr[m.sensor(f"{f}_vel").id] + np.arange(3) for f in ("left", "right")])
    self._water = m.body("payload_water").id
    self._payload = np.array([i for i in range(m.nbody) if m.body(i).name.startswith("payload_")])
    self._init_q = np.r_[0, 0, 0, 1, 0, 0, 0, DEFAULT].astype(np.float32)  # BHL start: base at the origin, then a drop

  @property
  def xml_path(self):
    return str(F1_XML)

  @property
  def action_size(self):
    return 12

  @property
  def mj_model(self):
    return self._mj_model

  @property
  def mjx_model(self):
    return self._mjx_model

  # ---------------- commands, disturbances ----------------
  def sample_command(self, key):
    k = jax.random.split(key, 8)
    u = lambda kk, lo, hi: jax.random.uniform(kk, minval=lo, maxval=hi)
    turn = jp.array([0.0, 0.0, u(k[0], 0.2, 1.0) * jp.sign(u(k[1], -1.0, 1.0))])
    walk = jp.array([u(k[2], 0.1, 0.5), 0.0, u(k[3], -0.5, 0.5) * (u(k[4], 0, 1) < 0.5)])
    general = jp.array([u(k[5], -0.3, 0.5), u(k[6], -0.2, 0.2), u(k[7], -1.0, 1.0)])
    mode = jax.random.choice(k[4], 4, p=jp.array(self._config.cmd_mix))
    return jp.stack([jp.zeros(3), turn, walk, general])[mode]

  def _steps(self, key, lo_hi):
    return jp.round(jax.random.uniform(key, minval=lo_hi[0], maxval=lo_hi[1]) / self.dt).astype(jp.int32)

  # ---------------- reset ----------------
  def reset(self, rng):
    c = self._config
    k = jax.random.split(rng, 16)
    qpos = jp.array(self._init_q)
    # BHL reset_root_state_uniform: xy +-0.5 m, any yaw; velocity +-0.5 m/s and rad/s (z 0)
    yaw = jax.random.uniform(k[0], minval=-jp.pi, maxval=jp.pi)
    qpos = qpos.at[0:2].set(jax.random.uniform(k[1], (2,), minval=-0.5, maxval=0.5))
    qpos = qpos.at[3:7].set(jp.array([jp.cos(yaw / 2), 0, 0, jp.sin(yaw / 2)]))
    qpos = qpos.at[7:].set(qpos[7:] * jax.random.uniform(k[2], (12,), minval=0.5, maxval=1.5))  # BHL joint scale
    qpos = qpos.at[7:].set(jp.clip(qpos[7:], self._lo, self._hi))
    qvel = jp.zeros(self.mjx_model.nv).at[0:2].set(jax.random.uniform(k[3], (2,), minval=-0.5, maxval=0.5))
    qvel = qvel.at[3:6].set(jax.random.uniform(k[4], (3,), minval=-0.5, maxval=0.5))
    data = mjx_env.make_data(self.mj_model, qpos=qpos, qvel=qvel, ctrl=qpos[7:], impl=self.mjx_model.impl.value,
                             naconmax=c.naconmax, njmax=c.njmax)
    data = mjx.forward(self.mjx_model, data)
    f_n, t_n = c.reset_wrench
    info = {
        "rng": k[5],
        "command": self.sample_command(k[6]),
        "cmd_left": self._steps(k[7], c.cmd_time),
        "last_act": jp.zeros(12),
        "target": qpos[7:],  # joint target in the drive (true joint coordinates)
        "zero_off": jax.random.uniform(k[8], (12,), minval=-c.zero_offset, maxval=c.zero_offset),
        "act_delay": jax.random.randint(k[9], (), 0, c.act_delay_steps + 1),
        "obs_delay": jax.random.randint(k[10], (), 0, c.obs_delay_steps + 1),
        "wrench": jp.r_[jax.random.uniform(k[11], (3,), minval=-f_n, maxval=f_n),
                        jax.random.uniform(k[12], (3,), minval=-t_n, maxval=t_n)],
        "recoil_level": jax.random.uniform(k[13], minval=c.recoil.force[0], maxval=c.recoil.force[1])
                        * jp.sign(jax.random.uniform(k[14], minval=-1, maxval=1)),
        "recoil_on": jp.zeros((), bool),
        "recoil_left": self._steps(k[15], c.recoil.off),
        "push_left": self._steps(k[15], c.push.interval),
        "air_time": jp.zeros(2),
        "contact_time": jp.zeros(2),
    }
    metrics = {f"reward/{n}": jp.zeros(()) for n in c.reward}
    metrics.update({"vx_err": jp.zeros(()), "wz_err": jp.zeros(())})
    reading = (data.qpos[3:7], data.qvel[3:6], data.qpos[7:19], data.qvel[6:18])
    contact = data.sensordata[self._contact_adr] > 0
    obs = self._get_obs(data, info, reading, contact)
    return mjx_env.State(data, obs, jp.zeros(()), jp.zeros(()), metrics, info)

  # ---------------- step ----------------
  def step(self, state, action):
    c, info = self._config, state.info
    info["rng"], k_push, k_dir, k_cmd, k_ct, k_rec = jax.random.split(info["rng"], 6)

    # velocity push (BHL push_by_setting_velocity, disabled in BHL; enabled here with smaller kicks)
    info["push_left"] -= 1
    do_push = (info["push_left"] <= 0) & c.push.enable
    th = jax.random.uniform(k_dir, maxval=2 * jp.pi)
    mag = jax.random.uniform(k_push, minval=c.push.magnitude[0], maxval=c.push.magnitude[1])
    qvel = state.data.qvel.at[0:2].add(do_push * mag * jp.array([jp.cos(th), jp.sin(th)]))
    info["push_left"] = jp.where(info["push_left"] <= 0, self._steps(k_push, c.push.interval), info["push_left"])
    data = state.data.replace(qvel=qvel)

    # recoil at the nozzle: along the torso heading, on and off
    info["recoil_left"] -= 1
    flip = info["recoil_left"] <= 0
    info["recoil_on"] = jp.where(flip, ~info["recoil_on"], info["recoil_on"])
    info["recoil_left"] = jp.where(flip, jp.where(info["recoil_on"], self._steps(k_rec, c.recoil.on),
                                                  self._steps(k_rec, c.recoil.off)), info["recoil_left"])
    q = data.qpos[3:7]
    fwd = quat_rotate(q, jp.array([1.0, 0.0, 0.0]))
    f_rec = info["recoil_on"] * info["recoil_level"] * jp.array([fwd[0], fwd[1], 0.0])
    nozzle = data.xpos[self._base] + quat_rotate(q, jp.array(NOZZLE))
    torque = jp.cross(nozzle - data.xipos[self._base], f_rec)
    xfrc = jp.zeros((self.mjx_model.nbody, 6)).at[self._base].set(info["wrench"] + jp.r_[f_rec, torque])
    data = data.replace(xfrc_applied=xfrc)

    # policy target -> drive, with the runtime target clip and the joint zero error (drive sees q + zero_off)
    target_meas = jp.clip(jp.array(DEFAULT) + ACTION_SCALE * action, self._lo, self._hi)
    new_target = target_meas - info["zero_off"]
    old_target = info["target"]

    def sub(d, i):
      d = d.replace(ctrl=jp.where(i < info["act_delay"], old_target, new_target))
      d = mjx.step(self.mjx_model, d)
      return d, (d.qpos[3:7], d.qvel[3:6], d.qpos[7:19], d.qvel[6:18])

    data, hist = jax.lax.scan(sub, data, jp.arange(self.n_substeps))
    reading = jax.tree.map(lambda x: x[self.n_substeps - 1 - info["obs_delay"]], hist)
    info["target"] = new_target

    contact = data.sensordata[self._contact_adr] > 0
    info["air_time"] = jp.where(contact, 0.0, info["air_time"] + self.dt)
    info["contact_time"] = jp.where(contact, info["contact_time"] + self.dt, 0.0)

    done = self._termination(data)
    rewards = self._rewards(data, action, info, contact, done)
    rewards = {n: v * c.reward[n] * self.dt for n, v in rewards.items()}  # Isaac Lab scales rewards by dt
    reward = sum(rewards.values())

    info["last_act"] = action
    info["cmd_left"] -= 1
    new_cmd = info["cmd_left"] <= 0
    info["command"] = jp.where(new_cmd, self.sample_command(k_cmd), info["command"])
    info["cmd_left"] = jp.where(new_cmd, self._steps(k_ct, c.cmd_time), info["cmd_left"])
    obs = self._get_obs(data, info, reading, contact)

    for n, v in rewards.items():
      state.metrics[f"reward/{n}"] = v
    v_yaw = self._v_yaw(data)
    state.metrics["vx_err"] = jp.abs(v_yaw[0] - info["command"][0])
    state.metrics["wz_err"] = jp.abs(self._wz(data) - info["command"][2])
    return state.replace(data=data, obs=obs, reward=reward, done=done.astype(reward.dtype))

  # ---------------- observation ----------------
  def _get_obs(self, data, info, reading, contact):
    quat, gyro, q, qd = reading
    grav = quat_rotate_inverse(quat, jp.array([0.0, 0.0, -1.0]))
    clean = jp.concatenate([info["command"], gyro, grav, q + info["zero_off"] - jp.array(DEFAULT), qd, info["last_act"]])
    info["rng"], key = jax.random.split(info["rng"])
    noise = (2 * jax.random.uniform(key, (45,)) - 1) * jp.array(OBS_NOISE) * self._config.noise
    m = self.mjx_model
    priv = jp.concatenate([
        clean, quat_rotate_inverse(data.qpos[3:7], data.qvel[0:3]), data.qpos[2:3], contact.astype(jp.float32),
        info["air_time"], info["contact_time"], info["wrench"], info["recoil_on"] * info["recoil_level"] * jp.ones(1),
        m.actuator_forcerange[0, 1:2] / 6.0, info["act_delay"] * jp.ones(1) / 4.0, info["obs_delay"] * jp.ones(1) / 4.0,
        m.body_mass[self._water] * jp.ones(1), m.body_ipos[self._base], m.geom_friction[0, 0:1],
    ])
    return {"state": clean + noise, "privileged_state": priv}

  # ---------------- rewards ----------------
  def _v_yaw(self, data):
    yaw = yaw_of(data.qpos[3:7])
    v = data.qvel[0:3]  # world frame
    return jp.array([jp.cos(yaw) * v[0] + jp.sin(yaw) * v[1], -jp.sin(yaw) * v[0] + jp.cos(yaw) * v[1]])

  def _wz(self, data):
    return quat_rotate(data.qpos[3:7], data.qvel[3:6])[2]

  def _termination(self, data):
    grav = quat_rotate_inverse(data.qpos[3:7], jp.array([0.0, 0.0, -1.0]))
    fall = -grav[2] < np.cos(TILT_LIMIT)
    low = data.qpos[2] < -0.25  # base origin; the torso is about 0.27 m lower than in the stand
    return fall | low | jp.isnan(data.qpos).any() | jp.isnan(data.qvel).any()

  def _rewards(self, data, action, info, contact, done):
    c = self._config
    cmd = info["command"]
    q, qd = data.qpos[7:19], data.qvel[6:18]
    quat = data.qpos[3:7]
    grav = quat_rotate_inverse(quat, jp.array([0.0, 0.0, -1.0]))
    w_b = data.qvel[3:6]
    v_b = quat_rotate_inverse(quat, data.qvel[0:3])
    s2 = c.tracking_std ** 2
    moving = (jp.linalg.norm(cmd[:2]) > 0.08) | (jp.abs(cmd[2]) > 0.1)
    standing = ~moving
    # Isaac Lab feet_air_time_positive_biped (BHL), gated by "moving" instead of |v_xy| > 0.1
    in_contact = info["contact_time"] > 0.0
    in_mode = jp.where(in_contact, info["contact_time"], info["air_time"])
    single = jp.sum(in_contact) == 1
    air = jp.minimum(jp.min(jp.where(single, in_mode, 0.0)), c.air_time_threshold)
    feet_v = data.sensordata[self._vel_adr][:, :2]
    out = jp.clip(self._soft_lo - q, 0.0) + jp.clip(q - self._soft_hi, 0.0)
    return {
        "track_lin_vel": jp.exp(-jp.sum(jp.square(cmd[:2] - self._v_yaw(data))) / s2),
        "track_ang_vel": jp.exp(-jp.square(cmd[2] - self._wz(data)) / s2),
        "termination": done.astype(jp.float32),
        "lin_vel_z": jp.square(v_b[2]),
        "ang_vel_xy": jp.sum(jp.square(w_b[:2])),
        "flat_orientation": jp.sum(jp.square(grav[:2])),
        "action_rate": jp.sum(jp.square(action - info["last_act"])),
        "torques": jp.sum(jp.square(data.actuator_force)),
        "dof_acc": jp.sum(jp.square(data.qacc[6:18])),
        "dof_pos_limits": jp.sum(out),
        "feet_air_time": air * moving,
        "feet_slide": jp.sum(jp.linalg.norm(feet_v, axis=-1) * contact),
        "joint_deviation_hip": jp.sum(jp.abs(q[self._hip] - DEFAULT[self._hip])),
        "joint_deviation_ankle_roll": jp.sum(jp.abs(q[self._ankle_roll] - DEFAULT[self._ankle_roll])),
        "stand_feet_air": standing * jp.sum(~contact),
        "no_step": moving * (jp.min(info["contact_time"]) > 0.5),  # both feet down for 0.5 s on a move command
    }


def domain_randomize(model, rng, mj_model, torque_range=(6.0, 9.0)):
  """Per-environment model changes (BHL startup events + payload + torque limit). Returns (model, in_axes)."""
  floor = mj_model.geom("floor").id
  base = mj_model.body("base").id
  water = mj_model.body("payload_water").id
  payload = np.array([i for i in range(mj_model.nbody) if mj_model.body(i).name.startswith("payload_")])
  upper = np.r_[base, payload]

  @jax.vmap
  def rand(key):
    k = jax.random.split(key, 10)
    u = lambda kk, lo, hi, shape=(): jax.random.uniform(kk, shape, minval=lo, maxval=hi)
    friction = model.geom_friction.at[floor, 0].set(u(k[0], 0.4, 1.2))
    scale = jp.ones(model.nbody).at[payload].set(u(k[1], 0.8, 1.2))  # payload parts +-20 %
    scale = scale.at[water].set(u(k[2], 0.0, 1.0))  # water: empty to full
    scale = scale.at[base].set(u(k[3], 0.9, 1.1))  # BHL torso structure +-10 %
    body_mass = model.body_mass * scale
    body_inertia = model.body_inertia * scale[:, None]
    body_ipos = model.body_ipos.at[upper].add(u(k[4], -0.03, 0.03, (3,)))  # payload CoM +-3 cm
    kp = KP * u(k[5], 0.8, 1.2, (12,))
    kd = KD * u(k[6], 0.8, 1.2, (12,))
    gainprm = model.actuator_gainprm.at[:, 0].set(kp)
    biasprm = model.actuator_biasprm.at[:, 1].set(-kp).at[:, 2].set(-kd)
    lim = u(k[7], *torque_range)
    forcerange = jp.stack([-lim * jp.ones(12), lim * jp.ones(12)], axis=1)
    armature = model.dof_armature.at[6:].set(u(k[8], 0.004, 0.008, (12,)))
    frictionloss = model.dof_frictionloss.at[6:].set(u(k[9], 0.05, 0.15, (12,)))
    return friction, body_mass, body_inertia, body_ipos, gainprm, biasprm, forcerange, armature, frictionloss

  keys = ["geom_friction", "body_mass", "body_inertia", "body_ipos", "actuator_gainprm", "actuator_biasprm",
          "actuator_forcerange", "dof_armature", "dof_frictionloss"]
  vals = rand(rng)
  in_axes = jax.tree_util.tree_map(lambda x: None, model).tree_replace({k: 0 for k in keys})
  return model.tree_replace(dict(zip(keys, vals))), in_axes


def check():
  """CPU self-check: the training model has the masses of the model file, and the observation matches
  sim/sim2sim/run.py."""
  import sys
  sys.path.insert(0, str(HERE.parent / "sim2sim"))
  import run as s2s  # sim/sim2sim/run.py, read-only
  m = build_model()
  ref = mujoco.MjModel.from_xml_path(str(F1_XML))
  assert abs(m.body_subtreemass[1] - ref.body_subtreemass[1]) < 1e-6, f"mass differs from {F1_XML.name}"
  assert np.allclose(s2s.DEFAULT, DEFAULT) and s2s.JOINTS == JOINTS and np.allclose(s2s.NOISE, OBS_NOISE)
  # same random state in both models -> same 45 observation values (no noise, no latency)
  rng = np.random.default_rng(0)
  d_ref = mujoco.MjData(ref)
  q = rng.normal(size=4)
  d_ref.qpos[:] = np.r_[0.1, -0.2, 0.05, q / np.linalg.norm(q), DEFAULT + rng.uniform(-0.2, 0.2, 12)]
  d_ref.qvel[:] = rng.normal(size=18)
  mujoco.mj_forward(ref, d_ref)
  cmd, act = np.array([0.2, 0.0, 0.5], np.float32), rng.normal(size=12).astype(np.float32)
  want = s2s.observation(cmd, d_ref.sensor("imu_quat").data, d_ref.sensor("imu_gyro").data,
                         d_ref.qpos[7:19], d_ref.qvel[6:18], act)[0]
  env = F1Joystick(config_overrides={"impl": "jax"})
  info = {"command": jp.array(cmd), "zero_off": jp.zeros(12), "last_act": jp.array(act), "rng": jax.random.PRNGKey(0),
          "air_time": jp.zeros(2), "contact_time": jp.zeros(2), "wrench": jp.zeros(6), "recoil_on": jp.zeros((), bool),
          "recoil_level": jp.zeros(()), "act_delay": jp.zeros((), int), "obs_delay": jp.zeros((), int)}
  d = mujoco.MjData(m)
  d.qpos[:], d.qvel[:] = d_ref.qpos, d_ref.qvel
  dd = mjx.forward(env.mjx_model, mjx.put_data(m, d, impl="jax"))
  reading = (dd.qpos[3:7], dd.qvel[3:6], dd.qpos[7:19], dd.qvel[6:18])
  got = np.asarray(env._get_obs(dd, info, reading, jp.zeros(2, bool))["privileged_state"][:45])
  assert np.allclose(got, want, atol=1e-4), f"observation differs: {np.abs(got - want).max()}"
  print(f"check passed: model mass {m.body_subtreemass[1]:.2f} kg, observation matches run.py (max diff "
        f"{np.abs(got - want).max():.1e})")


if __name__ == "__main__":
  check()
