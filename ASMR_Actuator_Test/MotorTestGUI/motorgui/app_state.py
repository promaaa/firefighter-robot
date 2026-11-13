"""GUI와 무관한 핵심 상태: 설정, CAN 링크, 목표, 시뮬, 기록, 명령(안전 검사 포함).

명령 메서드는 실패하면 이유 문자열을, 성공하면 None을 돌려준다.
"""
import math
import time
from collections import deque

import mujoco
import numpy as np

from .can_link import CanLink
from .commander import Commander
from .config import TURN_TO_JOINT_DEG
from .sim import SimRobot, joint_infos, load_model
from . import protocol as P

TARGET_HIST_LEN = 4000


class App:
    def __init__(self, cfg, mjcf_path):
        self.cfg = cfg
        self._t0 = time.perf_counter()
        self.cmdr = Commander(cfg.names)
        self.link = CanLink(cfg, self.cmdr, self.now)
        self.mjcf_path = mjcf_path
        self.model, self.model_note = load_model(mjcf_path)
        self.infos = joint_infos(self.model, cfg)
        self.sim = SimRobot(self.model, self.infos)
        self.real_data = mujoco.MjData(self.model)
        self.target_data = mujoco.MjData(self.model)
        self.sim_idle_mode = "follow"          # follow: 실물 따라가기 / passive: 힘 풀림(중력)
        self.send_real = True
        self.tgt_hist = {n: deque(maxlen=TARGET_HIST_LEN) for n in cfg.names}
        self.range_rec = {n: [math.inf, -math.inf] for n in cfg.names}
        self.logs = deque(maxlen=1000)
        self.dirty = False
        self.sweep_active = False              # 가동범위 기록 중
        self.sweep_t0 = 0.0
        self._follow = {}
        self._sim_hold = {}
        self.log(f"모델: {mjcf_path} ({self.model_note})")
        missing = [n for n, i in self.infos.items() if i is None]
        if missing:
            self.log(f"MJCF에 없는 관절: {', '.join(missing)}")

    # ------------------------------------------------------------------ 기본
    def now(self):
        return time.perf_counter() - self._t0

    def log(self, text):
        self.logs.append((self.now(), text))

    def node(self, name):
        return self.link.nodes[self.cfg.joint(name).node]

    def online(self, name):
        return self.node(name).online(self.now())

    def closed_loop(self, name):
        return self.node(name).closed_loop(self.now())

    def real_q(self, name):
        """실측 관절 각도 [rad] (연결 안 됐으면 None)."""
        j = self.cfg.joint(name)
        ns = self.link.nodes[j.node]
        if ns.pos is None or not ns.online(self.now()):
            return None
        return j.q_from_turn(ns.pos, ns.k)

    def real_qd(self, name):
        j = self.cfg.joint(name)
        ns = self.link.nodes[j.node]
        if ns.vel is None or not ns.online(self.now()):
            return None
        return j.direction * ns.vel * 2 * math.pi / 15.0

    def real_tau(self, name):
        j = self.cfg.joint(name)
        ns = self.link.nodes[j.node]
        if ns.iq is None or not ns.online(self.now()):
            return None
        return j.torque_from_iq(ns.iq)

    def status(self, name):
        """ok | uncal | offline | error | nomodel"""
        if self.infos.get(name) is None:
            return "nomodel"
        ns = self.node(name)
        if not ns.online(self.now()) or ns.pos is None:
            return "offline"
        if ns.axis_error:
            return "error"
        if not self.cfg.joint(name).calibrated:
            return "uncal"
        return "ok"

    def real_connected(self):
        return self.link.connected

    # ------------------------------------------------------------------ 매 프레임
    def update(self):
        t = self.now()
        # 실물 자세 (연결 안 된 관절은 0)
        for name, info in self.infos.items():
            if info is None:
                continue
            q = self.real_q(name)
            self.real_data.qpos[info.qadr] = 0.0 if q is None else q
            tg = self.cmdr.target_q(name, t)
            self.target_data.qpos[info.qadr] = tg if tg is not None else (0.0 if q is None else q)
            self._follow[name] = q
            if q is not None:
                r = self.range_rec[name]
                r[0], r[1] = min(r[0], q), max(r[1], q)
            self.tgt_hist[name].append((t, math.nan if tg is None else tg))
        mujoco.mj_kinematics(self.model, self.real_data)
        mujoco.mj_kinematics(self.model, self.target_data)
        self.sim.advance(t, self._sim_targets)

    def _sim_targets(self, t):
        out = {}
        for name in self.sim.infos:
            tg = self.cmdr.target_q(name, t)
            if tg is not None:
                self._sim_hold.pop(name, None)
                out[name] = tg
            elif self.sim_idle_mode == "follow":
                q = self._follow.get(name)
                if q is not None:
                    self._sim_hold.pop(name, None)
                    out[name] = q
                else:   # 실물 값이 없으면 지금 시뮬 자세 유지
                    out[name] = self._sim_hold.setdefault(name, self.sim.q(name))
            else:
                out[name] = None
        return out

    # ------------------------------------------------------------------ 연결
    def connect(self, interface, channels, bitrate, virtual_kwargs=None):
        """channels: 포트 이름 목록 (CANable 하나면 1개, 두 개면 2개). 문제가 있으면 이유 문자열."""
        if isinstance(channels, str):
            channels = [channels]
        channels = [c for c in (channels or []) if c] or [""]
        if len(channels) > 2:
            return "CAN 포트는 두 개까지 씁니다"
        if len(channels) == 2 and channels[0].upper() == channels[1].upper():
            return f"포트 1과 포트 2가 같습니다 ({channels[0]}). 두 번째 CANable의 포트를 고르거나 '사용 안 함'으로 두세요"
        if interface != "virtual":   # 가상 연습은 저장할 기본 어댑터로 남기지 않음
            self.cfg.can_interface, self.cfg.can_bitrate = interface, int(bitrate)
            self.cfg.can_channel = channels[0]
            self.cfg.can_channel2 = channels[1] if len(channels) > 1 else ""
        if interface == "virtual":
            vk = dict(cfg=self.cfg, mjcf_path=str(self.mjcf_path))
            vk.update(virtual_kwargs or {})
            virtual_kwargs = vk
        self.link.start(interface, channels, bitrate, self.cfg.poll_hz, virtual_kwargs)
        return None

    def disconnect(self):
        if self.sweep_active:
            self.sweep_cancel()
        if self.link.connected and self.any_real_enabled():
            self.idle_all()
            time.sleep(0.05)
        self.link.stop()
        for n in self.cfg.names:
            tg = self.cmdr.get(n)
            if tg is not None and tg.real:
                self.cmdr.clear(n)

    def any_real_enabled(self):
        return any(self.closed_loop(n) for n in self.cfg.names)

    # ------------------------------------------------------------------ 명령
    def _use_real(self, name):
        return self.send_real and self.link.connected

    def enable(self, name):
        j = self.cfg.joint(name)
        t = self.now()
        if self._use_real(name):
            ns = self.link.nodes[j.node]
            if not ns.online(t) or ns.pos is None:
                return f"{j.short}: node {j.node} 응답 없음 (연결 상태 확인)"
            if self.sweep_active:
                return "가동범위 기록 중에는 토크를 켜지 않습니다 (먼저 '끝' 또는 '취소')"
            self.link.send_enable(j.node)
            self.cmdr.hold(name, j.q_from_turn(ns.pos, ns.k), t, real=True)
            self.log(f"{j.short}: 토크 켜기 (node {j.node}, 지금 위치 유지)")
        else:
            q = self.sim.q(name)
            if q is None:
                return f"{j.short}: 시뮬 모델에 없는 관절"
            self.cmdr.hold(name, q, t, real=False)
            self.log(f"{j.short}: 시뮬에서만 위치 유지 (실물 {'연결 안 됨' if self.send_real else '보내기 꺼짐'})")
        return None

    def disable(self, name):
        j = self.cfg.joint(name)
        if self.link.connected:      # 힘 풀기는 '실물에 보내기'와 상관없이 항상 보낸다
            self.link.send_disable(j.node)
        self.cmdr.clear(name)
        self.log(f"{j.short}: 힘 풀기")
        return None

    def clear_errors(self, name):
        j = self.cfg.joint(name)
        if not self.link.connected:
            return "실물이 연결되지 않았습니다"
        self.link.send_clear_errors(j.node)
        self.log(f"{j.short}: 에러 지우기 요청")
        return None

    def idle_all(self):
        if self.link.connected:
            self.link.send_idle_all()
        self.cmdr.clear_all()
        self.log("모두 힘 풀기")

    def _check_move(self, name, q_to):
        """(q_to_clamped, 오류 메시지, 안내 메시지)"""
        j = self.cfg.joint(name)
        info = self.infos.get(name)
        tg = self.cmdr.get(name)
        if tg is None:
            if self._use_real(name):
                return None, f"{j.short}: 먼저 '토크 켜기'를 누르세요", None
            err = self.enable(name)   # 시뮬 전용이면 자동으로 유지 시작
            if err:
                return None, err, None
            tg = self.cmdr.get(name)
        if tg.real and not self.closed_loop(name):
            return None, f"{j.short}: 실물 축이 아직 위치 제어 상태가 아닙니다 ({P.state_text(self.node(name).state)})", None
        note = None
        if info is not None and (j.calibrated or not tg.real):
            qc = info.clamp(q_to, math.radians(self.cfg.limit_margin_deg))
            if abs(qc - q_to) > 1e-6:
                note = (f"{j.short}: 목표 {math.degrees(q_to):+.1f}° -> 시뮬 관절 범위(여유 "
                        f"{self.cfg.limit_margin_deg:g}°) 안의 {math.degrees(qc):+.1f}°로 줄임")
                q_to = qc
        step = abs(q_to - tg.eval(self.now()))
        if math.degrees(step) > self.cfg.max_step_deg + 1e-6:
            return None, (f"{j.short}: 한 번에 {math.degrees(step):.1f}° 이동은 최대 {self.cfg.max_step_deg:g}°를 넘습니다 "
                          f"(나눠서 움직이거나 '한 번 최대'를 늘리세요)"), None
        return q_to, None, note

    def move_rel(self, name, d_deg):
        tg = self.cmdr.get(name)
        base = tg.q1 if tg is not None else (self.real_q(name) if self._use_real(name) else self.sim.q(name))
        if base is None:
            return f"{self.cfg.joint(name).short}: 지금 각도를 모릅니다"
        return self._move(name, base + math.radians(d_deg), f"상대 {d_deg:+g}°")

    def move_abs(self, name, deg):
        j = self.cfg.joint(name)
        if self._use_real(name) and not j.calibrated:
            return f"{j.short}: 영점이 없어 절대 각도 명령을 쓸 수 없습니다 ('영점·방향' 탭에서 먼저 영점 설정)"
        return self._move(name, math.radians(deg), f"절대 {deg:+g}°")

    def _move(self, name, q_to, what):
        q_to, err, note = self._check_move(name, q_to)
        if err:
            return err
        if note:
            self.log(note)
        tg = self.cmdr.move(name, q_to, self.now(), math.radians(self.cfg.max_speed_dps))
        j = self.cfg.joint(name)
        self.log(f"{j.short}: {what} -> 목표 {math.degrees(q_to):+.2f}° ({tg.T:.2f} s"
                 f"{'' if tg.real else ', 시뮬만'})")
        return None

    # ------------------------------------------------------------------ 영점·방향
    def set_angle_here(self, name, deg):
        """지금 실측 위치를 관절 각도 deg로 정의 (zero_turn 계산)."""
        j = self.cfg.joint(name)
        ns = self.link.nodes[j.node]
        if ns.pos is None or not ns.online(self.now()):
            return f"{j.short}: 실측 위치가 없습니다 (연결 확인)"
        if self.cmdr.get(name) is not None and self.cmdr.get(name).real:
            return f"{j.short}: 토크가 켜진 동안에는 영점을 바꾸지 않습니다 (먼저 '힘 풀기')"
        j.zero_turn = ns.pos + ns.k - j.direction * deg / TURN_TO_JOINT_DEG
        self.range_rec[name] = [math.inf, -math.inf]
        self.dirty = True
        self.log(f"{j.short}: 지금 자세 = {deg:+.2f}°로 영점 설정 (zero_turn {j.zero_turn:.5f})")
        return None

    def zero_all_here(self):
        done, skipped = [], []
        for j in self.cfg.joints:
            err = self.set_angle_here(j.name, 0.0) if self.online(j.name) else "offline"
            (skipped if err else done).append(j.short)
        return done, skipped

    def set_direction(self, name, d):
        j = self.cfg.joint(name)
        d = -1 if d < 0 else 1
        if d == j.direction:
            return None
        if self.cmdr.get(name) is not None and self.cmdr.get(name).real:
            return f"{j.short}: 토크가 켜진 동안에는 방향을 바꾸지 않습니다 (먼저 '힘 풀기')"
        j.direction = d
        self.range_rec[name] = [math.inf, -math.inf]
        self.dirty = True
        self.log(f"{j.short}: 방향 {d:+d}. 영점을 0° 자세에서 잡았다면 그대로 맞고, 다른 각도에서 잡았다면 다시 잡으세요")
        return None

    def set_torque_sign(self, name, s):
        j = self.cfg.joint(name)
        j.torque_sign = -1 if s < 0 else 1
        j.torque_sign_src = "GUI에서 바꿈"
        self.dirty = True

    def set_boot_ref(self, name, deg):
        j = self.cfg.joint(name)
        j.boot_ref_deg = float(deg)
        self.dirty = True

    def clear_zero(self, name):
        j = self.cfg.joint(name)
        if self.cmdr.get(name) is not None and self.cmdr.get(name).real:
            return f"{j.short}: 토크가 켜진 동안에는 영점을 지우지 않습니다"
        j.zero_turn = None
        self.link.nodes[j.node].k = 0
        self.range_rec[name] = [math.inf, -math.inf]
        self.dirty = True
        self.log(f"{j.short}: 영점 지움 (미보정)")
        return None

    def rewrap(self, names=None):
        """지금 자세가 boot_ref_deg +-12도 안이라고 보고 정수 turn을 다시 고름."""
        out = []
        for name in names or self.cfg.names:
            j = self.cfg.joint(name)
            ns = self.link.nodes[j.node]
            if ns.pos is None or (self.cmdr.get(name) is not None and self.cmdr.get(name).real):
                continue
            k = j.unwrap_k(ns.pos)
            if k != ns.k:
                out.append(f"{j.short} {ns.k:+d}->{k:+d} turn")
            ns.k = k
        self.log("각도 다시 맞춤: " + (", ".join(out) if out else "바뀐 관절 없음"))

    def reset_range_rec(self):
        for n in self.range_rec:
            self.range_rec[n] = [math.inf, -math.inf]

    # ------------------------------------------------------------------ 가동범위 훑어서 영점
    # '시작' -> 힘을 푼 채 관절을 손으로 양쪽 기구 끝까지 돌림 -> '끝'. 엔코더 최소/최대(모터 turn)가
    # 기구 끝 각도(JointCfg.stop_lo_deg / stop_hi_deg)라고 보고 영점을 계산한다.
    # 양 끝이 다 있으면 두 영점의 평균을 쓴다 (손으로 끝에 밀어붙인 만큼 넘어간 양이 양쪽에서 상쇄됨).
    SWEEP_SPAN_TOL_DEG = 3.0     # 기록 폭과 기구 끝 사이 폭의 차이가 이 안이면 '일치'

    def sweep_start(self):
        if not self.link.connected:
            return "실물 로봇에 연결되어 있지 않습니다"
        if self.sweep_active:
            return None
        on = [self.cfg.joint(n).short for n in self.cfg.names if self.closed_loop(n)]
        if on:
            return "토크가 켜진 관절이 있습니다 (" + ", ".join(on) + "). 먼저 모두 힘을 푸세요"
        self.link.start_record()
        self.sweep_active = True
        self.sweep_t0 = self.now()
        self.log("가동범위 기록 시작: 힘을 푼 채 관절을 손으로 양쪽 기구 끝까지 천천히 돌리세요")
        return None

    def sweep_cancel(self):
        self.link.stop_record()
        self.sweep_active = False
        self.log("가동범위 기록 취소 (영점 그대로)")

    def sweep_span(self, name):
        """이 관절의 기록 폭 [deg] (기록 없으면 None)."""
        ns = self.node(name)
        if ns.rec_n == 0 or not math.isfinite(ns.rec_pmin):
            return None
        return (ns.rec_pmax - ns.rec_pmin) * TURN_TO_JOINT_DEG

    def sweep_finish(self):
        self.link.stop_record()
        self.sweep_active = False
        res = [self._sweep_result(j) for j in self.cfg.joints]
        n_ok = sum(r["status"] == "ok" for r in res)
        self.log(f"가동범위 기록 끝 ({self.now() - self.sweep_t0:.0f} s): 양 끝 폭이 맞는 관절 {n_ok}개")
        return res

    def _sweep_result(self, j):
        """한 관절의 기록 결과와 영점 후보.

        cands: {"lo"|"hi"|"avg": zero_turn}, basis: 기본 선택, apply: 기본으로 적용할지,
        q_now: 후보 영점으로 본 지금 각도 [deg], shift: 지금 영점 대비 각도 읽음이 바뀌는 양 [deg].
        """
        ns = self.link.nodes[j.node]
        lo, hi = j.stop_lo_deg, j.stop_hi_deg
        r = {"name": j.name, "short": j.short, "node": j.node, "stop_lo": lo, "stop_hi": hi,
             "span": self.sweep_span(j.name), "model_span": None if lo is None or hi is None else hi - lo,
             "cands": {}, "basis": None, "apply": False, "status": "none", "msg": "", "q_now": {}, "shift": {}}
        if r["span"] is None or ns.rec_n < 5:
            r["msg"] = "기록 없음 (응답 없는 관절)"
            return r
        if ns.session != self.link.rec_session.get(j.node, ns.session):
            r["status"], r["msg"] = "bad", "기록 중에 끊겼다 다시 연결됨 (재부팅이면 값이 바뀜) → 다시 기록"
            return r
        if lo is None and hi is None:
            r["status"], r["msg"] = "nostop", "기구 끝 없음 → '지금 자세 = 0°'로 영점"
            return r
        if r["span"] < 5.0:
            r["status"], r["msg"] = "still", "거의 안 움직임"
            return r
        d, k = j.direction, ns.k
        p_hi = (ns.rec_pmax if d > 0 else ns.rec_pmin) + k      # 관절 각도가 가장 컸을 때의 모터 위치
        p_lo = (ns.rec_pmin if d > 0 else ns.rec_pmax) + k
        if lo is not None:
            r["cands"]["lo"] = p_lo - d * lo / TURN_TO_JOINT_DEG
        if hi is not None:
            r["cands"]["hi"] = p_hi - d * hi / TURN_TO_JOINT_DEG
        if lo is not None and hi is not None:
            r["cands"]["avg"] = 0.5 * (r["cands"]["lo"] + r["cands"]["hi"])
            r["basis"] = "avg"
            diff = r["span"] - r["model_span"]
            if abs(diff) <= self.SWEEP_SPAN_TOL_DEG:
                r["status"], r["apply"] = "ok", True
                r["msg"] = f"양 끝 폭 일치 (차이 {diff:+.1f}°)"
            elif diff < 0:
                r["status"] = "warn"
                r["msg"] = f"기구 폭보다 {-diff:.1f}° 좁음 → 한쪽 끝까지 못 갔을 수 있음"
            else:
                r["status"] = "warn"
                r["msg"] = f"기구 폭보다 {diff:.1f}° 넓음 → 기구 끝 값·방향 확인"
        else:
            end = lo if lo is not None else hi
            r["basis"] = "lo" if lo is not None else "hi"
            # 끝까지 갔는지 폭으로 확인할 수 없어서 기본은 체크 안 함 (다른 관절을 돌릴 때 같이 흔들렸을 수도 있음)
            r["status"], r["apply"] = "one", False
            r["msg"] = f"한쪽 끝({end:+.2f}°)만 있음 → 그 끝까지 댔다면 체크"
        p_now = (ns.pos if ns.pos is not None else ns.rec_pmax) + k
        for key, z in r["cands"].items():
            r["q_now"][key] = d * (p_now - z) * TURN_TO_JOINT_DEG
            if j.zero_turn is not None:
                r["shift"][key] = d * (j.zero_turn - z) * TURN_TO_JOINT_DEG
        return r

    def apply_sweep(self, choices):
        """choices: {name: zero_turn}. 토크가 켜진 관절은 건너뜀. 바꾼 관절 이름(짧은) 목록을 돌려줌."""
        done = []
        for name, z in choices.items():
            j = self.cfg.joint(name)
            if self.cmdr.get(name) is not None and self.cmdr.get(name).real:
                continue
            j.zero_turn = float(z)
            self.range_rec[name] = [math.inf, -math.inf]
            done.append(j.short)
            self.log(f"{j.short}: 가동범위 기록으로 영점 설정 (zero_turn {j.zero_turn:.5f})")
        if done:
            self.dirty = True
        return done

    # ------------------------------------------------------------------ 시뮬
    def sync_sim_to_real(self):
        pose = {n: self.real_q(n) for n in self.cfg.names}
        n_ok = sum(q is not None for q in pose.values())
        self.sim.set_pose(pose)
        t = self.now()
        for n in self.cfg.names:
            tg = self.cmdr.get(n)
            if tg is not None and not tg.real and pose.get(n) is not None:
                self.cmdr.hold(n, pose[n], t, real=False)
        self.log(f"시뮬 자세를 실물에 맞춤 ({n_ok}개 관절)")

    def reset_sim(self):
        for n in self.cfg.names:
            tg = self.cmdr.get(n)
            if tg is not None and not tg.real:
                self.cmdr.clear(n)
        self._sim_hold.clear()
        self.sim.reset()
        self.log("시뮬을 0 자세로 초기화")

    # ------------------------------------------------------------------ 기록 (플롯용)
    def series(self, name, window):
        """선택 관절의 최근 window초 기록 -> dict of numpy (x는 '지금부터 몇 초 전', 음수)."""
        t = self.now()
        j = self.cfg.joint(name)
        ns = self.link.nodes[j.node]
        out = {}
        enc = [r for r in list(ns.enc_hist) if r[0] >= t - window]
        if enc:
            a = np.asarray(enc)
            out["real"] = (a[:, 0] - t, np.degrees(j.direction * (a[:, 1] + ns.k - (j.zero_turn or 0.0))
                                                   * 2 * np.pi / 15.0),
                           np.degrees(j.direction * a[:, 2] * 2 * np.pi / 15.0))
        iq = [r for r in list(ns.iq_hist) if r[0] >= t - window]
        if iq:
            a = np.asarray(iq)
            out["real_tau"] = (a[:, 0] - t, j.direction * j.torque_sign * a[:, 2] * j.kt * 15.0)
            out["iq"] = (a[:, 0] - t, a[:, 2])
        tg = [r for r in list(self.tgt_hist[name]) if r[0] >= t - window]
        if tg:
            a = np.asarray(tg)
            out["target"] = (a[:, 0] - t, np.degrees(a[:, 1]))
        if name in self.sim.hist:
            sh = [r for r in list(self.sim.hist[name]) if r[0] >= t - window]
            if sh:
                a = np.asarray(sh)
                out["sim"] = (a[:, 0] - t, np.degrees(a[:, 1]), np.degrees(a[:, 2]), a[:, 3])
        return out
