#!/usr/bin/env python3
"""Radxa에서 CAN 지연을 잰다 (보행 정책이 Radxa에서 돌 때와 같은 경로). 표준 라이브러리만 쓴다.

PC에서 MotorTestGUI/latency/latency.py가 이 파일과 can_bridge.py, joints.json을 ~/motor_latency/에 복사하고 실행한다.
GUI(Radxa 경유)는 끊어 둔다. CANable을 이 프로그램이 독점한다.

  python3 latency_test.py rtt  [--n 300]                       읽기만: 엔코더 요청→응답 시간, 12개 한 번에 읽는 시간
  python3 latency_test.py loop [--rates 25,50,...,400]         읽기만: 제어 주기를 훑으며 '12개 읽기 + 유지 목표 12개'가 몇 Hz까지 되나
  python3 latency_test.py step --joint left_knee_pitch --yes   관절 하나: 계단 목표 ±deg를 n번. 각도와 Iq를 한꺼번에 요청 (둘 다 약 2 ms마다)
  python3 latency_test.py sine --joint left_knee_pitch --yes   관절 하나: --hz(기본 50)로 사인 목표, 사이사이 빠르게 기록

공통: --out 폴더 (기본 runs/날짜_시각_시험), --slcan auto | --socketcan can_left,can_right, --bitrate 500000
움직이는 시험의 안전: 그 관절만 위치 제어. |Iq| > --guard-a (기본 8 A)나 목표-실측 > 12°가 0.3 s 이어지면 바로 IDLE.
Ctrl+C, SSH 끊김(표준 입력 닫힘), 오류에서도 그 관절을 IDLE로 둔다. 끝나면 IDLE.

기록 (CSV, 시각은 이 컴퓨터의 perf_counter 나노초):
  samples.csv  kind(enc|iq|cmd|hb), node, t_tx_ns(요청 보낸 시각), t_rx_ns(응답 받은 시각), v1, v2
  meta.json    시험 설정, 관절 설정, 버스 배치, 처음 자세
"""
import argparse
import json
import math
import os
import queue
import random
import signal
import struct
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_bridge as cb  # noqa: E402

HEARTBEAT, SET_AXIS_STATE, ENC, CTRL_MODES, INPUT_POS, IQ, VBUS, CLEAR = 0x01, 0x07, 0x09, 0x0B, 0x0C, 0x14, 0x17, 0x18
IDLE, CLOSED = 1, 8
GEAR = 15.0
TURN_DEG = 360.0 / GEAR          # 모터 1회전 = 관절 24°


def now_ns():
    return time.perf_counter_ns()


class Can:
    """두 버스를 열고, 받은 프레임을 (node, cmd)별 대기열에 넣는다."""

    def __init__(self, a):
        self.devs = cb.open_devices(a)
        if not self.devs:
            raise SystemExit("CAN 장치가 없습니다 (CANable을 Radxa에 꽂았는지)")
        self.stop = threading.Event()
        self.node_bus = {}
        self.hb = {}                       # node -> (axis_error, state, t_ns)
        self.waiters = {}                  # (node, cmd) -> queue
        self.lock = threading.Lock()
        for i, d in enumerate(self.devs):
            threading.Thread(target=self._loop, args=(i,), daemon=True).start()

    def _loop(self, i):
        try:
            self.devs[i].run(lambda cid, fl, dlc, data: self._on(i, cid, fl, dlc, data), self.stop)
        except Exception as e:  # noqa: BLE001
            print(f"버스 {i + 1} 수신 중단: {e}", flush=True)

    def _on(self, bus, cid, flags, dlc, data):
        t = now_ns()
        if flags & (cb.F_EXT | cb.F_RTR):
            return
        node, cmd = cid >> 5, cid & 0x1F
        self.node_bus.setdefault(node, bus)
        if cmd == HEARTBEAT:
            err, st = struct.unpack_from("<IB", data)
            self.hb[node] = (err, st, t)
            return
        q = self.waiters.get((node, cmd))
        if q is not None:
            q.put((t, data))

    def wait_nodes(self, nodes, timeout=1.5):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            if all(n in self.node_bus for n in nodes):
                return True
            time.sleep(0.02)
        return False

    def send(self, node, cmd, data=b"", rtr=False):
        bus = self.node_bus.get(node)
        targets = range(len(self.devs)) if bus is None else (bus,)
        t = None
        for b in targets:
            t0 = now_ns()          # 쓰기 직전 시각 (쓰기 뒤에 찍으면 그사이 스레드가 멈췄을 때 응답보다 늦게 찍힘)
            self.devs[b].send((node << 5) | cmd, cb.F_RTR if rtr else 0, 8, data.ljust(8, b"\x00"))
            t = t0 if t is None else t
        return t

    def queue_for(self, node, cmd):
        with self.lock:
            q = self.waiters.get((node, cmd))
            if q is None:
                q = self.waiters[(node, cmd)] = queue.Queue()
        return q

    def ask(self, node, cmd, timeout=0.02):
        """RTR 하나 → (t_tx, t_rx, data) 또는 None."""
        q = self.queue_for(node, cmd)
        while not q.empty():
            q.get_nowait()
        t_tx = self.send(node, cmd, rtr=True)
        try:
            t_rx, data = q.get(timeout=timeout)
        except queue.Empty:
            return None
        return t_tx, t_rx, data

    def ask_many(self, nodes, cmd, timeout=0.02):
        """여러 node에 한꺼번에 RTR → {node: (t_tx, t_rx, data)} (정책의 '12개 읽기'와 같은 방식)."""
        qs = {n: self.queue_for(n, cmd) for n in nodes}
        for q in qs.values():
            while not q.empty():
                q.get_nowait()
        t_tx = {n: self.send(n, cmd, rtr=True) for n in nodes}
        out, t_end = {}, time.monotonic() + timeout
        for n in nodes:
            left = t_end - time.monotonic()
            try:
                t_rx, data = qs[n].get(timeout=max(0.0, left))
                out[n] = (t_tx[n], t_rx, data)
            except queue.Empty:
                pass
        return out

    def ask_cmds(self, node, cmds, timeout=0.02):
        """한 node에 RTR 여러 개를 한꺼번에 → {cmd: (t_tx, t_rx, data)} (각도와 Iq를 같은 순간에 받으려고)."""
        qs = {c: self.queue_for(node, c) for c in cmds}
        for q in qs.values():
            while not q.empty():
                q.get_nowait()
        t_tx = {c: self.send(node, c, rtr=True) for c in cmds}
        out, t_end = {}, time.monotonic() + timeout
        for c in cmds:
            try:
                t_rx, data = qs[c].get(timeout=max(0.0, t_end - time.monotonic()))
                out[c] = (t_tx[c], t_rx, data)
            except queue.Empty:
                pass
        return out

    def close(self):
        self.stop.set()
        time.sleep(0.25)
        for d in self.devs:
            d.close()


# ---------------------------------------------------------------------- 관절 설정 (PC의 GUI 설정에서)
def load_joints(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def find_joint(joints, key):
    key = str(key).lower().replace(" ", "_")
    for j in joints["joints"]:
        names = {j["name"].lower(), j["name"].lower().replace("leg_", "").replace("_joint", ""),
                 j["short"].lower().replace(" ", "_"), str(j["node"])}
        if key in names:
            return j
    raise SystemExit(f"관절 '{key}'를 찾지 못했습니다. 예: left_knee_pitch, 7")


def unwrap_k(j, p):
    return int(round(j["direction"] * j.get("boot_ref_deg", 0.0) / TURN_DEG + j["zero_turn"] - p))


def q_deg(j, p, k):
    return j["direction"] * (p + k - j["zero_turn"]) * TURN_DEG


def p_turn(j, q, k):
    return j["zero_turn"] + j["direction"] * q / TURN_DEG - k


# ---------------------------------------------------------------------- 기록
class Log:
    def __init__(self, out, meta):
        os.makedirs(out, exist_ok=True)
        self.out = out
        self.meta = meta
        self.rows = []

    def add(self, kind, node, t_tx, t_rx, v1, v2=0.0):
        self.rows.append((kind, node, t_tx, t_rx, v1, v2))

    def save(self, extra=None):
        if extra:
            self.meta.update(extra)
        with open(os.path.join(self.out, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)
        with open(os.path.join(self.out, "samples.csv"), "w", encoding="utf-8") as f:
            f.write("kind,node,t_tx_ns,t_rx_ns,v1,v2\n")
            for r in self.rows:
                f.write(f"{r[0]},{r[1]},{r[2]},{r[3]},{r[4]:.7g},{r[5]:.7g}\n")
        print(f"기록: {self.out} (표본 {len(self.rows)}개)", flush=True)


def pctl(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    i = min(len(xs) - 1, max(0, int(round(p / 100.0 * (len(xs) - 1)))))
    return xs[i]


def stats_ms(xs):
    xs = [x / 1e6 for x in xs]
    return {"n": len(xs), "mean": round(sum(xs) / len(xs), 3) if xs else None, "p50": round(pctl(xs, 50), 3),
            "p95": round(pctl(xs, 95), 3), "p99": round(pctl(xs, 99), 3), "max": round(max(xs), 3) if xs else None}


# ---------------------------------------------------------------------- 시험
def t_rtt(can, a, log, nodes):
    print(f"[rtt] node마다 엔코더 RTR 하나씩 {a.n}번", flush=True)
    per = {n: [] for n in nodes}
    miss = {n: 0 for n in nodes}
    for _ in range(a.n):
        for n in nodes:
            r = can.ask(n, ENC)
            if r is None:
                miss[n] += 1
                continue
            per[n].append(r[1] - r[0])
            log.add("enc", n, r[0], r[1], *struct.unpack_from("<ff", r[2]))
        time.sleep(0.002)
    print(f"[rtt] 12개 한꺼번에 읽기 {a.n}번 (정책 방식)", flush=True)
    all_dur, ages = [], []
    for _ in range(a.n):
        t0 = now_ns()
        got = can.ask_many(nodes, ENC)
        t1 = now_ns()
        if len(got) == len(nodes):
            all_dur.append(t1 - t0)
        for n, (tx, rx, data) in got.items():
            ages.append(t1 - rx)
            log.add("enc_all", n, tx, rx, *struct.unpack_from("<ff", data))
        time.sleep(0.01)
    res = {"rtt_ms_per_node": {str(n): stats_ms(v) for n, v in per.items()},
           "rtt_ms_all": stats_ms([x for v in per.values() for x in v]),
           "missed": {str(n): m for n, m in miss.items() if m},
           "read_all_ms": stats_ms(all_dur),
           "read_all_complete": f"{len(all_dur)}/{a.n}"}
    print(json.dumps({k: v for k, v in res.items() if k != "rtt_ms_per_node"}, ensure_ascii=False, indent=1), flush=True)
    return res


def loop_once(can, a, log, nodes, hz, seconds):
    """hz로 seconds 동안: 12개 읽기 + (IDLE 축에만) 지금 위치를 목표로 보내기. 제어기 한 주기와 같은 CAN 일."""
    period = 1.0 / hz
    last_pos = {}
    ticks = []
    t_next = time.perf_counter()
    t_stop = t_next + seconds
    tmo = min(0.015, max(0.004, 0.8 * period))
    while t_next < t_stop:
        delay = t_next - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        t_start = now_ns()
        got = can.ask_many(nodes, ENC, timeout=tmo)
        t_read = now_ns()
        for n, (tx, rx, data) in got.items():
            p, v = struct.unpack_from("<ff", data)
            last_pos[n] = p
            log.add("enc", n, tx, rx, p, v)
        for n in nodes:
            if n in last_pos and can.hb.get(n, (0, IDLE, 0))[1] == IDLE:   # 위치 제어 중인 축은 건드리지 않음
                can.send(n, INPUT_POS, struct.pack("<fhh", last_pos[n], 0, 0))
        t_end = now_ns()
        ticks.append((int(t_next * 1e9), t_start, t_read, t_end, len(got)))
        log.add("tick", int(round(hz)), t_start, t_end, (t_read - t_start) / 1e6, len(got))
        t_next += period
        if t_next < time.perf_counter() - period:      # 밀렸으면 따라잡지 않고 지금부터 (실제 제어기처럼)
            t_next = time.perf_counter()
    per = [ticks[i + 1][1] - ticks[i][1] for i in range(len(ticks) - 1)]
    achieved = (len(ticks) - 1) / ((ticks[-1][1] - ticks[0][1]) / 1e9) if len(ticks) > 1 else 0.0
    return {"hz": hz, "achieved_hz": round(achieved, 2), "ticks": len(ticks), "tick_period_ms": stats_ms(per),
            "tick_start_late_ms": stats_ms([t[1] - t[0] for t in ticks]),
            "read_12_ms": stats_ms([t[2] - t[1] for t in ticks]), "cycle_ms": stats_ms([t[3] - t[1] for t in ticks]),
            "incomplete_reads": sum(1 for t in ticks if t[4] < len(nodes)),
            "replies_per_tick_mean": round(sum(t[4] for t in ticks) / max(1, len(ticks)), 2)}


def t_loop(can, a, log, nodes):
    rates = [float(r) for r in a.rates.split(",")] if a.rates else [a.hz]
    print(f"[loop] 주기 {rates} Hz를 각 {a.seconds:g} s: 12개 읽기 + 유지 목표 보내기 (IDLE이면 움직이지 않음)",
          flush=True)
    out = []
    for hz in rates:
        r = loop_once(can, a, log, nodes, hz, a.seconds)
        out.append(r)
        print(f"  {hz:6g} Hz → 실제 {r['achieved_hz']:7.2f} Hz, 12개 읽기 p50 {r['read_12_ms']['p50']} / "
              f"p99 {r['read_12_ms']['p99']} ms, 못 읽은 틱 {r['incomplete_reads']}/{r['ticks']}", flush=True)
        time.sleep(0.3)
    return {"rates": out}


class Mover:
    """관절 하나를 위치 제어로 움직이는 시험의 공통 부분 (켜기, 보호, IDLE)."""

    def __init__(self, can, a, log, j):
        self.can, self.a, self.log, self.j = can, a, log, j
        self.node = j["node"]
        self.on = False
        self.target_p = None
        self._bad_since = {}               # 조건별 (iq, err) 시작 시각
        self.aborted = None

    def enable(self, p0):
        n = self.node
        err = self.can.hb.get(n, (0, IDLE, 0))[0]
        if err:
            self.can.send(n, CLEAR)
            time.sleep(0.05)
        self.can.send(n, CTRL_MODES, struct.pack("<ii", 3, 1))
        self.set_target(p0)
        self.can.send(n, SET_AXIS_STATE, struct.pack("<I", CLOSED))
        self.on = True
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            if self.can.hb.get(n, (0, 0, 0))[1] == CLOSED:
                return
            time.sleep(0.02)
        raise RuntimeError("위치 제어로 들어가지 않았습니다 (heartbeat)")

    def set_target(self, p):
        self.target_p = p
        t = self.can.send(self.node, INPUT_POS, struct.pack("<fhh", p, 0, 0))
        self.log.add("cmd", self.node, t, t, p, 0.0)
        return t

    def poll(self, seconds, k):
        """seconds 동안 엔코더와 Iq를 한꺼번에 요청하며(둘 다 약 2 ms마다) 기록하고 보호 검사."""
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            got = self.can.ask_cmds(self.node, (ENC, IQ), timeout=0.01)
            for cmd, r in got.items():
                v1, v2 = struct.unpack_from("<ff", r[2])
                self.log.add("iq" if cmd == IQ else "enc", self.node, r[0], r[1], v1, v2)
                if cmd == IQ:
                    key, bad = "iq", (f"|Iq| {v2:+.1f} A > {self.a.guard_a:g} A" if abs(v2) > self.a.guard_a else None)
                else:
                    e = abs(q_deg(self.j, v1, k) - q_deg(self.j, self.target_p, k))
                    key, bad = "err", (f"목표-실측 {e:.1f}° > 12°" if e > 12.0 else None)
                if bad:
                    since = self._bad_since.setdefault(key, time.monotonic())
                    if time.monotonic() - since > 0.3:
                        self.aborted = bad
                        raise RuntimeError(f"보호 정지: {bad}")
                else:
                    self._bad_since.pop(key, None)
            hb = self.can.hb.get(self.node)
            if hb and hb[0]:
                self.aborted = f"axis error 0x{hb[0]:X}"
                raise RuntimeError(f"드라이버 에러 0x{hb[0]:X}")

    def idle(self):
        for _ in range(3):
            self.can.send(self.node, SET_AXIS_STATE, struct.pack("<I", IDLE))
        self.on = False


def start_pose(can, a, joints, j):
    """움직일 관절의 지금 위치와, 범위 안에서 움직일 방향."""
    r = None
    for _ in range(5):
        r = can.ask(j["node"], ENC, timeout=0.05)
        if r:
            break
    if r is None:
        raise SystemExit(f"node {j['node']} 엔코더 응답 없음")
    p0 = struct.unpack_from("<ff", r[2])[0]
    k = unwrap_k(j, p0)
    q0 = q_deg(j, p0, k)
    lo, hi = j["range_deg"]              # 시뮬 관절 범위에서 이미 2° 안쪽
    if not (lo - 6 <= q0 <= hi + 6):     # 24° 칸 오류는 잡고, 기구 끝에 기댄 몇 도는 허용 (범위 안으로 천천히 옮김)
        raise SystemExit(f"{j['short']} 지금 {q0:+.1f}°가 범위 {lo:+.0f}~{hi:+.0f}° 밖입니다. "
                         "24° 칸이 틀렸을 수 있습니다 (다리를 펴고 GUI에서 각도 다시 맞추기)")
    amp = a.deg
    base = min(max(q0, lo), hi)          # 기구 끝에 붙어 있으면 범위 안쪽에서 시작 (먼저 천천히 옮김)
    sign = 1.0 if (hi - base) >= (base - lo) else -1.0       # 여유가 큰 쪽으로
    span = amp * (2.0 if a.test == "sine" else 1.0)
    if not (lo <= base + sign * span <= hi):
        raise SystemExit(f"{j['short']} 지금 {q0:+.1f}°에서 {span:g}° 움직일 여유가 없습니다 (범위 {lo:+.0f}~{hi:+.0f}°). "
                         "관절을 범위 가운데쪽으로 옮기고 다시")
    others = can.ask_many([x["node"] for x in joints["joints"]], ENC, timeout=0.05)
    pose = {}
    for x in joints["joints"]:
        g = others.get(x["node"])
        if g:
            px = struct.unpack_from("<ff", g[2])[0]
            pose[x["name"]] = round(q_deg(x, px, unwrap_k(x, px)), 3)
    pose[j["name"]] = round(base, 3)
    return p0, k, q0, base, sign, pose


def move_slow(mv, j, k, q_from, q_to, seconds=1.2, period=0.02):
    t0 = time.perf_counter()
    while True:
        s = min(1.0, (time.perf_counter() - t0) / seconds)
        mv.set_target(p_turn(j, q_from + (q_to - q_from) * (s * s * (3 - 2 * s)), k))
        mv.poll(period, k)
        if s >= 1.0:
            return


def t_step(can, a, log, joints, j):
    p0, k, q0, base, sign, pose = start_pose(can, a, joints, j)
    p_base = p_turn(j, base, k)
    dp = sign * a.deg / TURN_DEG * j["direction"]
    print(f"[step] {j['short']} (node {j['node']}): 지금 {q0:+.2f}°, 시작점 {base:+.2f}°, "
          f"목표 {base:+.2f} ↔ {base + sign * a.deg:+.2f}° 를 {a.n}번 (각 {a.hold:g} s)", flush=True)
    log.meta.update({"p0": p0, "k": k, "q_start_deg": q0, "q0_deg": base, "step_deg": sign * a.deg, "pose_deg": pose})
    mv = Mover(can, a, log, j)
    try:
        mv.enable(p0)
        if abs(base - q0) > 0.05:
            move_slow(mv, j, k, q0, base)
        mv.poll(a.hold, k)
        for i in range(a.n):
            time.sleep(random.uniform(0, 0.003))       # 기록 주기와 엇갈리게 (표본 간격보다 짧은 시간까지 평균으로 보려고)
            mv.set_target(p_base + dp)
            mv.poll(a.hold, k)
            time.sleep(random.uniform(0, 0.003))
            mv.set_target(p_base)
            mv.poll(a.hold, k)
            print(f"  {i + 1}/{a.n}", flush=True)
        # 범위 밖(기구 끝)에서 시작했으면 되돌아가지 않고 범위 안에서 힘을 푼다
    finally:
        mv.idle()
    return {"aborted": mv.aborted}


def t_sine(can, a, log, joints, j):
    p0, k, q0, base, sign, pose = start_pose(can, a, joints, j)
    freqs = [float(f) for f in a.freqs.split(",")]
    center = base + sign * a.deg          # 범위 안쪽으로 진폭만큼 옮긴 점을 가운데로
    print(f"[sine] {j['short']} (node {j['node']}): 가운데 {center:+.2f}° ±{a.deg:g}°, {a.hz:g} Hz 목표, "
          f"주파수 {freqs} Hz × {a.cycles}주기", flush=True)
    pose[j["name"]] = round(center, 3)
    log.meta.update({"p0": p0, "k": k, "q_start_deg": q0, "q0_deg": center, "center_deg": center, "freqs": freqs,
                     "pose_deg": pose})
    mv = Mover(can, a, log, j)
    period = 1.0 / a.hz
    segments = []
    try:
        mv.enable(p0)
        move_slow(mv, j, k, q0, center, seconds=1.5, period=period)   # 가운데로 천천히
        for f in freqs:
            n_cyc = a.cycles + 2                       # 앞뒤 한 주기씩 진폭을 키우고 줄임
            dur = n_cyc / f
            t_seg = time.perf_counter()
            seg_ns = now_ns()
            t_next = t_seg
            while True:
                t = time.perf_counter() - t_seg
                if t >= dur:
                    break
                ramp = min(1.0, t * f, (dur - t) * f)
                q_t = center + a.deg * ramp * math.sin(2 * math.pi * f * t)
                mv.set_target(p_turn(j, q_t, k))      # 정책처럼 period마다 목표 하나 (사이는 유지)
                t_next += period
                mv.poll(max(0.0, t_next - time.perf_counter()), k)
            segments.append({"freq": f, "t0_ns": seg_ns, "dur_s": dur})
            print(f"  {f:g} Hz 끝", flush=True)
        move_slow(mv, j, k, center, base, seconds=1.5, period=period)   # 시작점으로 (범위 안)
    finally:
        mv.idle()
    log.meta["segments"] = segments
    return {"aborted": mv.aborted}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("test", choices=["rtt", "loop", "step", "sine"])
    ap.add_argument("--joints", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "joints.json"))
    ap.add_argument("--joint", help="step/sine: 움직일 관절 (예: left_knee_pitch 또는 node 번호)")
    ap.add_argument("--yes", action="store_true", help="step/sine: 관절이 움직입니다. 확인했으면 붙임")
    ap.add_argument("--n", type=int, default=None, help="rtt: 반복 수 (기본 300) / step: 계단 수 (기본 10)")
    ap.add_argument("--hz", type=float, default=None, help="sine: 목표를 보내는 주기 (기본 50 Hz)")
    ap.add_argument("--rates", default="25,50,100,150,200,300,400", help="loop: 훑을 제어 주기 [Hz]")
    ap.add_argument("--seconds", type=float, default=5.0, help="loop: 주기마다 시간")
    ap.add_argument("--targets", action="store_true", default=True)
    ap.add_argument("--deg", type=float, default=None, help="step: 계단 크기 (기본 4°) / sine: 진폭 (기본 4°), 최대 10°")
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--freqs", default="0.5,1,2,4,6")
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--guard-a", type=float, default=8.0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--slcan", default="auto")
    ap.add_argument("--socketcan", default="")
    ap.add_argument("--pycan", default="")
    ap.add_argument("--bitrate", type=int, default=500000)
    a = ap.parse_args()
    a.n = a.n or (300 if a.test == "rtt" else 10)
    a.hz = a.hz or 50.0
    a.deg = min(10.0, abs(a.deg if a.deg is not None else 4.0))
    moving = a.test in ("step", "sine")
    if moving and not a.joint:
        ap.error("step/sine에는 --joint가 필요합니다")
    if moving and not a.yes:
        ap.error("step/sine은 관절이 움직입니다. 로봇이 매달려 있고 주변이 비었으면 --yes를 붙이세요")

    joints = load_joints(a.joints)
    nodes = [x["node"] for x in joints["joints"]]
    j = find_joint(joints, a.joint) if moving else None
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs",
                                f"{stamp}_{a.test}" + (f"_{j['short'].replace(' ', '_')}" if j else ""))
    can = Can(a)
    import socket
    meta = {"test": a.test, "started": stamp, "args": {k: v for k, v in vars(a).items()}, "host": socket.gethostname(),
            "devices": [{"name": d.name, "kind": d.kind, "serial": d.serial} for d in can.devs], "joint": j}
    log = Log(out, meta)
    mover_node = j["node"] if j else None

    def emergency(*_):
        if mover_node is not None:
            for _ in range(3):
                try:
                    can.send(mover_node, SET_AXIS_STATE, struct.pack("<I", IDLE))
                except OSError:
                    pass
        print("중단 → IDLE", flush=True)
        try:
            log.save({"result": {"error": "중단 (Ctrl+C·SSH 끊김)"}})
        except Exception:  # noqa: BLE001
            pass
        os._exit(130)

    for sig in ("SIGINT", "SIGTERM", "SIGHUP"):
        if hasattr(signal, sig):
            signal.signal(getattr(signal, sig), emergency)
    if not sys.stdin.isatty():         # SSH로 실행: PC 쪽이 끊기면 표준 입력이 닫힘 → IDLE
        def stdin_watch():
            try:
                while sys.stdin.buffer.read(1):
                    pass
            except (OSError, ValueError):
                pass
            emergency()
        threading.Thread(target=stdin_watch, daemon=True).start()

    result = {}
    try:
        if not can.wait_nodes(nodes):
            missing = [n for n in nodes if n not in can.node_bus]
            print(f"응답 없는 node: {missing} (드라이버 전원·배선 확인). 있는 node로 계속합니다", flush=True)
            nodes = [n for n in nodes if n in can.node_bus]
        log.meta["node_bus"] = {str(n): b for n, b in can.node_bus.items()}
        log.meta["heartbeat_state"] = {str(n): v[1] for n, v in can.hb.items()}
        if moving:
            others_on = [n for n, v in can.hb.items() if v[1] != IDLE and n != j["node"]]
            if others_on:
                print(f"주의: 위치 제어 중인 다른 node {others_on}", flush=True)
        if a.test == "rtt":
            result = t_rtt(can, a, log, nodes)
        elif a.test == "loop":
            result = t_loop(can, a, log, nodes)
        elif a.test == "step":
            result = t_step(can, a, log, joints, j)
        else:
            result = t_sine(can, a, log, joints, j)
    except Exception as e:  # noqa: BLE001
        result = {"error": str(e)}
        print(f"오류: {e}", flush=True)
        if mover_node is not None:
            for _ in range(3):
                can.send(mover_node, SET_AXIS_STATE, struct.pack("<I", IDLE))
    finally:
        log.save({"result": result})
        can.close()
    print("DONE " + out, flush=True)
    return 0 if "error" not in result else 1


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)          # 표준 입력 감시 스레드가 막혀 있어도 바로 끝냄
