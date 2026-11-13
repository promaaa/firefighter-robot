"""길들이기(08_break_in.py) 기록을 모아 논문 형식 그림(영어 캡션)과 요약표를 만든다.

    python analysis\\break_in_report.py                          # BasicTest 폴더에서
    python analysis\\break_in_report.py --out analysis\\break_in_20261109

입력: data/IDxx/break_in.csv (세션 요약), break_in_<시각>.csv (30초 구간 기록),
      xdrive_IDxx_after_setup_*.json (상저항·인덕턴스), check_*.json (보드 serial)
출력: fig1_6512_break_in, fig2_5010_break_in, table1_6512_summary, table2_5010_summary
      (.png 300 dpi, .pdf), summary.csv

특수 처리 (각 폴더의 README.md 참고):
  - ID12: 처음 3.18분은 6512 상수(ID08)로 돈 기록 -> 토크 x 0.0836/0.067 로 환산해 첫 세션으로 넣음
  - ID08: 길들이기 중 마찰이 다른 6512의 약 1/7 (감속기가 물리지 않은 것으로 보임) -> 통계와 시간 그래프에서 뺌
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import to_rgb
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D
from matplotlib.textpath import TextToPath

BASE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(BASE))
from bench import joints, params  # noqa: E402

KT = {m: params.MOTORS[m]["kt"] for m in params.MOTORS}
PRE_SESSIONS = {"12": ("as_ID08_6512_config_*", KT["5010"] / KT["6512"])}
SUSPECT = {"08": "reducer likely not engaged"}
FC_MODEL = {m: params.MOTORS[m]["coulomb"] for m in params.MOTORS}
MOTOR_NAME = {"6512": "MAD M6C12 (6512), hip and knee", "5010": "MAD 5010, ankle"}

# 색: 관절 종류 (고정 순서, dataviz 기본 팔레트 1-6번), 선: 왼쪽 실선 / 오른쪽 점선
JOINT_ORDER = ["hip_roll", "hip_yaw", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"]
JOINT_COLOR = dict(zip(JOINT_ORDER, ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]))
JOINT_LABEL = {j: j.replace("_", " ") for j in JOINT_ORDER}
SIDE_LS = {"L": "-", "R": (0, (3.2, 1.6))}
INK, INK2, MUTED, GRID, AXIS, WARN = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#d03b3b"

FONT = "Arial"
plt.rcParams.update({
    "font.family": FONT, "font.size": 7.5, "axes.labelsize": 7.5, "axes.titlesize": 8,
    "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
    "mathtext.fontset": "custom", "mathtext.rm": FONT, "mathtext.it": f"{FONT}:italic",
    "mathtext.bf": f"{FONT}:bold", "axes.unicode_minus": True, "savefig.dpi": 300,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2,
    "figure.facecolor": "white", "axes.facecolor": "white",
})
FIG_W = 7.16  # in, two-column width
CAP_SIZE = 7.5


# ---------------- 데이터 ----------------

def read_summary(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def fnum(s):
    return float(s) if s not in ("", None) else np.nan


def read_log(path, scale):
    with open(path, encoding="utf-8") as f:
        meta = json.loads(f.readline()[1:])
        rows = list(csv.DictReader(f))
    s = {k: np.array([fnum(r[k]) for r in rows]) for k in
         ("t_min", "v_cmd_turn_s", "v_turn_s", "tau_Nm", "fet_C", "vbus_V")}
    s["tau_Nm"] = s["tau_Nm"] * scale
    s["kind"] = np.array([r["kind"] for r in rows])
    s["meta"] = meta
    return s


def pairs(log):
    """정방향·역방향 30초 구간 한 쌍씩 평균 -> (시간, 마찰)."""
    run = log["kind"] == "run"
    t, tau = log["t_min"][run], np.abs(log["tau_Nm"][run])
    n = len(t) // 2 * 2
    return t[:n].reshape(-1, 2).mean(1), tau[:n].reshape(-1, 2).mean(1)


def window_rate(t, f, lo, hi):
    """[lo, hi] 분 구간 직선 맞춤 기울기 -> %/h (구간 평균 대비)."""
    m = (t >= lo) & (t <= hi)
    if m.sum() < 4:
        return np.nan
    slope = np.polyfit(t[m], f[m], 1)[0]
    return slope * 60.0 / f[m].mean() * 100.0


def newest(d, pattern):
    files = sorted(d.glob(pattern))
    return files[-1] if files else None


def load_unit(d):
    aid = d.name[2:]
    if not (d / "break_in.csv").exists():
        return None
    j = joints.by_node(int(aid))
    name, _, motor = j
    rows = []
    if aid in PRE_SESSIONS:
        pat, scale = PRE_SESSIONS[aid]
        for sub in sorted(d.glob(pat)):
            rows += [dict(r, _dir=sub, _scale=scale) for r in read_summary(sub / "break_in.csv")]
    rows += [dict(r, _dir=d, _scale=1.0) for r in read_summary(d / "break_in.csv")]
    rows.sort(key=lambda r: r["time"])
    sessions, off = [], 0.0
    for r in rows:
        log_path = r["_dir"] / f"break_in_{r['time']}.csv"
        log = read_log(log_path, r["_scale"]) if log_path.exists() else None
        sc = r["_scale"]
        sessions.append({"time": r["time"], "off": off, "minutes": float(r["minutes"]), "log": log,
                         "scaled": sc != 1.0, "stop": r["stop"],
                         "slow_start": fnum(r["slow_start_Nm"]) * sc, "slow_end": fnum(r["slow_end_Nm"]) * sc,
                         "run_first": fnum(r["run_first_Nm"]) * sc, "run_last": fnum(r["run_last_Nm"]) * sc,
                         "fet_max": fnum(r["fet_max_C"])})
        off += float(r["minutes"])
    u = {"id": aid, "motor": motor, "side": "L" if "_left_" in name else "R",
         "joint": name.split("_", 2)[2][:-6], "sessions": sessions, "total": off,
         "suspect": SUSPECT.get(aid)}
    first, last = sessions[0], sessions[-1]
    u["slow0"], u["slow1"] = first["slow_start"], last["slow_end"]
    u["run0"], u["run1"] = first["run_first"], last["run_last"]
    u["fet_max"] = np.nanmax([s["fet_max"] for s in sessions])
    logged = [s for s in sessions if s["log"] is not None]
    main = max(logged, key=lambda s: s["minutes"])
    t, f = pairs(main["log"])
    u["rate_early"] = window_rate(t, f, 0, 20)
    u["rate_late"] = window_rate(t, f, main["minutes"] - 20, main["minutes"])
    u["main_min"] = main["minutes"]
    js = newest(d, f"xdrive_ID{aid}_after_setup_*.json")
    if js:
        mc = json.load(open(js, encoding="utf-8"))["axis0"]["motor"]["config"]
        u["R_mohm"], u["L_uh"] = mc["phase_resistance"] * 1e3, mc["phase_inductance"] * 1e6
    else:
        u["R_mohm"] = u["L_uh"] = np.nan
    ck = newest(d, "check_*.json")
    u["serial"] = json.load(open(ck, encoding="utf-8")).get("serial", "") if ck else ""
    u["schedule"] = " + ".join(f"{s['minutes']:.1f}" + ("*" if s["scaled"] else "") +
                               ("†" if s["log"] is None else "") for s in sessions)
    return u


def group_stats(units, key):
    v = np.array([u[key] for u in units if not u["suspect"]])
    return v.mean(), v.std(ddof=1), v.std(ddof=1) / v.mean() * 100


# ---------------- 그림 공통 ----------------

def style(ax, ygrid=True):
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_linewidth(0.7)
    ax.tick_params(width=0.6, length=2.5, pad=2)
    if ygrid:
        ax.yaxis.grid(True, color=GRID, lw=0.5)
        ax.set_axisbelow(True)


START_GRAY = MUTED  # 시작점 색 (끝점은 관절 색)


def start_end(ax, x, y0, y1, col, lw=1.8, n=48):
    """시작(회색, 빈 원) -> 끝(관절 색, 채운 원). 선은 회색에서 관절 색으로 그라데이션."""
    g, c = np.array(to_rgb(START_GRAY)), np.array(to_rgb(col))
    ys = np.linspace(y0, y1, n + 1)
    segs = [[(x, ys[i]), (x, ys[i + 1])] for i in range(n)]
    w = (np.arange(n) + 0.5) / n
    ax.add_collection(LineCollection(segs, colors=[tuple(g + (c - g) * t) for t in w], linewidths=lw,
                                     capstyle="butt", zorder=2))
    ax.plot(x, y0, "o", ms=4.6, mfc="white", mec=START_GRAY, mew=1.1, zorder=3)
    ax.plot(x, y1, "o", ms=4.8, mfc=col, mec="white", mew=0.6, zorder=4)


def panel_title(ax, letter, text):
    ax.set_title(rf"$\mathbf{{({letter})}}$ {text}", loc="left", color=INK, pad=4)


def wrap_lines(segments, width_pt, size):
    """segments: [(text, bold)] -> 줄 목록, 각 줄은 [(text, bold)]. 실제 글자 폭으로 줄바꿈."""
    ttp = TextToPath()
    props = {False: FontProperties(family=FONT, size=size), True: FontProperties(family=FONT, size=size, weight="bold")}
    width = lambda s, b: ttp.get_text_width_height_descent(s, props[b], ismath="$" in s)[0]
    lines, cur, cur_w = [], [], 0.0
    space = width(" ", False)
    for text, bold in segments:
        for word in text.split(" "):
            w = width(word, bold)
            need = w + (space if cur else 0)
            if cur and cur_w + need > width_pt:
                lines.append(cur)
                cur, cur_w = [], 0.0
                need = w
            cur.append((word, bold))
            cur_w += need
    if cur:
        lines.append(cur)
    return lines, width, space


def add_caption(fig, subfig, segments, width_in, size=CAP_SIZE, line_h=1.32, left_in=0.12):
    lines, width, space = wrap_lines(segments, (width_in - 2 * left_in) * 72, size)
    w_in, h_in = subfig.get_figure().get_size_inches()[0], subfig.bbox.height / fig.dpi
    for i, line in enumerate(lines):
        x = left_in
        y = h_in - (i + 0.9) * size * line_h / 72
        for word, bold in line:
            subfig.text(x / w_in, y / h_in, word, fontsize=size, fontweight="bold" if bold else "normal",
                        color=INK, ha="left", va="baseline", fontfamily=FONT)
            x += (width(word, bold) + space) / 72


def caption_height(segments, width_in, size=CAP_SIZE, line_h=1.32, left_in=0.12):
    lines, _, _ = wrap_lines(segments, (width_in - 2 * left_in) * 72, size)
    return (len(lines) + 0.6) * size * line_h / 72


def make_figure(plot_h, segments):
    """segments가 None이면 캡션 없이 그림만."""
    if segments is None:
        fig = plt.figure(figsize=(FIG_W, plot_h))
        return fig, fig, None, None
    cap_h = caption_height(segments, FIG_W)
    fig = plt.figure(figsize=(FIG_W, plot_h + cap_h))
    top, bottom = fig.subfigures(2, 1, height_ratios=[plot_h, cap_h], hspace=0)
    return fig, top, bottom, segments


def save(fig, out, name):
    for ext in ("png", "pdf"):
        fig.savefig(out / f"{name}.{ext}", dpi=300)
    plt.close(fig)
    print(f"saved {out / name}.png/.pdf")


def end_labels(ax, items, x_text, min_gap, x_line_end):
    """선 끝 라벨 (텍스트는 잉크색). items: [(y, text)] -> 겹치지 않게 y를 벌림."""
    items = sorted(items)
    ys = [y for y, _ in items]
    for _ in range(200):
        moved = False
        for i in range(1, len(ys)):
            if ys[i] - ys[i - 1] < min_gap - 1e-9:
                mid = (ys[i] + ys[i - 1]) / 2
                ys[i - 1], ys[i] = mid - min_gap / 2, mid + min_gap / 2
                moved = True
        if not moved:
            break
    for (y0, txt), y in zip(items, ys):
        ax.plot([x_line_end + 0.4, x_text - 0.4], [y0, y], color=MUTED, lw=0.4, clip_on=False)
        ax.text(x_text, y, txt, fontsize=6.5, color=INK2, va="center", ha="left", clip_on=False)


# ---------------- 모터 종류별 그림 ----------------

FRICTION_YLIM = {"6512": (0.0, 0.50), "5010": (0.0, 0.50)}  # 0부터: 순위보다 차이의 크기를 보려고
TEMP_YLIM = (0, 40)
SIDE_WORD = {"L": "left", "R": "right"}

CAPTIONS = {
    "6512": [("Fig. 1.", True), (
        "Break-in of the MAD M6C12 (“6512”) hip and knee actuators. Each unit was spun at 10 turn/s on the motor "
        "side (4.2 rad/s at the joint) with the direction reversed every 30 s, under a 5 A current limit on a 12 V "
        "supply. (a) Friction torque at the break-in speed, averaged over each forward/reverse pair (1-min resolution). "
        "(b) Slow-speed friction (1 turn/s motor, 0.42 rad/s joint), checked every 5 min. (c) Driver-board FET "
        "temperature. In (a)–(c) the time axis is the cumulative break-in time, and curves are broken between "
        "sessions, which were separated by rest periods (ID01 and ID02: 20 min and 60 min on consecutive days; ID07: "
        "2.1 + 3.2 + 80.5 min, the first session without an interval log; the others: a 3-min trial followed by an "
        "80-min session). (d) Slow-speed friction at the first check, before break-in with the unit cold (open), and at "
        "the last check, with the unit warm (filled). (e) Break-in-speed friction over the first and the last 2 min. "
        "(f) Rate of change of the break-in-speed friction from a straight-line fit over the first 20 min (open) and the "
        "last 20 min (filled) of each unit’s longest session, relative to the window mean; the first window also "
        "contains the warm-up. CV: coefficient of variation (sample SD / mean). Dotted lines: Coulomb friction of the "
        "course actuator model (0.377 N·m). Joint torque = $k_t$ $I_q$ · 15 with the bench-verified "
        "$k_t$ = 0.067 N·m/A. ID08 (red) ran at ≈ 0.05 N·m, about one seventh of the other units, "
        "which suggests that its reducer was not engaged; it is left out of (a)–(c), (f) and the CV.", False)],
    "5010": [("Fig. 2.", True), (
        "Break-in of the MAD 5010 ankle actuators, with the same protocol and panels as Fig. 1: (a) friction at the "
        "break-in speed (10 turn/s motor, 4.2 rad/s joint); (b) slow-speed friction (1 turn/s motor), checked every "
        "5 min; (c) driver-board FET temperature; (d) slow-speed friction at the first check (cold, open) and the last "
        "check (warm, filled); (e) break-in-speed friction over the first and the last 2 min; (f) rate of change over "
        "the first (open) and the last (filled) 20 min of each unit’s longest session. Sessions: ID11 and ID13, "
        "a single 80-min session; ID12, a 3.2-min trial recorded with 6512 constants (torque rescaled by 0.0836/0.067) "
        "and an 80-min session; ID14, 3 + 10 + 70 min. Dotted lines: Coulomb friction of the course actuator model "
        "(0.317 N·m). Joint torque = $k_t$ $I_q$ · 15 with $k_t$ = 0.0836 N·m/A, which has not "
        "yet been verified on the bench; a different $k_t$ would scale every torque in this figure by the same "
        "factor.", False)],
}


def motor_figure(units_all, motor, out, name):
    units = sorted([u for u in units_all if u["motor"] == motor], key=lambda u: u["id"])
    ok = [u for u in units if not u["suspect"]]
    fig, top, _, _ = make_figure(4.9, None)  # 캡션 없이 (CAPTIONS는 논문용으로 남겨 둠)
    n_leg = len(units) + 1  # 액추에이터마다 한 줄 + course model
    ncol = min(5, n_leg)
    rows = -(-n_leg // ncol)
    plot_h = 4.9
    gs = top.add_gridspec(2, 3, left=0.075, right=0.955, top=1 - (0.30 + 0.17 * rows) / plot_h, bottom=0.11,
                          hspace=0.80, wspace=0.55)
    ax = [[top.add_subplot(gs[r, c]) for c in range(3)] for r in range(2)]
    ylim = FRICTION_YLIM[motor]
    gap = (ylim[1] - ylim[0]) * 0.052

    # (a)-(c) 시간 이력
    for c, t in enumerate(["friction at break-in speed", "slow-speed friction", "FET temperature"]):
        style(ax[0][c])
        panel_title(ax[0][c], "abc"[c], t)
    labs = [[], [], []]
    for u in units:  # 0부터 그리면 ID08도 축 안에 들어오므로 빨간색으로 함께 그림 (통계에서는 뺌)
        col = WARN if u["suspect"] else JOINT_COLOR[u["joint"]]
        ls = SIDE_LS[u["side"]]
        last = None
        for s in u["sessions"]:
            lg = s["log"]
            if lg is None:
                continue
            off = s["off"]
            t, f = pairs(lg)
            ck = lg["kind"] == "check"
            ax[0][0].plot(off + t, f, color=col, ls=ls, lw=0.95, solid_capstyle="round")
            ax[0][1].plot(off + lg["t_min"][ck], lg["tau_Nm"][ck], color=col, ls=ls, lw=0.95, marker="o", ms=2.4,
                          mew=0, solid_capstyle="round")
            ax[0][2].plot(off + lg["t_min"], lg["fet_C"], color=col, ls=ls, lw=0.85)
            last = (f[-3:].mean(), lg["tau_Nm"][ck][-1], lg["fet_C"][-1])
        for k in range(3):
            labs[k].append((last[k], f"ID{u['id']}", bool(u["suspect"])))
    for c in (0, 1):
        a = ax[0][c]
        a.set_ylim(*ylim)
        a.axhline(FC_MODEL[motor], color=MUTED, lw=0.8, ls=(0, (1, 1.6)), zorder=1)
        a.set_ylabel("friction torque (N·m)")
    ax[0][2].set_ylim(*TEMP_YLIM)
    ax[0][2].set_ylabel("temperature (°C)")
    for c in range(3):
        a = ax[0][c]
        a.set_xlim(0, 86)
        a.set_xticks([0, 20, 40, 60, 80])
        a.set_xlabel("cumulative break-in time (min)")
        if c < 2:  # 4개 이하면 모두, 많으면 범례에 맡기고 최고·최저 + 이상 유닛만
            good = [(y, t) for y, t, bad in labs[c] if not bad]
            bad = [(y, t) for y, t, b in labs[c] if b]
            items = good if len(good) <= 4 else [max(good), min(good)]
            end_labels(a, items + bad, 89, gap, 86)

    # (d)-(f) 개체 편차와 수렴
    xpos = {u["id"]: i for i, u in enumerate(units)}

    def cat_axis(a):
        a.set_xticks(range(len(units)))
        a.set_xticklabels([f"{u['id']}\n{u['side']}" for u in units], fontsize=6.5, linespacing=1.0)
        a.set_xlim(-0.6, len(units) - 0.4)
        a.tick_params(axis="x", length=0)
        a.set_xlabel("actuator ID / side", labelpad=2)

    def dumbbell(a, k0, k1, letter, title):
        style(a)
        panel_title(a, letter, title)
        a.axhline(FC_MODEL[motor], color=MUTED, lw=0.8, ls=(0, (1, 1.6)), zorder=1)
        for u in units:
            xx = xpos[u["id"]]
            col = WARN if u["suspect"] else JOINT_COLOR[u["joint"]]
            start_end(a, xx, u[k0], u[k1], col)
        m0, _, cv0 = group_stats(units, k0)
        m1, _, cv1 = group_stats(units, k1)
        # 통계 글자는 점이 없는 모서리에
        pts = [((xpos[u["id"]] + 0.6) / len(units), (u[k] - ylim[0]) / (ylim[1] - ylim[0]))
               for u in units for k in (k0, k1)]
        corners = [(0.98, 0.97, "right", "top"), (0.02, 0.97, "left", "top"),
                   (0.98, 0.03, "right", "bottom"), (0.02, 0.03, "left", "bottom")]

        def free(c):
            xr = (0.40, 1.0) if c[2] == "right" else (0.0, 0.60)
            yr = (0.78, 1.0) if c[3] == "top" else (0.0, 0.22)
            return not any(xr[0] <= px <= xr[1] and yr[0] <= py <= yr[1] for px, py in pts)

        cx, cy, ha, va = next((c for c in corners if free(c)), corners[0])
        a.text(cx, cy, f"mean {m0:.3f} → {m1:.3f} N·m\nCV {cv0:.1f}% → {cv1:.1f}%", ha=ha, va=va, fontsize=6.3,
               color=INK2, transform=a.transAxes, linespacing=1.15)
        a.set_ylim(*ylim)
        a.set_ylabel("friction torque (N·m)")
        cat_axis(a)

    dumbbell(ax[1][0], "slow0", "slow1", "d", "slow-speed friction, start → end")
    dumbbell(ax[1][1], "run0", "run1", "e", "break-in-speed friction, start → end")
    a = ax[1][2]
    style(a)
    panel_title(a, "f", "friction change rate")
    a.axhline(0, color=AXIS, lw=0.8, zorder=1)
    for u in ok:
        xx, col = xpos[u["id"]], JOINT_COLOR[u["joint"]]
        start_end(a, xx, u["rate_early"], u["rate_late"], col)
    for u in units:
        if u["suspect"]:
            a.text(xpos[u["id"]], 0.5, "n/a", ha="center", va="center", fontsize=6, color=WARN,
                   transform=a.get_xaxis_transform())
    lo = min(min(u["rate_early"], u["rate_late"]) for u in ok)
    hi = max(max(u["rate_early"], u["rate_late"]) for u in ok)
    a.set_ylim(np.floor(lo / 10) * 10 - 2, max(5, np.ceil(hi / 5) * 5))
    a.set_ylabel("friction change (% h$^{-1}$)")
    cat_axis(a)

    # 범례: 액추에이터별 (색 = 관절, 선 = 좌우) + 열린/채운 원
    handles = [Line2D([], [], color=JOINT_COLOR[u["joint"]], ls=SIDE_LS[u["side"]], lw=1.4,
                      label=f"ID{u['id']}  {SIDE_WORD[u['side']]} {JOINT_LABEL[u['joint']]}") for u in ok]
    handles += [Line2D([], [], color=WARN, ls=SIDE_LS[u["side"]], lw=1.4,
                       label=f"ID{u['id']}  {SIDE_WORD[u['side']]} {JOINT_LABEL[u['joint']]} (suspect, not in mean/CV)")
                for u in units if u["suspect"]]
    handles.append(Line2D([], [], color=MUTED, lw=0.8, ls=(0, (1, 1.6)), label=f"course model $F_c$"))
    top.legend(handles=handles, loc="upper center", ncol=ncol, frameon=False,
               bbox_to_anchor=(0.5, 1.0), handlelength=2.2, columnspacing=1.3, fontsize=6.6)
    marks = [Line2D([], [], ls="", marker="o", ms=4.6, mfc="white", mec=START_GRAY, mew=1.1,
                    label="start (gray):  (d) first check, cold   (e) first 2 min   (f) first 20 min"),
             Line2D([], [], ls="", marker="o", ms=4.8, mfc=INK2, mec="white", mew=0.6,
                    label="end (joint color):  (d) last check, warm   (e) last 2 min   (f) last 20 min")]
    # 아래 줄 패널 제목 바로 위 (서브피규어 좌표, gridspec 값으로 계산)
    g_top, g_bot, g_h = gs.top, gs.bottom, gs.hspace
    row2_top = g_bot + (g_top - g_bot) / (2 + g_h)
    top.legend(handles=marks, loc="lower center", ncol=2, frameon=False,
               bbox_to_anchor=(0.5, row2_top + 0.25 / plot_h), bbox_transform=top.transSubfigure,
               columnspacing=2.0, handletextpad=0.3, fontsize=6.6)
    save(fig, out, name)


# ---------------- 표 ----------------

TABLE_NOTES = {  # 표 아래 한 줄 각주 (긴 캡션 대신)
    "6512": [("Slow-speed friction: first check (cold) → last check (warm). Break-in-speed friction: first 2 min → last "
              "2 min. Rate: last 20 min of the longest session. † Stopped by DC-bus undervoltage, no interval log. "
              "Red: friction ≈ 1/7 of the other units (reducer likely not engaged), not in the mean row.", False)],
    "5010": [("Columns as in the 6512 table. * Recorded with 6512 constants, torque × 0.0836/0.067. Torques assume "
              "$k_t$ = 0.0836 N·m/A (not yet verified on the bench).", False)],
}

TABLE_CAPTIONS = {
    "6512": [("Table 1.", True), (
        "Break-in summary of the 6512 actuators. Slow-speed friction: first check before break-in (cold) → last "
        "check (warm); break-in-speed friction: first 2 min → last 2 min; the change is given in parentheses. "
        "Rate: friction change over the last 20 min of the longest session. R and L: phase resistance and inductance "
        "from the motor calibration stored on the board. The last row gives mean ± sample SD (CV) without ID08. "
        "† Session stopped by DC-bus undervoltage, no interval log. ID08 (red): friction about one seventh of the "
        "other units throughout the break-in, so its reducer was most likely not engaged; check the assembly and "
        "repeat a 3-min trial.", False)],
    "5010": [("Table 2.", True), (
        "Break-in summary of the 5010 actuators; columns as in Table 1. * Recorded with 6512 constants; torque "
        "rescaled by 0.0836/0.067. Torques assume $k_t$ = 0.0836 N·m/A, not yet verified on the bench.", False)],
}


def motor_table(units_all, motor, out, name):
    units = sorted([u for u in units_all if u["motor"] == motor], key=lambda u: u["id"])
    ok = [u for u in units if not u["suspect"]]
    cols = [("ID", 0.03, "l"), ("joint", 0.105, "l"), ("board serial", 0.095, "l"), ("sessions (min)", 0.115, "l"),
            ("total\n(min)", 0.048, "r"), ("slow-speed friction\n(N·m)", 0.165, "r"),
            ("break-in-speed friction\n(N·m)", 0.165, "r"), ("rate\n(%/h)", 0.062, "r"),
            ("FET max\n(°C)", 0.062, "r"), ("R\n(mΩ)", 0.075, "r"), ("L\n(µH)", 0.072, "r")]
    neg = lambda t: t.replace("-", "−")  # 음수는 수학 기호 −
    body = []
    for u in units:
        d0 = (u["slow1"] / u["slow0"] - 1) * 100
        d1 = (u["run1"] / u["run0"] - 1) * 100
        body.append(([u["id"], f"{SIDE_WORD[u['side']]} {JOINT_LABEL[u['joint']]}", u["serial"], u["schedule"],
                      f"{u['total']:.1f}", neg(f"{u['slow0']:.3f} → {u['slow1']:.3f} ({d0:+.0f}%)"),
                      neg(f"{u['run0']:.3f} → {u['run1']:.3f} ({d1:+.0f}%)"), neg(f"{u['rate_late']:+.1f}"),
                      f"{u['fet_max']:.1f}", f"{u['R_mohm']:.1f}", f"{u['L_uh']:.1f}"], bool(u["suspect"])))

    def cell(k, fmt):
        mean, sd, cv = group_stats(units, k)
        return f"{fmt.format(mean)} ± {fmt.format(sd)} ({cv:.0f}%)"

    def plain(k, fmt):
        v = np.array([u[k] for u in ok])
        return neg(f"{fmt.format(v.mean())} ± {v.std(ddof=1):.1f}")

    group = [f"mean (n = {len(ok)})", "", "", "", f"{np.mean([u['total'] for u in ok]):.1f}",
             f"{cell('slow0', '{:.3f}')}\n→ {cell('slow1', '{:.3f}')}",
             f"{cell('run0', '{:.3f}')}\n→ {cell('run1', '{:.3f}')}", plain("rate_late", "{:+.1f}"),
             "", plain("R_mohm", "{:.1f}"), plain("L_uh", "{:.1f}")]
    row_h, head_h, grp_h = 0.205, 0.36, 0.40
    plot_h = 0.10 + head_h + row_h * len(body) + grp_h + 0.10
    fig, top, bottom, seg = make_figure(plot_h, TABLE_NOTES[motor])
    add_caption(fig, bottom, seg, FIG_W, size=6.4)
    a = top.add_axes([0, 0, 1, 1])
    a.axis("off")
    H, left, right = plot_h, 0.012, 0.995
    scale = (right - left) / sum(w for _, w, _ in cols)
    xs, x = [], left
    for _, w, al in cols:
        xs.append((x, x + w * scale, al))
        x += w * scale

    def rule(yv, lw, color=INK):
        a.plot([left, right], [yv, yv], color=color, lw=lw, transform=a.transAxes)

    def put(row, yc, size=6.4, color=INK, weight="normal"):
        for (x0, x1, al), txt in zip(xs, row):
            if txt:
                a.text(x0 + 0.003 if al == "l" else x1 - 0.003, yc, txt, ha="left" if al == "l" else "right",
                       va="center", fontsize=size, color=color, fontweight=weight, transform=a.transAxes,
                       linespacing=1.1)

    y = 1 - 0.10 / H
    rule(y, 0.9)
    put([c for c, _, _ in cols], y - head_h / 2 / H, weight="bold")
    y -= head_h / H
    rule(y, 0.5)
    for row, bad in body:
        put(row, y - row_h / 2 / H, color=WARN if bad else INK)
        y -= row_h / H
    rule(y, 0.5, AXIS)
    put(group, y - grp_h / 2 / H, size=6.2, color=INK2)
    y -= grp_h / H
    rule(y, 0.9)
    save(fig, out, name)


def write_csv(units, out):
    keys = ["id", "joint", "side", "motor", "serial", "schedule", "total", "slow0", "slow1", "run0", "run1",
            "rate_early", "rate_late", "main_min", "fet_max", "R_mohm", "L_uh", "suspect"]
    with open(out / "summary.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for u in units:
            w.writerow([round(u[k], 4) if isinstance(u[k], float) else u[k] for k in keys])
    print(f"saved {out / 'summary.csv'}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=str(BASE / "data"))
    ap.add_argument("--out", default=str(BASE / "analysis" / "break_in_20261109"))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    units = [u for u in (load_unit(d) for d in sorted(Path(a.data).glob("ID*"))) if u]
    units.sort(key=lambda u: (u["motor"] != "6512", u["id"]))
    for u in units:
        print(f"ID{u['id']} {u['motor']} {u['side']} {u['joint']:11s} total {u['total']:5.1f}  slow {u['slow0']:.3f}->"
              f"{u['slow1']:.3f}  run {u['run0']:.3f}->{u['run1']:.3f}  rate {u['rate_early']:+6.1f} -> "
              f"{u['rate_late']:+5.1f} %/h  FET {u['fet_max']:.1f}  R {u['R_mohm']:.1f} L {u['L_uh']:.1f}"
              f"{'  SUSPECT' if u['suspect'] else ''}")
    for m in ("6512", "5010"):
        g = [u for u in units if u["motor"] == m]
        for k in ("slow0", "slow1", "run0", "run1"):
            mean, sd, cv = group_stats(g, k)
            print(f"  {m} {k}: {mean:.3f} +- {sd:.3f} (CV {cv:.1f}%)")
    write_csv(units, out)
    motor_figure(units, "6512", out, "fig1_6512_break_in")
    motor_figure(units, "5010", out, "fig2_5010_break_in")
    motor_table(units, "6512", out, "table1_6512_summary")
    motor_table(units, "5010", out, "table2_5010_summary")


if __name__ == "__main__":
    main()
