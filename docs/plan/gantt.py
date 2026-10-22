"""Team schedule board as a Gantt chart: lanes, bars and milestones of the team plan (docs/plan/README.md).

    uv run python docs/plan/gantt.py      # -> docs/plan/gantt.svg

Week 1 starts on Sunday 11 October 2026. Edit TASKS and MILESTONES together with the README.
"""
from datetime import date, timedelta
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

W1 = date(2026, 10, 11)
TODAY = date(2026, 10, 22)
HW, SW, INT = "#c8553d", "#2f6690", "#2b2d33"
TASKS = [  # lane, bar text, first week, last week (inclusive), colour
    ("Legs, gearboxes, torso", "print, assemble, bench-test", 1, 3, HW),
    ("Water module", "bench tests", 2, 3, HW),
    ("Water module", "build and mount", 4, 6, HW),
    ("Simulation and training", "model, RL policy, sim-to-sim", 1, 5, SW),
    ("Real walking tests", "gantry to free walking", 3.5, 6, SW),
    ("Fire detection", "data, train, run onboard", 2, 5, SW),
    ("Targeting", "calibrate, close loop", 5, 8, SW),
    ("Integration and demo", "weekly tests, rehearsal", 4, 8, INT),
]
MILESTONES = [  # code, name, date, extra label drop (rows)
    ("M1 · W3", "Robot assembly complete", date(2026, 10, 31), 0.0),
    ("M2 · W6", "Robot stands on gantry", date(2026, 11, 21), 0.0),
    ("M3 · W8", "Walks to fire and aims", date(2026, 12, 5), 0.9),
    ("M4 · W9", "Final demo, 10 Dec", date(2026, 12, 10), 0.0),
]


def week_x(w):
    """x position of the start of week w (1-based, may be fractional)."""
    return w - 1


def main():
    plt.rcParams["font.family"] = ["Inter", "DejaVu Sans"]
    plt.rcParams["svg.fonttype"] = "path"
    plt.rcParams["svg.hashsalt"] = "sobang"
    lanes = list(dict.fromkeys(t[0] for t in TASKS))
    fig, ax = plt.subplots(figsize=(13, 6.0))
    for i in range(10):
        if i % 2 == 0:
            ax.axvspan(i, i + 1, color="#f2f1ee", zorder=0)
        start = W1 + timedelta(days=7 * i)
        ax.text(i + 0.5, len(lanes) + 0.55, f"{i + 1}", ha="center", fontsize=10, color="#555555")
        ax.text(i + 0.5, len(lanes) + 0.15, f"{start:%m/%d}", ha="center", fontsize=8.5, color="#777777")
    for lane, text, a, b, col in TASKS:
        y = len(lanes) - 1 - lanes.index(lane)
        ax.barh(y, week_x(b + 1) - week_x(a) - 0.08, left=week_x(a) + 0.04, height=0.62, color=col, zorder=2)
        ax.text(week_x(a) + 0.16, y, text, va="center", fontsize=9.5, color="white", zorder=3)
    ax.set_yticks(range(len(lanes)), lanes[::-1], fontsize=10)
    ymil = -1.0
    ax.axhline(ymil, color="#999999", lw=0.8, zorder=1)
    for code, name, d, drop in MILESTONES:
        x = ((d - W1).days + 1) / 7
        demo = "demo" in name
        col = "#c8553d" if demo else "#2b2d33"
        ax.plot(x, ymil, marker="D", ms=9, color=col, zorder=3)
        if drop:
            ax.plot([x, x], [ymil - 0.25, ymil - 0.35 - drop], color=col, lw=0.8, zorder=2)
        ax.text(x, ymil - 0.42 - drop, code, ha="center", va="top", fontsize=9, weight="bold", color=col)
        ax.text(x, ymil - 0.80 - drop, name, ha="center", va="top", fontsize=8.5, color="#444444")
    xt = ((TODAY - W1).days + 0.5) / 7
    ax.plot([xt, xt], [ymil + 0.3, len(lanes) - 0.45], color="#c8553d", lw=1.2, ls="--", zorder=4)
    ax.text(xt + 0.05, len(lanes) - 0.42, "today", fontsize=8, color="#c8553d")
    ax.set_xlim(0, 10)
    ax.set_ylim(ymil - 2.5, len(lanes) + 0.9)
    ax.set_xticks([])
    for s in ("top", "right", "bottom", "left"):
        ax.spines[s].set_visible(False)
    ax.tick_params(axis="y", length=0)
    ax.text(-0.02, len(lanes) + 0.55, "WEEK", ha="right", fontsize=9, color="#555555", transform=ax.transData)
    fig.tight_layout()
    fig.savefig(Path(__file__).with_name("gantt.svg"), metadata={"Date": None})


if __name__ == "__main__":
    main()
