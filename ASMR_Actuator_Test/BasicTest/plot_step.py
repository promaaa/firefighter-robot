"""05_step_response.py 결과를 그림으로. 기록에 저장된 드라이버·모델 값으로 모델 응답을 다시 계산해 겹쳐 그린다.

    python plot_step.py data/ID01/step_20261104_153000.csv
    python plot_step.py data/ID01/step_....csv --inertia 0.003   # 부하 관성을 바꿔 다시 비교
"""
import argparse
import math
from pathlib import Path

from bench import model, params, util


def plot(path, inertia=None, show=True):
    import matplotlib.pyplot as plt

    meta, c = util.read_log(path)
    t, ref, pos = c["t"], c["target_turn"], c["pos_turn"]
    p0 = meta.get("p0_turn", pos[0])
    cfg = model.DriveConfig(**meta["drive"])
    plant = model.Plant(**meta["plant"])
    if inertia is not None:
        plant.inertia = inertia
    pred = model.simulate(cfg, plant, t, ref, pos[0])

    deg = params.turn_to_out_deg
    has_iq = any(not math.isnan(v) for v in c["iq"])
    fig, axes = plt.subplots(2 if has_iq else 1, 1, sharex=True, figsize=(10, 6 if has_iq else 4), squeeze=False)
    ax = axes[0][0]
    ax.step(t, [deg(r - p0) for r in ref], where="post", color="0.6", lw=1, label="목표")
    ax.plot(t, [deg(p - p0) for p in pos], ".-", ms=2, lw=1, label="실측 (엔코더)")
    ax.plot(t, [deg(p - p0) for p in pred], lw=1.2, label=f"모델 (관성 {plant.inertia:.4f})")
    ax.set_ylabel("출력 각도 [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right")
    ax.set_title(f"ID{meta.get('id')} {meta.get('motor')} {meta.get('link')} gains={meta.get('gains')}  "
                 f"pos_gain {cfg.pos_gain:g}, vel_gain {cfg.vel_gain:g}")
    if has_iq:
        axes[1][0].plot(t, c["iq"], lw=1)
        axes[1][0].set_ylabel("Iq [A]")
        axes[1][0].grid(alpha=0.3)
    axes[-1][0].set_xlabel("시간 [s]")
    fig.tight_layout()
    png = Path(path).with_suffix(".png")
    fig.savefig(png, dpi=130)
    print(f"그림 저장: {png}")
    if show:
        plt.show()


def _korean_font():
    import matplotlib
    matplotlib.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False


_korean_font()

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--inertia", type=float, default=None, help="모델 관절 관성 [kg*m^2] 덮어쓰기")
    ap.add_argument("--no-show", action="store_true")
    a = ap.parse_args()
    plot(a.csv, a.inertia, not a.no_show)
