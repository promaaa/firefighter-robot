"""스텝 응답 지표 (출력축 기준)."""
import math

from . import params


def edges(t, ref):
    """ref가 바뀌는 지점 -> [(시작 인덱스, 끝 인덱스(제외), r0, r1)]."""
    idx = [i for i in range(1, len(ref)) if ref[i] != ref[i - 1]]
    out = []
    for n, i in enumerate(idx):
        j = idx[n + 1] if n + 1 < len(idx) else len(ref)
        out.append((i, j, ref[i - 1], ref[i]))
    return out


def metrics(t, pos, i, j, r0, r1):
    """i..j 구간에서 r0 -> r1 스텝의 지표. 시간 [s], 각도 [출력 deg].

    진행률은 스텝 직전의 실제 위치에서 목표까지로 잰다 (마찰로 앞 스텝이 덜 간 만큼을 빼고).
    """
    tc = t[i]
    span = r1 - r0
    start = pos[i - 1] if i > 0 else pos[i]
    y = [(p - start) / (r1 - start) for p in pos[i:j]]
    tt = [x - tc for x in t[i:j]]

    def first(th):
        return next((tt[k] for k, v in enumerate(y) if v >= th), None)

    t05, t10, t90, t95 = first(0.05), first(0.10), first(0.90), first(0.95)
    tail = pos[i + int(0.8 * (j - i)):j]
    final = sum(tail) / len(tail)
    return {
        "step_deg": params.turn_to_out_deg(span),
        "delay_s": t05,
        "rise_s": (t90 - t10) if (t90 is not None and t10 is not None) else None,
        "t95_s": t95,
        "overshoot_pct": max(0.0, (max(y) - 1.0) * 100.0),
        "ss_err_deg": params.turn_to_out_deg(r1 - final) * math.copysign(1, span),
    }


def fmt_metrics(mm):
    def f(v, unit, scale=1.0, nd=3):
        return "  -  " if v is None else f"{v * scale:.{nd}f}{unit}"
    return (f"지연 {f(mm['delay_s'], 'ms', 1000, 1)}, 상승 {f(mm['rise_s'], 'ms', 1000, 1)}, "
            f"95% {f(mm['t95_s'], 'ms', 1000, 1)}, 오버슈트 {mm['overshoot_pct']:.1f}%, "
            f"정상상태 오차 {mm['ss_err_deg']:+.3f} deg")
