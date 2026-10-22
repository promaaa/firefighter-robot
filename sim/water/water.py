"""Water system of the final design F2 (two R2 tanks, Kamoer KLP02 24 V pump): pump and system curve, nozzle operating
points, jet range with air drag and jet break-up, spray time, recoil, and the fire size that the water can handle.

    uv run python sim/water/water.py          -> sim/water/results/water.md, water.json, *.png (about 20 s)
    uv run python sim/water/water.py check    self-check (about 5 s)

Status: tested in simulation. Every pump value comes from a datasheet or a shop page (see PUMPS). Nothing is measured.
The geometry (heights, volume) comes from cad/final/out_f2/r2.json, the tube path from out_f2/layout.json.
"""
import json
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / "aim"))
import ballistics  # noqa: E402  aim/ballistics.py: the constant-drag jet of the aim code

OUT = HERE / "results"
G, RHO, MU, SIGMA = 9.81, 998.0, 1.0e-3, 0.072  # water at 20 degC: kg/m3, Pa s, N/m
RHO_AIR, MU_AIR = 1.20, 1.81e-5  # air at 20 degC

# ---------------- geometry (cad/final, design F2: two R2 tanks outside the torso) ----------------
_R2 = json.loads((HERE.parent.parent / "cad/final/out_f2/r2.json").read_text())
NOZZLE_Z = 0.905  # m, nozzle over the floor at tilt 0 (layout.json nozzle_fitting). Tilt moves it a few cm: ignored.
WATER_TOP, WATER_FLOOR = (_R2["water_z_mm"][k] / 1000 for k in ("full_surface", "floor"))  # 0.6755, 0.5945 m
LIFT_FULL, LIFT_EMPTY = NOZZLE_Z - WATER_TOP, NOZZLE_Z - WATER_FLOOR  # 0.23 and 0.31 m
PUMP_INLET_Z = 0.703  # m, pump axis on the rear of the middle shelf (the CAD pump holder, layout.json pump_dwp385): suction
# lift 0.03 m (full) to 0.11 m (empty). The pump primes itself, so it can sit above the water. Included in LIFT_*.
TANK_L = _R2["volume_l"]  # L, both R2 tanks together (0.39 L each)
# ASSUMED tube path (cad/final/out_f2/layout.json suction_mm): the right tank outlet to the tee 0.29 m (left 0.20 m),
# tee + filter to the pump 0.06 m, pump to the deck slot 0.25 m, service loop around the pan axis 0.25 m, tilt arm
# to the nozzle 0.12 m. Sum 0.97 m, rounded up (the right line, full flow: conservative).
TUBE_L, TUBE_D = 1.0, 3.2e-3
DEAD_ML = 24.0  # mL, ASSUMED: about 12 mL per tank under the outlet (z 599) when the pump starts to suck air
# ---------------- pumps ----------------
# q in L/min, p in kPa at v_ref. Speed ~ voltage (ASSUMED). "centrifugal": q ~ V, p ~ V^2 (affinity laws).
# "displacement": q ~ V, stall pressure ~ stall torque ~ V.
# volts: the voltage the pump runs on. The KLP02 takes the 24 V motor bus after relay K1, through the DMC-101 pump
# controller: two 6S packs, 25.2 V full, 22.2 V nominal, 21.0 V low (3.5 V per cell). Pin 11 of the ROCK 3C switches it
# on and off (no PWM). The diode drop of the bus is ignored.
V_BUS = (21.0, 22.2, 25.2)  # V: low, nominal, full
PUMPS = {
    "KLP02": dict(label="Kamoer KLP02, 24 V, brushless, self-priming", kind="displacement", v_ref=24.0, volts=V_BUS[1],
                  q=[0.0, 0.7], p=[RHO * G * 30.0 / 1000, 0.0], amps=0.5, grams=250, self_priming=True,
                  config="filter and check valve",
                  source="Kamoer data for the 24 V brushless KLP02: flow 0.7 L/min or more, head 30 m or more (294 kPa), "
                         "suction lift 4 m or more, power 12 W or less (0.5 A at 24 V), about 250 g, switch control. "
                         "ESTIMATE from two datasheet points: a straight line from the free flow to the head at zero "
                         "flow (ASSUMED shape). The datasheet gives minimum values."),
    "DWP-385": dict(label="motorbank DWP-385, 2-stage gear (comparison only)", kind="displacement", v_ref=12.0,
                    volts=11.1, q=[0.0, 0.9], p=[167.0, 0.0], amps=1.1, grams=130, self_priming=True, config="filter and check valve",
                    source="motorbank DWP-385 12 V datasheet image on eleparts goods 3850995 (read 2026-10-15): "
                           "6-18 V, 900 cc/min and 0.4-0.65 A at no load, 540 cc/min and 0.7-1.1 A rated, locking "
                           "pressure 1.7 kgf/cm2 or more, 130 g, intermittent running. Straight line from free flow "
                           "to the locking pressure (ASSUMED). It passes 540 cc/min at 67 kPa. Run at 11.1 V (3S)."),
}
PUMP_SHAPES = {"linear (used)": 1.0, "quadratic": 2.0}  # p = p0 (1 - (q/q0)^n), sensitivity of two-point curves

# config: the fittings of the recommended build.
# ---------------- losses (ASSUMED K values, typical handbook values for barbs and small fittings) ----------------
BARB_D = 2.0e-3  # m, bore of an M5 hose barb for 3 mm tube
K_BARB = 1.0  # contraction into the barb bore + expansion into the tube, at the barb bore velocity
K_FILTER = 3.0  # inline mesh filter body, at the tube velocity
K_CHECK, P_CRACK = 2.0, 10.0  # check valve body (tube velocity) and cracking pressure (kPa): Sansyo SB, 10 kPa
CONFIGS = {"minimal": dict(barbs=4, filt=False, check=False),  # tank outlet barb + tee (2 barbs) + nozzle barb
           "filter": dict(barbs=6, filt=True, check=False),  # + filter with 2 barbs
           "filter and check valve": dict(barbs=8, filt=True, check=True)}  # + check valve with 2 barbs

# ---------------- nozzle ----------------
# A 3D-printer brass nozzle: 2 mm bore, cone, short land at the orifice = a short convergent nozzle.
# ASSUMED: velocity coefficient Cv 0.95, discharge coefficient Cd 0.85 (contraction Cc = Cd / Cv = 0.89).
CV, CD = 0.95, 0.85
NOZZLES_MM = (0.8, 1.0, 1.2, 1.5, 2.0)
ELEV_DEG = (-10, -5, 0, 5, 10, 15, 20, 25, 30)


def tube_dp(q, L=TUBE_L, d=TUBE_D):
    """Pa, Darcy friction. f = max(laminar 64/Re, Blasius): turbulent from Re about 1200 (conservative)."""
    v = q / (math.pi * d * d / 4)
    re = max(RHO * v * d / MU, 1e-9)
    return max(64 / re, 0.316 * re ** -0.25) * L / d * RHO * v * v / 2


def fittings_dp(q, barbs, filt, check):
    vb, vt = q / (math.pi * BARB_D ** 2 / 4), q / (math.pi * TUBE_D ** 2 / 4)
    dp = barbs * K_BARB * RHO * vb * vb / 2 + (K_FILTER * RHO * vt * vt / 2 if filt else 0.0)
    return dp + ((K_CHECK * RHO * vt * vt / 2 + P_CRACK * 1e3) if check else 0.0)


def pump_dp(pump, q, volts, shape=1.0):
    """Pa at flow q (m3/s). Two-point curves can take another shape exponent for the sensitivity table."""
    s = volts / pump["v_ref"]
    qq = q * 60000 / s  # L/min at the reference speed
    if len(pump["q"]) == 2 and shape != 1.0:
        p = pump["p"][0] * max(0.0, 1 - (qq / pump["q"][1]) ** shape)
    else:
        p = float(np.interp(qq, pump["q"], pump["p"], right=0.0))
    return (s * s if pump["kind"] == "centrifugal" else s) * p * 1e3


def operating_point(pump, d_mm, volts=None, lift=LIFT_EMPTY, config=None, shape=1.0, tube_l=TUBE_L, cd=CD):
    """Flow (L/min), nozzle pressure (kPa), exit speed (m/s), jet diameter (mm). None when the pump cannot lift."""
    a = math.pi * (d_mm / 1000) ** 2 / 4
    cfg = CONFIGS[config or pump["config"]]
    volts = volts or pump["volts"]

    def p_nozzle(q):  # pressure left for the nozzle at flow q
        return pump_dp(pump, q, volts, shape) - RHO * G * lift - tube_dp(q, tube_l) - fittings_dp(q, **cfg)

    def f(q):
        return p_nozzle(q) - RHO / 2 * (q / (cd * a)) ** 2

    q_hi = max(pump["q"]) / 60000 * volts / pump["v_ref"]
    if f(1e-12) <= 0:
        return None
    q = brentq(f, 1e-12, q_hi)
    p = RHO / 2 * (q / (cd * a)) ** 2
    return dict(q_lpm=q * 60000, p_kpa=p / 1e3, v=CV * math.sqrt(2 * p / RHO), dj_mm=d_mm * math.sqrt(cd / CV),
                p_pump_kpa=pump_dp(pump, q, volts, shape) / 1e3)


# ---------------- jet: intact column, then drops with drag ----------------
def breakup(v, dj_mm):
    """Intact length (m) of the jet column. Laminar: Weber's form L/d = ln(d/2 delta0) (We^0.5 + 3 We/Re) with
    ln(d/2 delta0) = 13.4 (Grant and Middleman 1966, via Trettel 2020, eq. 12). Turbulent: L/d = 8.51 We^0.32
    (Grant and Middleman 1966, confirmed by De Jarlais et al. 1983). We = rho v^2 d / sigma."""
    d = dj_mm / 1000
    we, re = RHO * v * v * d / SIGMA, RHO * v * d / MU
    return dict(we=we, oh=MU / math.sqrt(RHO * SIGMA * d), re=re, we_air=RHO_AIR * v * v * d / SIGMA,
                laminar=d * 13.4 * (we ** 0.5 + 3 * we / re), turbulent=d * 8.51 * we ** 0.32)


def cd_sphere(re):
    """Schiller and Naumann (1933) for Re < 1000, Newton value 0.44 above."""
    return 24 / re * (1 + 0.15 * re ** 0.687) if re < 1000 else 0.44


# Model cases. "best": long laminar column (short convergent nozzle), Rayleigh drops 1.89 d (Trettel 2020), drag
# times 0.6 for drops in a train (ASSUMED from two spheres in line 2 diameters apart: Cd/Cd_single = 0.62,
# Zapryanov and Toshev, quoted in OSTI report 6417393). "low": short turbulent column, drops of the jet size,
# drag of single drops.
MODELS = {"no drag": None,
          "best estimate": dict(breakup="laminar", drop=1.89, draft=0.6),
          "low estimate": dict(breakup="turbulent", drop=1.0, draft=1.0)}


def jet(v0, elev_deg, dj_mm, model, h0=NOZZLE_Z, n=200):
    """Path (x, z arrays) of the jet to the floor and the range (m). model: a MODELS value."""
    th = math.radians(elev_deg)
    vx, vz = v0 * math.cos(th), v0 * math.sin(th)
    t_floor = (vz + math.sqrt(vz * vz + 2 * G * h0)) / G
    t_b = t_floor if model is None else min(t_floor, breakup(v0, dj_mm)[model["breakup"]] / v0)
    t1 = np.linspace(0, t_b, n)
    x, z = vx * t1, h0 + vz * t1 - G * t1 * t1 / 2
    if t_b >= t_floor:
        return x, z, float(x[-1])
    D = model["drop"] * dj_mm / 1000

    def rhs(_, s):
        u, w = s[2], s[3]
        sp = math.hypot(u, w)
        k = 3 * model["draft"] * cd_sphere(max(RHO_AIR * sp * D / MU_AIR, 1e-6)) * RHO_AIR / (4 * RHO * D)
        return [u, w, -k * sp * u, -G - k * sp * w]

    floor = lambda _, s: s[1]  # noqa: E731
    floor.terminal, floor.direction = True, -1
    sol = solve_ivp(rhs, (0, 20), [x[-1], z[-1], vx, vz - G * t_b], events=floor, max_step=0.01, rtol=1e-8)
    xe, ze = sol.y_events[0][0][:2]
    return np.r_[x, sol.y[0], xe], np.r_[z, sol.y[1], ze], float(xe)


def equivalent_drag(v0, elev_deg, target_range, h0=NOZZLE_Z):
    """Constant k (1/m) of aim/ballistics.jet_path that gives the same floor range: a start value for
    ControllerConfig.drag before the jet is measured."""
    def rng(k):
        p = np.array(ballistics.jet_path(v0, math.radians(elev_deg), k, z_stop=-h0 - 0.01))
        return float(np.interp(-h0, p[::-1, 1], p[::-1, 0])) - target_range
    return brentq(rng, 0.0, 5.0) if rng(0.0) > 0 else 0.0


# ---------------- tables ----------------
def nozzle_table(pump, volts, lift=LIFT_EMPTY, config=None):
    rows = []
    for d in NOZZLES_MM:
        op = operating_point(pump, d, volts, lift, config)
        if op is None:
            rows.append(dict(d=d, op=None))
            continue
        bu = breakup(op["v"], op["dj_mm"])
        rng = {name: {e: jet(op["v"], e, op["dj_mm"], m)[2] for e in ELEV_DEG} for name, m in MODELS.items()}
        usable = TANK_L * 1000 - DEAD_ML - math.pi * TUBE_D ** 2 / 4 * TUBE_L * 1e6
        rows.append(dict(d=d, op=op, breakup=bu, ranges=rng, spray_s=usable / (op["q_lpm"] * 1000 / 60),
                         recoil_n=RHO * op["q_lpm"] / 60000 * op["v"],
                         grams_s=RHO * op["q_lpm"] / 60))
    return rows


def best_of(r, model):
    e = max(ELEV_DEG, key=lambda k: r["ranges"][model][k])
    return r["ranges"][model][e], e


# ---------------- fire ----------------
# Sardqvist (2002), "Water and other extinguishing agents", Swedish Rescue Services Agency, ISBN 91-7253-265-3:
# p. 313: heating water from 10 degC and evaporating it at 100 degC takes 2640 kJ/kg.
# Fig. 6.42, p. 318: water applied by hand needs 9-15 L/(m2 min) of fire floor area (design values with a margin).
Q_WATER = 2640.0  # kJ/kg
HAND_RATE = (9.0, 15.0)  # L/(m2 min) of fire floor area
# Peak heat release per floor area of a loose paper fire. NIST Fire Calorimetry Database, Test090 and Test091
# (waste paper baskets): 21.3 and 33.7 kW peak. ASSUMED basket footprint 0.07 m2 (30 cm diameter): 300-480 kW/m2.
HRR_PAPER = (300.0, 480.0)  # kW/m2


def fire_capacity(grams_s):
    """Upper limit of the heat the flow can take (all water evaporates), and the fire floor area it covers at the
    hand-application rates, with the peak heat release of a loose paper fire of that area."""
    area = [grams_s / (r * 1000 / 60) for r in HAND_RATE[::-1]]  # m2, at 15 then 9 L/(m2 min)
    return dict(kw_max=grams_s / 1000 * Q_WATER, area_m2=area, side_cm=[100 * math.sqrt(a) for a in area],
                hrr_kw=[area[0] * HRR_PAPER[0], area[1] * HRR_PAPER[1]])


# ---------------- plots ----------------
INK, INK2, SURF = "#0b0b0b", "#52514e", "#fcfcfb"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]


def style(ax):
    ax.set_facecolor(SURF)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.grid(True, color="#e4e3df", lw=0.6)
    ax.tick_params(colors=INK2)


def plot_curves(pump, volts, path):
    fig, ax = plt.subplots(figsize=(7.5, 4.6), facecolor=SURF)
    style(ax)
    q = np.linspace(0, max(pump["q"]) * volts / pump["v_ref"], 200)
    ax.plot(q, [pump_dp(pump, x / 60000, volts) / 1e3 for x in q], color=INK, lw=2, label="pump")
    for i, d in enumerate(NOZZLES_MM):
        a = math.pi * (d / 1000) ** 2 / 4
        need = [(RHO * G * LIFT_EMPTY + tube_dp(x / 60000) + fittings_dp(x / 60000, **CONFIGS[pump["config"]])
                 + RHO / 2 * (x / 60000 / (CD * a)) ** 2) / 1e3 for x in q]
        ax.plot(q, need, color=SERIES[i], lw=1.6)
        op = operating_point(pump, d, volts)
        if op:
            ax.plot(op["q_lpm"], op["p_pump_kpa"], "o", ms=8, color=SERIES[i], mec=SURF, mew=2)
            ax.annotate(f"{d} mm", (op["q_lpm"], op["p_pump_kpa"]), xytext=(6, 4), textcoords="offset points",
                        color=INK2, fontsize=9)
    ax.set_ylim(0, max(pump["p"]) * 1.25 * (volts / pump["v_ref"]) ** (2 if pump["kind"] == "centrifugal" else 1))
    ax.set_xlabel("Flow (L/min)", color=INK2)
    ax.set_ylabel("Pressure (kPa)", color=INK2)
    ax.set_title(f"{pump['label']}\npump curve at {volts:g} V, system curve per nozzle ({pump['config']})",
                 color=INK, fontsize=10)
    ax.legend(frameon=False, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_jets(op, d, path):
    fig, ax = plt.subplots(figsize=(8, 4), facecolor=SURF)
    style(ax)
    for e, c in zip((-10, 0, 15, 30), SERIES):
        for name, ls in (("no drag", ":"), ("best estimate", "-"), ("low estimate", "--")):
            x, z, _ = jet(op["v"], e, op["dj_mm"], MODELS[name])
            ax.plot(x, z, ls, color=c, lw=1.6 if ls == "-" else 1.0, label=f"{e:+d}°" if ls == "-" else None)
    bu = breakup(op["v"], op["dj_mm"])
    ax.axvline(bu["laminar"], color=INK2, lw=0.8)
    ax.annotate("column breaks up (best estimate)", (bu["laminar"], 0.05), xytext=(4, 0), textcoords="offset points",
                color=INK2, fontsize=8)
    ax.set_xlim(0, None)
    ax.set_ylim(0, None)
    ax.set_aspect("equal")
    ax.set_xlabel("Distance from the nozzle (m)", color=INK2)
    ax.set_ylabel("Height (m)", color=INK2)
    ax.set_title(f"{d} mm nozzle, {op['v']:.1f} m/s: solid = best estimate, dashed = low estimate, "
                 "dotted = no drag", color=INK, fontsize=9)
    ax.legend(frameon=False, title="Elevation", fontsize=8, title_fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def plot_ranges(rows, path, name):
    fig, ax = plt.subplots(figsize=(7.5, 4.2), facecolor=SURF)
    style(ax)
    for r, c in zip(rows, SERIES):
        if r["op"] is None:
            continue
        lo = [r["ranges"]["low estimate"][e] for e in ELEV_DEG]
        hi = [r["ranges"]["best estimate"][e] for e in ELEV_DEG]
        ax.fill_between(ELEV_DEG, lo, hi, color=c, alpha=0.15, lw=0)
        ax.plot(ELEV_DEG, hi, color=c, lw=2)
        ax.annotate(f"{r['d']} mm", (ELEV_DEG[-1], hi[-1]), xytext=(4, 0), textcoords="offset points", color=INK2,
                    fontsize=9, va="center")
    ax.set_xlabel("Elevation (degrees)", color=INK2)
    ax.set_ylabel("Range to the floor (m)", color=INK2)
    ax.set_title(f"{name}: range from 0.9 m, line = best estimate, band down to the low estimate", color=INK,
                 fontsize=10)
    ax.set_xlim(ELEV_DEG[0], ELEV_DEG[-1] + 4)
    ax.set_ylim(0, None)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------- report ----------------
def md_table(head, rows):
    return "\n".join(["| " + " | ".join(head) + " |", "|" + "---|" * len(head)] +
                     ["| " + " | ".join(str(c) for c in r) + " |" for r in rows])


def report(res):
    L = ["# Water system numbers (generated by sim/water/water.py)", "",
         "Status: tested in simulation. Pump values: datasheets or shop pages (see `PUMPS` in the script). Not measured.",
         f"Geometry: nozzle {NOZZLE_Z:.3f} m over the floor, lift {LIFT_FULL:.2f} m (full tank) to {LIFT_EMPTY:.2f} m "
         f"(empty), {TUBE_L} m of {TUBE_D * 1000:.1f} mm tube, {TANK_L:.3f} L in two R2 tanks (F2). "
         f"Ranges: from the nozzle to the floor.", ""]
    for key, block in res["pumps"].items():
        pump = PUMPS[key]
        L += [f"## {pump['label']} at {block['volts']:g} V, fittings: {pump['config']}", "",
              f"Source: {pump['source']}", ""]
        rows = []
        for r in block["rows"]:
            if r["op"] is None:
                rows.append([r["d"], "no flow"] + [""] * 10)
                continue
            op, bu = r["op"], r["breakup"]
            b, eb = best_of(r, "best estimate")
            lo, el = best_of(r, "low estimate")
            nd, en = best_of(r, "no drag")
            rows.append([r["d"], f"{op['q_lpm']:.2f}", f"{r['grams_s']:.1f}", f"{op['p_kpa']:.1f}", f"{op['v']:.1f}",
                         f"{1000 * bu['laminar']:.0f} / {1000 * bu['turbulent']:.0f}",
                         f"{nd:.1f} ({en:+d}°)", f"{b:.1f} ({eb:+d}°)", f"{lo:.1f} ({el:+d}°)",
                         f"{r['spray_s'] / 60:.1f}", f"{1000 * r['recoil_n']:.0f}"])
        L += [md_table(["Nozzle (mm)", "Flow (L/min)", "Flow (g/s)", "Nozzle pressure (kPa)", "Exit speed (m/s)",
                        "Intact column, laminar / turbulent (mm)", "Max range, no drag (m)",
                        "Max range, best estimate (m)", "Max range, low estimate (m)", "Spray time, both tanks full (min)",
                        "Recoil (mN)"], rows), ""]
        L += ["Range to the floor per elevation (best estimate / low estimate, m):", ""]
        L += [md_table(["Nozzle (mm)"] + [f"{e:+d}°" for e in ELEV_DEG],
                       [[r["d"]] + [f"{r['ranges']['best estimate'][e]:.1f} / {r['ranges']['low estimate'][e]:.1f}"
                                    for e in ELEV_DEG] for r in block["rows"] if r["op"]]), ""]
    s = res["sensitivity"]
    L += ["## Sensitivity (recommended pump and nozzle)", "", md_table(["Case", "Flow (L/min)", "Exit speed (m/s)",
                                                                       "Best-estimate range at +15° (m)"],
                                                                      [[k, f"{v['q_lpm']:.2f}", f"{v['v']:.1f}",
                                                                        f"{v['range15']:.1f}"] for k, v in s.items()]),
          "", f"Equivalent constant drag for `aim/` (`ControllerConfig.drag`): {res['k_equiv']:.3f} 1/m "
              f"(fit to the best estimate at +15°).",
          f"Jet speed against the bus voltage: v0 = {res['recommended']['v']:.1f} m/s × (V / {V_BUS[1]:g} V)^"
          f"{res['v0_exponent']:.2f} (fit from {V_BUS[0]:g} V and {V_BUS[2]:g} V, ASSUMED speed ~ voltage).", ""]
    f = res["fire"]
    L += ["## Fire capacity (recommended pump and nozzle)", "",
          f"- Water flow {f['grams_s']:.1f} g/s. If all of it evaporates, it takes {f['kw_max']:.0f} kW. This is an "
          f"upper limit: on a real fire much of the water runs off.",
          f"- At the hand-application rates of {HAND_RATE[0]:g} to {HAND_RATE[1]:g} L/(m2 min), the flow covers "
          f"{10000 * f['area_m2'][0]:.0f} to {10000 * f['area_m2'][1]:.0f} cm2 of fire floor area, a square of "
          f"{f['side_cm'][0]:.0f} to {f['side_cm'][1]:.0f} cm.",
          f"- A loose paper fire of that area peaks at about {f['hrr_kw'][0]:.0f} to {f['hrr_kw'][1]:.0f} kW.", ""]
    return "\n".join(L)


REC_PUMP, REC_NOZZLE = "KLP02", 1.0  # pump and nozzle of docs/hardware/water.md


def main():
    OUT.mkdir(exist_ok=True)
    res = {"pumps": {}}
    for key, pump in PUMPS.items():
        res["pumps"][key] = {"volts": pump["volts"], "rows": nozzle_table(pump, pump["volts"])}
        plot_curves(pump, pump["volts"], OUT / f"curves_{key}.png")
    pump, d = PUMPS[REC_PUMP], REC_NOZZLE
    rows = res["pumps"][REC_PUMP]["rows"]
    r = next(r for r in rows if r["d"] == d)
    plot_ranges(rows, OUT / "ranges.png", f"{REC_PUMP} at {pump['volts']:g} V")
    plot_jets(r["op"], d, OUT / "jets.png")
    cases = {"baseline: empty tank, filter and check valve": dict(),
             "full tank": dict(lift=LIFT_FULL),
             "no check valve": dict(config="filter"),
             "no filter, no check valve": dict(config="minimal"),
             "tube 0.7 m": dict(tube_l=0.7),
             "tube 1.3 m": dict(tube_l=1.3),
             f"bus full, {V_BUS[2]:g} V": dict(volts=V_BUS[2]),
             f"bus low, {V_BUS[0]:g} V": dict(volts=V_BUS[0]),
             "datasheet voltage, 24 V": dict(volts=24.0),
             "nozzle Cd 0.65 (sharp orifice)": dict(cd=0.65),
             "quadratic pump curve": dict(shape=2.0)}
    sens = {}
    for name, kw in cases.items():
        op = operating_point(pump, d, **kw)
        sens[name] = dict(q_lpm=op["q_lpm"], v=op["v"], range15=jet(op["v"], 15, op["dj_mm"], MODELS["best estimate"])[2])
    res["sensitivity"] = sens
    res["k_equiv"] = equivalent_drag(r["op"]["v"], 15, r["ranges"]["best estimate"][15])
    lo, hi = (operating_point(pump, d, volts=v)["v"] for v in (V_BUS[0], V_BUS[2]))
    res["v0_exponent"] = math.log(hi / lo) / math.log(V_BUS[2] / V_BUS[0])  # v0 ~ v0_nominal (V / V_nominal)^exponent
    res["fire"] = fire_capacity(r["grams_s"]) | {"grams_s": r["grams_s"]}
    res["recommended"] = dict(pump=REC_PUMP, nozzle_mm=d, volts=pump["volts"], amps=pump["amps"],
                              watts=pump["amps"] * 24.0, q_lpm=r["op"]["q_lpm"], v=r["op"]["v"],
                              spray_min=r["spray_s"] / 60, usable_l=r["spray_s"] * r["op"]["q_lpm"] / 60,
                              range15=r["ranges"]["best estimate"][15], range15_low=r["ranges"]["low estimate"][15])
    (OUT / "water.json").write_text(json.dumps(res, indent=1, default=float))
    (OUT / "water.md").write_text(report(res))
    print(report(res))


def check():
    pump = PUMPS["KLP02"]
    op = operating_point(pump, 1.0, 24.0, config="minimal")
    a = math.pi * 1e-6 / 4
    # the nozzle flow equation holds at the operating point
    assert abs(op["q_lpm"] / 60000 - CD * a * math.sqrt(2 * op["p_kpa"] * 1e3 / RHO)) < 1e-9
    # energy balance: pump pressure = lift + tube + fittings + nozzle
    q = op["q_lpm"] / 60000
    lhs = op["p_pump_kpa"] * 1e3
    rhs = RHO * G * LIFT_EMPTY + tube_dp(q) + fittings_dp(q, **CONFIGS["minimal"]) + op["p_kpa"] * 1e3
    assert abs(lhs - rhs) < 1.0, (lhs, rhs)
    # no-drag range equals the closed form
    v, th, h = 7.0, math.radians(20), NOZZLE_Z
    x_exact = v * math.cos(th) * (v * math.sin(th) + math.sqrt((v * math.sin(th)) ** 2 + 2 * G * h)) / G
    assert abs(jet(v, 20, 1.0, None)[2] - x_exact) < 1e-9
    # drag only shortens the jet, and the low estimate is the shortest
    rn, rb, rl = (jet(v, 20, 1.0, m)[2] for m in MODELS.values())
    assert rl < rb < rn, (rn, rb, rl)
    # more lift or a filter means less flow; a higher voltage means more flow
    assert operating_point(pump, 1.0, 24.0, lift=0.5, config="minimal")["q_lpm"] < op["q_lpm"]
    assert operating_point(pump, 1.0, 24.0, config="filter and check valve")["q_lpm"] < op["q_lpm"]
    assert operating_point(pump, 1.0, 25.2, config="minimal")["v"] > op["v"]
    # the two-point curve passes the datasheet points: 0.7 L/min free flow and 30 m head at zero flow (24 V)
    assert abs(pump_dp(pump, 0.7 / 60000, 24.0)) < 1e-6 and abs(pump_dp(pump, 0.0, 24.0) - RHO * G * 30.0) < 1e-6
    # the equivalent drag reproduces the target range in aim/ballistics
    k = equivalent_drag(v, 15, 0.8 * jet(v, 15, 1.0, None)[2])
    p = np.array(ballistics.jet_path(v, math.radians(15), k, z_stop=-h - 0.01))
    assert abs(np.interp(-h, p[::-1, 1], p[::-1, 0]) - 0.8 * jet(v, 15, 1.0, None)[2]) < 0.01
    # Weber laminar form: about 0.35 m intact length for a 1 mm jet at 7 m/s (We 680, Re 7000)
    assert 0.3 < breakup(7.0, 1.0)["laminar"] < 0.6
    print("water.py check: OK")


if __name__ == "__main__":
    check() if sys.argv[1:] == ["check"] else main()
