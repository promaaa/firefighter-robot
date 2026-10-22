"""Electrical numbers of the final design F2: motor bus power and regen from the walking simulation, the brake
chopper, the pump on the motor bus, the 3S and 5 V budget, battery time, and the fuse and wire check of harness.csv.

    uv run python docs/hardware/power_calc.py          -> power_calc.md, power_calc.json, brake_chopper.png (1 min)
    uv run python docs/hardware/power_calc.py check    fuse and wire check of harness.csv + chopper self-check (1 s)

Status: tested in simulation. The walking runs use sim/sim2sim/hw_risks.py (case 7: K 20, D 2, 6 Nm, zero errors
of 2 degrees, 10 ms command latency) on the F2 model (sim/payload/models/bhl_F2.xml, two R2 tanks, 15.99 kg). Nothing is measured.
The pump numbers come from sim/water/results/water.json (Kamoer KLP02 24 V with the DMC-101, after relay K1).
"""
import csv
import json
import math
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUT = HERE

# ---------------- motor bus ----------------
P_STANDBY = 1.5  # W per drive, ASSUMED (ODrive v3.6 class board, gate drivers and MCU on, motor idle)
V_BATT = 25.2  # V, full 6S pack. The diode drop (about 0.3 V at low current) is ignored: worst case for regen.
V_TRIP = 26.0  # V, course dc_bus_overvoltage_trip_level
C_BUS = 12 * 5 * 100e-6  # F, 5 x 100 uF per XDrive MINI (photos, as in hw_risks.py)
I_POS_MAX = 12.0  # A, course dc_max_positive_current per drive: the brake current counts in it
RAMP = (25.5, 25.9)  # V, dc_bus_overvoltage_ramp_start / _end (proposal)
# Brake options: (number of drives with a resistor on AUX, ohm per resistor)
CHOPPERS = {"none": (0, 1.0), "1 x 2.2 ohm": (1, 2.2), "2 x 2.2 ohm": (2, 2.2), "1 x 4.7 ohm": (1, 4.7),
            "2 x 4.7 ohm": (2, 4.7)}
DUTY_MAX = 0.95  # ODrive 0.5.1 brake duty limit (low_level.cpp: "to allow bootstrap caps to charge")
SEEDS = 10
SUB = 10  # chopper sub-steps per physics step: the ramp time constant (0.2 ms) is shorter than 0.5 ms


def chopper(bus, dt, n_res, ohm, ramp=RAMP, v_batt=V_BATT, c=C_BUS):
    """Bus voltage with the diode (no current back to the batteries), the drive capacitors and n_res brake
    resistors with the ODrive over-voltage ramp: duty 0 at ramp[0], 1 at ramp[1].
    bus: net drive input power per step (W, + = drives take power). Returns peak V, resistor energy and power."""
    v, e_r, p_peak, v_peak, events, on, e_ev = v_batt, 0.0, 0.0, v_batt, [], False, 0.0
    h = dt / SUB
    for p in bus:
        if p >= 0 and v <= v_batt:  # the batteries feed the drives, nothing to do
            if on:
                events.append(e_ev)
                on, e_ev = False, 0.0
            continue
        for _ in range(SUB):
            duty = min(max((v - ramp[0]) / (ramp[1] - ramp[0]), 0.0), 1.0) if n_res else 0.0
            pr = n_res * duty * v * v / ohm
            e = max(0.5 * c * v * v - (p + pr) * h, 0.5 * c * v_batt * v_batt)  # the diode holds v >= v_batt
            v = math.sqrt(2 * e / c)
            e_r, e_ev, p_peak, v_peak = e_r + pr * h, e_ev + pr * h, max(p_peak, pr), max(v_peak, v)
        on = on or pr > 0
    if on:
        events.append(e_ev)
    return dict(v_peak=v_peak, e_r=e_r, p_peak=p_peak, e_event=max(events, default=0.0), n_events=len(events))


def walk_job(cmd_name, seed):
    import hw_risks as hw  # sim/sim2sim/hw_risks.py
    log = []
    r = hw.simulate(hw.CASES["7"][1], hw.s2s.COMMANDS[cmd_name], seed, log=log)
    a = np.array(log)
    tau, qd = a[:, :12], a[:, 12:]
    p = tau * qd + 1.5 * hw.R_PHASE * (tau / hw.N / hw.KT) ** 2  # W per drive, as hw_risks.bus_metrics
    regen = p.sum(axis=1)  # no standby power: worst case for the brake sizing
    bus = regen + 12 * P_STANDBY  # battery side
    dt = hw.s2s.PHYSICS_DT
    w = round(0.02 / dt)  # 20 ms window for the fuse and wire peaks
    avg20 = np.convolve(bus, np.ones(w) / w, mode="valid")
    out = dict(cmd=cmd_name, seed=seed, fall=r["fall"], seconds=len(bus) * dt,
               p_batt=float(np.maximum(bus, 0).mean()), p_net=float(bus.mean()), p_peak=float(bus.max()),
               p_peak20=float(avg20.max()), p_regen=float(-regen.min()), p_drive_peak=float(p.max()),
               e_regen=float(-np.minimum(regen, 0).sum() * dt), p_drive_regen=float(-p.min()))
    out["chopper"] = {k: chopper(regen, dt, n, ohm) for k, (n, ohm) in CHOPPERS.items()}
    # every drive has its own resistor and dumps only its own regen (brake_current = -Ibus): the bus never charges
    own = -np.minimum(p, 0)
    out["chopper"]["12 x own resistor, no ramp"] = dict(
        v_peak=chopper(np.maximum(p, 0).sum(axis=1), dt, 0, 1.0)["v_peak"], e_r=float(own.sum() * dt),
        p_peak=float(own.max()), e_event=float("nan"), n_events=0)
    if cmd_name == "vx 0.2 + yaw 0.5" and seed == 0:  # one trace for the plot
        i0 = int(np.argmin(regen))
        s = slice(max(i0 - 400, 0), i0 + 400)
        out["trace"] = dict(t=(np.arange(len(regen))[s] * dt).tolist(), bus=regen[s].tolist(),
                            v={k: trace_v(regen[s], dt, *CHOPPERS[k]) for k in ("none", "2 x 4.7 ohm")})
    return out


def trace_v(bus, dt, n_res, ohm):
    v, vs, h = V_BATT, [], dt / SUB
    for p in bus:
        for _ in range(SUB):
            duty = min(max((v - RAMP[0]) / (RAMP[1] - RAMP[0]), 0.0), 1.0) if n_res else 0.0
            e = max(0.5 * C_BUS * v * v - (p + n_res * duty * v * v / ohm) * h, 0.5 * C_BUS * V_BATT ** 2)
            v = math.sqrt(2 * e / C_BUS)
        vs.append(v)
    return vs


def motor_bus():
    sys.path[:0] = [str(ROOT / "sim/sim2sim"), str(ROOT / "sim/payload")]
    import hw_risks as hw
    hw.MODEL = "bhl_F2"  # set before the pool forks, as sim/payload/final_check.py does
    jobs = [(c, s) for c in hw.s2s.COMMANDS for s in range(SEEDS)]
    with Pool(os.cpu_count()) as pool:
        return pool.starmap(walk_job, jobs, chunksize=1)


# ---------------- 3S and 5 V budget ----------------
# (rail, load, typical A, peak A, source). 5 V loads are at 5 V, 12 V loads at the 3S voltage.
LOADS = [
    ("5V", "ROCK 3C at full CPU load", 1.2, 2.0, "estimate 5-6 W (parts.md). Radxa asks for a 5 V 3 A supply"),
    ("5V", "2 x CANable V2.0 Pro (isolated)", 0.2, 0.25, "ASSUMED 100 mA each (isolated DC-DC + STM32G431)"),
    ("5V", "Camera B0201", 0.2, 0.3, "listing: 300 mA max"),
    ("5V", "IMU IM10A on UART", 0.012, 0.02, "listing: 12 mA"),
    ("5V", "Bus servo adapter, USB side", 0.02, 0.05, "ASSUMED"),
    ("12V", "Pan-tilt: 2 x ST3215", 0.3, 5.4, "Waveshare: stall 2.7 A each at 12 V. Typical ASSUMED"),
    ("12V", "Push-lock switch lamps, meters", 0.05, 0.05, "ASSUMED 15-20 mA each"),
]
BUCK_EFF, BOOST_EFF = 0.90, 0.90  # ASSUMED converter efficiencies
COIL_W = 24 * (0.176 + 0.1)  # W: K1 AEP18024 coil 176 mA (datasheet) + K2 KKA-K4 coil about 0.1 A (ASSUMED)
V3S = (12.6, 11.1, 10.5)  # V: full, nominal, lowest use (3.5 V per cell)
PACK_3S_WH, PACK_6S_WH, USABLE = 11.1 * 6.0, 22.2 * 5.0, 0.8
PUMP_DUTY = 0.1  # ASSUMED: the pump sprays 10 % of the mission time


def budget():
    loads = [list(x) for x in LOADS]
    i5 = [sum(x[k] for x in loads if x[0] == "5V") for k in (2, 3)]
    i12 = [sum(x[k] for x in loads if x[0] == "12V") for k in (2, 3)]
    p_in = [5.0 * i5[k] / BUCK_EFF + COIL_W / BOOST_EFF + 11.1 * i12[k] for k in (0, 1)]
    return dict(loads=loads, i5=i5, i12=i12, p3s=p_in, i3s_nom=p_in[0] / 11.1, i3s_peak_low=p_in[1] / V3S[2] * 1.0,
                hours_3s=PACK_3S_WH * USABLE / p_in[0])


# ---------------- harness check ----------------
# Copper wire: chassis-wiring ampacity (A) and resistance (ohm/m). Source: PowerStream "Wire gauge and current
# limits" (chassis wiring column), https://www.powerstream.com/Wire_Size.htm. Silicone insulation takes more heat,
# so these are conservative.
AMPACITY = {10: 55, 12: 41, 14: 32, 16: 22, 18: 16, 20: 11, 22: 7, 24: 3.5, 26: 2.2}
OHM_M = {10: 3.277e-3, 12: 5.211e-3, 14: 8.286e-3, 16: 13.17e-3, 18: 20.95e-3, 20: 33.31e-3, 22: 52.96e-3,
         24: 84.22e-3, 26: 133.9e-3}


def check_harness(path=HERE / "harness.csv"):
    """Every wire: its fuse (own or upstream) is not above the wire ampacity, the continuous current is at most
    80 % of the fuse, and the drop over the wire is at most 3 % of the rail voltage (one way, x 2 for a pair)."""
    rows, errors = list(csv.DictReader(open(path))), []
    for r in rows:
        if None in r or None in r.values():
            errors.append(f"{r['id']}: wrong number of fields (a comma inside a field?)")
            continue
        if not r["awg"].isdigit():  # USB cables
            continue
        awg, ic, fuse = int(r["awg"]), float(r["i_cont_a"]), float(r["fuse_a"] or 0)
        pair = r["pair"] == "yes"
        if fuse and fuse > AMPACITY[awg]:
            errors.append(f"{r['id']}: fuse {fuse} A > {AMPACITY[awg]} A for {awg} AWG")
        if fuse and ic > 0.8 * fuse:
            errors.append(f"{r['id']}: continuous {ic} A > 80 % of the {fuse} A fuse")
        if not fuse and ic > 0.8 * AMPACITY[awg]:
            errors.append(f"{r['id']}: no fuse and {ic} A > 80 % of the wire ampacity")
        drop = (2 if pair else 1) * float(r["length_mm"]) / 1000 * OHM_M[awg] * ic
        if float(r["v_nom"]) and drop > 0.03 * float(r["v_nom"]):
            errors.append(f"{r['id']}: drop {drop:.2f} V > 3 % of {r['v_nom']} V")
        r["drop_v"] = drop
    return rows, errors


# ---------------- report ----------------
def summary(runs):
    walk = [r for r in runs if r["cmd"] != "stand" and not r["fall"]]
    stand = [r for r in runs if r["cmd"] == "stand" and not r["fall"]]
    ch = {k: dict(v_peak=max(r["chopper"][k]["v_peak"] for r in runs),
                  p_peak=max(r["chopper"][k]["p_peak"] for r in runs),
                  e_event=max(r["chopper"][k]["e_event"] for r in runs),
                  p_mean=max(r["chopper"][k]["e_r"] / r["seconds"] for r in runs),
                  events_s=max(r["chopper"][k]["n_events"] / r["seconds"] for r in runs)) for k in runs[0]["chopper"]}
    return dict(runs=len(runs), falls=sum(bool(r["fall"]) for r in runs),
                p_batt_walk=float(np.mean([r["p_batt"] for r in walk])),
                p_batt_walk_max=max(r["p_batt"] for r in walk),
                p_batt_stand=float(np.mean([r["p_batt"] for r in stand])),
                p_peak=max(r["p_peak"] for r in runs), p_peak20=max(r["p_peak20"] for r in runs),
                p_regen=max(r["p_regen"] for r in runs), e_regen_mean_w=max(r["e_regen"] / r["seconds"] for r in runs),
                p_drive_peak=max(r["p_drive_peak"] for r in runs),
                p_drive_regen=max(r["p_drive_regen"] for r in runs), chopper=ch)


def plot(trace, path):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.5, 5), sharex=True, facecolor="#fcfcfb")
    t = np.array(trace["t"]) - trace["t"][0]
    for ax in (a1, a2):
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color="#e4e3df", lw=0.6)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    a1.plot(1000 * t, trace["bus"], color="#2a78d6", lw=1.5)
    a1.axhline(0, color="#52514e", lw=0.8)
    a1.set_ylabel("Net drive power (W)")
    a1.set_title("Largest regen stretch, vx 0.2 + yaw 0.5, seed 0 (12 drives, F2, case 7)", fontsize=10)
    for (k, v), c in zip(trace["v"].items(), ("#eb6834", "#1baf7a")):
        a2.plot(1000 * t, v, color=c, lw=1.8, label=f"brake: {k}")
    a2.axhline(V_TRIP, color="#e34948", lw=1, ls="--")
    a2.annotate("drive trip 26 V", (2, V_TRIP), xytext=(0, 3), textcoords="offset points", fontsize=8, color="#52514e")
    a2.set_ylabel("Bus voltage (V)")
    a2.set_xlabel("Time (ms)")
    a2.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main():
    runs = motor_bus()
    s = summary(runs)
    plot(next(r["trace"] for r in runs if "trace" in r), OUT / "brake_chopper.png")
    pump = json.loads((ROOT / "sim/water/results/water.json").read_text())["recommended"]
    b = budget()
    harness, errors = check_harness()
    res = dict(motor=s, budget=b, pump=pump, harness_errors=errors)
    p_pump = pump["watts"] * PUMP_DUTY  # W, mean pump power from the motor bus
    res["hours_6s_sim"] = 2 * PACK_6S_WH * USABLE / (s["p_batt_walk"] + p_pump)
    (OUT / "power_calc.json").write_text(json.dumps(res, indent=1, default=float))
    L = ["# Power numbers (generated by docs/hardware/power_calc.py)", "",
         f"Status: tested in simulation. {s['runs']} walking runs of 20 s on F2 (bhl_F2), case 7, {s['falls']} falls.", "",
         "## Motor bus (24 V, two 6S packs through the diode)", "",
         f"- Battery power, walking, mean of the runs: {s['p_batt_walk']:.0f} W (highest run {s['p_batt_walk_max']:.0f} W). "
         f"Standing: {s['p_batt_stand']:.0f} W. Both include {12 * P_STANDBY:.0f} W drive standby (ASSUMED).",
         f"- Peak bus power: {s['p_peak']:.0f} W instant, {s['p_peak20']:.0f} W over 20 ms. "
         f"Peak into one drive: {s['p_drive_peak']:.0f} W.",
         f"- Peak regen of the 12 drives together: {s['p_regen']:.0f} W. Peak regen of one drive: "
         f"{s['p_drive_regen']:.0f} W ({s['p_drive_regen'] / 24:.1f} A at 24 V). Mean regen power: at most "
         f"{s['e_regen_mean_w']:.1f} W in one run.",
         f"- Pump ({pump['pump']}, through the DMC-101, after relay K1): {pump['watts']:.0f} W or less while it sprays "
         f"({pump['amps']:.1f} A at 24 V, datasheet). At {100 * PUMP_DUTY:.0f} % spray time (ASSUMED): {p_pump:.1f} W on average.",
         f"- Battery time, two 6S at 80 % usable, simulated walking power plus the mean pump power: {res['hours_6s_sim']:.1f} h.", "",
         "| Brake option | Peak bus voltage (V) | Peak resistor power (W) | Largest energy per event (J) "
         "| Mean resistor power, worst run (W) | Events per second | Brake current limit per drive (A) |",
         "|---|---|---|---|---|---|---|"]
    for k, c in s["chopper"].items():
        ohm = CHOPPERS[k][1] if k in CHOPPERS and CHOPPERS[k][0] else None
        lim = f"{DUTY_MAX * RAMP[1] / ohm:.1f}" if ohm else "-"
        ev = "-" if c["e_event"] != c["e_event"] else f"{c['e_event']:.2f}"
        L.append(f"| {k} | {c['v_peak']:.2f} | {c['p_peak']:.0f} | {ev} | {c['p_mean']:.2f} "
                 f"| {c['events_s']:.1f} | {lim} |")
    L += ["", f"Ramp {RAMP[0]} V to {RAMP[1]} V, bus capacitance {1000 * C_BUS:.0f} mF, start at {V_BATT} V. "
          f"The brake current counts in dc_max_positive_current ({I_POS_MAX:g} A).", "",
          "## 3S rail (computer, relay coils, pan-tilt)", "",
          "| Rail | Load | Typical (A) | Peak (A) | Source |", "|---|---|---|---|---|"]
    L += [f"| {x[0]} | {x[1]} | {x[2]:.2f} | {x[3]:.2f} | {x[4]} |" for x in b["loads"]]
    L += ["", f"- 5 V rail: {b['i5'][0]:.2f} A typical, {b['i5'][1]:.2f} A peak.",
          f"- 3S input: {b['p3s'][0]:.1f} W typical ({b['i3s_nom']:.2f} A at 11.1 V), {b['p3s'][1]:.1f} W peak "
          f"({b['i3s_peak_low']:.1f} A at 10.5 V).",
          f"- 3S battery time at the typical load: {b['hours_3s']:.1f} h.", "",
          "## Harness check", "", "All rows pass." if not errors else "\n".join(f"- {e}" for e in errors), ""]
    (OUT / "power_calc.md").write_text("\n".join(L))
    print("\n".join(L))


def check():
    # a constant regen of 100 W for 50 ms without a resistor charges 6 mF from 25.2 V by E = 5 J
    bus = np.full(100, -100.0)
    v = chopper(bus, 0.0005, 0, 1.0)["v_peak"]
    assert abs(0.5 * C_BUS * (v * v - V_BATT ** 2) - 5.0) < 1e-6, v
    # with a resistor, the voltage settles where the resistor takes the regen power, inside the ramp
    c = chopper(np.full(2000, -100.0), 0.0005, 1, 3.3)
    assert RAMP[0] < c["v_peak"] < RAMP[1] and abs(c["p_peak"] - 100) < 5, c
    # drive power: the batteries never take power back
    assert chopper(np.full(100, 50.0), 0.0005, 1, 3.3)["v_peak"] == V_BATT
    _, errors = check_harness()
    assert not errors, errors
    print("power_calc.py check: OK")


if __name__ == "__main__":
    check() if sys.argv[1:] == ["check"] else main()
