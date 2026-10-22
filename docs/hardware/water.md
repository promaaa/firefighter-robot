# Water system of the final design F2

Status: the water parts are **planned**. The numbers are **tested in simulation** with `sim/water/water.py`. Nothing here is built or measured.

- **Not verified:** The KLP02 pump curve is an estimate from two datasheet points: 0.7 L/min free flow and 30 m of head at zero flow. Both are minimum values. The fitting losses, the nozzle coefficients, the jet break-up length and the drag of the drops are also estimates. The bench tests in section 9 measure them.
- **Assumed:** The pump speed follows its supply voltage. All numbers are at 22.2 V, the nominal voltage of the 6S motor bus. The bus goes from 21.0 V (low) to 25.2 V (full).
- **Assumed:** The tube from the right tank (the longer line) to the nozzle is 1.0 m long. The nozzle sits 0.905 m above the floor (`cad/final/out_f2/layout.json`).
- **Open:** The DMC-101 datasheet is not available yet. We do not know its input voltage range, its control input or whether it changes the pump speed.
- **Open:** Kamoer does not publish the KLP02 size or port size. The CAD pump holder (`cad/final`) fits the DWP-385 gear pump of the earlier plan. A KLP02 holder will follow when the pump arrives and we measure it.
- **Open:** The two tanks feed the pump through one tee. When the robot leans, one tank outlet can take air first. The jet then stops with water still in the other tank (section 3.3, test T8).

## 1. Summary

| Item | Value |
|---|---|
| Pump | Kamoer KLP02, 24 V version, brushless, self-priming (suction lift 4 m or more), 12 W or less, about 250 g |
| Controller | DMC-101 DC pump controller, between the bus and the pump |
| Power (proposed wiring) | 24 V motor bus after relay K1, fuse F14 3 A. The e-stop opens K1, so it stops the pump with the leg motors. |
| Switch | on/off from ROCK 3C pin 11 (GPIO3_A1) through an opto-isolator, no PWM. Software: `robot/pump.py` |
| Nozzle | 1.0 mm brass 3D-printer nozzle (M6). Order 1.2 and 1.5 mm too, for the bench test. |
| Flow | 0.42 L/min (7.0 g/s) |
| Jet speed at the nozzle | 10.0 m/s at 22.2 V (9.5 m/s at 21.0 V, 11.0 m/s at 25.2 V) |
| Range to the floor, from 0.9 m | best estimate 2.7 m at −10°, 3.8 m at 0°, 5.3 m at +15°, 6.1 m at +30°. Low estimate 2.1 to 2.8 m. |
| Spray time, 0.749 L usable | 1.8 min (107 s) |
| Recoil | 0.07 N |
| Current | 0.5 A or less at 24 V (datasheet: 12 W or less) |
| Fire it can handle | a 17 to 22 cm square tray of paper or small wood sticks (8 to 22 kW peak), and any candle group |

All values: tested in simulation, design F2, filter and check valve, empty tanks, 22.2 V.

Terms:

- **Pump curve:** the pressure that the pump gives at each flow.
- **System curve:** the pressure that the tube, the fittings, the lift and the nozzle need at each flow. The pump runs where the two curves cross (the **operating point**).
- **Self-priming:** the pump can suck air out of its inlet tube and pull the water up.
- **Break-up length:** the distance over which the jet stays one smooth column. After it, the jet is a stream of drops.

## 2. Pump and controller

### 2.1 Pump curve

- Kamoer data for the 24 V brushless KLP02: flow 0.7 L/min or more, head 30 m or more, suction lift 4 m or more.
- It takes 12 W or less, weighs about 250 g and has "switch control".
- 30 m of water head is 294 kPa (2.9 bar). We draw a straight line from 294 kPa at zero flow to 0.7 L/min at zero pressure, at 24 V.
- At 22.2 V the line goes from 272 kPa to 0.65 L/min (flow and pressure follow the voltage, Assumed).
- A curved (quadratic) pump curve through the same two points gives 0.48 L/min and 11.4 m/s (section 5.4).
- The DWP-385 gear pump of the earlier plan gives 0.39 L/min and 9.2 m/s on the same plumbing, at 11.1 V (`sim/water/results/water.md`).

![Pump and system curves of the KLP02](../../sim/water/results/curves_KLP02.png)

Figure 1. KLP02 at 22.2 V. Each coloured curve is the system curve of one nozzle. The dots are the operating points.

### 2.2 Wiring and control (proposed)

1. The motor bus feeds the pump branch after relay K1, through fuse F14 (3 A).
2. The DMC-101 takes the bus voltage and drives the pump.
3. ROCK 3C pin 11 drives an opto-isolator. The opto-isolator switches the DMC-101 on and off. The full wiring is in [electrical.md](electrical.md).
4. `robot/pump.py` switches the one GPIO line. It stops the pump after 12 s on (`max_spray_s`) and when the actuator node stops for 0.3 s (`keepalive_s`).
5. The pump takes 12 W or less while it sprays. At 10 % spray time (Assumed), this is 1.2 W on average from the motor packs ([power_calc.md](power_calc.md)).

**Open:** regen of the leg drives lifts the bus to about 25.6 V with the brake chopper ([power_calc.md](power_calc.md)). Check that the DMC-101 and the KLP02 accept this voltage.

### 2.3 Jet speed and bus voltage

- The 6S bus voltage falls from 25.2 V to 21.0 V as the packs discharge. If the pump speed follows the voltage, the jet speed falls from 11.0 m/s to 9.5 m/s.
- Fit of the model: v0 ≈ 10.0 m/s × (V / 22.2 V)^0.8.
- The leg drives report the bus voltage (`legs/robot.py`, `vbus()`). **Open:** the aim code does not use it yet for F2.
- Tests T3 and T4 at 21.0 V and 25.2 V show if the speed follows the voltage. If the DMC-101 or the pump holds the speed, v0 is constant.

## 3. Plumbing

### 3.1 Path

Tube lengths from `cad/final/out_f2/layout.json` (`suction_mm`, `tubes_mm`).

1. Each R2 tank has a brass M5 × 3 mm outlet barb at its front end, low on the inner face (centre z 0.599 m).
2. 3 × 6 mm silicone tube runs from each outlet, across the front posts, to a 3 mm tee. Left line 0.20 m, right line 0.29 m.
3. One tube runs from the tee to the inline filter (100 mesh) on the middle shelf, then 0.06 m to the pump inlet.
4. The pump sits on the rear of the middle shelf, axis at z 0.703 m. It lifts the water 0.03 to 0.11 m on the suction side (test T2).
5. From the pump outlet, 0.25 m of tube runs through the hose channel to the deck. A 0.25 m service loop goes around the pan axis.
6. The check valve sits on the tilt arm, arrow toward the nozzle. An M6 female × 3 mm barb holds the M6 nozzle (PTFE tape).

Put a small cable tie on every barb joint. Right tank to nozzle: about 0.97 m. The model uses 1.0 m.

### 3.2 Losses (Assumed)

| Part | Value | Basis |
|---|---|---|
| Lift, water surface to nozzle | 0.23 m (full tanks) to 0.31 m (empty tanks) | `cad/final/out_f2/r2.json`, `layout.json` |
| Tube | 1.0 m, 3.2 mm bore, all the flow in one line. Friction f = max(64/Re, 0.316 Re^-0.25) | Darcy and Blasius |
| Barb | K = 1.0 at a 2.0 mm bore, 8 barbs (tank outlet, tee in and out, filter 2, check valve 2, nozzle) | typical handbook value |
| Inline filter | K = 3.0 at the tube speed | typical handbook value |
| Check valve | 10 kPa opening pressure + K = 2.0 | Sansyo SB: "opens at 10 kPa or more" |
| Nozzle | Cv = 0.95, Cd = 0.85 | a short cone with a short land, like a 3D-printer nozzle |

The operating point solves: pump pressure = ρ g lift + tube friction + fitting losses + nozzle pressure.
The nozzle gives Q = Cd A √(2 p / ρ) and v = Cv √(2 p / ρ).

### 3.3 Open risk: one tee for two tanks

- While the robot stands level, both lines carry water and the levels stay about equal.
- When the robot leans to one side, the outlet of the higher tank uncovers first. The pump sucks air through that line and the jet stops.
- Status: not tested in simulation. The water model has no air in the lines.
- Fixes: a check valve in each tank line, or an outlet at each tank end. A check valve costs about 0.3 m of range at +15° (section 5.4).

## 4. Operating points

Tested in simulation. KLP02 at 22.2 V, design F2, empty tanks, filter and check valve. Full tables: `sim/water/results/water.md`.

| Nozzle (mm) | Flow (L/min) | Flow (g/s) | Nozzle pressure (kPa) | Exit speed (m/s) | Max range, best / low estimate (m) | Spray time, 0.749 L (min) | Recoil (mN) |
|---|---|---|---|---|---|---|---|
| 0.8 | 0.35 | 5.8 | 93 | 13.0 | 6.8 / 2.6 | 2.1 | 75 |
| **1.0** | **0.42** | **7.0** | **55** | **10.0** | **6.1 / 2.8** | **1.8** | **70** |
| 1.2 | 0.46 | 7.7 | 32 | 7.6 | 4.8 / 2.6 | 1.6 | 59 |
| 1.5 | 0.49 | 8.2 | 15 | 5.2 | 3.1 / 2.1 | 1.5 | 43 |
| 2.0 | 0.51 | 8.5 | 5 | 3.0 | 1.6 / 1.3 | 1.5 | 26 |

- We choose the 1.0 mm nozzle. It gives 20 % more flow than 0.8 mm. Its low-estimate range (2.8 m) is the longest of all nozzles.
- The 1.2 mm nozzle gives 10 % more water per second for 1.1 m less range at +15°. Use it if the fires are close.
- Larger nozzles add little flow, because the barbs then take most of the pressure.

## 5. Jet range

### 5.1 Model

1. **Intact column, no drag.** A smooth water column has much mass for its front area. The model treats it as a drag-free projectile.
2. **Break-up length.** Best estimate (laminar jet): L/d = 13.4 (We^0.5 + 3 We/Re), Weber's form with the Grant and Middleman constant [1, 2]. Low estimate (turbulent jet): L/d = 8.51 We^0.32 [2, 3].
3. **Drops.** After break-up, each drop is a sphere of 1.89 d in the best estimate [1] and of d in the low estimate.
4. **Drag of the drops:** Schiller-Naumann, Cd = 24/Re (1 + 0.15 Re^0.687) [4].
5. **Drop train.** A drop in the wake of the drop in front has less drag: Cd/Cd_single = 0.62 for two spheres 2 diameters apart [5]. The best estimate uses 0.6, the low estimate 1.0.

For the 1.0 mm jet at 10.0 m/s: We = 1310, Re = 9400, gas Weber number 1.6. This is the first wind-induced regime (0.4 < We_gas < 13) [1].
The intact column is 0.46 m (best estimate) or 0.08 m (low estimate). A 3D-printer nozzle is a short cone, so we expect a jet nearer the laminar case. **Not verified.**

### 5.2 Range per elevation

Range to the floor from the nozzle at 0.905 m. KLP02 at 22.2 V, 1.0 mm. Best estimate / low estimate, in m.

| −10° | −5° | 0° | +5° | +10° | +15° | +20° | +25° | +30° |
|---|---|---|---|---|---|---|---|---|
| 2.7 / 2.1 | 3.2 / 2.3 | 3.8 / 2.4 | 4.3 / 2.6 | 4.9 / 2.7 | 5.3 / 2.7 | 5.7 / 2.8 | 6.0 / 2.7 | 6.1 / 2.7 |

Without drag, the jet would reach 10.2 m at +30°.

![Jet paths](../../sim/water/results/jets.png)

Figure 2. Jet paths of the 1.0 mm nozzle at four elevations. Solid: best estimate. Dashed: low estimate. Dotted: no drag.

### 5.3 Values for the aim code

- `aim/ballistics.py` uses one constant drag k: dv/dt = −g − k |v| v. k = 0.11 1/m with v0 = 10.0 m/s fits the best estimate at +15°.
- Tank time: 0.749 L / 0.42 L/min = 107 s of pump time. Use these values until test T4 gives measured values.

### 5.4 Sensitivity (KLP02, 1.0 mm)

| Case | Flow (L/min) | Exit speed (m/s) | Best-estimate range at +15° (m) |
|---|---|---|---|
| Baseline: empty tanks, filter and check valve, 22.2 V | 0.42 | 10.0 | 5.3 |
| Full tanks | 0.42 | 10.0 | 5.4 |
| No check valve | 0.44 | 10.5 | 5.6 |
| No filter, no check valve | 0.45 | 10.7 | 5.7 |
| Tube 0.7 m or 1.3 m instead of 1.0 m | 0.42 | 9.9 to 10.0 | 5.3 to 5.4 |
| Bus full, 25.2 V | 0.47 | 11.0 | 5.9 |
| Bus low, 21.0 V | 0.40 | 9.5 | 5.1 |
| Datasheet voltage, 24 V | 0.45 | 10.6 | 5.7 |
| Nozzle Cd 0.65 (sharp orifice) | 0.38 | 11.8 | 6.0 |
| Quadratic pump curve instead of a straight line | 0.48 | 11.4 | 6.2 |

- The check valve costs about 0.3 m of range. Keep it only if test T6 shows drips that matter.
- The bus voltage moves the range at +15° by −0.2 to +0.6 m.
- The tank level and the tube length hardly matter.

## 6. Spray time and recoil

- Usable water = 0.781 L − 24 mL under the outlets (Assumed) − 8 mL in the tube = 0.749 L.
- KLP02 with 1.0 mm at 22.2 V: 1.8 min (107 s) of spray for one fill of both tanks.
- Recoil F = ρ Q v [6]: 0.07 N, less than the weight of 7 g. It does not matter for the walk or for the tilt servo.

## 7. What fire the robot can put out

- Heating water from 10 °C and making steam at 100 °C takes 2640 kJ/kg [7, p. 313]. So 7.0 g/s can take at most 18 kW. On a real fire much of the water runs off, so 18 kW is an upper limit.
- Water applied by hand needs 9 to 15 L/(m² min) of fire floor area [7, Fig. 6.42]. 7.0 g/s covers 280 to 467 cm²: a square of 17 to 22 cm.
- Small fires: one candle 77 W [9]. A waste basket with 224 to 350 g of paper peaks at 21 to 34 kW [10]. A 14 L basket with 0.2 kg of paper peaks at about 10 to 20 kW [11].
- **Candle group:** yes, any number that the robot can aim at. The limit is the aim, not the flow.
- **Paper tray:** a metal tray up to about 17 × 17 cm with loose paper (about 8 kW peak) is inside the capacity. 22 × 22 cm is the upper limit (about 22 kW).
- **Small wood crib:** 9 sticks of 1 × 1 × 15 cm in 3 layers, about 6 to 9 kW [12]. This is inside the capacity.
- **Total water:** a small tray needs about 10 to 30 s of direct spray (Assumed). So one fill can put out about 4 small fires.
- **Proposed test fire:** a 15 × 15 cm steel tray with at most 30 g of crumpled paper, or the 9-stick crib.
- **No liquid fires.** Water sinks below burning oil and can throw it out ("slop over") [7, p. 254]. Fire services say: never use water on a grease fire [13]. A liquid fire would also spread toward the robot and its LiPo packs.

## 8. Safety rules for indoor fire tests

1. Ask the TA which SNU rule covers open flames in the lab. Get the permission before the first fire test.
2. Write a short risk assessment and give a safety briefing before each session [14].
3. Put a class ABC or CO2 extinguisher within reach [15]. Put a fire blanket and a bucket of sand next to it.
4. Burn only paper or dry wood: at most 30 g of paper or one 9-stick crib per test. Do not burn plastics or liquids.
5. Put the fire in a steel tray on a non-combustible sheet. Keep 1 m clear around it.
6. Ventilate the room. Do not cover or switch off smoke detectors without the building manager's permission.
7. One person watches the fire and holds the extinguisher. A second person holds the robot e-stop.
8. Keep the robot at least 1 m from the flame. The jet reaches 2 m or more.
9. Never leave a flame. After the test, pour water on the residue until it is cold.
10. Wear safety glasses. Do not wear synthetic clothes near the flame.
11. Leak-test both tanks (test T1) before the first test with the robot. Keep the pump, the DMC-101 and the 24 V connections dry. Cover the relay terminals.
12. Charge LiPo packs in a LiPo bag and never leave a charging pack [16]. If a pack smokes, put it in the LiPo bag or under sand.

## 9. Bench tests

Status: planned. Do the tests in this order.
Tools: kitchen scale (1 g), stopwatch, measuring jug, 5 m tape, plastic floor sheet, and a phone with slow-motion video and an inclinometer app.
Supply: a bench supply (21 to 25.2 V, 1 A) or one 6S pack.

| Test | What to do | Pass or result |
|---|---|---|
| T1 Leak | Close each tank outlet, fill the tank, weigh it on a dry towel, wait 60 min, weigh again. Hold it 1 min at 30° to each side. Repeat on the robot. | 1 g or less lost, no wet spot |
| T2 Priming | Start with a dry pump and an empty hose. Switch the pump on. Time the first water and the steady jet. Run the tanks empty, refill, run again. | steady jet within 10 s, also after a refill |
| T3 Flow | Catch the whole jet in a jug for 30.0 s (software timer). Flow (mL/min) = mass (g) × 2. Do 3 runs per nozzle (1.0, 1.2, 1.5 mm) at 22.2 V, and the 1.0 mm runs at 21.0 V and 25.2 V. Write down the current. | flow and current |
| T4 Range | Nozzle at 0.90 m. Tape marks every 0.25 m to 6 m. Spray 5 s at −10° to +30° in 5° steps. Film from the side. Write down the near edge, centre and far edge of the wet zone. | range per angle |
| T5 Spray pattern | A3 sheet at 1.5 m and 2.5 m. 3 s shots, 5 times at the same aim. Measure the wet spot and the spread of the centres. | spot size and spread |
| T6 Response and drips | Time "pump on" to the jet and "pump off" to the end of the jet. Count the drops in 60 s after "off" at −10°. Repeat without the check valve. | delays and drops |
| T7 Endurance | Fill both tanks. Spray them empty at 22.2 V. Time it. Measure the pump and DMC-101 temperature right after. | within 20 % of 1.8 min, below 60 °C |
| T8 Air through the tee | Robot on a stand with both full tanks. Tilt it 5°, 10° and 15° left and right. Spray until the jet stops. Weigh the water left. | less than 30 mL left |

From the 0° shot of T4, compute v0 = x √(g / (2 h)), with the centre distance x and h = 0.90 m.
Fit `ControllerConfig.drag` until `aim/sim_demo.py` (`landing_point()`) gives the measured +30° range.
From the slow-motion video, write down where the smooth column turns into drops.
If T8 fails, put a check valve in each tank line and repeat.

| Test | Nozzle (mm) | Voltage (V) | Measured | Unit | Predicted (section) |
|---|---|---|---|---|---|
| T3 flow, 3 runs | 1.0 | 22.2 | | mL/min | 420 (4) |
| T3 flow | 1.0 | 21.0 / 25.2 | | mL/min | 400 / 470 (5.4) |
| T3 flow | 1.2 / 1.5 | 22.2 | | mL/min | 460 / 490 (4) |
| T3 current | 1.0 | 22.2 | | A | 0.5 or less (1) |
| T4 range at −10° / 0° | 1.0 | 22.2 | | m | 2.1 to 2.7 / 2.4 to 3.8 (5.2) |
| T4 range at +15° / +30° | 1.0 | 22.2 | | m | 2.7 to 5.3 / 2.7 to 6.1 (5.2) |
| T4 v0 from the 0° shot | 1.0 | 21.0 / 22.2 / 25.2 | | m/s | 9.5 / 10.0 / 11.0 (2.3) |
| T4 break-up length | 1.0 | 22.2 | | m | 0.08 to 0.46 (5.1) |
| T7 spray time, both tanks full | 1.0 | 22.2 | | min | 1.8 (6) |
| T8 water left at 10° lean, left / right | 1.0 | 22.2 | | mL | less than 30 |

## 10. Water parts

| Part | Qty | Note |
|---|---|---|
| Kamoer KLP02 24 V pump | 1 + 1 spare | team decision. **Open:** shop and price |
| DMC-101 DC pump controller | 1 | **Open:** datasheet |
| Brass nozzle M6, 1.0, 1.2 and 1.5 mm (3D-printer nozzle kit) | 1 kit | Gmarket 4039267557 (8,780 KRW) or AliExpress 1005004605881926 |
| Brass M6 female × 3 mm hose barb | 2 | AliExpress 1005005947571818 |
| Brass M5 male × 3 mm barb | 5 | AliExpress 33050288194: tank outlets and spares |
| 3 mm tee | 1 | for the two tank lines |
| Silicone tube 3 × 6 mm | 2 m | navimro K15538679 |
| Check valve Sansyo SB, 3 mm, opens at 10 kPa | 1 (3 if T8 fails) | navimro K51663439 |
| Inline filter ARAM, 3 mm, 100 mesh | 2 | navimro K92574648 |

We read the prices on 2026-10-15. The pump port size decides the reducing barbs.

## 11. Sources

1. Trettel B. 2020, "Reevaluating the jet breakup regime diagram", Atomization and Sprays 30(7):517-556, doi:10.1615/AtomizSpr.2020033171.
2. Grant R.P., Middleman S. 1966, "Newtonian jet stability", AIChE J. 12(4):669-678, doi:10.1002/aic.690120411 (read through [1] and [3]).
3. De Jarlais G., Ishii M., Linehan J. 1983, OSTI 6068163, https://www.osti.gov/servlets/purl/6068163
4. Schiller-Naumann drag law, accuracy stated in https://arxiv.org/pdf/1305.1283
5. Zapryanov and Toshev, two spheres in line, quoted in OSTI report 6417393 (1989), https://www.osti.gov/servlets/purl/6417393
6. Jet reaction F = ρ Q v = 2 A Δp (USDA Forest Service TechTip, https://www.fs.usda.gov/t-d/pubs/html/02511318/02511318.htm).
7. Särdqvist S. 2002, *Water and other extinguishing agents*, Swedish Rescue Services Agency, ISBN 91-7253-265-3, https://rib.msb.se/filer/pdf/23061.pdf
8. Takahashi S. 1986, Fire Safety Science 1:1197-1206, https://publications.iafss.org/publications/fss/1/1197
9. Hamins A., Bundy M., Dillon S. 2005, "Characterization of candle flames", J. Fire Prot. Eng. 15(4):265-285.
10. NIST Fire Calorimetry Database, Test090 and Test091 (waste paper baskets), doi:10.18434/mds2-2314
11. Särdqvist S. 1993, *Initial Fires*, Lund report 3070, https://lup.lub.lu.se/record/605521
12. Lawson J.R., Quintiere J.G. 1985, NBSIR 85-3196, https://nvlpubs.nist.gov/nistpubs/Legacy/IR/nbsir85-3196.pdf, with the heat of wood of [7, p. 316]
13. Indiana Department of Homeland Security, *Cooking Safety*, https://www.in.gov/dhs/get-prepared/files/cooking-safety.pdf
14. NFPA 45 (2015), chapter 12, through https://www.nsta.org/blog/protecting-students-fires
15. MIT EHS 2017, lithium battery safety guidance, https://ehs.mit.edu/wp-content/uploads/2019/09/Lithium_Battery_Safety_Guidance.pdf
16. Colorado State University 2025, lithium battery guidelines, https://policylibrary.colostate.edu/attachments/790.Lithium-based%20batteries%20process%20guidelines%20final.pdf

Source [8] gives the critical water rate of a wood crib: 1.3 to 3.0 g/(m² s) of exposed fuel surface.

## 12. Files

`sim/water/water.py` makes all numbers and figures of this document. Its README is `sim/water/README.md`. Run it from the repository root:

```
uv run python sim/water/water.py
```

Licence: this document is CC BY-SA 4.0. The code is MIT (root `LICENSE`).
