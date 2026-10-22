# Parts to order

Parts for the final design F2 that are **not** in the course BOM. The course BOM already has the batteries, D1, K1, U1, the e-stop, the push-lock switches, the meters, the shunt, the CANables, the EC5, XT30 and XT60 connectors and 18 AWG wire. Ask the TA how many EC5 (3 needed) and XT60 pairs (2 needed) the course gives.

Status: planned. We read the prices on 2026-10-15. Prices change: check each one before you order. "Search result" means the price comes from the search page, not from the product page. AliExpress prices have no shipping.

**Assumed:** where a shop shows the price without VAT, we added 10 % VAT. Devicemart ships free above 66,000 KRW.

The reference designators (U2, F14 …) are the ones of the [electrical drawings](electrical.md). The printed parts and the screws of the print kit are in [cad/final/kit/hardware.md](../../cad/final/kit/hardware.md).

## Totals

| Group | KRW |
|---|---|
| Water system | 41,422 |
| Electrical | 187,013 |
| Mounting hardware | 24,072 |
| **Total of the priced rows** | **252,507** |

The rows marked — have no price yet: the pump, the DMC-101, the opto-isolator, the tee, the stoppers, the M5 × 55 screws, the strap and the tube adapters.

| Shop | KRW |
|---|---|
| devicemart | 162,477 |
| eleparts | 40,308 |
| navimro | 23,925 |
| Gmarket | 8,780 |
| AliExpress | 8,717 |
| mechasolution | 8,300 |

## 1. Water system

| # | Part | Qty | Unit KRW | Line KRW | Shop | Link | Price source |
|---|---|---|---|---|---|---|---|
| W1 | Kamoer KLP02 diaphragm pump, 24 V version (1 spare recommended) | 1 | — | — | team | https://www.kamoer.com/us/product/params.html?id=3006 | the team orders it |
| W2 | DMC-101 DC pump controller | 1 | — | — | team |  | the team orders it |
| W3 | MK8 large-bore brass nozzle kit, 1.0 / 1.2 / 1.5 / 3 mm | 1 | 8,780 | 8,780 | Gmarket | https://item.gmarket.co.kr/Item?goodsCode=4039267557 | search result. Check the M6 thread. |
| W4 | Brass M6 female × 3 mm hose barb (nozzle holder) | 2 | 3,920 | 7,840 | AliExpress | https://ko.aliexpress.com/item/1005005947571818.html | search list, no shipping |
| W5 | Brass M5 male × 3 mm hose barb, 5 pieces (tank outlets: 2 needed) | 1 | 877 | 877 | AliExpress | https://ko.aliexpress.com/item/33050288194.html | search list, no shipping |
| W6 | Silicone tube 3 × 6 mm, per m (1.5 m needed) | 2 | 1,749 | 3,498 | navimro | https://www.navimro.com/p/K15538679/ | shop page |
| W7 | Hose tee for 3 mm tube (joins the two tank lines) | 1 | — | — | AliExpress |  | to check |
| W8 | Check valve Sansyo SB, 3 mm, opens at 10 kPa. Add a second one if bench test T8 shows air at the tee. | 1 | 14,289 | 14,289 | navimro | https://www.navimro.com/p/K51663439/ | shop page |
| W9 | Inline filter ARAM, 3 mm, 100 mesh (1 spare) | 2 | 3,069 | 6,138 | navimro | https://www.navimro.com/p/K92574648/ | shop page |
| W10 | Silicone stopper for a 16 mm bore, with a vent hole (tank fill necks) | 2 | — | — |  |  | to check |
| W11 | Tube adapters to the KLP02 ports | 1 | — | — |  |  | to check when the pump arrives: port size not published |
| | **Subtotal of the priced rows** | | | **41,422** | | | |

## 2. Electrical

| # | Part | Qty | Unit KRW | Line KRW | Shop | Link | Price source |
|---|---|---|---|---|---|---|---|
| E1 | Brake resistor RARA RH50C4R70J, 4.7 Ω 50 W aluminium (RB1, RB2, 1 spare) | 3 | 4,840 | 14,520 | devicemart | https://www.devicemart.co.kr/goods/view?no=33451 | shop page. Made to order (2 weeks or more) when the stock runs out. |
| E2 | Buck 12 V → 5 V, Pololu D36V50F5, 5.5 A (U2) | 1 | 51,000 | 51,000 | devicemart | https://www.devicemart.co.kr/goods/view?no=12536510 | search result. Ships from abroad in about 1 week. |
| E3 | Spare buck SZH-EK366, 9–36 V → 5 V 5 A | 1 | 2,420 | 2,420 | devicemart | https://www.devicemart.co.kr/goods/view?no=1386375 | search result |
| E4 | Opto-isolator module, 1 channel, PC817 type (U8) | 2 | — | — | devicemart |  | to check. Choose it after the DMC-101 datasheet (electrical.md, section 5). |
| E5 | In-line fuse holder MTA 0100335, mini blade, 30 A, 4 mm² leads (F1–F4, 1 spare) | 5 | 3,790 | 18,950 | eleparts | https://www.eleparts.co.kr/goods/view?no=18224538 | shop page |
| E6 | In-line mini fuse holder Coms BE151 (F5, F10–F14, 1 spare) | 7 | 1,450 | 10,150 | eleparts | https://www.eleparts.co.kr/goods/view?no=4063235 | shop page |
| E7 | Mini blade fuse MTA MINIVAL 2 A, pack of 10 (F5, F12) | 1 | 2,530 | 2,530 | eleparts | https://www.eleparts.co.kr/goods/view?no=18406815 | shop page |
| E8 | Mini blade fuse MTA MINIVAL 5 A, pack of 10 (F11) | 1 | 2,120 | 2,120 | eleparts | https://www.eleparts.co.kr/goods/view?no=18406818 | shop page |
| E9 | Mini blade fuse MTA MINIVAL 10 A, pack of 10 (F10, F13) | 1 | 1,930 | 1,930 | eleparts | https://www.eleparts.co.kr/goods/view?no=18406811 | shop page |
| E10 | Mini auto fuse 3 A (F14) | 5 | 165 | 825 | devicemart | https://www.devicemart.co.kr/goods/view?no=10901863 | search result |
| E11 | Mini auto fuse 20 A (F1–F4) | 10 | 165 | 1,650 | devicemart | https://www.devicemart.co.kr/goods/view?no=10903008 | search result |
| E12 | Varistor Bourns MOV-10D390K, 39 V (RV1, 1 spare) | 2 | 538 | 1,076 | eleparts | https://www.eleparts.co.kr/goods/view?no=1195942 | shop page |
| E13 | Schottky diode SB560, 5 A 60 V (D2, pump flyback if needed, spares) | 4 | 228 | 912 | eleparts | https://www.eleparts.co.kr/goods/view?no=17046478 | shop page |
| E14 | Lever connector Wago 221-413 type, 3-way (X1, 12 V actuator node) | 6 | 550 | 3,300 | devicemart | https://www.devicemart.co.kr/goods/view?no=10827022 | search result. Brand not confirmed. |
| E15 | Ring lug JOT 6-6 (6 mm², 6 mm hole) for D1 and K1 | 10 | 110 | 1,100 | devicemart | https://www.devicemart.co.kr/goods/view?no=12506996 | search result |
| E16 | Ring lug JOT 6-8 (6 mm², 8 mm hole) for RS1 | 10 | 110 | 1,100 | devicemart | https://www.devicemart.co.kr/goods/view?no=12506997 | search result |
| E17 | DC plug 5.5 × 2.1 mm, solder type (servo adapter) | 2 | 490 | 980 | devicemart | https://www.devicemart.co.kr/goods/view?no=4581 | search result |
| E18 | Silicone wire 10 AWG, red, per m (W05) | 1 | 1,900 | 1,900 | mechasolution | https://www.mechasolution.com/goods/goods_view.php?goodsNo=604948 | shop page |
| E19 | Silicone wire 12 AWG, per m: 2 red + 2 black (W01–W09) | 4 | 1,600 | 6,400 | mechasolution | https://www.mechasolution.com/goods/goods_view.php?goodsNo=604947 | shop page |
| E20 | Silicone wire 16 AWG, 1 m red + 1 m black, DFRobot FIT0584 (W20) | 1 | 6,300 | 6,300 | devicemart | https://www.devicemart.co.kr/goods/view?no=16057418 | search result. Ships from abroad in 1 week. |
| E21 | Wire kit 22 AWG, 6 colours × 10 m, SZH-WR002 | 1 | 17,050 | 17,050 | devicemart | https://www.devicemart.co.kr/goods/view?no=10918438 | search result |
| E22 | Dupont jumper wires F/F, 40 pieces, 20 cm | 1 | 1,200 | 1,200 | devicemart | https://www.devicemart.co.kr/goods/view?no=1321192 | search result |
| E23 | Heat-shrink set NT-DIY, 280 pieces | 1 | 8,800 | 8,800 | devicemart | https://www.devicemart.co.kr/goods/view?no=1320937 | search result |
| E24 | Relay KKA-K4, 24 V coil, 40 A (K2, 1 spare) | 2 | 7,150 | 14,300 | devicemart | https://www.devicemart.co.kr/goods/view?no=14822397 | search result. Coil current not listed. |
| E25 | Crimp spade terminal set, 180 pieces, incl. 6.3 mm (K2 terminals) | 1 | 16,500 | 16,500 | devicemart | https://www.devicemart.co.kr/goods/view?no=15356628 | search result |
| | **Subtotal of the priced rows** | | | **187,013** | | | |

## 3. Mounting hardware

| # | Part | Qty | Unit KRW | Line KRW | Shop | Link | Price source |
|---|---|---|---|---|---|---|---|
| M1 | Brass insert M3 × 5 × 5 mm (pump holder, buck, modules) | 30 | 88 | 2,640 | eleparts | https://www.eleparts.co.kr/goods/view?no=11045755 | shop page. Check that it is a heat-set insert. |
| M2 | Stainless hex socket screw kit NT-KIT-LS012, M3–M5, 6–30 mm | 1 | 18,000 | 18,000 | devicemart | https://www.devicemart.co.kr/goods/catalog?code=001500070037 | search result |
| M3 | Spring nut M5 for 20-series profile. 32 needed (cad/final/kit/hardware.md), 20 already ordered: order 12 more. | 12 | 286 | 3,432 | devicemart | https://www.devicemart.co.kr/goods/view?no=12530574 | search result |
| M4 | Stainless socket head screw M5 × 55 with washer (tanks to the post side slots) | 8 | — | — | devicemart |  | to check. The M2 kit stops at 30 mm. |
| M5 | Hook-and-loop strap 20 × 250 mm (pump) | 1 | — | — |  |  | to check |
| | **Subtotal of the priced rows** | | | **24,072** | | | |

## 4. From lab stock

- Three resistors: R1 10 kΩ (e-stop pull-up), R8 10 kΩ (pump pull-down), R7 470 Ω (opto LED).
- Thermal paste (brake resistors), PTFE tape (nozzle and barbs), cable ties, crimp ferrules and a crimp tool.
- 18 AWG wire for W10, W11, W29 and W32 comes from the course BOM.

## 5. Alternatives

| Part | When | KRW | Link |
|---|---|---|---|
| M6 brass nozzle, 1.0 or 1.5 mm, single | if the W3 kit is not M6 | 1,517 each | https://ko.aliexpress.com/item/1005004605881926.html |
| Pololu #3779 shunt regulator | if the AUX brake path does not work | 37,400 | https://www.pololu.com/product/3779 |
| RH50C2R20J, 2.2 Ω 50 W | if the 4.7 Ω part is out of stock. Then raise `dc_max_positive_current` on the two brake drives (electrical.md, 4.3). | 4,840 | https://www.devicemart.co.kr/goods/view?no=33447 |
| Littelfuse FHAC0002ZXJ ATO holder, 30 A, 12 AWG, IP67 | if the MTA holders are out of stock | 16,248 | https://www.digikey.kr/en/products/result?keywords=FHAC0002ZXJ |

Each XDrive MINI ships with one brake resistor. Its value is not published. Measure it when the drives arrive: if it is 4 to 5 Ω, you do not need E1.
