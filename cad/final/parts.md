# Torso parts: size, mass, mount, power

Research date: 2026-10-15. Pump: 2026-10-22. The machine-readable copy is `parts.json` in this folder. `kit.py` and `layout.py` read the sizes and masses from it.

**Not verified:** IMU shell hole spacing, CANable V2.0 Pro hole pattern, HAM5925 size, switch and e-stop depths, HAV1808 and HAV1625a sizes, XDrive MINI size. Measure these parts when they arrive.

**Not verified:** ROCK 3C height, mass and power draw. They are estimates.

**Assumed:** the HAM5925 makes 24 V for the e-stop chain and the relay coil. The diode joins the two 6S packs. The course sheet does not say this.

**Different from the course sheet:** the Korean 6S link is a VEGA 5200 mAh XT90S pack (751 g). The sheet text names the Zeee pack (603 g).

## How to read the table

- Size is L x W x H in mm. It is the outside box of the part.
- For switches and the e-stop, size is [bezel or head width, width, depth behind the panel].
- Confidence: "datasheet" = maker document, "listing" = shop page, "estimate" = our estimate from a similar part.
- Confidence is for the main size. The notes say which other values are estimates.
- Qty 0 = reference, alternative or old part, not on the robot. The CAD layout still places the DWP-385 pump and the DFR0457 module as stand-ins.

## Parts table

| ID | Part | Qty | Size L x W x H mm | Mass g | Mount | Confidence | Source |
|---|---|---|---|---|---|---|---|
| rock3c | Radxa ROCK 3C (RK3566) computer | 1 | 87.3 x 56 x 22 | 45 | 4 holes 2.8 mm (M2.5). Pattern 58 x 49 mm. Hole centres 3.5 mm from the board edges. Same pattern as a Raspberry Pi 3 or 4. | datasheet | [link](https://dl2.radxa.com/rock3/docs/hw/3c/radxa_rock3c_product_brief.pdf) |
| imu_im10a | Hiwonder IM10A 10-axis IMU, metal shell version | 1 | 59.5 x 49.5 x 11.7 | 50 | Metal shell with 2 side flanges and 2 holes in each flange (4 holes). Hole size and spacing are not published. Measure the part. The bare board is 42.2 x 29.6 mm with 4 corner holes and brass standoffs. | listing | [link](https://www.hiwonder.com/products/imu-module?variant=40375875338327) |
| lipo_6s_zeee | Zeee 6S LiPo 5000 mAh 22.2 V 120C, EC5, hard case | 2 | 138 x 47 x 48.5 | 603 | No holes. Hold it with a strap or a tray. Leave space for the EC5 lead and the JST-XH balance lead at one end. | listing | [link](https://www.amazon.com/dp/B0DGPMD46S) |
| lipo_6s_vega_alt | VEGA 6S LiPo 5200 mAh 22.2 V 70C, XT90S (Korean link in the course sheet) | 0 | 154 x 51 x 44 | 751 | No holes. Strap or tray. | listing | [link](https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=66947) |
| lipo_3s_dinogy | Dinogy Graphene 3S LiPo 11.1 V 6000 mAh 80C | 1 | 138 x 46 x 30 | 362 | No holes. Strap or tray. | listing | [link](https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=102623) |
| boost_ham5925 | HAM5925 DC/DC boost converter 150 W, IN 10-32 V, OUT 12-35 V, 10 A, CV/CC | 1 | 65 x 35 x 21 | 45 | Probably 4 holes about 3 mm near the board corners. Not verified. | estimate | [link](https://www.icbanq.com/P017179148) |
| diode_mbrt400100 | MBRT400100 Schottky diode module, 100 V 400 A, three-tower package | 1 | 92.7 x 20.3 x 18.3 | 75 | 2 holes 7.0 to 7.5 mm, 80 mm apart, on an isolated base plate. Bolt it to metal to remove heat. | datasheet | [link](https://www.diodemodule.com/rectifier-module/schottky/mbrt400100.pdf) |
| relay_aep18024 | Panasonic AEP18024 DC power relay, 80 A, 1 Form A (SPST-NO), 24 V coil | 1 | 75.5 x 40 x 79 | 400 | 2 holes 6 mm on a diagonal: 63.5 mm apart in L and 26 mm apart in W. M5 screws, 3 to 4 N m. Any mounting direction. | datasheet | [link](https://api.pim.na.industrial.panasonic.com/file_stream/main/fileversion/398) |
| switch_pushlock_24v | Kacon (Konhung) push-lock switch 1A1B, 24 V DC LED, 22 mm, NF22-BA3C1 | 1 | 30 x 30 x 45 | 25 | Panel hole 22.3 mm (NF22 series). Ring nut. | estimate | [link](https://www.icbanq.com/P017196445) |
| switch_pushlock_12v | Kacon (Konhung) push-lock switch 1A1B, 12 V DC LED, 16 mm, KBL16AT-L2C1 | 1 | 19 x 19 x 40 | 15 | Panel hole 16 mm. Ring nut. | estimate | [link](https://www.icbanq.com/P017197713) |
| estop_a20 | E-stop: Menics A20 series 22 mm mushroom operator with 3 x A20-E01P (1 NC) contact blocks | 1 | 40 x 40 x 60 | 60 | Panel hole 22.3 mm (standard 22 mm series). Ring nut. | estimate | [link](https://www.wolfautomation.com/contact-block-1nc-for-pushbuttons-o22mm) |
| canable_v2_pro | Makerbase MKS CANable V2.0 Pro, isolated USB to CAN FD | 2 | 52 x 16 x 12 | 8 | Hole pattern not verified. The non-Pro V2.0 (45.0 x 16.2 mm) has 4 holes 2.2 mm, 34.7 x 12.6 mm apart. The Pro board may differ. | listing | [link](https://www.devicemart.co.kr/goods/view?no=15775527) |
| meter_hav1808 | HAV1808 panel voltmeter 3.5-30 V, waterproof | 1 | 48 x 29 x 22 | 15 | Panel mount. Cut-out about 45.5 x 26.5 mm if it is the 0.56 inch type. | estimate | [link](https://www.icbanq.com/P017177967) |
| meter_hav6001 | HAV6001 panel volt and current meter 0-100 V, 100 A | 1 | 48 x 29 x 21 | 20 | Panel cut-out 45.5 x 26.5 mm (some listings say 46 x 27 mm). | listing | [link](https://www.icbanq.com/P017179301) |
| shunt_100a | Shunt 100 A / 75 mV for HAV6001 | 1 | 105 x 40 x 25 | 80 | 2 holes in an insulating base, near the ends. | estimate | [link](https://www.sigmanortec.ro/en/voltmeter-with-digital-ammeter-0-100v-0-100a-with-shunt) |
| meter_hav1625a | HAV1625a panel ammeter 4-30 V, 10 A | 1 | 48 x 29 x 22 | 20 | Panel mount. Cut-out about 45.5 x 26.5 mm if it is the 0.56 inch type. | estimate | [link](https://www.icbanq.com/P017179547) |
| xdrive_mini | Makerbase XDrive MINI motor driver (ODrive v3.6 based), reference only | 12 | ? | ? | ? | estimate | [link](https://www.devicemart.co.kr/goods/view?no=15775528) |
| camera_b0201 | Arducam B0201 USB camera, Sony IMX291 2 MP, M12 lens 120 deg | 1 | 38 x 38 x 25 | 25 | 38 x 38 mm board. Holes fit 34 x 34 mm and 28 x 28 mm patterns (welectron listing). Probably M2. | listing | [link](https://welectron.com/Arducam-B0201-2MP-IMX291-120-Low-Light-Camera-with-USB-20-UVC-S-Mount_1) |
| ref_bhl_battery | Reference: BHL battery, Zeee 6S 5000 mAh 120C (BHL BOM link) | 0 | 138 x 47 x 48.5 | 603 | ? | listing | [link](https://docs.google.com/spreadsheets/d/1AQEHcH_nPkXYfor2-h7bwNIUMmsePtAm53epnsWgZXc) |
| ref_bhl_pc | Reference: BHL computer, Beelink Mini S12 (Intel N95) | 0 | 115 x 102 x 39 | 280 | ? | listing | [link](https://www.amazon.com/dp/B0BVLPCDVW) |
| acrylic_b4_5mm | Acrylic plate 5 mm, B4 | 1 | 364 x 257 x 5 | 557 | Cut and drill as needed. | estimate | [link](https://smartstore.naver.com/sol321/products/10278086239) |
| profile_2020_200 | Aluminium profile 2020, 200 mm | 2 | 200 x 20 x 20 | 90 | 6 mm slots, M5 centre hole (typical 2020). | estimate | [link](https://www.yjprofile.co.kr/goods/goods_view.php?goodsNo=1000000042) |
| buck_12v_to_5v | Pololu D36V50F5 step-down regulator, 5 V 5.5 A, for the ROCK 3C (NOT in the course BOM) | 1 | 25.4 x 25.4 x 10 | 7 | Rail pocket on the pod basket lip, foam tape. | listing | [link](https://www.pololu.com/product/4091) |
| pump_klp02 | Kamoer KLP02 24 V brushless pump, with a DMC-101 DC pump controller | 1 | not published | about 250 | Not drawn yet. A KLP02 holder follows when the pump arrives and is measured. | datasheet | Kamoer |
| pump_dwp385 | motorbank DWP-385 12 V gear pump, self-priming. No longer used: the printed pump holder fits it | 0 | 99 x 34 x 48 | 130 | Printed pump holder on the rear of the middle shelf, 2 saddles and a strap. | estimate | [link](https://www.eleparts.co.kr/goods/view?no=3850995) |
| mosfet_dfr0457 | DFRobot DFR0457 MOSFET module, the switch of the DWP-385. Not used with the KLP02 | 0 | 44 x 32 x 15 | 10 | Rail pocket on the pod basket lip, foam tape. | estimate | [link](https://www.icbanq.com/P007616427) |
| relay_k2_kka_k4 | Relay K2 KKA-K4, 24 V coil, 40 A | 1 | 28 x 28 x 35 | 30 | Pod plate, foam tape and a zip tie. | estimate | [link](https://www.devicemart.co.kr/goods/view?no=14822397) |
| brake_resistor_rh50 | Brake resistor RARA RH50C4R70J, 4.7 Ohm 50 W | 2 | 71 x 30 x 16 | 31 | 4 x M3, ASSUMED 39.7 x 21.4 mm pattern. Printed mount on the front face of the front bottom rail. | estimate | [link](https://www.devicemart.co.kr/goods/view?no=33451) |

## Mass in the torso

- Batteries (2 x 6S Zeee + 1 x 3S): 1568 g.
- All parts in the table with qty > 0 and a known mass, batteries and the camera included: 2808 g.
- This sum has no XDrive drivers, no DMC-101, no opto-isolator, no wires, no acrylic plate and no profiles.
- The CAD layout (`layout.py`) uses the DWP-385 (130 g) and the DFR0457 (10 g) instead of the KLP02 (about 250 g).
- With the VEGA 6S packs, add 296 g.

## Electrical data and notes per part

### rock3c: Radxa ROCK 3C (RK3566) computer

- Electrical: 5 V only. Input is USB-C at 5 V (no USB PD), or 5 V on 40-pin header pins 2 and 4. Radxa asks for 5 V 3 A, or 5 V 4 A with an M.2 SSD. GPIO is 3.3 V (max 3.63 V). USB: 1 USB 3.0, 2 USB 2.0, 1 USB 2.0 OTG, 2.8 A total for the 4 ports. Other ports: Gigabit Ethernet, HDMI, 2-lane MIPI CSI (15-pin FPC), 2-lane MIPI DSI. Fan header: 5 V, 2-pin 1.25 mm, PWM. PWM pins: 7 (PWM14_M0), 16 (PWM8_M0), 18 (PWM9_M0), 19 (PWM15_IR_M1), 21 (PWM12_M1), 23 (PWM14_M1), 24 (PWM13_M1). Pins 7 and 23 are the same PWM14 channel. UARTs on pins 8/10 (UART2 debug), 3/5, 16/18, 32/33. I2C on pins 3/5 and 27/28.
- Notes: Board outline 85.0 x 56.0 mm from the Radxa v1.4 DXF. The Ethernet and USB jacks stick out 2.3 mm past one short edge, so L is 87.3 mm. Height 22 mm is an estimate: about 16 mm USB stack, 1.6 mm board, about 4 mm of parts and M.2 socket under the board. A heatsink 10 mm or lower stays under the USB stack. A heatsink with a fan adds about 4 to 6 mm. Mass is an estimate (Raspberry Pi 3 class boards are 42 to 50 g). Power draw is an estimate: about 2 W idle and 5 to 6 W at full CPU load, plus USB loads. The v1.4 schematic (sheet 29) shows USB-C VBUS goes straight to the 5 V rail through a 5 V TVS diode. 12 V on USB-C will damage the board. Schematic: https://dl.radxa.com/rock3/docs/hw/3c/v1400/radxa_rock_3c_v1400_schematic.pdf. 2D DXF: https://dl.radxa.com/rock3/docs/hw/3c/v1400/radxa_rock_3c_v1400_2d_dxf.zip

### imu_im10a: Hiwonder IM10A 10-axis IMU, metal shell version

- Electrical: USB-C to a USB-serial chip (Linux: /dev/ttyUSB0). Also 3.3 V TTL UART and I2C (400 kHz) on pin headers. Supply 5 V or 3.3 V, 12 mA. Default 9600 baud, settable 4800 to 921600. Output 10 Hz default, settable 0.2 to 200 Hz. At 200 Hz only acceleration, angular velocity and angle can be on. Protocol: WitMotion WIT standard protocol, 11-byte frames, header 0x55.
- Notes: Maker is Hiwonder. The course sheet links the metal shell variant (USD 89.99). The bare board variant is USD 79.99, 42.2 x 29.6 mm, 5.7 g. The sheet price USD 70.20 is the same as the BHL battery price, so it is probably a copy error. The core module is a WitMotion JY901B (label visible in the Hiwonder board photo). Frames: 0x55, type, 8 data bytes (four int16, low byte first), checksum = low 8 bits of the sum of the first 10 bytes. 0x51 acc = v/32768*16 g, temperature = v/100 degC. 0x52 gyro = v/32768*2000 deg/s. 0x53 roll, pitch, yaw = v/32768*180 deg. 0x54 magnetic field (raw LSB). 0x56 pressure and height (two int32). 0x59 quaternion = v/32768. Commands: unlock FF AA 69 88 B5, then the setting, then save FF AA 00 00 00. Rate 200 Hz: FF AA 03 0B 00. Baud 115200: FF AA 04 06 00. Use 115200 baud or more for 200 Hz (3 frames at 200 Hz need 66 kbit/s). The Hiwonder manual names a CP2102 chip but also a CH340 driver. A community driver (kscalelabs/imu) uses the CH341 driver. Protocol: https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol. Hiwonder manual: https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/1.User_Manual.html. Hiwonder downloads (PC software, ROS packages, code): https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/Appendix.html. Open-source driver with the frame parser: https://github.com/kscalelabs/imu/tree/master/imu/drivers/hiwonder

### lipo_6s_zeee: Zeee 6S LiPo 5000 mAh 22.2 V 120C, EC5, hard case

- Electrical: 6S1P, 22.2 V nominal, 25.2 V full, 111 Wh. 120C. EC5 discharge plug, JST-XH balance plug.
- Notes: Size tolerance is +/-2 mm (Zeee listing). This is the US link in the course sheet and also the battery in the BHL BOM. The Korean link in the course sheet is a different pack. See lipo_6s_vega_alt. Two packs are 1206 g.

### lipo_6s_vega_alt: VEGA 6S LiPo 5200 mAh 22.2 V 70C, XT90S (Korean link in the course sheet)

- Electrical: 6S1P, 22.2 V nominal, 115 Wh. 70C. XT90S anti-spark plug as sold. The course wants EC5.
- Notes: The Korean link (rcbank goods 66947, KRW 125,000) in the course sheet points to this pack, not to the Zeee pack. If the TA buys this pack, set qty to 2 here and 0 on lipo_6s_zeee. It is 16 mm longer and 148 g heavier per pack. The listing also gives 154 x 44 x 51 mm in a second place.

### lipo_3s_dinogy: Dinogy Graphene 3S LiPo 11.1 V 6000 mAh 80C

- Electrical: 3S1P, 11.1 V nominal, 12.6 V full, 66.6 Wh. 80C. The shop sells it with Deans, XT60, EC5, XT90 or XT90S. The course sheet asks for EC5.
- Notes: This is the Korean link in the course sheet (KRW 76,400). Brand is Dinogy. We did not find a Dinogy datasheet for this size.

### boost_ham5925: HAM5925 DC/DC boost converter 150 W, IN 10-32 V, OUT 12-35 V, 10 A, CV/CC

- Electrical: Non-isolated boost. Input 10-32 V. Output 12-35 V, adjustable. 10 A input max. 150 W with a fan, about 100 W without. Output must be higher than the input. Similar modules have no reverse-polarity protection. Screw terminals for input and output.
- Notes: icbanq gives no size. Size and mass are from the common 150 W 10-32 V to 12-35 V boost module listings (65 x 35 x 21 mm). The CV/CC version with current pots can be a little bigger. Measure the part.

### diode_mbrt400100: MBRT400100 Schottky diode module, 100 V 400 A, three-tower package

- Electrical: Common-cathode pair (2 x 200 A). 3 lug terminals with 1/4-20 UNC screws. Centre terminal is the common cathode. VF max 0.84 V per leg at 200 A. Isolated base plate.
- Notes: Size values are the datasheet maxima. Body (nameplate) is 60.5 to 62.5 mm long. Mass is an estimate from similar three-tower modules. Our reading: the two anodes take one 6S pack each and the cathode feeds the motor bus. This joins the two packs without back-feeding. The 'R' version has the opposite polarity.

### relay_aep18024: Panasonic AEP18024 DC power relay, 80 A, 1 Form A (SPST-NO), 24 V coil

- Electrical: Coil 24 V DC, 176 mA, 136 ohm, 4.2 W. Operate at max 18 V (75 %). Release at min 2 V. Max coil voltage 32 V (133 %). Coil plug Yazaki 7283-1020 with 300 mm leads (included). Contacts 80 A at 400 V DC. Load terminals are M5 studs, 44.9 mm apart, and are polarized: terminal 1 is (+) input.
- Notes: Body is 75 mm high. 79 mm includes the M5 bolts on top. The datasheet says 'Not recommended for new design' from 1 October 2025. Panasonic rates contact life with a varistor across the coil. A diode makes the life shorter. Use wire of 15 mm2 or more on the load terminals for 80 A (datasheet). It is the heaviest single electrical part.

### switch_pushlock_24v: Kacon (Konhung) push-lock switch 1A1B, 24 V DC LED, 22 mm, NF22-BA3C1

- Electrical: 1 NO + 1 NC, push-lock (maintained). LED lamp 24 V DC. NF22 series contacts are rated 3 A.
- Notes: Size is [bezel width, bezel width, depth behind the panel]. Bezel and depth are estimates for a 22 mm pushbutton. The 22.3 mm panel hole and 3 A rating are from NF22 series listings (Digi-Key, KOINO brand). Measure before you fix the panel layout.

### switch_pushlock_12v: Kacon (Konhung) push-lock switch 1A1B, 12 V DC LED, 16 mm, KBL16AT-L2C1

- Electrical: 1 NO + 1 NC, push-lock. LED lamp 12 V DC.
- Notes: Size is [bezel width, bezel width, depth behind the panel]. Only the 16 mm size and the 12 V LED come from the listing. The rest is an estimate for a 16 mm metal pushbutton.

### estop_a20: E-stop: Menics A20 series 22 mm mushroom operator with 3 x A20-E01P (1 NC) contact blocks

- Electrical: 3 NC contacts in series or separate. The course note says 24 V DC is available on the e-stop chain.
- Notes: Size is [head diameter, head diameter, depth behind the panel with 3 blocks]. The head sticks out about 30 to 35 mm in front of the panel. A20-E01P is a Menics A20 series 1 NC block. We did not find the Menics drawing. A Menics 3 NC e-stop part is A20B-V4E03R. Measure the part from ASMR2024.

### canable_v2_pro: Makerbase MKS CANable V2.0 Pro, isolated USB to CAN FD

- Electrical: USB-C, powered from USB 5 V (3.3-5 V). STM32G431C8T6 and ADM3050 isolated CAN FD transceiver, 5.7 kV rms signal isolation. CAN 2.0A/B and CAN FD, 1 Mbit/s classic. Screw terminal for CAN_H, CAN_L and GND. 120 ohm termination: on-board jumper (shorting cap), not a switch. BOOT jumper for DFU. Firmware candleLight (gs_usb, SocketCAN) or slcan.
- Notes: PCB size 52.0 x 16.0 x 1.6 mm is from the Amazon 'Macrobase CANable PRO V2.0' listing. H 12 mm is an estimate (screw terminal). Mass is an estimate. Makerbase publishes no Pro drawing (github.com/makerbase-mks/CANable-MKS has the V2.0 only). A known bug: after a quick reset it may not boot. Leave it unpowered about 2 s (github.com/SebKuzminsky/mks-canable-2.0-pro). Jumper method: Makerbase README for V2.0, CANable docs for the Pro, and seller listings.

### meter_hav1808: HAV1808 panel voltmeter 3.5-30 V, waterproof

- Electrical: 2-wire. Powered from the measured voltage, 3.5-30 V. Good for 6S (25.2 V max).
- Notes: icbanq gives no size. Size is a guess for a 0.56 inch panel meter. Some 3.5-30 V 2-wire meters are the smaller 0.28 inch type (about 23 x 15 mm). Measure the part.

### meter_hav6001: HAV6001 panel volt and current meter 0-100 V, 100 A

- Electrical: Uses an external shunt. 100 A needs a 100 A / 75 mV shunt. Some kits ship a 50 A shunt. Meter supply 4.5-30 V. Above 30 V it needs its own supply.
- Notes: Size and shunt facts are from listings of the common 0-100 V 100 A dual meter (Grobotronics, Sigmanortec, eBuild). icbanq gives no size. See shunt_100a.

### shunt_100a: Shunt 100 A / 75 mV for HAV6001

- Electrical: Goes in the negative lead of the motor bus. 2 big bolts for the bus wires, 2 small screws for the sense wires.
- Notes: 105 x 40 mm is from one listing. Height and mass are estimates. Check which shunt comes in the box.

### meter_hav1625a: HAV1625a panel ammeter 4-30 V, 10 A

- Electrical: Probably an internal shunt (10 A range). Supply 4-30 V.
- Notes: icbanq gives no size and no shunt type. Size is a guess for a 0.56 inch panel meter. Measure the part.

### xdrive_mini: Makerbase XDrive MINI motor driver (ODrive v3.6 based), reference only

- Electrical: Single-axis FOC driver. CAN on a 4-pin JST-SH connector. USB-C for setup. 24 V bus from the 6S packs.
- Notes: Not verified. A devicemart category page shows 25.8 x 21 x 1.6 mm next to this item. That looks too small for a 120 A peak driver. It sits on each actuator, not in the torso.

### camera_b0201: Arducam B0201 USB camera, Sony IMX291 2 MP, M12 lens 120 deg

- Electrical: USB 2.0 UVC. 5 V, 300 mA max. 1 m cable.
- Notes: H 25 mm (board plus M12 lens and holder) and the 25 g mass without cable are estimates. Shipping weight with cable is 70 g. Robotshop lists 36 x 36 mm for a similar Arducam board. Measure the hole spacing before printing a bracket.

### ref_bhl_battery: Reference: BHL battery, Zeee 6S 5000 mAh 120C (BHL BOM link)

- Electrical: 6S, 22.2 V.
- Notes: The BHL BOM sheet links the same Amazon item as the course (B0DGPMD46S), qty 1. The BHL paper says 6S 4000 mAh instead. We found no mass for a 4000 mAh pack. The course robot carries 2 of these plus a 3S pack: 1568 g against 603 g on the BHL.

### ref_bhl_pc: Reference: BHL computer, Beelink Mini S12 (Intel N95)

- Electrical: 12 V 3 A on a DC 5.5 x 2.5 mm jack. About 15 W.
- Notes: Link from the BHL BOM sheet. 280 g is from retailer listings (some say 41 mm high). The ROCK 3C is about 235 g lighter. The 12 V DC 5.5 x 2.5 mm jack explains the DC5525-XT60 cable and the 3S pack in the course sheet.

### acrylic_b4_5mm: Acrylic plate 5 mm, B4

- Electrical: Insulator.
- Notes: Korean shops use JIS B4 = 257 x 364 mm. 257 x 364 x 5 mm = 467.7 cm3, x 1.19 g/cm3 = 557 g. ISO B4 is 250 x 353 mm: 441.2 cm3 = 525 g. Mass is for the full uncut plate.

### profile_2020_200: Aluminium profile 2020, 200 mm

- Electrical: ?
- Notes: Maker Kyunghee Profile. The shop gives no kg/m. 2020 profiles weigh 0.40 to 0.50 kg/m (Framingtech 0.40, Systeal V-slot 0.456, Haluminium 0.50). 200 mm is 80 to 100 g. 90 g is the middle.

### buck_12v_to_5v: Pololu D36V50F5 step-down regulator for the ROCK 3C (NOT in the course BOM)

- Electrical: 5.5-50 V in, 5 V out, 5.5 A typical, with reverse-voltage, over-current and over-temperature protection. It feeds header pins 2 and 4 (`docs/hardware/electrical.md`).
- Notes: Team choice. 25.4 x 25.4 mm from Pololu. Height and mass are estimates.

### pump_klp02: Kamoer KLP02 24 V brushless pump, with a DMC-101 DC pump controller

- Kamoer data for the 24 V brushless KLP02: flow 0.7 L/min or more, head 30 m or more, suction lift 4 m or more, 12 W or less, about 250 g, "switch control".
- Size: not published. Port size: not published. The DMC-101 datasheet is not available yet.
- Proposed wiring: 24 V from the motor bus after relay K1, so the e-stop cuts it. Fuse F14 3 A. On/off from ROCK 3C pin 11 (GPIO3_A1) through an opto-isolator into the DMC-101. The software keeps one on/off GPIO (`robot/pump.py`).
- The printed pump holder fits the DWP-385. We will draw a KLP02 holder when the pump arrives and is measured.

### Old pump parts, relay K2, brake resistors

- `pump_dwp385` and `mosfet_dfr0457` are the pump and the pump switch of the earlier plan. The printed pump holder and the basket pocket still fit them.
- Relay K2 cuts the 12 V of the pan-tilt servos when the e-stop is pressed.
- The sizes and masses of these parts come from `docs/hardware/electrical.md` and shop listings. Head size, ear span and hole patterns are guesses. See `parts.json`.

### Assumed power paths

Our reading of the course sheet. The sheet has no wiring diagram. The current wiring is in `docs/hardware/electrical.md`.

- Motor bus: 6S packs A and B → diode (one anode each) → relay contacts → 100 A shunt → 12 XDrive MINI drivers.
- Computer: 3S pack → 12 V push-lock switch → 12 V to 5 V step-down converter → ROCK 3C.
- E-stop: 3S pack → HAM5925 boost (24 V) → 3 NC e-stop contacts → 24 V push-lock switch → relay coil.
- USB from the ROCK 3C: 2 CANable adapters, IMU, camera.

### ROCK 3C (item 1)

- Radxa product brief: "Radxa ROCK 3C can only be powered by +5V."
- The two inputs are USB-C at 5 V and 5 V on header pins 2 and 4.
- We checked the v1.4 schematic (sheet 29, "Power_DC IN"). USB-C VBUS goes straight to the 5 V rail.
- The board has no USB PD chip and no DC barrel jack. The CC pins have 5.1 kohm resistors, so a USB-C charger gives only 5 V.
- **CAUTION:** do not connect the 3S pack (9.9 to 12.6 V) to the board. 12 V on USB-C or on pins 2/4 destroys the board. You cannot undo this.
- Use a 12 V to 5 V step-down (buck) converter between the 3S pack and the board.
- Converter output: fixed 5.1 V, 5 A or more. Radxa asks for 5 V 3 A, or 5 V 4 A with an M.2 SSD.
- The USB loads add about 0.6 A: 2 CANable adapters, IMU 12 mA, camera 300 mA max.
- Option A: a converter with a USB-C output, into the ROCK 3C USB-C port.
- Option B: a converter with screw terminals, to header pin 2 or 4 (5 V) and pin 6 (GND). Option B has no USB-C plug that can come loose.
- The course sheet says "C type & DC 5.5mm x 2.5mm" and lists a "DC5525-XT60" cable for the PC.
- These lines match the BHL computer, a Beelink Mini S12 (Intel N95). It takes 12 V on a DC 5.5 x 2.5 mm jack.
- We think the course sheet kept these lines from the BHL BOM. Ask the TA which 12 V to 5 V converter they use.
- A "DC 5.5 x 2.5 mm to USB-C" cable is safe only with a built-in 5 V step-down. A plain adapter cable passes 12 V.
- Power draw (estimate, not measured): about 2 W idle, 5 to 6 W at full CPU load, plus the USB loads.
- GPIO is 3.3 V, max 3.63 V. Do not connect 5 V signals to the header.

### HAM5925 boost converter (item 5)

- Input 10 to 32 V. Output 12 to 35 V, set with a trimmer. The course note says "12V to 24V".
- We think it makes 24 V from the 3S pack for the e-stop chain and the relay coil. The e-stop note says "24V DC available".
- The input minimum is 10 V. A nearly empty 3S pack is at about 9.9 V. The converter can stop at the end of the 3S charge.
- The load is small. The relay coil takes 176 mA at 24 V.
- Set the output to 24.0 V with a meter before you connect the relay.

### AEP18024 relay (item 7)

- Coil: 24 V DC, 176 mA, 136 ohm, 4.2 W.
- The relay closes at 18 V or more. It opens below about 2 V. The maximum coil voltage is 32 V.
- Panasonic asks for pure DC within +/-5 % of 24 V.
- Put a varistor across the coil. Panasonic rates contact life with a varistor. A diode across the coil makes the contact life shorter.
- The coil plug is a Yazaki 7283-1020 with 300 mm leads. It comes with the relay.
- Contacts: 80 A at 400 V DC. The load terminals are polarized. Connect terminal 1 (+) to the battery side.
- Use wire of 15 mm2 or more on the load terminals for 80 A (datasheet). The course sheet lists 18 AWG (0.82 mm2) wire.
- The datasheet says "Not recommended for new design" from 1 October 2025.

### IMU IM10A (item 2)

- The IMU takes 5 V and data from one ROCK 3C USB port. It uses 12 mA.
- Default 9600 baud and 10 Hz. Set 115200 baud and 200 Hz before use (commands in `parts.json`).
- Protocol: WitMotion WIT standard protocol. https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol
- Hiwonder manual and downloads: https://docs.hiwonder.com/projects/IMU-Module/en/latest/docs/1.User_Manual.html

### CANable V2.0 Pro (item 10)

- Each adapter takes 5 V from one ROCK 3C USB port.
- The CAN side is isolated from USB (ADM3050 isolated transceiver).
- Connect CAN_H, CAN_L and GND to the XDrive bus.
- A CAN bus needs 120 ohm at both ends. Put the termination jumper on when the adapter is at one end of its bus.
