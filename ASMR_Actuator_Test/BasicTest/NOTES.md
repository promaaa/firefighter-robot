# BasicTest 개발 메모 (XDrive MINI, ODrive fw 0.5.1)

> **작업 절차는 노션 문서를 따릅니다:** [⚙️ 모터 세팅](https://app.notion.com/p/40f9f50be3fc800f87f4f70a8d9c05ec) · [📊 액추에이터 측정 결과](https://app.notion.com/p/40f9f50be3fc81b38370ca54d97d96ea)  
> 이 파일은 개발하면서 남긴 메모입니다. 일부 내용(예: "측정 순서"의 예시 ID)은 지난 내용일 수 있습니다.


인수 점검 artifact(액추에이터 인수 점검)의 3–8단계를 스크립트로 옮긴 것이다. 시험 스크립트(00–09, check_wiring, sensor_mode, set_gains)는
`save_configuration()`을 부르지 않는다. 설정은 RAM에서만 바꾸고, 끝날 때(Ctrl+C 포함) 축을 IDLE로 두고 원래 값으로 되돌린다.
드라이버 설정을 저장하는 것은 `setup_driver.py` 하나뿐이다.

## 설치된 것 (2026-11-04)

| 항목 | 위치 / 버전 |
|---|---|
| Python 3.12.10 | winget 사용자 설치, `py -3.12`. 사용자 PATH에 추가됨 (기본 `python`은 여전히 miniconda) |
| 가상환경 | `%USERPROFILE%\odrv051`: odrive 0.5.1.post0 (`odrivetool --version` 확인), python-can 4.6.1, gs_usb, libusb-package |
| Zadig 2.9 | `tools\zadig-2.9.exe` (Akeo 서명 확인). 장치를 꽂아야 쓸 수 있어 현장에서 실행 |

0.5.4 환경은 만들지 않았다. 수업 문서의 코스 펌웨어가 MKS 0.5.1 기반이기 때문이다. 01이 펌웨어 버전을 출력하니
0.5.1이 아니면 그때 artifact의 0.5.4 설치 명령을 쓰면 된다.

## 매번 시작할 때

```bat
%USERPROFILE%\odrv051\Scripts\activate
cd %USERPROFILE%\Desktop\ASMR_ActuatorTest\BasicTest
python 00_env_check.py
```

PowerShell이면 `& $env:USERPROFILE\odrv051\Scripts\Activate.ps1`.
USB는 한 번에 한 프로그램만 연결된다. 스크립트를 돌리기 전에 `odrivetool` 창을 닫는다.

## 현장에서 처음 한 번: Zadig (ODrive USB)

1. 드라이버에 DC 전원을 켜고 USB 연결 (USB 전원만으로는 켜지지 않음)
2. `tools\zadig-2.9.exe` → Options → **List All Devices** → **ODrive 3.6 Native Interface (Interface 2)** 선택.
   USB ID **1209 0D32**와 Interface 2인지로 찾는다. List All Devices를 켜지 않으면 "0 devices found"로 보인다.
   Windows가 이 인터페이스에 WinUSB를 자동으로 붙이지만 DeviceInterfaceGUID가 등록되지 않아 libusb가 열지 못한다
   (2026-11-04 libusb 로그로 확인). 그래서 Zadig로 드라이버를 다시 설치해야 한다
3. 대상 드라이버 **libusb-win32** → Install(Replace) Driver. STM32 BOOTLOADER나 Interface 0/1(직렬)은 건드리지 않는다
4. `python 00_env_check.py`에서 `libusb0 ... 있음`, ODrive MI_02 드라이버가 `libusb0`인지 확인 → `odrivetool` → `Connected to ODrive ... as odrv0`

## CAN 어댑터 (MKS CANable V2.0 Pro)

- **출고 펌웨어 slcan**: Windows가 COM 포트로 잡는다(드라이버 설치 불필요). 기본값 `--interface slcan`, COM 포트는 자동 탐색하고 안 되면 `--channel COM5`
- **HW팀이 candleLight로 바꿨다면**: 00에서 `1D50:606F`, 드라이버가 WinUSB인지 확인(아니면 Zadig로 WinUSB). `--interface gs_usb`
- 비트레이트 기본 500000 (수업 문서). 드라이버의 `can.config.baud_rate`와 같아야 한다(01이 보여 줌, 0.5.1 기본은 250000)
- 버스 양 끝 120 Ω 종단

## 드라이버 처음부터 설정 (2026-11-05 결정)

HW팀이 해 둔 설정은 무시하고 **수업 문서 절차(코스 펌웨어 → `xdrive_config.txt`)대로 처음부터** 설정한다.
다른 점은 아래와 같다.

| 항목 | 수업 | 우리 | 이유 |
|---|---|---|---|
| `dc_bus_undervoltage_trip_level` | 21 V | **10 V** | 길들이기 등에 12 V 50 A SMPS 사용. **액추에이터 모델링이 끝나면 21 V로 올린다** |
| `brake_resistance` | 0 (저항 없음) | **실측 저항값 (2 Ω 50 W 저항을 달고 `--brake-resistance`)** | SMPS는 모터가 되돌려 보내는 전기를 받지 못해, 급감속 때 전압이 올라 과전압 차단에 걸리기 쉽다. 저항이 열로 버린다. 저항을 떼면 0으로 되돌린다 |
| 동작 시험 목표 | 절대 위치 1, 0, 0.5 turn | 현재 위치 기준 상대값 | 시작 자세와 무관하게 같은 크기만 움직이게 |
| `axis1.config.can_heartbeat_rate_ms` | 100 (기본) | **0 (끔)** | 모든 보드의 axis1이 같은 ID 63으로 heartbeat를 보내 버스에서 겹치지 않게 (펌웨어 `interface_can.cpp`에서 0이면 안 보냄) |
| node ID | 예시 23 | 아래 표 | 관절별 고유 번호 |

### 1) 코스 펌웨어 올리기

- 파일: `tools\firmware\xdrive-mini-fw-instant-start-v0.5.1-instant-start.1.hex`
  (릴리스 v0.5.1-instant-start.1, SHA-256 `06e74f7c…7df5`. 같은 폴더에 .elf도 있음)
- 두 방법 모두 STM32CubeProgrammer가 필요하다 (ST 계정 로그인 후 다운로드: https://www.st.com/en/development-tools/stm32cubeprog.html)
- odrivetool의 `dfu`/업그레이드 명령은 쓰지 않는다 (표준 ODrive 펌웨어가 올라가 보드가 동작하지 않게 됨)

**A. USB-C만으로 (DFU, 추가 장비 없음).** 수업 문서 방법은 아니다. XDrive Mini에 BOOT 버튼이 있다는 제3자 안내(FFBeast)를 따른다.
STM32 칩 안의 ROM 부트로더를 쓰는데, 이 부트로더는 지워지지 않으므로 실패해도 다시 시도할 수 있다.
1. USB-C는 꽂은 채로 DC 전원을 끈다 → BOOT 버튼을 누른 채 DC 전원을 켠다 → 버튼을 놓는다
2. 장치 관리자나 00_env_check에 `STM32 BOOTLOADER`(USB ID 0483:DF11)가 보이면 DFU 모드
3. CubeProgrammer: 오른쪽 위 연결 방식을 **USB** → 새로고침 → Connect → Erasing & Programming → .hex 선택 → Verify 체크 → Start Programming
4. Disconnect → DC 전원을 껐다 켜면 새 펌웨어로 시작 (RESET 버튼으로는 DFU에 들어가지 못한다)

**B. ST-Link V2로 (수업 문서 방법).** 수업 BOM에 팀당 1개 포함(47,500원). A가 안 될 때나 ST-Link를 받았으면 이쪽. 보드는 DC 전원으로 켜고 ST-Link를 보드의 SWD 핀(SWDIO, SWCLK, GND)에 연결.
핀 위치는 보드 인쇄를 확인. CubeProgrammer: ST-LINK 선택 → Connect → 이후 A의 3–4와 같다

### 2) 설정 스크립트

```bat
python setup_driver.py --joint left_hip_roll --print   :: 장치 없이 명령만 미리 보기
python setup_driver.py --joint left_hip_roll           :: 실제 설정 (단계마다 확인)
```

0 백업 → 1 erase → 2 전원 → 3 모터 → 4 엔코더 → 5 제어기(재부팅) → 6 보정(모터·엔코더, 재부팅) → 7 동작 시험 →
8 CAN → 9 PASSTHROUGH(재부팅, ±0.2 turn 시험) → 10 설정 후 백업. 단계마다 명령을 보여 주고 확인을 받으며,
erase는 `erase`를 직접 입력해야 진행한다. 중간에 멈추면 `--from-step N`으로 이어서 한다.
6단계 끝에 재부팅 후에도 보정값이 남아 있는지 확인하고, 부팅 위치로 펌웨어 종류를 추정해 알려 준다
(보정값 유지는 출고·코스 펌웨어 모두 같아서 구분 근거가 되지 않는다).
저장되는 게인은 수업 값이고, 학습 등가 게인은 시험 스크립트가 RAM에서만 바꾼다(`--gains train`).

### 관절별 CAN node ID ([bench/joints.py](bench/joints.py))

수업 문서에는 번호 규칙이 없어서 원본 Berkeley Humanoid Lite 저수준 코드(`robot/humanoid.py`)의 다리 번호를 따랐다.
왼쪽은 홀수, 오른쪽은 짝수다. 9·10은 원본에서 팔 관절 번호라 비워 둔다. 0은 설정 안 된 보드의 기본값이라 쓰지 않는다.

| 관절 | 왼쪽 | 오른쪽 | 모터 |
|---|---|---|---|
| hip_roll | 1 | 2 | 6512 |
| hip_yaw | 3 | 4 | 6512 |
| hip_pitch | 5 | 6 | 6512 |
| knee_pitch | 7 | 8 | 6512 |
| ankle_pitch | 11 | 12 | 5010 |
| ankle_roll | 13 | 14 | 5010 |

원본은 왼쪽 다리를 `can0`, 오른쪽 다리를 `can1`의 두 버스로 나눈다. 번호가 겹치지 않으니 한 버스로 묶어도 된다.
모든 보드의 axis1은 63번으로 두고 heartbeat를 끈다.

## 측정 순서

액추에이터 모델링 측정은 **08 길들이기 → (몇 시간 식힘) → 07 무부하 마찰 → 09 Kd → 03 강성·부하 마찰** 순서로 한다.
길들이기 전 마찰은 10–20% 크고, 마찰은 식은 상태에서 가장 먼저 잰다 (2026-11-06 ID01·ID02 결과, 노션 측정 결과 페이지 5절).

**번호 변경 (2026-11-09):** 처음 측정한 골반 두 대는 hip yaw(ID03·ID04)가 아니라 hip roll이었다. 골반(base)에 달린 모터가
hip roll을 돌린다. 모델 링크는 그 링크를 움직이는 관절 이름을 따르고, 링크에 달린 모터는 한 칸 아래 관절을 돌린다
(허벅지 `leg_*_hip_pitch`의 모터 = 무릎 6512, 정강이 `leg_*_knee_pitch`의 모터 = 발목 pitch 5010).
`set_node_id.py`로 보드 node를 바꾸고(328934703435: 3 -> 1, 3272346E3435: 4 -> 2) `data/ID03`·`ID04`를 `ID01`·`ID02`로 옮겼다.
그 폴더 안의 json·csv 내용(`"id": "03"` 등)과 노션 이전 기록의 ID03·ID04는 지금의 ID01·ID02다.

`--id`는 데이터 폴더 이름이다. 관절 node ID(예: `01`)를 쓰면 `setup_driver.py` 기록과 같은 폴더에 모인다.
`--motor`는 6512(고관절·무릎, 큰 모터) 또는 5010(발목). 결과는 `data/ID01/`에 쌓인다.

| artifact 단계 | 명령 | 모터 |
|---|---|---|
| 3·4 백업·설정 대조 | `python 01_backup_and_check.py --id 01 --motor 6512` | 안 움직임 |
| 5 부팅 위치 | `python 02_units_zero.py --id 01 --motor 6512 --boot` (전원 껐다 켤 때마다) | 안 움직임 |
| 5 감속비·방향·연속성 | `python 02_units_zero.py --id 01 --motor 6512` | +10°, +90° 왕복 (5 A, 5 turn/s) |
| 6 강성·부하 마찰 (현재 설정) | `python 03_stiffness.py --id 01 --motor 6512 --gains asis --mass 2.486 --arm 0.1` | 암을 +-6° 자동 왕복 |
| 6 강성·부하 마찰 (학습 등가) | `python 03_stiffness.py --id 01 --motor 6512 --gains train --mass 2.486 --arm 0.1` | 암을 +-6° 자동 왕복 |
| 6 큰 부하 (6 Nm 근처) | `python 03_stiffness.py --id 01 --motor 6512 --gains train --mass 6.3 --arm 0.1` | 전류 한계는 기본 10 A 유지, 추 밑에 받침 |
| 길들이기 | `python 08_break_in.py --id 01 --motor 6512 --minutes 60 --plot` | 출력축 무부하, 정·역 회전 |
| 무부하 마찰 (식은 상태) | `python 07_friction_sweep.py --id 01 --motor 6512 --plot` | 출력축 무부하, 여러 바퀴 |
| Kd | `python 09_damping.py --id 01 --motor 6512 --gains train` | 출력축 무부하, 6 Nm 한계로 수십 도 |
| 7 토크–전류 | `python 04_torque_friction.py torque --id 01 --motor 6512` | 토크 모드, **암 고정 필수** |
| 7 마찰 | `python 04_torque_friction.py friction --id 01 --motor 6512` | IDLE |
| 7 백래시 | `python 04_torque_friction.py backlash --id 01 --motor 6512 --gains train` | 위치 유지 |
| 8 스텝 응답 (USB) | `python 05_step_response.py --id 01 --motor 6512 --gains train --steps 3,10 --plot` | 스텝 |
| 8 CAN 확인 | `python 06_can_check.py scan` → `latency --node 1` → `move --node 1 --deg 10` | move만 움직임 |
| 8 스텝 응답 (CAN) | `python set_gains.py --motor 6512 --gains train` 후 `python 05_step_response.py --id 01 --motor 6512 --link can --node 1 --plot` | 스텝 |

각 스크립트는 `-h`로 옵션을 볼 수 있다. 모든 단계를 다 할 필요는 없다. 꼭 할 것은 01, 02와 위의 모델링 측정(08 → 07 → 09 → 03)이다.

### 측정 방법 메모

- **01**: odrivetool `backup-config`와 같은 내용을 `xdrive_ID01.json`으로 저장한다. 되돌릴 때는 `odrivetool restore-config 파일`(RAM에만 씀)
- **03 강성·부하 마찰** (2026-11-05 자동 왕복으로 변경): 마찰 때문에 한쪽에서 접근한 처짐으로는 Kp가 틀린다. 예전에는 손으로 더 눌렀다 놓아(d2)
  양쪽을 쟀는데, 누르는 정도에 따라 마찰이 0.11~0.62 Nm로 흔들렸다. 이제 모터가 암을 +-6°, 6°/s로 여러 번 왕복하며
  +방향/-방향 모터 토크의 차이로 마찰, 평균으로 중력을 구하고, 암만 / 추를 건 두 번의 차이로 Kp와 kt 비율을 구한다. 결과는 `stiffness_auto.csv`
- **05 지표**: 지연(5%), 상승(10–90%), 95% 도달, 오버슈트, 정상상태 오차. 마찰 때문에 6512는 약 0.24°, 5010은 약 0.8° 덜 가서 멈출 수 있으니
  3° 스텝은 95%에 못 미칠 수 있다. 모델 곡선은 같은 명령을 학습 모델(관성·마찰)과 드라이버 제어기 식으로 계산한 것이다.
  암을 달았으면 `--arm-inertia`로 관성을 넣고, 나중에 `python plot_step.py 파일.csv --inertia 0.03`처럼 다시 그려 볼 수 있다
- 시간 값(USB 샘플링, CAN 왕복)은 Windows USB·스케줄링 지연이 섞인 참고용이다. 배포 판단은 Ubuntu PC에서 다시 잰다

## 배선을 바꾼 뒤: check_wiring.py

모터 3상이나 DC 전원 연결 방식을 바꾸면 접촉 저항이 생겼는지 확인한다. 보드에 저장하지 않는다(RAM 값 원복).

```bat
python check_wiring.py --id 03 --motor 6512 --note "바꾼 연결 설명"
python check_wiring.py --id 03 --motor 6512 --repeat 8    :: 재는 동안 선을 살살 흔들어 접촉 불량 찾기
```

- **A. 모터 3상**: 모터 보정(5 A)으로 상저항을 다시 재서 저장된 값과 비교한다. 1.15배 넘으면 주의, 1.5배 넘으면 문제.
  악어 케이블일 때 513 mOhm, 납땜 후 69 mOhm (M6C12 두 대 67.9 / 68.9 mOhm)
- **B. DC 전원**: 같은 측정을 15 A로 3초 하면서 Vbus 강하를 재서 전원 경로 저항을 구한다. 정상은 수십 mOhm.
  최대 DC 전류(12 A)일 때 예상 Vbus가 저전압 차단(10 V)에 가까우면 문제
- 출력축은 1도 미만 움찔하고 삑 소리가 난다. 결과는 `data/IDxx/wiring_check.csv`에 쌓인다

## 코스 펌웨어에서 확인한 것

수업 문서가 권장하는 코스 펌웨어(`omletkang/xdrive-mini-fw-instant-start`, MKS ODrive 0.5.1 수정판)의 소스와
`xdrive_config.txt`를 읽고 artifact에 더한 내용이다.

- **코스 펌웨어가 바꾼 것** (커밋 7f1a413, MKS 원본 대비 `encoder.cpp` 세 곳):
  1. 부팅 때 위치 추정을 절대 엔코더 값으로 시작한다. 출고 펌웨어는 0에서 시작해 실제 위치로 따라가며 튀거나 속도가 순간 크게 잡힌다
  2. SPI가 바쁠 때 `ERROR_ABS_SPI_NOT_READY` 에러로 축을 끄지 않고 한 주기 건너뛴다 (긴 길들이기·로봇 구동 때 중요)
  3. 절대 엔코더는 보정 직후가 아니어도 `pre_calibrated = True`를 넣을 수 있다 (restore-config 등)
  - 보정값을 저장해 두면 재부팅 후 다시 보정하지 않는 것은 **출고 펌웨어도 같다**(`abs_spi_cb`가 `pre_calibrated`면 준비 완료로 둠).
    하드웨어팀이 출고 펌웨어로 위치 제어에 성공한 것과 맞는다
- **펌웨어 버전 표시**: MKS 출고 펌웨어는 `0.0.0`, 코스 펌웨어 릴리스(v0.5.1-instant-start.1)는 `0.5.1`로 표시된다
  (2026-11-05 같은 보드에서 올리기 전후로 확인). 버전만 봐도 구분된다
- **부팅 직후 위치**: 코스 펌웨어는 `pos_estimate`가 **0~1 turn (출력 0~24°)**, 출고 펌웨어는 **-0.5~0.5 turn**에서 시작한다
  (artifact의 ±0.5 예상은 출고 펌웨어 동작). 부팅 직후 음수면 출고, 0.5보다 크면 코스 펌웨어다. `02 --boot`로 확인
- **xdrive_config.txt 게인**: 6512 pos_gain 12.6 / vel_gain 0.223 (관절 Kp 100.6, Kd 7.99 = 학습의 1.12배 / 1.39배),
  5010 20 / 0.1 (Kp 71.6, Kd 3.58 = 학습의 **3.3배 / 7.3배**). HW팀이 이 파일대로 설정했다면 발목은 학습과 크게 다르다
- 그 밖의 값: current_lim 18, torque_constant 0.067 (5010은 0.0836), encoder bandwidth 3000, vel_limit 50,
  CAN 500k, node ID 예시 23, brake_resistance 0 + dc_max_negative_current −3 A (브레이크 저항 없음: 손으로 빠르게 돌리면 과전압 에러 가능)
- CAN Simple: Get 계열은 RTR 요청, 위치를 주기적으로 보내지 않음, CAN 메시지를 받을 때마다 워치독 갱신

## 장치 없이 리허설

USB 스크립트는 `--sim`, CAN은 `--interface virtual`. 학습 모델 값으로 움직이는 가짜 드라이버를 쓰고, 게인은 코스 설정(xdrive_config)으로 시작한다.
그래서 가상 CAN으로 05를 돌릴 때는 `--model-gains course`를 줘야 모델과 맞는다. 실측을 대신하지 않는다.

```bat
python 03_stiffness.py --id 99 --motor 6512 --gains train --sim
python 05_step_response.py --id 99 --motor 5010 --link can --interface virtual --node 5 --model-gains course --plot
```

## 파일

```
00_env_check.py        설치·장치·Windows 드라이버 확인
01_backup_and_check.py 설정 백업 + 학습 기준 대조 (읽기만)
02_units_zero.py       부팅 위치 기록 / 감속비·방향·연속성
03_stiffness.py        강성, kt 비율, 부하 마찰 (추를 건 채 자동 왕복)
04_torque_friction.py  torque | friction | backlash
05_step_response.py    스텝 응답 기록(USB/CAN) + 모델 비교
06_can_check.py        scan | monitor | latency | move
07_friction_sweep.py   무부하 마찰: 여러 속도로 돌리며 Iq -> 쿨롱·점성 (추·암 필요 없음)
08_break_in.py         길들이기: 정·역 번갈아 회전하며 마찰 추이 기록 (누적 시간 관리, 안전 정지)
09_damping.py          Kd: 목표를 일정 속도로 움직여 피드포워드 없음/있음의 위치 지연 차이로 감쇠 측정 (추·암 없음)
set_gains.py           RAM에서만 게인 변경 (CAN 시험 전, 원복 안 함, 재부팅하면 원래대로)
setup_driver.py        드라이버 처음부터 설정 (수업 xdrive_config 순서, 저장함)
check_wiring.py        배선 바꾼 뒤 상저항(3상)·전원 경로 저항(DC) 확인 (저장 안 함)
sensor_mode.py         현재 각도를 한 줄로 계속 표시. --hold면 위치 유지하며 밀린 각·전류·토크 (RAM만)
plot_step.py           05 결과 그림 (모델 다시 계산)
bench/                 공통: params(학습값·단위), joints(관절별 node ID), model(제어기+관절 모델), odrv_usb, can_simple, sim, stepan, util
data/IDxx/             결과
tools/zadig-2.9.exe, tools/firmware/ (코스 펌웨어 .hex/.elf)
```

## 끝나면 공유할 것

- `data/IDxx/` 폴더 전체 (백업 JSON, check, units, boot_pos, stiffness, torque, friction, backlash, step csv/png, can_latency)
- 기록표 사진, 암 길이와 무게, 슬로모션 영상, HW팀 답변(영점 방법, PC 코드, 어댑터 펌웨어, 비트레이트)
