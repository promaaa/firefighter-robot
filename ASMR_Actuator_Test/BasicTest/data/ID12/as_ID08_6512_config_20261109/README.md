# 2026-11-09 오전: 이 보드를 ID08(6512 무릎)로 잘못 설정했을 때의 기록

- 보드 serial 325634613435. 실제로는 오른쪽 발목 pitch(5010)이며, 같은 날 10:53에 `setup_driver.py --joint right_ankle_pitch`로 다시 설정해 지금은 node 12.
- 10:21–10:29: 출고 백업, 코스 펌웨어 0.5.1, `setup_driver.py --joint right_knee_pitch`(6512 값: kt 0.067, 게인 12.6 / 0.223).
  모터 보정은 5010에서 했으므로 상저항 232.2 mOhm, 인덕턴스 103.4 uH는 5010 값.
- 10:32: `08_break_in.py --id 08 --motor 6512 --minutes 3` (10 turn/s, 3.18분).
  토크는 kt 0.067로 계산되어 있어 5010(kt 0.0836)으로는 x1.248: 저속 마찰 0.205 -> 약 0.256 Nm, 10 turn/s 마찰 0.199 -> 약 0.248 Nm.
  이 3분은 `data/ID12/break_in.csv` 누적 시간에 들어 있지 않다.
