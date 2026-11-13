# ID07 (left knee pitch, 6512) 기록 메모

- 보드 serial 325834543435. 팀원에게서 받은 데이터 (2026-11-09, "Actuator data.zip").
- 길들이기는 이 보드가 node 5였을 때 `--id 05`로 돌렸다 (팀원 확인: "길들이는 과정에서는 id 설정을 잘못"). 2026-11-09에
  구간 기록 meta의 id를 05 -> 07로 고치고 그림(.png) 제목도 다시 그렸다 (원래 값은 meta의 "note"). 2026-11-09 11:04–11:07에
  설정을 지우고 `setup_driver.py`로 node 7로 다시 설정했다 (before_setup 두 개: node 5, 지운 뒤 node 0).
- `break_in.csv` 첫 줄(20261108_203440, 2.07분)은 DC_BUS_UNDER_VOLTAGE로 멈춘 세션이고 구간 기록 파일은 받지 못했다.
  누적 85.8분 = 2.07 + 3.18 + 80.54.
