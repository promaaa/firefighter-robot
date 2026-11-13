"""MotorTestGUI: 시뮬레이션과 실물 로봇이 같게 세팅됐는지 보고, 관절을 조금씩 움직여 보는 프로그램.

  PowerShell (VS Code 터미널 기본):
    & "$env:USERPROFILE\\odrv051\\Scripts\\Activate.ps1"
    cd "$env:USERPROFILE\\Desktop\\ASMR_ActuatorTest\\MotorTestGUI"
  cmd(명령 프롬프트)라면 첫 두 줄 대신:
    %USERPROFILE%\\odrv051\\Scripts\\activate
    cd %USERPROFILE%\\Desktop\\ASMR_ActuatorTest\\MotorTestGUI
  그다음:
    python motor_test_gui.py                     # 실물 (위쪽에서 CAN 어댑터·포트 1·2 고르고 '연결')
    python motor_test_gui.py --virtual            # 장치 없이 가상 로봇으로 연습 (바로 연결, 버스 2개)
    python motor_test_gui.py --virtual --virtual-flip "L knee pitch,R hip yaw"   # 방향이 틀린 관절 흉내

- 왼쪽 3D: CAN으로 읽은 엔코더 값 -> 관절 각도 -> 수업 MJCF 모델 (실물)
- 오른쪽 3D: 몸통을 고정한 같은 모델을 MuJoCo로 진행, 실물과 같은 목표 궤적을 받음 (시뮬)
- 오른쪽 패널: 관절 선택, 실시간 그래프(각도·속도·토크), 명령(토크 켜기/힘 풀기, 상대·절대 이동), 영점·방향
- 위: CAN 연결 상태(CANable 1~2개), 노드 12개 상태, Vbus, 버스 통계, 모두 힘 풀기(Esc)

설정은 config/robot_config.json에 저장된다 (node, 방향, 영점, 토크 부호, 이동 제한).
필요한 패키지: requirements.txt (PySide6-Essentials, pyqtgraph, mujoco + BasicTest의 python-can 등).
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="설정 파일 (기본 config/robot_config.json)")
    ap.add_argument("--mjcf", default=None, help="시뮬 MJCF 파일 (기본: 수업 Robot Model v1)")
    ap.add_argument("--virtual", action="store_true", help="가상 로봇으로 바로 연결")
    ap.add_argument("--virtual-flip", default="", help="가상 로봇에서 방향을 뒤집을 관절 (쉼표로, 예: 'L knee pitch')")
    ap.add_argument("--virtual-nodes", default="", help="가상 로봇에 있을 node 번호 (쉼표로, 비우면 12개 전부)")
    ap.add_argument("--virtual-buses", type=int, choices=(1, 2), default=2,
                    help="가상 로봇의 CAN 버스 수. 2(기본): 왼다리 버스 1, 오른다리 버스 2 / 1: 한 버스에 12개")
    ap.add_argument("--virtual-powered", default="",
                    help="처음부터 토크가 켜져 있는 가상 node (쉼표로). 연결하면 프로그램이 IDLE로 돌리는지 확인용")
    ap.add_argument("--connect", action="store_true", help="저장된 CAN 설정으로 바로 연결")
    args = ap.parse_args()

    from PySide6.QtWidgets import QApplication, QMessageBox

    from motorgui import theme
    from motorgui.config import CONFIG_PATH, DEFAULT_MJCF, RobotConfig

    qapp = QApplication(sys.argv)
    qapp.setApplicationName("MotorTestGUI")
    theme.apply(qapp)

    cfg = RobotConfig.load(args.config or CONFIG_PATH)
    if args.mjcf:
        cfg.mjcf = str(Path(args.mjcf).resolve())
    mjcf = cfg.mjcf_path()
    if not mjcf.exists():
        if mjcf == DEFAULT_MJCF:
            r = QMessageBox.question(None, "로봇 모델 없음",
                                     "수업 로봇 모델(MJCF + STL, 약 10 MB)이 없습니다.\n"
                                     "수업 문서 사이트에서 내려받을까요?")
            if r != QMessageBox.Yes:
                return 1
            import fetch_model
            try:
                fetch_model.fetch()
            except Exception as e:  # noqa: BLE001
                QMessageBox.critical(None, "내려받기 실패", f"{e}\n\npython fetch_model.py 로 다시 시도하세요.")
                return 1
        else:
            QMessageBox.critical(None, "MJCF 없음", f"파일이 없습니다: {mjcf}")
            return 1

    from motorgui.app_state import App
    from motorgui.main_window import MainWindow

    core = App(cfg, mjcf)
    vk = {"buses": args.virtual_buses}
    if args.virtual_flip:
        vk["flips"] = [s.strip() for s in args.virtual_flip.split(",") if s.strip()]
    if args.virtual_nodes:
        vk["nodes"] = {int(s) for s in args.virtual_nodes.split(",") if s.strip()}
    if args.virtual_powered:
        vk["powered"] = {int(s) for s in args.virtual_powered.split(",") if s.strip()}
    if args.virtual:
        cfg.can_interface = "virtual"
    win = MainWindow(core, virtual_kwargs=vk, autoconnect=args.virtual or args.connect)
    win.show()
    return qapp.exec()


if __name__ == "__main__":
    sys.exit(main())
