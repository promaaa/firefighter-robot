"""CAN node ID만 바꾼다 (보드 번호를 잘못 매겼을 때). 다른 설정은 건드리지 않는다.

    python set_node_id.py --joint left_hip_roll     # bench/joints.py 표에서 node 1
    python set_node_id.py --node 1                  # 직접 지정
    python set_node_id.py --node 1 --print          # 장치 없이 odrivetool 명령만 출력

보드는 한 개만 꽂는다. serial과 지금 node ID를 보여 주고 확인을 받은 뒤
먼저 재부팅하고(다른 스크립트가 강제 종료되어 RAM에 남긴 임시 값이 같이 저장되지 않게),
axis0.config.can_node_id만 쓰고 save_configuration() -> reboot() -> 다시 읽어 확인한다.
odrivetool 셸 대신 쓰는 이유: odrivetool 0.5.1 셸은 IPython 9.x에서 열리지 않는다
(AttributeError: module 'IPython.terminal' has no attribute 'embed').
"""
import argparse

from bench import joints, odrv_usb, util
from setup_driver import Runner


def label(node):
    j = joints.by_node(node)
    return f"node {node} ({j[0][4:-6]}, {j[2]})" if j else f"node {node} (표에 없음)"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", help="관절 (예: left_hip_roll). node ID를 표에서 가져옴")
    ap.add_argument("--node", type=int, help="새 CAN node ID (1-62)")
    ap.add_argument("--print", dest="dry", action="store_true", help="장치 없이 명령만 출력")
    ap.add_argument("--timeout", type=float, default=15.0, help="USB 장치 검색 시간 [s]")
    a = ap.parse_args()
    if a.joint:
        node = joints.find(a.joint)[1]
        if a.node is not None and a.node != node:
            ap.error(f"--joint {a.joint}는 node {node}인데 --node {a.node}를 줬습니다")
        a.node = node
    if a.node is None:
        ap.error("--joint 또는 --node 가 필요합니다")
    if not 1 <= a.node <= 62:
        ap.error("node ID는 1-62 (63은 axis1용, 0은 미설정 보드 기본값)")

    if a.dry:
        r = Runner(None, True)
        r.reboot()
        r.set("axis0.config.can_node_id", a.node, note=label(a.node))
        r.save()
        r.reboot()
        return

    a.sim = False
    odrv = odrv_usb.connect(a)
    serial = f"{odrv_usb.read(odrv, 'serial_number', 0):012X}"
    old = odrv.axis0.config.can_node_id
    print(f"\n보드 serial {serial}")
    print(f"  지금: {label(old)}")
    print(f"  바꿀 값: {label(a.node)}")
    if old == a.node:
        print("이미 같은 값입니다. 바꿀 것 없음.")
        return
    if odrv.axis0.current_state != odrv_usb.STATE_IDLE:
        raise RuntimeError(f"축이 IDLE이 아닙니다 (state {odrv_usb.state_name(odrv.axis0.current_state)}). "
                           "IDLE에서만 저장하세요.")
    print("같은 CAN 버스에 이 번호를 쓰는 보드가 이미 있으면 안 됩니다.")
    util.confirm("이 보드의 node ID를 바꾸고 저장할까요?")

    r = Runner(odrv, False)
    print("저장된 설정만 남기려고 먼저 재부팅합니다.")
    r.reboot()
    if r.odrv.axis0.config.can_node_id != old:
        raise RuntimeError("재부팅 뒤 node ID가 처음 읽은 값과 다릅니다. 다른 보드가 연결되었는지 확인하세요.")
    if r.odrv.axis0.current_state != odrv_usb.STATE_IDLE:
        raise RuntimeError("재부팅 뒤 축이 IDLE이 아닙니다 (startup_closed_loop_control 확인).")
    r.set("axis0.config.can_node_id", a.node)
    r.save()
    r.reboot()
    back = r.odrv.axis0.config.can_node_id
    if back != a.node:
        raise RuntimeError(f"재부팅 후 node ID가 {back}로 읽힙니다 (저장 실패)")
    print(f"\n완료: serial {serial}, {label(old)} -> {label(a.node)} (재부팅 후 확인)")
    print("  액추에이터 번호 스티커도 바꾸고, 이후 스크립트는 --id "
          f"{a.node:02d} 로 실행하세요. CAN 확인: python 06_can_check.py scan")


if __name__ == "__main__":
    util.run(main)
