"""CAN 점검 (CAN Simple, fw 0.5.1). 기본 어댑터는 출고 펌웨어(slcan, COM 포트), 500 kbps.

    python 06_can_check.py scan                                  # heartbeat로 node ID 찾기 + Vbus/위치 읽기
    python 06_can_check.py monitor --node 1                      # 위치·속도·Iq 10 Hz 출력 (Ctrl+C로 끝)
    python 06_can_check.py latency --node 1  --n 2000            # Set Input Pos + RTR 왕복 시간
    python 06_can_check.py move --node 1  --deg 10               # CAN으로 closed loop -> +10 deg -> 원위치 -> IDLE

candleLight 펌웨어면 --interface gs_usb, 장치 없이 연습은 --interface virtual.
"""
import argparse
import statistics
import time

from bench import can_simple, odrv_usb, params, util


def cmd_scan(bus, args):
    print(f"heartbeat 듣는 중 ({args.duration:g} s)...")
    found = can_simple.scan(bus, args.duration)
    if not found:
        print("아무 heartbeat도 없습니다. 비트레이트(드라이버 can.config.baud_rate), CAN H/L 배선, 120 옴 종단,"
              " 드라이버 전원을 확인하세요.")
        return
    for nid, (err, state, n) in sorted(found.items()):
        node = can_simple.Node(bus, nid)
        try:
            vbus = node.get_vbus()
            pos, vel = node.get_encoder_estimates()
            extra = (f"Vbus {vbus:.2f} V, pos {pos:.4f} turn (출력 {params.turn_to_out_deg(pos):.2f} deg), "
                     f"RTT {node.last_rtt * 1000:.2f} ms")
        except TimeoutError as e:
            extra = f"RTR 응답 없음 ({e})"
        print(f"node {nid:2d}: heartbeat {n}회, axis error 0x{err:X}, state {odrv_usb.state_name(state)} | {extra}")


def cmd_monitor(node, args):
    print("Ctrl+C로 끝")
    while True:
        pos, vel = node.get_encoder_estimates()
        iq_set, iq = node.get_iq()
        hb = node.heartbeat or (0, 0)
        print(f"pos {pos:9.4f} turn ({params.turn_to_out_deg(pos):8.2f} deg)  vel {vel:7.3f} turn/s  "
              f"Iq {iq:6.2f} A  state {hb[1]} err 0x{hb[0]:X}")
        time.sleep(0.1)


def cmd_latency(node, args):
    pos0, _ = node.get_encoder_estimates()
    hb = node.wait_heartbeat(0.5)
    closed = hb is not None and hb[1] == odrv_usb.STATE_CLOSED_LOOP
    print(f"{args.n}회: Set Input Pos({'현재 위치 유지' if closed else '축 IDLE이라 효과 없음'}) + "
          f"Get Encoder Estimates(RTR) 왕복")
    rtts, fails = [], 0
    t_start = time.perf_counter()
    for _ in range(args.n):
        t0 = time.perf_counter()
        node.set_input_pos(pos0)
        try:
            node.get_encoder_estimates()
        except TimeoutError:
            fails += 1
            continue
        rtts.append((time.perf_counter() - t0) * 1000)
    total = time.perf_counter() - t_start
    rtts.sort()
    q = lambda p: rtts[min(len(rtts) - 1, int(p * len(rtts)))]
    print(f"성공 {len(rtts)} / 실패 {fails}, {args.n / total:.0f} 주기/s")
    print(f"왕복 [ms]: 평균 {statistics.fmean(rtts):.3f}, 중앙 {q(0.5):.3f}, 95% {q(0.95):.3f}, "
          f"99% {q(0.99):.3f}, 최대 {rtts[-1]:.3f}")
    bits = 12 * (108 + 44 + 108)  # 8바이트 프레임 약 108 bit, RTR 요청 약 44 bit
    print(f"참고: 12축이면 한 주기 명령12+요청12+응답12 프레임, 약 {bits} bit -> "
          f"{args.bitrate // 1000} kbps에서 버스 점유만 {bits / args.bitrate * 1000:.1f} ms (비트 스터핑 제외)")
    print("Windows 값에는 USB·스케줄링 지연이 섞여 참고용입니다. 배포 판단은 Ubuntu PC에서 다시 재세요.")
    util.append_csv(util.data_dir(args.id) / "can_latency.csv", {
        "time": util.stamp(), "interface": args.interface, "bitrate": args.bitrate, "node": args.node,
        "n": len(rtts), "fail": fails, "mean_ms": round(statistics.fmean(rtts), 4), "p50_ms": round(q(0.5), 4),
        "p95_ms": round(q(0.95), 4), "p99_ms": round(q(0.99), 4), "max_ms": round(rtts[-1], 4)})


def cmd_move(node, args):
    print(f"CAN으로 closed loop 진입 -> 출력 +{args.deg:g} deg -> 원위치 -> IDLE. 암 주변을 비우세요.")
    util.confirm("준비되면")
    try:
        node.clear_errors()
        node.set_controller_modes(odrv_usb.CONTROL_POSITION, odrv_usb.INPUT_PASSTHROUGH)
        p0, _ = node.get_encoder_estimates()
        node.set_input_pos(p0)
        node.set_axis_state(odrv_usb.STATE_CLOSED_LOOP)
        if not node.wait_state(odrv_usb.STATE_CLOSED_LOOP, 2.0):
            raise RuntimeError(f"closed loop 진입 실패 (heartbeat {node.heartbeat}). USB로 dump_errors 확인")
        for deg in (args.deg, 0.0):
            target = p0 + params.out_deg_to_turn(deg)
            t_end = time.perf_counter() + args.hold
            while time.perf_counter() < t_end:  # 배포처럼 주기적으로 목표 전송 (25 Hz)
                node.set_input_pos(target)
                pos, _ = node.get_encoder_estimates()
                time.sleep(params.POLICY_DT)
            print(f"  명령 {deg:+g} deg -> 엔코더 {params.turn_to_out_deg(pos - p0):+.3f} deg")
    finally:
        node.set_axis_state(odrv_usb.STATE_IDLE)
        print("축 IDLE")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["scan", "monitor", "latency", "move"])
    can_simple.add_args(ap, node_required=False)
    ap.add_argument("--id", default="00", help="액추에이터 번호 (기록 파일용)")
    ap.add_argument("--duration", type=float, default=1.5, help="scan 시간 [s]")
    ap.add_argument("--n", type=int, default=2000, help="latency 반복 횟수")
    ap.add_argument("--deg", type=float, default=10.0, help="move 크기 [출력 deg]")
    ap.add_argument("--hold", type=float, default=1.5, help="move 각 위치 유지 [s]")
    args = ap.parse_args()
    if args.cmd != "scan" and args.node is None:
        ap.error(f"{args.cmd} 에는 --node 가 필요합니다 (먼저 scan)")

    bus = can_simple.open_bus(args)
    try:
        if args.cmd == "scan":
            cmd_scan(bus, args)
        else:
            node = can_simple.Node(bus, args.node)
            try:
                node.get_vbus()
            except TimeoutError as e:
                raise RuntimeError(f"{e}. scan으로 node ID를 확인하세요.")
            {"monitor": cmd_monitor, "latency": cmd_latency, "move": cmd_move}[args.cmd](node, args)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    util.run(main)
