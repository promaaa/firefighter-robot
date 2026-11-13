"""8단계: 스텝 응답 기록 + 학습 모델 예측과 비교.

USB (이 스크립트가 RAM에서 게인을 바꾸고 끝나면 되돌림):
    python 05_step_response.py --id 01 --motor 6512 --gains train --steps 3,10 --plot
CAN (0.5.1 CAN으로는 게인을 못 바꾸므로 set_gains.py로 RAM에 먼저 맞춰 둠):
    python 05_step_response.py --id 01 --motor 6512 --link can --interface slcan --node 1  --steps 3,10 --plot

각 스텝은 p0 -> p0+step -> p0 왕복. 출력축 deg 단위. 결과: data/ID01/step_<시각>.csv (+ .png)
암은 아래로 늘어뜨리거나 떼고 한다(중력 토크 없이). 암을 달았다면 --arm-inertia 로 관성을 넣는다.
USB 샘플링은 수백 Hz, Windows 시간 측정에는 USB·스케줄링 지연이 섞인다(참고용).
"""
import argparse
import math
import time

from bench import can_simple, model, odrv_usb, params, stepan, util


class UsbIO:
    def __init__(self, axis, with_iq):
        self.axis, self.with_iq, self.n = axis, with_iq, 0

    def command(self, pos):
        self.axis.controller.input_pos = pos

    def read(self):
        a = self.axis
        pos, vel = a.encoder.pos_estimate, a.encoder.vel_estimate
        iq = a.motor.current_control.Iq_measured if self.with_iq else math.nan
        self.n += 1
        if self.n % 50 == 0:
            odrv_usb.check_alive(a)
        return pos, vel, iq, math.nan


class CanIO:
    stream = True  # 배포처럼 매 주기 목표를 보냄

    def __init__(self, node, with_iq):
        self.node, self.with_iq = node, with_iq

    def command(self, pos):
        self.node.set_input_pos(pos)

    def read(self):
        pos, vel = self.node.get_encoder_estimates()
        rtt = self.node.last_rtt
        iq = self.node.get_iq()[1] if self.with_iq else math.nan
        hb = self.node.heartbeat
        if hb and hb[1] != odrv_usb.STATE_CLOSED_LOOP:
            raise RuntimeError(f"축이 closed loop에서 빠졌습니다 (heartbeat error 0x{hb[0]:X}, state {hb[1]})")
        return pos, vel, iq, rtt


def record(io, p0, steps_turn, pre, hold, rate):
    seq, t = [(0.0, p0)], pre
    for s in steps_turn:
        seq += [(t, p0 + s), (t + hold, p0)]
        t += 2 * hold
    t_total = t
    rows, k, target = [], 0, p0
    period = 1.0 / rate if rate else 0.0
    t0 = time.perf_counter()
    next_t = 0.0
    while (now := time.perf_counter() - t0) < t_total:
        while k + 1 < len(seq) and seq[k + 1][0] <= now:
            k += 1
        if seq[k][1] != target or getattr(io, "stream", False):
            target = seq[k][1]
            io.command(target)
        ta = time.perf_counter() - t0
        pos, vel, iq, rtt = io.read()
        tb = time.perf_counter() - t0
        rows.append((round((ta + tb) / 2, 6), target, pos, vel, iq, round((tb - ta) * 1000, 3)))
        if period:
            next_t = max(next_t + period, tb)
            rem = next_t - (time.perf_counter() - t0)
            if rem > 0:
                time.sleep(rem)
    return rows


def drive_for_model(args, snap=None):
    """모델에 넣을 드라이버 설정. USB면 실제 값, CAN이면 가정."""
    m = params.MOTORS[args.motor]
    if snap:
        return model.DriveConfig(snap["pos_gain"], snap["vel_gain"], snap["vel_integrator_gain"], snap["vel_limit"],
                                 snap["current_lim"], snap["torque_constant"], snap["torque_lim"])
    pg, vg, ig = params.gains_for(args.motor, args.model_gains)
    print(f"[모델 가정] CAN으로는 게인을 읽을 수 없어 '{args.model_gains}' 게인 {pg:g}/{vg:g}, "
          f"torque_constant = kt {m['kt']}, current_lim {args.current_lim or params.CURRENT_LIM:g} A로 계산합니다.")
    return model.DriveConfig(pg, vg, ig, args.vel_limit, args.current_lim or params.CURRENT_LIM, m["kt"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    odrv_usb.add_args(ap)
    can_simple.add_args(ap, node_required=False)
    ap.add_argument("--link", choices=["usb", "can"], default="usb")
    ap.add_argument("--gains", default="train", choices=["asis", "train", "course", "default"], help="USB에서만")
    ap.add_argument("--model-gains", default="train", choices=["train", "course", "default"],
                    help="CAN일 때 모델 계산에 가정할 게인 (set_gains.py로 맞춘 것과 같게)")
    ap.add_argument("--steps", default="3,10", help="스텝 크기 목록 [출력 deg]")
    ap.add_argument("--pre", type=float, default=0.3, help="첫 스텝 전 대기 [s]")
    ap.add_argument("--hold", type=float, default=0.8, help="각 스텝 유지 [s]")
    ap.add_argument("--rate", type=float, default=0.0, help="샘플링 [Hz], 0이면 최대한 빠르게 (CAN 권장 500)")
    ap.add_argument("--iq", action="store_true", help="Iq도 기록 (샘플링이 느려짐)")
    ap.add_argument("--vel-limit", type=float, default=params.VEL_LIMIT)
    ap.add_argument("--current-lim", type=float, default=None, help="RAM에서 바꿀 전류 한계 [A] (USB). 기어 허용치 확인 후")
    ap.add_argument("--arm-inertia", type=float, default=0.0, help="출력축에 단 부하 관성 [kg*m^2]")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    m = params.MOTORS[args.motor]
    steps_turn = [params.out_deg_to_turn(float(s)) for s in args.steps.split(",")]
    out_dir = util.data_dir(args.id)
    print(f"스텝 {args.steps} deg (모터 {', '.join(f'{s:.4f}' for s in steps_turn)} turn), 링크 {args.link}")
    print("출력축이 스텝만큼 빠르게 움직입니다. 암 주변을 비우세요.")

    rows, vbus = None, None
    if args.link == "usb":
        odrv = odrv_usb.connect(args)
        axis = odrv.axis0
        with odrv_usb.RamSession(odrv) as s:
            s.set_gains(args.motor, args.gains)
            s.set("controller.config.control_mode", odrv_usb.CONTROL_POSITION)
            s.set("controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH)
            s.set("controller.config.vel_limit", args.vel_limit)
            if args.current_lim:
                s.set("motor.config.current_lim", args.current_lim)
            snap = odrv_usb.drive_snapshot(axis)
            cfg = drive_for_model(args, snap)
            warn_saturation(cfg, m, steps_turn)
            util.confirm("시작")
            odrv_usb.enter_closed_loop(axis)
            p0 = odrv_usb.average(axis, 0.3, with_iq=False)["pos"]
            rate = args.rate or (1000.0 if args.sim else 0.0)  # 시뮬레이션은 읽기가 즉시 끝나므로 속도 제한
            rows = record(UsbIO(axis, args.iq), p0, steps_turn, args.pre, args.hold, rate)
            odrv_usb.wait_settled(axis, timeout=1.0)
            vbus = odrv_usb.read(odrv, "vbus_voltage")
    else:
        if args.node is None:
            ap.error("--link can 에는 --node 가 필요합니다")
        if args.interface == "virtual":
            args.sim_motor = args.motor
        bus = can_simple.open_bus(args)
        node = can_simple.Node(bus, args.node)
        cfg = drive_for_model(args)
        warn_saturation(cfg, m, steps_turn)
        try:
            vbus = node.get_vbus()
        except TimeoutError as e:
            raise RuntimeError(f"{e}. node ID, 비트레이트, 종단저항, 배선을 확인하세요.")
        util.confirm("시작")
        try:
            node.clear_errors()
            node.set_controller_modes(odrv_usb.CONTROL_POSITION, odrv_usb.INPUT_PASSTHROUGH)
            node.set_vel_limit(args.vel_limit)
            p0, _ = node.get_encoder_estimates()
            node.set_input_pos(p0)
            node.set_axis_state(odrv_usb.STATE_CLOSED_LOOP)
            if not node.wait_state(odrv_usb.STATE_CLOSED_LOOP, 2.0):
                raise RuntimeError(f"closed loop 진입 실패 (heartbeat {node.heartbeat})")
            rows = record(CanIO(node, args.iq), p0, steps_turn, args.pre, args.hold, args.rate or 500.0)
            time.sleep(0.3)
        finally:
            node.set_axis_state(odrv_usb.STATE_IDLE)
            print("축 IDLE. (CAN으로 바꾼 control/input mode, vel_limit은 RAM 값이라 재부팅하면 원래대로)")
            bus.shutdown()
    if not rows:
        return

    plant = model.Plant.for_motor(args.motor, args.arm_inertia)
    gains = args.gains if args.link == "usb" else f"model-{args.model_gains}"
    if vbus is not None and vbus < 20.0:
        print(f"참고: Vbus {vbus:.1f} V. 24 V일 때보다 최고 속도가 낮아(역기전력) 큰 스텝·고속 구간은 24 V 결과와 다를 수 있습니다.")
    meta = {"id": args.id, "motor": args.motor, "link": args.link, "gains": gains, "steps_deg": args.steps,
            "vbus_V": vbus,
            "p0_turn": p0, "drive": vars(cfg), "plant": vars(plant), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
    path = out_dir / f"step_{util.stamp()}.csv"
    util.write_log(path, meta, ["t", "target_turn", "pos_turn", "vel_turn_s", "iq", "read_ms"], rows)

    t = [r[0] for r in rows]
    ref = [r[1] for r in rows]
    pos = [r[2] for r in rows]
    dts = [b - a for a, b in zip(t, t[1:])]
    read_ms = sorted(r[5] for r in rows)
    print(f"\n샘플 {len(rows)}개, 평균 {1 / (sum(dts) / len(dts)):.0f} Hz, 최대 간격 {max(dts) * 1000:.1f} ms, "
          f"읽기 시간 중앙값 {read_ms[len(read_ms) // 2]:.2f} ms / 95% {read_ms[int(0.95 * len(read_ms))]:.2f} ms")
    pred = model.simulate(cfg, plant, t, ref, pos[0])
    print(f"모델: Kp {params.joint_gains(cfg.pos_gain, cfg.vel_gain, cfg.torque_constant, plant.kt)[0]:.1f}, "
          f"관성 {plant.inertia:.4f}, 쿨롱 {plant.coulomb}, 점성 {plant.viscous}")
    for i, j, r0, r1 in stepan.edges(t, ref):
        mm = stepan.metrics(t, pos, i, j, r0, r1)
        mp = stepan.metrics(t, pred, i, j, r0, r1)
        print(f"\n[{mm['step_deg']:+.1f} deg] 실측: {stepan.fmt_metrics(mm)}")
        print(f"{'':12s}모델: {stepan.fmt_metrics(mp)}")

    if args.plot:
        import plot_step
        plot_step.plot(path)


def warn_saturation(cfg, m, steps_turn):
    kp, _ = params.joint_gains(cfg.pos_gain, cfg.vel_gain, cfg.torque_constant, m["kt"])
    need = kp * params.turn_to_out_rad(max(abs(s) for s in steps_turn))
    lim = min(cfg.current_lim * cfg.torque_constant, cfg.torque_lim) / cfg.torque_constant * m["kt"] * params.GEAR_RATIO
    if need > lim:
        print(f"참고: 가장 큰 스텝의 초기 토크 {need:.1f} Nm > 토크 한계 {lim:.1f} Nm (current_lim {cfg.current_lim:g} A). "
              "처음엔 전류 제한에 걸립니다(모델도 같은 제한으로 계산).")


if __name__ == "__main__":
    util.run(main)
