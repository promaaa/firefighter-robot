"""드라이버를 처음부터 설정한다. 수업 문서(코스 펌웨어 + xdrive_config.txt) 순서를 그대로 따른다.

    python setup_driver.py --joint left_hip_roll                # node ID와 모터 종류를 bench/joints.py 표에서
    python setup_driver.py --motor 6512 --node 1                # 직접 지정
    python setup_driver.py --joint left_hip_roll --print        # 장치 없이 odrivetool 명령만 출력
    python setup_driver.py --joint left_hip_roll --from-step 6  # 중간 단계부터 다시

xdrive_config.txt와 다른 점:
  - dc_bus_undervoltage_trip_level 21 -> 10 V. 12 V SMPS를 쓰기 때문이며, 액추에이터 모델링이 끝나면 21 V로 올린다
  - 동작 시험의 목표를 절대 위치(1, 0, 0.5 turn) 대신 현재 위치 기준 상대값으로 준다
  - axis1(안 쓰는 축)의 heartbeat를 끈다. 모든 보드의 axis1이 같은 ID 63으로 heartbeat를 보내지 않게
  - node ID는 bench/joints.py 표 (예시 23 대신)
펌웨어(코스 instant-start)는 이 스크립트 전에 올린다 (노션 "모터 세팅" 문서 5장: USB DFU 또는 ST-Link).
단계마다 할 명령을 보여 주고 확인을 받은 뒤 실행한다. 저장은 그 단계 안에서 한다.
"""
import argparse
import json
import math
import threading
import time

from bench import joints, odrv_usb, params, util

UNDERVOLTAGE = 10.0
OVERVOLTAGE = 26.0
CAN_BAUD = params.CAN_BITRATE

STATE_MOTOR_CAL = 4
STATE_ENCODER_OFFSET_CAL = 7
MOTOR_TYPE_HIGH_CURRENT = 0
ENCODER_MODE_SPI_ABS_AMS = 257
INPUT_MODE_TRAP_TRAJ = 5


class Runner:
    """명령을 odrivetool 문법으로 출력하고, --print가 아니면 실행·확인한다."""

    def __init__(self, odrv, dry):
        self.odrv, self.dry = odrv, dry
        self.serial = None
        if odrv is not None:
            import fibre.utils
            self.serial = fibre.utils.get_serial_number_str(odrv)

    @staticmethod
    def show(line, note=""):
        print(f"    {line:58s}" + (f"# {note}" if note else ""))

    def set(self, path, value, shown=None, note=""):
        self.show(f"odrv0.{path} = {shown or value}", note)
        if self.dry:
            return
        obj, attr = odrv_usb.resolve(self.odrv, path)
        setattr(obj, attr, value)
        back = getattr(obj, attr)
        same = (math.isclose(back, value, rel_tol=1e-5, abs_tol=1e-6) if isinstance(value, float)
                else back == value)
        if not same:
            raise RuntimeError(f"{path}에 {value}를 썼는데 {back}로 읽힙니다")

    def call(self, line, path, *args, disconnects=False):
        self.show(f"odrv0.{line}")
        if self.dry:
            return
        obj, attr = odrv_usb.resolve(self.odrv, path)
        fn = getattr(obj, attr)
        if not disconnects:
            fn(*args)
            return
        # 보드가 바로 재부팅해 응답이 오지 않으므로 별도 스레드로 부르고 다시 연결한다
        threading.Thread(target=_quiet_call, args=(fn, *args), daemon=True).start()
        self.reconnect()

    def save(self):
        self.call("save_configuration()", "save_configuration")

    def reboot(self):
        self.call("reboot()", "reboot", disconnects=True)

    def reconnect(self):
        import odrive
        print("    (재부팅 기다린 뒤 다시 연결...)")
        time.sleep(3.0)
        odrv = odrive.find_any(serial_number=self.serial, timeout=20)
        if odrv is None:
            raise RuntimeError("재부팅 후 다시 연결되지 않습니다. 전원·USB 확인 후 --from-step 으로 이어서 하세요.")
        self.odrv = odrv
        print(f"    다시 연결됨 (Vbus {odrv.vbus_voltage:.2f} V)")

    def run_state(self, state, name, timeout=60.0):
        self.show(f"odrv0.axis0.requested_state = {name}")
        if self.dry:
            return
        axis = self.odrv.axis0
        old = odrv_usb.axis_errors(axis)
        if old:
            print("      이전 에러를 지우고 시작: " + "; ".join(old))
            odrv_usb.clear_errors(axis)
        axis.requested_state = state
        t0 = time.time()
        while True:
            time.sleep(0.2)
            if time.time() - t0 > 1.0 and axis.current_state == odrv_usb.STATE_IDLE:
                break
            if time.time() - t0 > timeout:
                axis.requested_state = odrv_usb.STATE_IDLE
                raise RuntimeError(f"{name}이(가) {timeout:.0f}초 안에 끝나지 않았습니다")
        errs = odrv_usb.axis_errors(axis)
        if errs:
            hints = odrv_usb.error_hints(errs)
            raise RuntimeError(f"{name} 실패:\n  " + "\n  ".join(errs) +
                               ("\n확인할 것:\n  - " + "\n  - ".join(hints) if hints else ""))
        print(f"      완료 ({time.time() - t0:.1f} s), 에러 없음")

    def move_test(self, offsets):
        """closed loop에서 현재 위치 p0 기준 상대 이동 후 IDLE."""
        self.show("odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL")
        for off in offsets:
            self.show(f"odrv0.axis0.controller.input_pos = p0 {off:+g}", f"출력 {params.turn_to_out_deg(off):+.1f} deg")
        self.show("odrv0.axis0.requested_state = AXIS_STATE_IDLE")
        if self.dry:
            return
        axis = self.odrv.axis0
        try:
            odrv_usb.enter_closed_loop(axis)
            p0 = axis.encoder.pos_estimate
            for off in offsets:
                target = p0 + off
                axis.controller.input_pos = target
                t0 = time.time()
                while time.time() - t0 < 5.0:
                    odrv_usb.check_alive(axis)
                    if abs(axis.encoder.pos_estimate - target) < 0.01 and abs(axis.encoder.vel_estimate) < 0.05:
                        break
                    time.sleep(0.02)
                time.sleep(0.3)
                d = axis.encoder.pos_estimate - p0
                print(f"      명령 {off:+.2f} turn -> {d:+.4f} turn (출력 {params.turn_to_out_deg(d):+.2f} deg)")
                if abs(d - off) > 0.1:  # 정상은 마찰로 0.01 turn 정도 덜 감
                    raise RuntimeError(
                        f"명령 {off:+.2f} turn인데 {d:+.4f} turn에 있습니다. 모터가 명령대로 움직이지 않습니다.\n"
                        "확인할 것:\n"
                        "  - 힘을 뺀 상태에서 출력축을 손으로 돌려 걸리는 데가 없는지\n"
                        "  - python sensor_mode.py 로 출력축을 손으로 한 바퀴 돌렸을 때 '시작 기준'이 360 deg로 "
                        "읽히는지 (자석·magnet extender·보드 위치)\n"
                        "  - python check_wiring.py --id <ID> --motor <6512|5010> --skip-dc (모터 3상 연결)\n"
                        "  - 액추에이터를 다시 조립했다면 6단계 보정부터 다시 (--from-step 6)")
        finally:
            axis.requested_state = odrv_usb.STATE_IDLE


def _quiet_call(fn, *args):
    try:
        fn(*args)
    except Exception:
        pass


# ---------------- 단계 ----------------

def step_backup(r, a):
    """0. 지금 설정 백업 (바꾸기 전)"""
    if r.dry:
        r.show("(odrivetool backup-config 와 같은 내용을 파일로 저장)")
        return
    from odrive.configuration import get_dict
    path = util.data_dir(a.id) / f"xdrive_ID{a.id}_before_setup_{util.stamp()}.json"
    with open(path, "w") as f:
        json.dump(get_dict(r.odrv, False), f, indent=1)
    print(f"    백업: {path}")


def step_erase(r, a):
    """1. 저장된 설정 지우기 (보드가 재부팅됨)"""
    r.call("erase_configuration()", "erase_configuration", disconnects=True)
    if not r.dry:
        ax = r.odrv.axis0
        print(f"      startup 보정 {ax.config.startup_encoder_offset_calibration}, "
              f"startup closed loop {ax.config.startup_closed_loop_control} (둘 다 False여야 함)")


def step_board(r, a):
    """2. 메인보드(전원) 설정"""
    r.set("config.brake_resistance", a.brake_resistance,
          note="수업 0 (저항 없음)" if a.brake_resistance == 0 else "실제로 달린 브레이크 저항 값")
    r.set("config.dc_bus_undervoltage_trip_level", a.undervoltage, note="수업 21 V. 12 V SMPS용으로 낮춤")
    r.set("config.dc_bus_overvoltage_trip_level", OVERVOLTAGE)
    r.set("config.dc_max_positive_current", 12.0)
    r.set("config.dc_max_negative_current", -3.0)
    r.set("config.max_regen_current", 0.0)
    r.save()


def step_motor(r, a):
    """3. 모터 설정"""
    p = "axis0.motor.config."
    r.set(p + "pole_pairs", 14)
    r.set(p + "calibration_current", 5.0)
    r.set(p + "resistance_calib_max_voltage", 2.0)
    r.set(p + "motor_type", MOTOR_TYPE_HIGH_CURRENT, "MOTOR_TYPE_HIGH_CURRENT")
    r.set(p + "current_lim", params.CURRENT_LIM)
    r.set(p + "requested_current_range", 25.0)
    r.set(p + "torque_constant", params.MOTORS[a.motor]["kt"], note=f"{a.motor} 실측 kt")
    r.save()


def step_encoder(r, a):
    """4. 엔코더 설정 (보드의 AS5047P)"""
    p = "axis0.encoder.config."
    r.set(p + "mode", ENCODER_MODE_SPI_ABS_AMS, "ENCODER_MODE_SPI_ABS_AMS")
    r.set(p + "abs_spi_cs_gpio_pin", 7)
    r.set(p + "cpr", 16384)
    r.set(p + "bandwidth", 3000.0)
    r.set(p + "calib_range", 10.0)
    r.save()


def step_controller(r, a):
    """5. 제어기 설정 (저장 후 재부팅)"""
    pg, vg = params.MOTORS[a.motor]["course_gains"]
    p = "axis0.controller.config."
    r.set(p + "control_mode", odrv_usb.CONTROL_POSITION, "CONTROL_MODE_POSITION_CONTROL")
    r.set(p + "vel_limit", params.VEL_LIMIT)
    r.set(p + "pos_gain", pg, note=f"수업 {a.motor} 값")
    r.set(p + "vel_gain", vg, note=f"수업 {a.motor} 값")
    r.set(p + "vel_integrator_gain", 0.0)
    r.set(p + "input_mode", INPUT_MODE_TRAP_TRAJ, "INPUT_MODE_TRAP_TRAJ")
    r.set("axis0.trap_traj.config.vel_limit", 10.0)
    r.set("axis0.trap_traj.config.accel_limit", 5.0)
    r.set("axis0.trap_traj.config.decel_limit", 5.0)
    r.save()
    r.reboot()


def step_calibrate(r, a):
    """6. 1회 보정: 모터(삑 소리) -> 엔코더 오프셋(출력축이 앞뒤로 돎) -> 저장·재부팅"""
    r.run_state(STATE_MOTOR_CAL, "AXIS_STATE_MOTOR_CALIBRATION")
    if not r.dry:
        mc = r.odrv.axis0.motor.config
        print(f"      상저항 {mc.phase_resistance * 1000:.1f} mOhm, 인덕턴스 {mc.phase_inductance * 1e6:.1f} uH")
    r.set("axis0.motor.config.pre_calibrated", True)
    r.run_state(STATE_ENCODER_OFFSET_CAL, "AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
    r.set("axis0.encoder.config.pre_calibrated", True)
    r.save()
    r.reboot()
    if r.dry:
        return
    ax = r.odrv.axis0
    pos = ax.encoder.pos_estimate
    mp, ep = ax.motor.config.pre_calibrated, ax.encoder.config.pre_calibrated
    ready = odrv_usb.read(ax, "encoder.is_ready")
    print(f"      재부팅 후: motor pre_calibrated {mp}, encoder pre_calibrated {ep}, encoder is_ready {ready}, "
          f"pos {pos:.4f} turn")
    if not (mp and ep and ready):
        raise RuntimeError("재부팅 후 보정값이 유지되지 않았습니다. dump_errors로 확인하세요.")
    # 보정값 유지는 출고·코스 펌웨어 모두 같으므로 부팅 위치로 구분한다
    print(f"      펌웨어 추정: {odrv_usb.firmware_from_boot_pos(pos)[1]}")


def step_move(r, a):
    """7. 동작 시험 (TRAP_TRAJ, 현재 위치 기준 +1, 0, +0.5 turn = 출력 +24, 0, +12 deg)"""
    r.move_test([1.0, 0.0, 0.5])


def step_can(r, a):
    """8. CAN 설정"""
    r.set("axis0.config.can_node_id", a.node, note=a.joint_name or "")
    r.call(f"can.set_baud_rate({CAN_BAUD})", "can.set_baud_rate", CAN_BAUD)
    r.set("axis1.config.can_node_id", joints.AXIS1_NODE_ID)
    r.set("axis1.config.can_heartbeat_rate_ms", 0, note="추가: 보드마다 같은 ID 63 heartbeat 방지")
    r.save()
    if not r.dry:
        baud = r.odrv.can.config.baud_rate
        if baud != CAN_BAUD:
            raise RuntimeError(f"CAN baud_rate가 {baud}로 읽힙니다")


def step_passthrough(r, a):
    """9. PASSTHROUGH로 바꾸고 저장·재부팅 후 +-0.2 turn 시험"""
    r.set("axis0.controller.config.input_mode", odrv_usb.INPUT_PASSTHROUGH, "INPUT_MODE_PASSTHROUGH")
    r.save()
    r.reboot()
    r.move_test([0.2, -0.2, 0.0])


def step_final(r, a):
    """10. 설정 후 백업"""
    if r.dry:
        r.show("(설정 후 백업 저장)")
        return
    from odrive.configuration import get_dict
    path = util.data_dir(a.id) / f"xdrive_ID{a.id}_after_setup_{util.stamp()}.json"
    with open(path, "w") as f:
        json.dump(get_dict(r.odrv, False), f, indent=1)
    print(f"    백업: {path}")
    print(f"    다음: python 01_backup_and_check.py --id {a.id} --motor {a.motor} --no-backup 로 대조")


STEPS = [step_backup, step_erase, step_board, step_motor, step_encoder, step_controller,
         step_calibrate, step_move, step_can, step_passthrough, step_final]
CAUTION = {
    1: "저장된 설정이 모두 지워집니다",
    6: "출력축이 앞뒤로 돕니다(엔코더 보정). 암을 떼거나 걸리는 데가 없게 하세요",
    7: "출력축이 +24 deg까지 돕니다",
    9: "출력축이 +-5 deg 돕니다",
}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", help="관절 (예: left_hip_roll). node ID와 모터 종류를 표에서 가져옴")
    ap.add_argument("--motor", choices=sorted(params.MOTORS))
    ap.add_argument("--node", type=int, help="CAN node ID (1-62)")
    ap.add_argument("--id", default=None, help="데이터 폴더 이름 (기본: node ID)")
    ap.add_argument("--undervoltage", type=float, default=UNDERVOLTAGE)
    ap.add_argument("--brake-resistance", type=float, default=0.0,
                    help="보드에 브레이크 저항이 실제로 달려 있을 때만 그 값 [ohm] (수업 설정은 0 = 없음)")
    ap.add_argument("--from-step", type=int, default=0)
    ap.add_argument("--print", dest="dry", action="store_true", help="장치 없이 명령만 출력")
    ap.add_argument("--timeout", type=float, default=15.0)
    a = ap.parse_args()

    a.joint_name = None
    if a.joint:
        a.joint_name, node, motor = joints.find(a.joint)
        a.node = a.node if a.node is not None else node
        a.motor = a.motor or motor
    if a.motor is None or a.node is None:
        ap.error("--joint 또는 --motor 와 --node 가 필요합니다")
    if not 1 <= a.node <= 62:
        ap.error("node ID는 1-62 (63은 axis1용, 0은 미설정 보드 기본값)")
    a.id = a.id or f"{a.node:02d}"

    print(f"설정 대상: node {a.node} {a.joint_name or ''}, 모터 {a.motor} ({params.MOTORS[a.motor]['name']}), "
          f"저전압 차단 {a.undervoltage:g} V")
    if a.dry:
        odrv = None
    else:
        a.sim = False
        odrv = odrv_usb.connect(a)
    r = Runner(odrv, a.dry)

    for i, step in enumerate(STEPS):
        if i < a.from_step:
            continue
        title = step.__doc__.split("\n")[0]
        print(f"\n[{title}]")
        if a.dry:
            step(r, a)
            continue
        if i in CAUTION:
            print(f"    !! {CAUTION[i]}")
        print("    실행할 명령:")
        step(Runner(None, True), a)
        if i == 1:
            if util.ask("    지우려면 erase 라고 입력 (다른 입력은 중단): ") != "erase":
                raise RuntimeError("erase 확인 안 됨. 중단합니다.")
        else:
            util.confirm("    실행할까요?")
        print("    실행:")
        try:
            step(r, a)
        except KeyboardInterrupt:
            try:
                r.odrv.axis0.requested_state = odrv_usb.STATE_IDLE
            finally:
                raise
    if not a.dry:
        print("\n설정 완료. 이제 node "
              f"{a.node}로 CAN 확인: python 06_can_check.py scan  (비트레이트 {CAN_BAUD})")


if __name__ == "__main__":
    util.run(main)
