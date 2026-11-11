"""Interactive hardware checks for the ROCK 3C. Run from the repository root with the board venv."""
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot"))

from robot_config import CONF  # noqa: E402


def run_command(*args):
    subprocess.run([sys.executable, *args], cwd=ROOT, check=False)


def _load_legs():
    legs_dir = str(ROOT / "legs")
    if legs_dir not in sys.path:
        sys.path.insert(0, legs_dir)
    import bringup
    import robot as leg_robot

    return bringup, leg_robot


def _detection_value(detection, key):
    return detection.get(key) if isinstance(detection, dict) else getattr(detection, key)


def _camera_watch(pump_test=False):
    import camera_node as camera

    detector_path = camera.detector_file(source="real")
    detect = camera.load_detector(detector_path)
    frames, closer, _world = camera.open_source("real")
    pump = None
    pulse_until = 0.0
    active, on_streak, off_streak = False, 0, 0
    last_report, last_pump_state = 0.0, False
    pulse_s = min(1.0, float(CONF["pump"]["max_spray_s"]))

    try:
        if pump_test:
            from pump import open_pump

            pump = open_pump(log=lambda message: print(message, flush=True))

        print(f"Camera: {detector_path.relative_to(ROOT) if detector_path.is_relative_to(ROOT) else detector_path}")
        print(f"Image: {camera.W}x{camera.H}. Target point (u, v) is in pixels from the top-left; v is the detected box's bottom edge.")
        print("Watching for fire. Press Ctrl+C to stop.")
        for timestamp, rgb in frames:
            detections = detect(rgb, timestamp)
            best = max(detections, key=lambda d: float(_detection_value(d, "confidence")), default=None)
            if best is not None:
                on_streak += 1
                off_streak = 0
                if not active and on_streak >= 3:
                    active = True
                    print("FIRE DETECTED")
                    if pump_test:
                        pulse_until = time.monotonic() + pulse_s
                        print(f"Pump trigger: one {pulse_s:g} s pulse")
            else:
                off_streak += 1
                on_streak = 0
                if active and off_streak >= 3:
                    active = False
                    pulse_until = 0.0
                    print("FIRE NOT DETECTED")

            now = time.monotonic()
            if active and best is not None and now - last_report >= 0.5:
                u = float(_detection_value(best, "u"))
                v = float(_detection_value(best, "v"))
                confidence = float(_detection_value(best, "confidence"))
                bbox = tuple(round(float(x), 1) for x in _detection_value(best, "bbox"))
                print(f"  target point (u={u:.1f}, v={v:.1f}) px / {camera.W}x{camera.H}, "
                      f"confidence={confidence:.2f}, bbox={bbox}")
                last_report = now

            if pump is not None:
                pump.set(active and now < pulse_until)
                if pump.on != last_pump_state:
                    print("Pump output: ON" if pump.on else "Pump output: OFF")
                    last_pump_state = pump.on
    except KeyboardInterrupt:
        print("Camera test stopped.")
    finally:
        if pump is not None:
            pump.close()
        if closer is not None:
            closer.close()


def _camera_test():
    try:
        _camera_watch()
    except (OSError, RuntimeError, SystemExit, ImportError) as e:
        print(f"Camera test could not start: {e}")


def _pump_menu():
    while True:
        print("\nPump test")
        print("  1) Direct timed spray (uses the existing guided jet test)")
        print("  2) Camera detection trigger (one short pulse per detection event)")
        print("  q) Back")
        choice = input("Select [1/2/q]: ").strip().lower()
        if choice in ("q", "quit", "exit"):
            return
        if choice == "1":
            seconds = input("Spray duration in seconds [1]: ").strip() or "1"
            try:
                duration = float(seconds)
            except ValueError:
                print("Enter a number between 0 and the configured maximum spray time.")
                continue
            if not math.isfinite(duration) or not 0 < duration <= CONF["pump"]["max_spray_s"]:
                print(f"Duration must be greater than 0 and no more than {CONF['pump']['max_spray_s']:g} s.")
                continue
            height = input("Measured nozzle height in metres (Enter to enter it in the guided test): ").strip()
            command = ["robot/bench.py", "pump", "--seconds", str(duration), "--runs", "1"]
            if height:
                try:
                    if not math.isfinite(float(height)) or float(height) <= 0:
                        raise ValueError
                except ValueError:
                    print("Height must be a positive number in metres.")
                    continue
                command.extend(["--height", height])
            print("This test physically sprays water. Aim into a clear collection area and protect electronics.")
            run_command(*command)
        elif choice == "2":
            print("This test sprays one short pulse after fire is detected in 3 consecutive frames.")
            print("Keep the nozzle aimed into a clear collection area. Keep people and electronics out of the spray area.")
            if input('Type SPRAY to arm the camera-triggered pump test: ').strip() != "SPRAY":
                print("Pump test cancelled.")
                continue
            try:
                _camera_watch(pump_test=True)
            except (OSError, RuntimeError, SystemExit, ImportError) as e:
                print(f"Camera-triggered pump test stopped: {e}")
        else:
            print("Choose 1, 2 or q.")


def _motor_test():
    bringup, rb = _load_legs()
    print("\nJoint selection (menu row is 1-12; actual CAN node IDs are shown separately):")
    for row, (name, node, motor, *_rest) in enumerate(rb.JOINTS, 1):
        print(f"  {row:2d}) {name:20s} CAN node {node:2d}  motor {motor}")
    print("Select a row number, or q to leave the motor test.")

    buses = []
    try:
        buses, _sim = rb.open_buses()
        robot = rb.Robot(buses)
        if missing := robot.missing():
            print("Drives not found:", ", ".join(missing))
        while True:
            value = input("Joint row [1-12/q]: ").strip().lower()
            if value in ("q", "quit", "exit"):
                return
            if not value.isdigit() or not 1 <= int(value) <= len(rb.JOINTS):
                print("Enter a row from 1 to 12, or q.")
                continue
            i = int(value) - 1
            name, node = rb.JOINTS[i][0], rb.JOINTS[i][1]
            if i not in robot.idx:
                print(f"{name} (CAN node {node}) is not responding. Check `legs/bringup.py scan`.")
                continue

            print(f"\nSelected {name}, CAN node {node}. Expected positive movement: {bringup.positive(i)}.")
            print("Only this joint is commanded. It moves ±0.1 rad (about ±5.7°) for one cycle.")
            print("The robot must be supported, the limb clear, and the drives configured for the low test torque.")
            if input("Type yes when ready to move: ").strip().lower() != "yes":
                continue

            print("Select cycle period: 1) slow 4 s  2) medium 2 s  3) fast 1 s")
            speed = input("Speed [1-3, default 2]: ").strip() or "2"
            period = {"1": 4.0, "2": 2.0, "3": 1.0}.get(speed)
            if period is None:
                print("Invalid speed selection; returning to joint selection.")
                continue
            try:
                bringup.wiggle(robot, name, yes=True, period=period)
            except (ValueError, RuntimeError, OSError) as e:
                print(f"Motor test stopped: {e}")
            result = input("Did it move as expected and return near its start position? [y/N]: ").strip().lower()
            print("Recorded: PASS" if result.startswith("y") else "Recorded: CHECK")
    except (OSError, RuntimeError) as e:
        print(f"Could not start the motor test: {e}")
    finally:
        for bus in buses:
            bus.close()


def main():
    while True:
        print("\nROCK 3C hardware test menu")
        print("  1) Preflight checks")
        print("  2) CAN drive scan")
        print("  3) Individual leg motor test")
        print("  4) Live camera/fire detector test")
        print("  5) Pump test")
        print("  6) Pan/tilt test")
        print("  7) IMU test")
        print("  8) E-stop test instructions")
        print("  q) Exit")
        choice = input("Select a test: ").strip().lower()
        if choice in ("q", "quit", "exit"):
            return
        if choice == "1":
            run_command("robot/bench.py", "preflight")
        elif choice == "2":
            run_command("legs/bringup.py", "scan")
        elif choice == "3":
            _motor_test()
        elif choice == "4":
            _camera_test()
        elif choice == "5":
            _pump_menu()
        elif choice == "6":
            run_command("robot/bench.py", "pantilt")
        elif choice == "7":
            run_command("robot/bench.py", "imu")
        elif choice == "8":
            print("Follow the e-stop procedure in robot/README.md, section 11. It requires an assembled robot and a clear area.")
        else:
            print("Choose 1-8 or q.")


if __name__ == "__main__":
    try:
        main()
    except (EOFError, KeyboardInterrupt):
        print("\nExiting hardware test menu.")
