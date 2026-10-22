"""Step-by-step checks for the first day with the legs. Each step is one subcommand. Add --fake to try it without hardware.

    uv run python legs/bringup.py [--fake] STEP

scan                 list the drives on each bus (from their heartbeats) and test 50 encoder requests
vbus                 read the bus voltage of each drive
settings             print the odrivetool commands for each drive (--torque-limit, default 2 Nm at the joint)
angles               show the joint angles live, motors off. Ctrl+C stops.
calibrate            one time: move each joint by hand to its calibration stop. Saves legs/calibration.json.
check                legs hanging straight, then move each joint to its calibration stop. Finds a wrong zero.
wiggle JOINT         move one joint (name or node id) +-0.1 rad, after you confirm
hold                 ramp to the default pose in 2 s and hold it. Ctrl+C: damping, then idle.
"""
import argparse
import sys

import numpy as np

import odrive_can as oc
import robot as rb

# What you see when a joint angle grows (MuJoCo model, from the zero pose). The same for both legs.
POSITIVE = {"hip_roll": "the foot swings to the robot's left",
            "hip_yaw": "the foot swings to the robot's left and the toes turn right (the axis is tilted)",
            "hip_pitch": "the foot swings backward", "knee_pitch": "the knee bends",
            "ankle_pitch": "the toes go down", "ankle_roll": "the left edge of the sole goes up"}
# Where to push each joint for `calibrate` and `check` (the "stop" column of JOINTS). The same for both legs.
STOP_WORDS = {"hip_roll": "foot inward, toward the other leg", "hip_yaw": "toes inward, foot swings out",
              "hip_pitch": "leg backward", "knee_pitch": "knee fully straight", "ankle_pitch": "toes up",
              "ankle_roll": "outer edge of the sole down"}
NODE_NAME = {j[1]: j[0] for j in rb.JOINTS}


def positive(i):
    return POSITIVE[rb.NAMES[i].split("_", 1)[1]]


def stop_words(i):
    return STOP_WORDS[rb.NAMES[i].split("_", 1)[1]]


def scan(buses):
    for b in buses:
        b.poll(0.5)
        hb = b.heartbeats()
        print(f"{b.name}: {len(hb)} drive(s)")
        for n, (err, state, age) in sorted(hb.items()):
            print(f"  node {n:2d}  {NODE_NAME.get(n, 'NOT IN THE JOINT MAP'):18s} state {state}  error {oc.error_names(err)}")
        nodes = [n for n in hb if n in NODE_NAME]
        if nodes:
            got = sum(len(b.ask(nodes, oc.GET_ENCODER_ESTIMATES)) for _ in range(50))
            print(f"  encoder replies: {got} of {50 * len(nodes)}")


def settings(torque_limit):
    print("# odrivetool commands for each drive. Connect one drive by USB, run odrivetool, paste its block.")
    for name, node, motor, *_ in rb.JOINTS:
        s = rb.drive_settings(motor, torque_limit)
        print(f"\n# {name}: node {node}, motor {motor}, joint torque limit {torque_limit:g} Nm")
        print(f"odrv0.axis0.config.can_node_id = {node}\n"
              "odrv0.axis1.config.can_node_id = 63\n"
              "odrv0.axis1.config.can_heartbeat_rate_ms = 0\n"
              "odrv0.can.set_baud_rate(500000)\n"
              "odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL\n"
              "odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH\n"
              f"odrv0.axis0.controller.config.pos_gain = {s['pos_gain']:.4g}\n"
              f"odrv0.axis0.controller.config.vel_gain = {s['vel_gain']:.4g}\n"
              "odrv0.axis0.controller.config.vel_integrator_gain = 0\n"
              f"odrv0.axis0.motor.config.current_lim = {s['current_lim']:.3g}\n"
              "odrv0.axis0.config.watchdog_timeout = 1.0\n"
              "odrv0.axis0.config.enable_watchdog = True\n"
              "odrv0.save_configuration()\n"
              "odrv0.reboot()")


def angles(robot):
    calibrated = not np.isnan(robot.enc_ref[robot.idx]).any()
    if calibrated:
        robot.zero()
    print("Joint angles (deg), motors off. Ctrl+C stops." if calibrated else
          "No calibration yet: the angles count from the pose at the start (deg). Ctrl+C stops.")
    if not calibrated:
        robot.offset = robot.turns()
    short = {"left_": "L ", "right_": "R ", "ankle": "ank", "knee_pitch": "knee"}
    labels = [rb.NAMES[i] for i in robot.idx]
    for a, b in short.items():
        labels = [s.replace(a, b) for s in labels]
    print(" ".join(f"{s:>10s}" for s in labels))
    for _ in rb.ticks(5):
        q = robot.to_joint(robot.turns())
        print(" ".join(f"{np.degrees(q[i]):+10.1f}" for i in robot.idx))


def stop_readings(robot):
    """Motor turns of each joint at its calibration stop, while you move the joints by hand."""
    lo_side = np.array([j[6] == "lo" for j in rb.JOINTS])
    start = best = robot.turns()
    print("Move each joint by hand to this stop. Press Ctrl+C when all joints were there.")
    for i in robot.idx:
        print(f"  {rb.NAMES[i]:18s} {stop_words(i):34s} {np.degrees(rb.STOP[i]):+7.2f} deg")
    try:
        for _ in rb.ticks(20):
            p = robot.turns()
            s, sb = robot.sign * p, robot.sign * best
            best = np.where(np.where(lo_side, s < sb, s > sb), p, best)
    except KeyboardInterrupt:
        pass
    moved = np.degrees(np.abs(best - start) / rb.TURNS_PER_RAD)
    for i in robot.idx:
        if moved[i] < 3:
            print(f"  {rb.NAMES[i]}: moved only {moved[i]:.1f} deg. Was it at its stop?")
    return best


def calibrate(robot):
    p = stop_readings(robot)
    robot.calibrate(p, rb.STOP)
    print(f"Saved {robot.calib}. Now run: bringup.py check")


def check(robot):
    robot.zero()
    q = robot.to_joint(stop_readings(robot))
    bad, err = rb.wrong_zero(q), np.degrees(q - rb.STOP)
    print(f"{'joint':18s} {'stop error (deg)':>16s}")
    for i in robot.idx:
        print(f"{rb.NAMES[i]:18s} {err[i]:+16.1f}" + ("   WRONG ZERO: was the leg straight at the start?" if bad[i] else ""))


def wiggle(robot, joint, yes):
    i = rb.NAMES.index(joint) if joint in rb.NAMES else rb.NAMES.index(NODE_NAME[int(joint)])
    print(f"CAUTION: {rb.NAMES[i]} moves +-0.1 rad (+-6 deg) for 2 s. First it moves +: {positive(i)}.")
    print("Before: set the low limit in each drive (bringup.py settings, --torque-limit 2). Keep hands off the leg.")
    if not yes and input("Type yes to move: ").strip() != "yes":
        return
    if np.isnan(robot.enc_ref[i]):
        robot.offset = robot.turns()  # no calibration: the angle counts from the current pose
    else:
        robot.zero()
    q0, _ = robot.read()
    robot.enable([i])
    worst = 0.0
    try:
        for k in rb.ticks(rb.RATE):
            target = q0.copy()
            target[i] += 0.1 * np.sin(np.pi * k / rb.RATE)  # one period in 2 s
            q, _ = robot.read()
            if f := robot.faults():
                print("Stop:", "; ".join(f))
                break
            worst = max(worst, abs(q[i] - np.clip(target[i], robot.lo[i], robot.hi[i])))
            robot.command(target, [i])
            if k >= 2.5 * rb.RATE:
                break
    finally:
        robot.relax(1.0)
    print(f"Largest tracking error: {np.degrees(worst):.1f} deg (K = 20 Nm/rad, so 1 deg = 0.35 Nm)")


def hold(robot, yes):
    print("CAUTION: all 12 joints move to the default pose in 2 s. The robot must hang from the gantry.")
    if not yes and input("Type yes to move: ").strip() != "yes":
        return
    robot.zero()
    robot.enable()
    try:
        f = robot.move_to(rb.DEFAULT, 2.0, until=lambda: False)  # holds until Ctrl+C or a fault
        print("Stop:", "; ".join(f))
    except KeyboardInterrupt:
        print("Stop: Ctrl+C")
    finally:
        robot.relax()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["scan", "vbus", "settings", "angles", "calibrate", "check", "wiggle", "hold"])
    ap.add_argument("joint", nargs="?", help="for wiggle: joint name or node id")
    ap.add_argument("--fake", action="store_true", help="fake drives on a MuJoCo biped")
    ap.add_argument("--torque-limit", type=float, default=2.0, help="for settings: joint torque limit (Nm)")
    ap.add_argument("--yes", action="store_true", help="do not ask before a move")
    a = ap.parse_args()
    if a.step == "settings":
        return settings(a.torque_limit)
    buses, sim = rb.open_buses(a.fake)
    try:
        if a.step == "scan":
            return scan(buses)
        robot = rb.Robot(buses, calib=None if sim else rb.CALIB)
        if missing := robot.missing():
            print("Not found (the steps use the other drives):", ", ".join(missing))
        if sim:
            robot.calibrate(robot.turns(), rb.ZERO_POSE)  # the fake starts exactly in ZERO_POSE, like a jig
        if a.step == "vbus":
            for name, v in robot.vbus().items():
                print(f"{name:18s} {v:5.1f} V" + ("  LOW" if v < 22.0 else ""))
        elif a.step == "angles":
            angles(robot)
        elif a.step in ("calibrate", "check") and sim:
            print("Fake: you cannot move the fake joints by hand. The fake is calibrated in ZERO_POSE.")
        elif a.step == "calibrate":
            calibrate(robot)
        elif a.step == "check":
            check(robot)
        elif a.step == "wiggle":
            if not a.joint:
                sys.exit("wiggle needs a joint name or a node id")
            wiggle(robot, a.joint, a.yes)
        elif a.step == "hold":
            hold(robot, a.yes)
    except KeyboardInterrupt:
        pass
    finally:
        for b in buses:
            b.close()
        if sim:
            sim.close()


if __name__ == "__main__":
    main()
