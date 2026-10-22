"""Optional first-day check for the pan-tilt servos with an external Pico 2 relay board, its e-stop input and the UDP ports.

F2 has no Pico: the pump uses a ROCK 3C GPIO (robot/pump.py). On F2, use robot/bench.py.

python selftest.py                   real parts, asks before each step that moves or switches something
python selftest.py --mock --yes      no hardware, no questions (used by the tests)

Each step prints PASS, FAIL or SKIP, and a one-line hint on FAIL.
"""
import argparse
import importlib.util
import socket
import sys
import time
from pathlib import Path

import st3215

PAN, TILT = 1, 2  # servo ids
VOLT_RANGE = (9.0, 12.6)  # V, ST3215 12 V version and Bus Servo Adapter (A) input
MOVE_DEG, MOVE_DPS, MAX_ERR_DEG = 10.0, 20.0, 2.0  # test move, its speed, allowed position error
UDP_PORTS = [5600, 5601, 5602, 5603, 5604, 5605]
VID_PICO, VID_WCH = 0x2E8A, 0x1A86  # USB vendor ids: Raspberry Pi, WCH (CH343P on the servo adapter)
TICKS_PER_DEG = st3215.TICKS_PER_REV / 360


class PicoLink:
    def __init__(self, device):
        import serial
        self.ser = serial.Serial(device, 115200, timeout=0.2)

    def cmd(self, line):
        """Send one line, return the reply line (OK ... or ERR ...). Other lines (OFF ...) are skipped."""
        self.ser.write((line + "\n").encode())
        for _ in range(5):
            r = self.ser.readline().decode(errors="replace").strip()
            if r.startswith(("OK", "ERR")):
                return r
        return ""

    def lines(self, wait):
        time.sleep(wait)
        return self.ser.read(self.ser.in_waiting).decode(errors="replace").split()

    def press_estop(self, on):  # a person presses the real button
        pass


class MockPico:
    """Runs the logic of pico_fw/main.py on the laptop."""

    def __init__(self):
        spec = importlib.util.spec_from_file_location("pico_fw", Path(__file__).parent / "pico_fw" / "main.py")
        self.fw = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.fw)
        self.state, self.estop, self.last, self.out = {"P": 0, "T": 0, "E": 0}, False, time.monotonic(), []

    def _safety(self):
        r = self.fw.safety(self.state, self.estop, (time.monotonic() - self.last) * 1000)
        if r:
            self.out.append("OFF " + r)

    def cmd(self, line):
        self._safety()
        self.last = time.monotonic()
        return self.fw.handle(self.state, line)

    def lines(self, wait):
        time.sleep(wait)
        self._safety()
        out, self.out = self.out, []
        return " ".join(out).split()

    def press_estop(self, on):
        self.estop = on


class MockBus:
    def __init__(self):
        self.pos = {PAN: 2048, TILT: 2048}

    def ping(self, sid):
        return sid in self.pos

    def state(self, sid):
        return dict(position=self.pos[sid], speed=0, load=0, voltage=12.0, temperature=30)

    def torque(self, sid, on):
        pass

    def sync_write_goal(self, goals):
        for sid, (pos, _, _) in goals.items():
            self.pos[sid] = pos


def find_ports():
    """{"servo": device, "pico": device}, from the USB vendor id and then a probe."""
    from serial.tools import list_ports
    found = {}
    for p in list_ports.comports():
        if p.vid is None:
            continue
        print(f"  {p.device}: {p.description} (USB {p.vid:04x}:{p.pid:04x})")
        try:
            if p.vid == VID_PICO and PicoLink(p.device).cmd("S").startswith("OK"):
                found["pico"] = p.device
            elif p.vid == VID_WCH or st3215.Bus.open(p.device).ping(PAN):
                found.setdefault("servo", p.device)
        except OSError as e:
            print(f"  {p.device}: cannot open ({e})")
    return found


def step_ports(c):
    if c.mock:
        c.bus, c.pico = MockBus(), MockPico()
        return "PASS", "mock servo bus and mock Pico", ""
    found = {} if (c.servo_port and c.pico_port) else find_ports()
    servo, pico = c.servo_port or found.get("servo"), c.pico_port or found.get("pico")
    if not servo or not pico:
        c.abort = True
        return "FAIL", f"servo adapter: {servo}, Pico: {pico}", \
            "Plug both USB cables. Check the adapter jumper at B and that main.py runs on the Pico."
    c.bus, c.pico = st3215.Bus.open(servo), PicoLink(pico)
    return "PASS", f"servo adapter: {servo}, Pico: {pico}", ""


def step_servos(c):
    missing = [sid for sid in (PAN, TILT) if not c.bus.ping(sid)]
    if missing:
        c.abort = True
        return "FAIL", f"no reply from servo id {missing}", "Check the bus cable and the 12 V supply. Set the ids (README, Set the servo ids)."
    states = {sid: c.bus.state(sid) for sid in (PAN, TILT)}
    detail = ", ".join(f"id {sid}: {s['voltage']:.1f} V, {s['temperature']} C" for sid, s in states.items())
    if not all(VOLT_RANGE[0] <= s["voltage"] <= VOLT_RANGE[1] for s in states.values()):
        c.abort = True
        return "FAIL", detail, f"Voltage must be {VOLT_RANGE[0]} to {VOLT_RANGE[1]} V. Check the 12 V converter. The test stops here."
    return "PASS", detail, ""


def step_move(c):
    if not c.ask(f"Move pan and tilt by +-{MOVE_DEG:.0f} deg, slowly. Is the area around the gun clear?"):
        return "SKIP", "not confirmed", ""
    speed = round(MOVE_DPS * TICKS_PER_DEG)
    worst = 0.0
    for sid in (PAN, TILT):
        c.bus.torque(sid, True)
        start = c.bus.state(sid)["position"]
        for goal in (start + MOVE_DEG * TICKS_PER_DEG, start, start - MOVE_DEG * TICKS_PER_DEG, start):
            c.bus.sync_write_goal({sid: (round(goal), speed, 0)})
            time.sleep(0 if c.mock else MOVE_DEG / MOVE_DPS + 0.5)
            worst = max(worst, abs(c.bus.state(sid)["position"] - goal) / TICKS_PER_DEG)
    if worst > MAX_ERR_DEG:
        return "FAIL", f"largest position error {worst:.1f} deg", "Check the load on the servo (gun balance, cables) and the 12 V supply current."
    return "PASS", f"largest position error {worst:.2f} deg", ""


def step_relays(c):
    if not c.ask("Relay test: remove the water, or point the gun at a safe place. Ready?"):
        return "SKIP", "not confirmed", ""
    replies = []
    for ch in ("T", "P"):
        replies += [c.pico.cmd(ch + "1")]
        time.sleep(0.2)
        replies += [c.pico.cmd(ch + "0")]
    expected = ["OK P0 T1 E0", "OK P0 T0 E0", "OK P1 T0 E0", "OK P0 T0 E0"]
    if replies != expected:
        return "FAIL", f"replies {replies}", "Check that the e-stop is released and that main.py runs on the Pico."
    if not c.ask("Did relay 1 and then relay 2 click?"):
        return "FAIL", "Pico replies are correct, but the relays did not click", \
            "Check relay VCC on Pico pin 40, GND, IN1 on pin 19, IN2 on pin 20, and ACTIVE_LOW in main.py."
    return "PASS", "both relays switched on and off", ""


def step_watchdog(c):
    if not c.ask("Watchdog test: both relays switch on for 0.5 s. Water removed or gun safe?"):
        return "SKIP", "not confirmed", ""
    c.pico.cmd("P1")
    c.pico.cmd("T1")
    out = c.pico.lines(0.8)  # no line for 0.8 s: the Pico must switch both off by itself
    status = c.pico.cmd("S")
    if "TIMEOUT" not in out or not status.startswith("OK P0 T0"):
        c.pico.cmd("P0")
        c.pico.cmd("T0")
        return "FAIL", f"Pico said {out}, then {status}", "Flash pico_fw/main.py again. The timeout is TIMEOUT_MS = 500."
    return "PASS", f"both relays OFF after 0.5 s without a line ({status})", ""


def step_estop(c):
    if not c.ask("Press the e-stop now. Done?"):
        return "SKIP", "not confirmed", ""
    c.pico.press_estop(True)
    pressed, refused = c.pico.cmd("S"), c.pico.cmd("P1")
    if not c.ask("Turn the e-stop to release it. Done?"):
        return "SKIP", "not released", ""
    c.pico.press_estop(False)
    released = c.pico.cmd("S")
    if not (pressed.endswith("E1") and refused == "ERR ESTOP" and released.endswith("E0")):
        return "FAIL", f"pressed: {pressed}, P1: {refused}, released: {released}", \
            "Wire one NC contact of the e-stop from GP16 (pin 21) to GND (pin 23)."
    return "PASS", "e-stop input reads open when pressed and closed when released", ""


def step_udp(c):
    bad = []
    for port in UDP_PORTS:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
                rx.bind(("127.0.0.1", port))
                rx.settimeout(0.5)
                tx.sendto(b"selftest", rx.getsockname())
                if rx.recv(64) != b"selftest":
                    bad.append(port)
        except OSError:
            bad.append(port)
    if bad:
        return "FAIL", f"ports {bad} are in use or do not loop back", "Stop the program that uses them: ss -ulpn | grep 560"
    return "PASS", f"UDP loopback works on {len(UDP_PORTS)} ports", ""


STEPS = [("find the serial ports", step_ports), ("servo ids, voltage, temperature", step_servos),
         ("move pan and tilt", step_move), ("click the relays", step_relays), ("Pico watchdog", step_watchdog),
         ("e-stop input", step_estop), ("UDP loopback", step_udp)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--mock", action="store_true", help="no hardware")
    ap.add_argument("--yes", action="store_true", help="answer yes to every question (mock tests only)")
    ap.add_argument("--servo-port")
    ap.add_argument("--pico-port")
    c = ap.parse_args(argv)
    c.abort = False

    def ask(q):
        if c.yes:
            print(f"  {q} yes")
            return True
        return input(f"  {q} [y/N] ").strip().lower().startswith("y")

    c.ask = ask
    results = []
    for i, (name, fn) in enumerate(STEPS, 1):
        if c.abort and fn is not step_udp:
            status, detail, hint = "SKIP", "an earlier step failed", ""
        else:
            print(f"Step {i}: {name}")
            try:
                status, detail, hint = fn(c)
            except (OSError, st3215.ServoError) as e:
                status, detail, hint = "FAIL", str(e), "Check the USB cable and the port name."
        results.append(status)
        print(f"Step {i} {status}: {name}. {detail}" + (f"\n  Hint: {hint}" if hint else ""))
    try:  # leave both relays off
        if getattr(c, "pico", None):
            c.pico.cmd("P0")
            c.pico.cmd("T0")
    except OSError:
        pass
    print(f"Summary: {results.count('PASS')} PASS, {results.count('FAIL')} FAIL, {results.count('SKIP')} SKIP")
    return 1 if "FAIL" in results else 0


if __name__ == "__main__":
    sys.exit(main())
