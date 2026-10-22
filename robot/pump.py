"""Water pump on/off on one GPIO line, with two watchdogs and an optional e-stop input.

F2: header pin 11 drives an opto-isolator, which switches the DMC-101 controller of the KLP02 pump (config.toml [pump]).

The actuator node calls pump.set(on) at each tick. The pump switches off:
- when it has been on for max_spray_s: it stays off until set(False) arrives (the spray must end first),
- when set() was not called for keepalive_s (a hung actuator loop): a thread checks this every 50 ms,
- while the e-stop input reads "pressed",
- on close(), and on any exit of the node (the node calls close()).

GPIO uses libgpiod v2 (pip package gpiod, the "board" extra). Without it, use FileLine or MemoryLine.
FileLine keeps the pin value in a file, so the fake camera (--sim) and launch.py see the same "pin".

    python robot/pump.py --seconds 1            switch the real pump on for 1 s (config.toml [pump])
    python robot/pump.py --off                  force the pump line low (launch.py does this at the end)
"""
import argparse
import threading
import time
from pathlib import Path

from robot_config import CONF


class MemoryLine:
    def __init__(self):
        self.value = 0

    def set(self, v):
        self.value = int(v)

    def get(self):
        return self.value

    def release(self):
        pass


class FileLine(MemoryLine):
    """A mock pin shared between processes: the value is "0" or "1" in a file."""

    def __init__(self, path):
        self.path = Path(path)
        self.set(0)

    def set(self, v):
        self.path.write_text(str(int(bool(v))))

    def get(self):
        try:
            return int(self.path.read_text().strip() or 0)
        except FileNotFoundError:
            return 0


class GpiodLine:
    """One output (or input with pull-up) line of a gpiochip, with libgpiod v2."""

    def __init__(self, chip, line, output=True, active_low=False, consumer="robot-pump"):
        import gpiod  # board only: uv sync --extra board
        from gpiod.line import Bias, Direction, Value
        self.Value, self.line = Value, line
        s = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE, active_low=active_low) if output \
            else gpiod.LineSettings(direction=Direction.INPUT, bias=Bias.PULL_UP)
        self.req = gpiod.request_lines(chip, consumer=consumer, config={line: s})

    def set(self, v):
        self.req.set_value(self.line, self.Value.ACTIVE if v else self.Value.INACTIVE)

    def get(self):
        return int(self.req.get_value(self.line) == self.Value.ACTIVE)

    def release(self):
        self.req.release()


class Pump:
    def __init__(self, line, max_spray_s=CONF["pump"]["max_spray_s"], keepalive_s=CONF["pump"]["keepalive_s"], estop=None, log=print):
        self.line, self.estop_line, self.log = line, estop, log
        self.max_spray_s, self.keepalive_s = max_spray_s, keepalive_s
        self.on, self.t_on, self.t_set, self.latched = False, 0.0, time.monotonic(), False
        self.lock = threading.Lock()
        self.line.set(0)
        self.running = True
        threading.Thread(target=self._watch, daemon=True).start()

    @property
    def estop(self):
        """True while the e-stop input reads open (pressed or a broken wire)."""
        return bool(self.estop_line and self.estop_line.get())

    def _switch(self, on, why=""):
        if on != self.on:
            self.line.set(on)
            self.on, self.t_on = on, time.monotonic()
            if why:
                self.log(f"pump: off, {why}")

    def set(self, on):
        """on: the wanted state. Call it at each tick: the keepalive watchdog needs the calls."""
        with self.lock:
            now = self.t_set = time.monotonic()
            if not on:
                self.latched = False
                self._switch(False)
            elif self.estop:
                self._switch(False, "e-stop pressed")
            elif self.latched:
                pass
            elif self.on and now - self.t_on > self.max_spray_s:
                self.latched = True
                self._switch(False, f"on for more than {self.max_spray_s:g} s (max_spray_s)")
            else:
                self._switch(True)

    def _watch(self):
        while self.running:
            time.sleep(0.05)
            with self.lock:
                now = time.monotonic()
                if self.on and now - self.t_set > self.keepalive_s:
                    self._switch(False, f"no update for {self.keepalive_s:g} s (keepalive_s)")
                elif self.on and now - self.t_on > self.max_spray_s:
                    self.latched = True
                    self._switch(False, f"on for more than {self.max_spray_s:g} s (max_spray_s)")

    def close(self):
        self.running = False
        with self.lock:
            self._switch(False)
            self.line.set(0)
            self.line.release()
            if self.estop_line:
                self.estop_line.release()


def open_pump(mock=False, pump_file=None, log=print):
    """The pump of config.toml [pump]: real GPIO, or a mock line (in a file when pump_file is given)."""
    c = CONF["pump"]
    if mock:
        return Pump(FileLine(pump_file) if pump_file else MemoryLine(), log=log)
    estop = GpiodLine(c["estop_chip"], c["estop_line"], output=False, consumer="robot-estop") if c["estop_chip"] else None
    return Pump(GpiodLine(c["chip"], c["line"], active_low=c["active_low"]), estop=estop, log=log)


def force_off(mock=False, pump_file=None):
    """Last step of a safe stop, after the actuator node has exited: drive the pump line low. Returns a message."""
    try:
        if mock:
            if pump_file:
                FileLine(pump_file)  # writes 0
            return "pump line forced low (mock)"
        c = CONF["pump"]
        line = GpiodLine(c["chip"], c["line"], active_low=c["active_low"], consumer="robot-launch")
        line.set(0)
        line.release()
        return "pump line forced low"
    except Exception as e:  # noqa: BLE001  a stop must go on: report it
        return f"could not force the pump line low: {e!r}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seconds", type=float, default=0.0, help="switch the pump on for this time")
    ap.add_argument("--off", action="store_true", help="only force the line low")
    ap.add_argument("--mock", action="store_true")
    a = ap.parse_args()
    if a.off:
        print(force_off(a.mock))
    else:
        p = open_pump(a.mock)
        try:
            end = time.monotonic() + a.seconds
            while time.monotonic() < end:
                p.set(True)
                time.sleep(0.02)
        finally:
            p.close()
        print(f"pump was on for {a.seconds:g} s")
