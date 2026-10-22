"""Relay switch for the water: MicroPython for the Raspberry Pi Pico 2. Copy this file to the Pico as main.py.

Optional, not used on F2 (the F2 pump uses a ROCK 3C GPIO). actuators/selftest.py --mock runs this logic on a laptop.

Lines over USB serial, each ends with a newline:
  P1 / P0  pump relay on / off (relay channel 2)
  T1 / T0  gun trigger relay on / off (relay channel 1)
  H        heartbeat, no change
  S        status
Each line gets one reply line: "OK P0 T0 E0" (outputs and e-stop) or "ERR <reason>".
Safety: both outputs go OFF when no line arrives for 0.5 s, and while the e-stop input is open.
"""
import sys
import time

PIN_TRIGGER = 14  # GP14 -> relay module IN1 (channel 1, water-gun trigger)
PIN_PUMP = 15  # GP15 -> relay module IN2 (channel 2, 12 V pump)
PIN_ESTOP = 16  # GP16 -> one NC contact of the e-stop -> GND. Pressed (or wire broken) = high = stop
ACTIVE_LOW = True  # most 5 V relay modules switch on when IN is low. Check yours and change this if needed
TIMEOUT_MS = 500


def handle(state, line):
    """Apply one command line to state {"P": 0/1, "T": 0/1, "E": 0/1}. Returns the reply line."""
    cmd = line.strip().upper()
    if cmd in ("P1", "P0", "T1", "T0"):
        if cmd[1] == "1" and state["E"]:
            return "ERR ESTOP"
        state[cmd[0]] = int(cmd[1])
    elif cmd not in ("H", "S"):
        return "ERR UNKNOWN " + cmd[:20]
    return "OK P%d T%d E%d" % (state["P"], state["T"], state["E"])


def safety(state, estop, ms_since_line):
    """Force both outputs off on e-stop or line timeout. Returns a reason when it switched something off."""
    state["E"] = 1 if estop else 0
    if (estop or ms_since_line > TIMEOUT_MS) and (state["P"] or state["T"]):
        state["P"] = state["T"] = 0
        return "ESTOP" if estop else "TIMEOUT"
    return None


def main():
    import select
    from machine import WDT, Pin

    def out(pin, on):  # active-low: drive low for on, high impedance (input) for off
        if ACTIVE_LOW:
            Pin(pin, Pin.OUT, value=0) if on else Pin(pin, Pin.IN)
        else:
            Pin(pin, Pin.OUT, value=1 if on else 0)

    estop_pin = Pin(PIN_ESTOP, Pin.IN, Pin.PULL_UP)
    led = Pin("LED", Pin.OUT)
    state = {"P": 0, "T": 0, "E": 0}
    out(PIN_TRIGGER, 0)
    out(PIN_PUMP, 0)
    wdt = WDT(timeout=2000)  # the chip resets if this loop hangs; after a reset the pins are inputs = relays off
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf, last_line = "", time.ticks_ms()
    while True:
        wdt.feed()
        while poll.poll(10):  # read single characters, so a half line never blocks the loop
            c = sys.stdin.read(1)
            if c == "\n":
                last_line = time.ticks_ms()
                print(handle(state, buf))
                buf = ""
            elif c != "\r" and len(buf) < 32:
                buf += c
        reason = safety(state, estop_pin.value() == 1, time.ticks_diff(time.ticks_ms(), last_line))
        if reason:
            print("OFF " + reason)
        out(PIN_PUMP, state["P"])
        out(PIN_TRIGGER, state["T"])
        led.value(state["P"] or state["T"])


if __name__ == "__main__":
    main()
