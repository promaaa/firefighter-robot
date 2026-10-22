"""Feetech STS protocol (SCServo) for the STS3215 / Waveshare ST3215 serial bus servo, over pyserial.

Instruction packet: FF FF ID LEN INSTR PARAMS CHECKSUM, with LEN = len(PARAMS) + 2.
Status packet:      FF FF ID LEN ERROR PARAMS CHECKSUM.
CHECKSUM = ~(ID + LEN + INSTR_or_ERROR + sum(PARAMS)) & 0xFF.
Two-byte values are little endian (low byte first) on the STS series. Sources and checks: actuators/README.md.
"""
import time

BAUD = 1_000_000  # default baud rate of the STS3215
BROADCAST = 0xFE
PING, READ, WRITE, SYNC_WRITE = 0x01, 0x02, 0x03, 0x83

# Memory table (decimal addresses). EPROM: written values stay after power off, LOCK must be 0 to write them.
ID, BAUD_RATE, MIN_ANGLE, MAX_ANGLE, OFFSET, MODE = 5, 6, 9, 11, 31, 33
TORQUE_ENABLE, ACC, GOAL_POSITION, GOAL_TIME, GOAL_SPEED, LOCK = 40, 41, 42, 44, 46, 55
PRESENT_POSITION, PRESENT_SPEED, PRESENT_LOAD, PRESENT_VOLTAGE, PRESENT_TEMPERATURE = 56, 58, 60, 62, 63
MOVING, PRESENT_CURRENT = 66, 69

TICKS_PER_REV = 4096  # position steps for one turn


class ServoError(Exception):
    pass


def checksum(body):
    return ~sum(body) & 0xFF


def packet(sid, instr, params=b""):
    body = bytes([sid, len(params) + 2, instr]) + bytes(params)
    return b"\xff\xff" + body + bytes([checksum(body)])


def u16(v):
    return bytes([v & 0xFF, (v >> 8) & 0xFF])


def to_signed(raw, bit):
    """Sign-magnitude value: bit `bit` is the sign (15 for position and speed, 10 for load)."""
    return -(raw & ~(1 << bit)) if raw & (1 << bit) else raw


def from_signed(v, bit=15):
    return (-v) | (1 << bit) if v < 0 else v


def parse_status(data, sid=None):
    """(id, error flags, params) from one status packet. Raises ServoError on a bad packet."""
    i = data.find(b"\xff\xff")
    if i < 0 or len(data) < i + 6:
        raise ServoError(f"no status packet in {data.hex(' ')}")
    pid, n = data[i + 2], data[i + 3]
    body = data[i + 2:i + 4 + n - 1]
    if len(data) < i + 4 + n or len(body) != n + 1:
        raise ServoError(f"short status packet {data.hex(' ')}")
    if checksum(body) != data[i + 3 + n]:
        raise ServoError(f"bad checksum in {data.hex(' ')}")
    if sid is not None and pid != sid:
        raise ServoError(f"reply from servo {pid}, expected {sid}")
    return pid, data[i + 4], bytes(data[i + 5:i + 3 + n])


class Bus:
    """One servo bus. `port` is a pyserial Serial or any object with write(), read(n) and reset_input_buffer()."""

    def __init__(self, port):
        self.port = port
        self.errors = {}  # last error flags per servo id (0 = none)

    @classmethod
    def open(cls, device, baud=BAUD, timeout=0.02):
        import serial  # pyserial
        return cls(serial.Serial(device, baud, timeout=timeout))

    def _send(self, sid, instr, params=b"", reply_len=None):
        """Send one packet. With reply_len, read the status packet and return its params."""
        self.port.reset_input_buffer()
        self.port.write(packet(sid, instr, params))
        if reply_len is None or sid == BROADCAST:
            return None
        want = 6 + reply_len
        data = bytes(self.port.read(want))
        i = data.find(b"\xff\xff")
        if i > 0:  # noise before the header: read the missing bytes
            data = data[i:] + bytes(self.port.read(i))
        if len(data) < want:
            raise ServoError(f"servo {sid}: no reply ({len(data)} of {want} bytes)")
        _, err, params = parse_status(data, sid)
        self.errors[sid] = err
        return params

    def ping(self, sid):
        try:
            self._send(sid, PING, reply_len=0)
            return True
        except ServoError:
            return False

    def read(self, sid, addr, n):
        return self._send(sid, READ, bytes([addr, n]), reply_len=n)

    def write(self, sid, addr, data):
        self._send(sid, WRITE, bytes([addr]) + bytes(data), reply_len=0)

    def torque(self, sid, on):
        self.write(sid, TORQUE_ENABLE, [1 if on else 0])

    def sync_write_goal(self, goals):
        """goals: {id: (position ticks, speed ticks/s, acceleration)}. One broadcast packet, no reply."""
        params = bytearray([ACC, 7])
        for sid, (pos, speed, acc) in goals.items():
            params += bytes([sid, acc]) + u16(from_signed(int(pos))) + u16(0) + u16(int(speed))
        self._send(BROADCAST, SYNC_WRITE, params)

    def state(self, sid):
        """Present position (ticks), speed (ticks/s), load (0.1 %), voltage (V) and temperature (deg C), in one read."""
        p = self.read(sid, PRESENT_POSITION, 8)
        return dict(position=to_signed(p[0] | p[1] << 8, 15), speed=to_signed(p[2] | p[3] << 8, 15),
                    load=to_signed(p[4] | p[5] << 8, 10), voltage=p[6] / 10, temperature=p[7])

    def set_id(self, old, new):
        """Change the id of the only servo on the bus. Unlock the EPROM, write the id, lock again."""
        self.write(old, LOCK, [0])
        self.write(old, ID, [new])
        time.sleep(0.05)
        self.write(new, LOCK, [1])


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="List the servos on the bus, change an id, or read positions with the torque off.")
    ap.add_argument("device", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--set-id", nargs=2, type=int, metavar=("OLD", "NEW"), help="only one servo on the bus")
    ap.add_argument("--torque-off", action="store_true", help="torque off, then print the positions until Ctrl-C")
    a = ap.parse_args()
    bus = Bus.open(a.device)
    if a.set_id:
        bus.set_id(*a.set_id)
    found = [sid for sid in range(1, 11) if bus.ping(sid)]
    for sid in found:
        print(sid, bus.state(sid))
    if a.torque_off:
        for sid in found:
            bus.torque(sid, False)
        while True:
            print("  ".join(f"id {sid}: {bus.state(sid)['position']}" for sid in found))
            time.sleep(0.5)
