"""ODrive "CANSimple" messages of firmware v0.5.1, the base of the course firmware for the XDrive MINI.

Source: github.com/omletkang/xdrive-mini-fw-instant-start, ODrive_S-fw-v0.5.1/Firmware/communication/
can_simple.hpp (command IDs, lines 8-35) and can_simple.cpp (payloads). A frame ID is node_id << 5 | cmd_id.
The ID is a standard 11-bit ID. All payload numbers are little-endian.

This firmware is older than the current ODrive docs:
- It has no set_limits, set_pos_gain or set_vel_gains message. Set the gains and current_lim with odrivetool (USB).
- It sends no cyclic encoder message. Ask for the encoder estimates with a remote frame (RTR) in each cycle.
- The heartbeat holds only the axis error (uint32) and the axis state (uint32). It comes every 100 ms.
"""
import struct
import time

import can

# Command IDs (can_simple.hpp:9-33)
HEARTBEAT = 0x01              # drive -> host, every can_heartbeat_rate_ms (default 100 ms)
ESTOP = 0x02
SET_AXIS_STATE = 0x07
GET_ENCODER_ESTIMATES = 0x09  # remote frame. Reply: pos (turns), vel (turns/s), float32 each
SET_CONTROLLER_MODES = 0x0B
SET_INPUT_POS = 0x0C
SET_INPUT_VEL = 0x0D
SET_INPUT_TORQUE = 0x0E
SET_VEL_LIMIT = 0x0F          # v0.5.1 "set limits": the velocity limit only, no current limit
REBOOT = 0x16
GET_VBUS_VOLTAGE = 0x17       # remote frame. Reply: bus voltage (V), float32
CLEAR_ERRORS = 0x18

# Enum values (odrive-interface.yaml of the same firmware)
IDLE, CLOSED_LOOP = 1, 8                                     # AxisState
TORQUE_CONTROL, VELOCITY_CONTROL, POSITION_CONTROL = 1, 2, 3  # ControlMode
PASSTHROUGH, TRAP_TRAJ = 1, 5                                 # InputMode
AXIS_ERRORS = {0: "INVALID_STATE", 1: "DC_BUS_UNDER_VOLTAGE", 2: "DC_BUS_OVER_VOLTAGE",
               3: "CURRENT_MEASUREMENT_TIMEOUT", 4: "BRAKE_RESISTOR_DISARMED", 5: "MOTOR_DISARMED",
               6: "MOTOR_FAILED", 7: "SENSORLESS_ESTIMATOR_FAILED", 8: "ENCODER_FAILED", 9: "CONTROLLER_FAILED",
               10: "POS_CTRL_DURING_SENSORLESS", 11: "WATCHDOG_TIMER_EXPIRED", 12: "MIN_ENDSTOP_PRESSED",
               13: "MAX_ENDSTOP_PRESSED", 14: "ESTOP_REQUESTED", 17: "HOMING_WITHOUT_ENDSTOP", 18: "OVER_TEMP"}


def error_names(err):
    return " | ".join(AXIS_ERRORS.get(b, f"bit {b}") for b in range(32) if err >> b & 1) or "none"


def frame(node, cmd, data=b"", rtr=False):
    return can.Message(arbitration_id=node << 5 | cmd, data=data, dlc=8 if rtr else len(data),
                       is_extended_id=False, is_remote_frame=rtr)


def _milli(x):  # int16 in units of 0.001 (can_simple.cpp:283-284)
    return max(-32768, min(32767, round(x * 1000)))


def set_axis_state(node, state):
    return frame(node, SET_AXIS_STATE, struct.pack("<I", state))


def set_controller_modes(node, control_mode, input_mode):
    return frame(node, SET_CONTROLLER_MODES, struct.pack("<ii", control_mode, input_mode))


def set_input_pos(node, pos, vel_ff=0.0, torque_ff=0.0):
    """pos in motor turns (float32). vel_ff (turns/s) and torque_ff (Nm at the motor) are int16 x 0.001."""
    return frame(node, SET_INPUT_POS, struct.pack("<fhh", pos, _milli(vel_ff), _milli(torque_ff)))


def set_input_vel(node, vel, torque_ff=0.0):
    return frame(node, SET_INPUT_VEL, struct.pack("<ff", vel, torque_ff))


def set_input_torque(node, torque):
    return frame(node, SET_INPUT_TORQUE, struct.pack("<f", torque))


def set_vel_limit(node, vel_limit):
    return frame(node, SET_VEL_LIMIT, struct.pack("<f", vel_limit))


def estop(node):
    return frame(node, ESTOP)


def clear_errors(node):
    return frame(node, CLEAR_ERRORS)


def reboot(node):
    return frame(node, REBOOT)


def request(node, cmd):
    """Remote frame (RTR): the drive answers with a data frame with the same ID."""
    return frame(node, cmd, rtr=True)


FORMATS = {HEARTBEAT: "<II", GET_ENCODER_ESTIMATES: "<ff", GET_VBUS_VOLTAGE: "<f", SET_AXIS_STATE: "<I",
           SET_CONTROLLER_MODES: "<ii", SET_INPUT_POS: "<fhh", SET_INPUT_VEL: "<ff", SET_INPUT_TORQUE: "<f",
           SET_VEL_LIMIT: "<f"}


def decode(msg):
    """(node, cmd, values) of a frame. values is () for a remote frame or a command without a format here."""
    node, cmd = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
    fmt = FORMATS.get(cmd)
    if msg.is_remote_frame or fmt is None or len(msg.data) < struct.calcsize(fmt):
        return node, cmd, ()
    vals = struct.unpack_from(fmt, msg.data)
    if cmd == SET_INPUT_POS:
        vals = (vals[0], vals[1] / 1000, vals[2] / 1000)
    return node, cmd, vals


class Bus:
    """One CAN bus. self.last keeps the last values of each (node, cmd) with the time they came."""

    def __init__(self, channel, interface="socketcan"):
        self.name = channel
        self.can = can.Bus(channel=channel, interface=interface)
        self.last = {}

    def send(self, msg, tries=20):
        for k in range(tries):
            try:
                return self.can.send(msg)
            except can.CanOperationError:  # the Linux transmit queue is full (txqueuelen): wait 1 ms
                if k == tries - 1:
                    raise
                time.sleep(0.001)

    def _recv(self, timeout):
        msg = self.can.recv(timeout)
        if msg is None:
            return None
        node, cmd, vals = decode(msg)
        if vals:
            self.last[node, cmd] = (time.monotonic(), vals)
        return node, cmd, vals

    def poll(self, seconds=0.0):
        """Read frames for this many seconds. With 0, read only the frames that wait now."""
        end = time.monotonic() + seconds
        while self._recv(max(0.0, end - time.monotonic())) is not None:
            pass

    def ask(self, nodes, cmd, timeout=0.02):
        """Send a remote frame to each node and wait for the replies. Return {node: values}."""
        for n in nodes:
            self.send(request(n, cmd))
        got, end = {}, time.monotonic() + timeout
        while len(got) < len(nodes):
            r = self._recv(max(0.0, end - time.monotonic()))
            if r is None:
                break
            if r[1] == cmd and r[0] in nodes and r[2]:
                got[r[0]] = r[2]
        return got

    def heartbeats(self):
        """{node: (axis error, axis state, age in s)} of all heartbeats read so far."""
        now = time.monotonic()
        return {n: (*v, now - t) for (n, c), (t, v) in self.last.items() if c == HEARTBEAT}

    def close(self):
        self.can.shutdown()
