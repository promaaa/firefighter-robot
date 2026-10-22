"""Walking bridge: aim/run.py walk commands into the Berkeley Humanoid Lite (BHL) Python runtime.

AimGamepad replaces Se2Gamepad (BHL lowlevel, policy/gamepad.py). The gamepad still gives the mode buttons
(A + right bumper: RL mode, A + left bumper: init, X or a stick press: idle). The velocity comes from UDP:
the JSON lines of aim/run.py on 127.0.0.1:PORT, fields vx (m/s) and yaw_rate (rad/s).
legs/run_policy.py --udp uses the same class without the BHL code (the stand-in base class below).
Without a message for TIMEOUT seconds, the velocity command is zero, so the robot stands still.
How to plug it in: actuators/README.md.
"""
import json
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[1] / "robot"))
from robot_config import CONF, HOST  # noqa: E402

PORT = CONF["ports"]["walk_cmd"]
TIMEOUT = CONF["walk"]["timeout_s"]  # s
VX_MAX, YAW_MAX = CONF["walk"]["vx_max"], CONF["walk"]["yaw_max"]  # m/s, rad/s: the command range of the policy training
# Start kick: from a stand, the policy does not start to step at 0.25 m/s (robot/README.md, SIL findings).
# When vx goes from 0 to forward, the bridge sends at least KICK_VX for KICK_S seconds. KICK_S = 0 switches it off.
KICK_VX, KICK_S, MOVING = CONF["walk"]["start_kick_vx"], CONF["walk"]["start_kick_s"], 0.05  # m/s, s, m/s

try:
    from berkeley_humanoid_lite_lowlevel.policy.gamepad import Se2Gamepad as _Base
except ImportError:  # no BHL code: a small stand-in, so the bridge runs and tests without it
    class _Base:
        def __init__(self):
            self.commands = {"velocity_x": 0.0, "velocity_y": 0.0, "velocity_yaw": 0.0, "mode_switch": 0}
            self._stopped = threading.Event()

        def run(self):
            pass

        def stop(self):
            self._stopped.set()

        def _update_command_buffer(self):
            pass


def clip(x, m):
    return min(max(x, -m), m)


class AimGamepad(_Base):
    def __init__(self, *args, port=PORT, **kwargs):
        super().__init__(*args, **kwargs)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, port))
        self.sock.settimeout(0.05)
        self._vel, self._t, self._t_start = (0.0, 0.0), -float("inf"), None

    def run(self):
        super().run()  # the BHL gamepad thread, for the mode buttons
        self.start_udp()

    def start_udp(self):
        threading.Thread(target=self._udp_loop, daemon=True).start()

    def _udp_loop(self):
        while not self._stopped.is_set():
            try:
                m = json.loads(self.sock.recv(65536))
                vx, yaw = float(m["vx"]), float(m["yaw_rate"])
                if vx == vx and yaw == yaw:  # not NaN
                    self._vel, self._t = (clip(vx, VX_MAX), clip(yaw, YAW_MAX)), time.monotonic()
            except (socket.timeout, ValueError, KeyError, TypeError):
                pass
            self._apply()

    def _apply(self):
        now = time.monotonic()
        vx, yaw = self._vel if now - self._t < TIMEOUT else (0.0, 0.0)
        if vx <= MOVING:
            self._t_start = None
        elif self._t_start is None:
            self._t_start = now  # the robot starts to walk forward
        if self._t_start is not None and now - self._t_start < KICK_S:
            vx = max(vx, KICK_VX)
        self.commands.update(velocity_x=vx, velocity_y=0.0, velocity_yaw=yaw)

    def _update_command_buffer(self):  # the gamepad thread: keep its mode switch, replace the stick velocities
        super()._update_command_buffer()
        self._apply()


if __name__ == "__main__":  # print the command that the policy would get
    g = AimGamepad()
    g.start_udp()
    try:
        while True:
            print({k: round(v, 3) for k, v in g.commands.items()})
            time.sleep(0.2)
    except KeyboardInterrupt:
        g.stop()
