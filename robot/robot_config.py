"""Load robot/config.toml, the one config file of the robot runtime.

Every node reads its ports, devices, pins, ids and limits here, so a change in config.toml reaches all of them.

ROBOT_CONFIG=path        use another config file. launch.py --config passes it to every node.
ROBOT_PORT_OFFSET=n      add n to every UDP port, so a test can run next to a running robot pipeline.

    python robot/robot_config.py                 print the whole config (with the offset applied)
    python robot/robot_config.py can.bitrate     print one value (the install script uses this)
"""
import json
import os
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # repository root: relative paths in config.toml start here
PATH = Path(os.environ.get("ROBOT_CONFIG") or Path(__file__).with_name("config.toml"))


def load(path=PATH):
    with open(path, "rb") as f:
        conf = tomllib.load(f)
    offset = int(os.environ.get("ROBOT_PORT_OFFSET", "0"))
    conf["ports"] = {k: v + offset for k, v in conf["ports"].items()}
    return conf


def path(p):
    """A path from config.toml: relative paths start at the repository root."""
    return Path(p) if Path(p).is_absolute() else ROOT / p


CONF = load()
HOST = "127.0.0.1"  # all nodes run on the robot computer and talk over UDP on localhost


def get(key, conf=CONF):
    for k in key.split("."):
        conf = conf[k]
    return conf


if __name__ == "__main__":
    v = get(sys.argv[1]) if len(sys.argv) > 1 else CONF
    print(v if isinstance(v, (str, int, float)) else json.dumps(v, indent=1))
