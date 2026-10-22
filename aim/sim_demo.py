"""Toy simulation: one fire on the floor, a virtual camera, the controller loop, a PNG plot.

python sim_demo.py [F2|F1|A|B|E] [out.png] [--gif]
"""
import math
import sys

import numpy as np

from ballistics import jet_path
from controller import Controller
from detection import FireDetection
from geometry import CONCEPTS, camera_pose, nozzle_exit, project, robot_to_world, world_to_robot

FIRE_RADIUS = 0.15  # m
TURN_VX = 0.2  # m/s. In our simulations, the BHL policy turned fully only from this forward speed


def fake_detections(cfg, cam, pan, tilt, pose, fire, t, rng, noise_px=2.0, p_miss=0.1, p_outlier=0.03):
    """What a fire detector could output: the projected fire centre with pixel noise, misses and outliers."""
    p = world_to_robot(pose, fire)
    uv = project(cfg, cam, pan, tilt, p)
    k = cfg["cameras"][cam]["K"]
    if uv is None or not (0 <= uv[0] < k.width and 0 <= uv[1] < k.height) or rng.random() < p_miss:
        return []
    u, v = uv + rng.normal(0, noise_px, 2)
    if rng.random() < p_outlier:
        u, v = rng.uniform(0, k.width), rng.uniform(0, k.height)
    half = k.fx * FIRE_RADIUS / max(np.linalg.norm(p - camera_pose(cfg, cam, pan, tilt)[1]), 0.1)
    return [FireDetection(t, cam, u, v, (u - half, v - half, u + half, v + half), rng.uniform(0.7, 0.95))]


def landing_point(cfg, pan, tilt, v0, drag, z=0.0):
    """Where the jet crosses the height z, robot frame."""
    e, d = nozzle_exit(cfg, pan, tilt)
    r = jet_path(v0, tilt, drag, z_stop=z - e[2])[-1][0]
    h = d[:2] / np.linalg.norm(d[:2])
    return np.array([e[0] + r * h[0], e[1] + r * h[1], z])


def run(concept="F2", fire=(2.5, 3.0, 0.0), dt=0.1, t_max=60.0, seed=1):
    cfg = CONCEPTS[concept]
    cam = next(iter(cfg["cameras"]))
    rng = np.random.default_rng(seed)
    ctl = Controller(cfg)
    fire = np.array(fire, dtype=float)
    pose, pan, tilt = np.zeros(3), 0.0, 0.0
    health, servo_rate = 1.0, 4.0  # fire health 1 -> 0 after 3 s of water on it; servo speed rad/s
    log = []
    for i in range(int(t_max / dt)):
        t = i * dt
        dets = fake_detections(cfg, cam, pan, tilt, pose, fire, t, rng) if health > 0 else []
        cmd = ctl.step(t, dets, tuple(pose), pan, tilt)
        hit = None
        if cmd.pump_on:
            hit = robot_to_world(pose, landing_point(cfg, pan, tilt, ctl.cc.v0, ctl.cc.drag, fire[2]))
            if np.linalg.norm(hit[:2] - fire[:2]) < FIRE_RADIUS + 0.1:
                health -= dt / 3.0
        est = ctl.filter.x.copy() if ctl.filter.x is not None else None
        log.append(dict(t=t, pose=pose.copy(), pan=pan, tilt=tilt, cmd=cmd, est=est, hit=hit, health=health))
        if cmd.state == "DONE":
            break
        # robot and servo motion
        pan += np.clip(cmd.pan - pan, -servo_rate * dt, servo_rate * dt)
        tilt += np.clip(cmd.tilt - tilt, -servo_rate * dt, servo_rate * dt)
        pose[2] += cmd.yaw_rate * min(1.0, abs(cmd.vx) / TURN_VX) * dt  # no turn on the spot
        pose[0] += cmd.vx * math.cos(pose[2]) * dt
        pose[1] += cmd.vx * math.sin(pose[2]) * dt
    return cfg, ctl, fire, log


def plot(cfg, ctl, fire, log, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    states = ["SEARCH", "APPROACH", "AIM", "SPRAY", "CHECK", "VERIFY", "DONE", "ABORT"]
    fig, (top, side, tl) = plt.subplots(3, 1, figsize=(8, 13), gridspec_kw=dict(height_ratios=[3, 2, 1.2]))

    path = np.array([r["pose"] for r in log])
    top.plot(path[:, 0], path[:, 1], "k-", lw=1, label="robot path")
    # heading: short ticks that start at the robot position (long arrows looked like a robot closer to the fire)
    step = max(len(log) // 15, 1)
    top.quiver(path[::step, 0], path[::step, 1], np.cos(path[::step, 2]), np.sin(path[::step, 2]),
               color="0.5", angles="xy", scale_units="xy", scale=8, width=0.004, label="heading")
    x, y = path[-1, :2]
    top.plot(x, y, "ko", ms=7, label="robot at the end")
    top.plot([x, fire[0]], [y, fire[1]], "k--", lw=0.8)
    top.text((x + fire[0]) / 2, (y + fire[1]) / 2, f"  {math.hypot(fire[0] - x, fire[1] - y):.2f} m", fontsize=9)
    est = np.array([r["est"] for r in log if r["est"] is not None])
    if len(est):
        top.plot(est[:, 0], est[:, 1], ".", ms=3, color="tab:orange", label="filtered fire estimate")
    hits = np.array([r["hit"] for r in log if r["hit"] is not None])
    if len(hits):
        top.plot(hits[:, 0], hits[:, 1], "x", ms=4, color="tab:blue", label="water landing point")
    top.add_patch(plt.Circle(fire[:2], 0.15, color="tab:red", alpha=0.5, label="fire"))
    top.set(aspect="equal", xlabel="x (m)", ylabel="y (m)", title="Top view (odometry frame)")
    top.legend(loc="lower right", fontsize=8)

    # side view: vertical plane through the nozzle at the last aim
    last = next((r for r in reversed(log) if r["cmd"].pump_on), log[-1])
    pan, tilt = last["pan"], last["tilt"]
    e, _ = nozzle_exit(cfg, pan, tilt)
    jp = np.array(jet_path(ctl.cc.v0, tilt, ctl.cc.drag, z_stop=fire[2] - e[2]))
    re = math.hypot(e[0], e[1])
    side.plot(re + jp[:, 0], e[2] + jp[:, 1], color="tab:blue", label=f"jet, v0 = {ctl.cc.v0} m/s, tilt = {math.degrees(tilt):.1f} deg")
    side.plot([0, 0], [0, cfg["pivot"][2]], color="0.3", lw=6, label="robot (origin)")
    side.plot([0, re], [cfg["pivot"][2], e[2]], color="k", lw=2, label="pivot to nozzle exit")
    f = world_to_robot(last["pose"], fire)
    side.plot(math.hypot(f[0], f[1]), fire[2], "^", ms=12, color="tab:red", label="fire")
    side.axhline(0, color="0.6", lw=1)
    side.set(aspect="equal", xlabel="horizontal distance from the robot origin (m)", ylabel="z (m)", title="Side view at the last spray")
    side.legend(fontsize=8)

    t = [r["t"] for r in log]
    tl.step(t, [states.index(r["cmd"].state) for r in log], where="post", label="state")
    tl.fill_between(t, 0, [3 * r["cmd"].pump_on for r in log], step="post", alpha=0.3, color="tab:blue", label="pump on")
    tl.set(yticks=range(len(states)), yticklabels=states, xlabel="time (s)", title="Controller state")
    tl.legend(fontsize=8, loc="upper left")
    fig.tight_layout()
    fig.savefig(out, dpi=110)


def animate(cfg, ctl, fire, log, out, every=3):
    """Animated GIF: top view and jet side view."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    fig, (top, side) = plt.subplots(1, 2, figsize=(8, 3.6), gridspec_kw=dict(width_ratios=[1, 1.4]))
    path = np.array([r["pose"] for r in log])
    lim = (min(path[:, 0].min(), fire[0]) - 0.4, max(path[:, 0].max(), fire[0]) + 0.4,
           min(path[:, 1].min(), fire[1]) - 0.4, max(path[:, 1].max(), fire[1]) + 0.4)

    def draw(i):
        r = log[i]
        x, y, yaw = r["pose"]
        top.clear()
        side.clear()
        top.plot(path[: i + 1, 0], path[: i + 1, 1], "k-", lw=1)
        top.plot(x, y, "ko", ms=6)
        top.plot([x, x + 0.3 * math.cos(yaw)], [y, y + 0.3 * math.sin(yaw)], "k-", lw=2)
        top.add_patch(plt.Circle(fire[:2], 0.15, color="tab:red", alpha=0.15 + 0.6 * max(r["health"], 0)))
        if r["hit"] is not None:
            top.plot([x, r["hit"][0]], [y, r["hit"][1]], color="tab:blue", lw=2)
        top.set(xlim=lim[:2], ylim=lim[2:], aspect="equal", title=f"t = {r['t']:.1f} s   {r['cmd'].state}")
        e, _ = nozzle_exit(cfg, r["pan"], r["tilt"])
        re_ = math.hypot(e[0], e[1])
        side.plot([0, 0], [0, cfg["pivot"][2]], color="0.3", lw=5)
        side.plot([0, re_], [cfg["pivot"][2], e[2]], "k-", lw=2)
        if r["cmd"].pump_on:
            jp = np.array(jet_path(ctl.cc.v0, r["tilt"], ctl.cc.drag, z_stop=fire[2] - e[2]))
            side.plot(re_ + jp[:, 0], e[2] + jp[:, 1], color="tab:blue", lw=2)
        f = world_to_robot(r["pose"], fire)
        if r["health"] > 0:
            side.plot(math.hypot(f[0], f[1]), fire[2], "^", ms=10, color="tab:red")
        side.axhline(0, color="0.6", lw=1)
        side.set(xlim=(-0.2, 4.5), ylim=(-0.1, 1.2), aspect="equal", title="side view, jet when the pump is on", xlabel="m")

    draw(0)
    fig.tight_layout()
    frames = list(range(0, len(log), every)) + [len(log) - 1]
    FuncAnimation(fig, draw, frames=frames).save(out, writer=PillowWriter(fps=10 / every * 2), dpi=90)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--gif"]
    concept = args[0] if args else "F2"
    out = args[1] if len(args) > 1 else f"sim_demo_{concept}.png"
    cfg, ctl, fire, log = run(concept)
    r = log[-1]
    print(f"concept {concept}: final state {r['cmd'].state} at t = {r['t']:.1f} s, fire health {max(r['health'], 0):.2f}, "
          f"sprays {ctl.sprays}, robot at ({r['pose'][0]:.2f}, {r['pose'][1]:.2f}) m")
    plot(cfg, ctl, fire, log, out)
    print("saved", out)
    if "--gif" in sys.argv:
        animate(cfg, ctl, fire, log, out.rsplit(".", 1)[0] + ".gif")
        print("saved", out.rsplit(".", 1)[0] + ".gif")
