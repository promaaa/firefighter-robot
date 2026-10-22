"""Detection rate against distance: robot camera images of one fire, procedural and real flame sprites, YOLO detector and
baseline colour detector.

    MUJOCO_GL=egl uv run python sim/mission/sprite_check.py          about 5 min, writes results/sprite_check.md and figures

The robot stands in the rooms of missions 1000 to 1007, at 1 to 5 m from one full-strength tray fire (10, 15, 20 or 30 cm).
The pan and tilt point the camera near the fire (random offsets), and each placement gives images at 5 random sprite times.
The images get the mission exposure and pixel noise. A fire counts as detected when a detection box overlaps its true
visible flame box (mission.match, as in the campaign tables), and also with IoU 0.25 or more (the criterion of vision/).
"""
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE)]
import mission  # noqa: E402
import world  # noqa: E402

OUT = HERE / "results"
RUNS = Path.home() / "runs" / "firefighter" / "mission" / "sprites"
SEEDS, TRAYS, DISTS, TIMES = range(1000, 1008), (0.10, 0.15, 0.20, 0.30), np.arange(1.0, 5.01, 0.5), 5
YOLO_CONF, AIM_CONF = 0.28, 0.5  # vision/detector.py threshold, aim/ min_conf


def iou(a, b):
    i = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def placements(spec, m, rng):
    """(distance, robot x, y, yaw, pan, tilt) for each distance, robot inside the room and clear of furniture, or None."""
    f = spec["fires"][0]
    boxes = world.obstacle_boxes(m)
    lim = np.array(spec["room"]) / 2 - 0.4
    out = []
    for r in DISTS:
        for _ in range(200):
            a = rng.uniform(-math.pi, math.pi)
            x, y = f["x"] + r * math.cos(a), f["y"] + r * math.sin(a)
            free = all(not (b[0, 0] - 0.3 < x < b[1, 0] + 0.3 and b[0, 1] - 0.3 < y < b[1, 1] + 0.3) for b in boxes if b[1, 2] > 0.05)
            if abs(x) < lim[0] and abs(y) < lim[1] and free:
                bearing = math.atan2(f["y"] - y, f["x"] - x)
                yaw = bearing + rng.uniform(-0.5, 0.5)
                pan = float(np.clip(bearing - yaw + rng.uniform(-0.35, 0.35), *world.PAN_LIM))
                tilt = float(np.clip(math.atan2(1.25 * f["d"] - 0.8, r) + rng.uniform(-0.15, 0.15), *world.TILT_LIM))
                out.append((float(r), x, y, yaw, pan, tilt))
                break
    return out


def run():
    sys.path.insert(0, str(ROOT / "vision"))
    yolo = mission.load_detector(ROOT / "vision" / "detector.py")
    base = mission.load_detector(mission.BASELINE)
    sprites = {k: world.flame_sprites(k) for k in ("procedural", "real")}
    recs, examples = [], []
    for seed in SEEDS:
        for j, d in enumerate(TRAYS):
            spec = world.random_spec(seed)
            f0 = spec["fires"][0]
            phase = (seed - SEEDS[0]) * len(TRAYS) + j  # real clip phase % 5, mirrored every other round: each clip about as often
            spec["fires"] = [dict(x=f0["x"], y=f0["y"], d=d, phase=phase)]
            m = mujoco.MjModel.from_xml_string(world.build_xml(spec))
            dd = mujoco.MjData(m)
            mujoco.mj_resetDataKeyframe(m, dd, 0)
            ren = mujoco.Renderer(m, world.H, world.W, max_geom=4000)
            opt = mujoco.MjvOption()
            opt.geomgroup[4] = 1  # the ceiling, as the robot camera sees it
            cam = m.camera("rgb").id
            rng = np.random.default_rng([seed, int(100 * d)])
            for r, x, y, yaw, pan, tilt in placements(spec, m, rng):
                dd.qpos[:3] = x, y, -world.DZ
                dd.qpos[3:7] = math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)
                dd.qpos[m.joint("pan").qposadr[0]], dd.qpos[m.joint("tilt").qposadr[0]] = pan, tilt
                mujoco.mj_forward(m, dd)
                ren.update_scene(dd, camera="rgb", scene_option=opt)
                rgb0, depth = world.render_rgbd(ren)
                for k, t in enumerate(rng.uniform(0, 2, TIMES)):
                    noise = rng.normal(0, 2.5, rgb0.shape).astype(np.float32)
                    for kind, spr in sprites.items():
                        img, info = world.composite_fires(rgb0, depth, dd.cam_xpos[cam].copy(), dd.cam_xmat[cam].reshape(3, 3).copy(),
                                                          world.K.fy, spec["fires"], [1.0], float(t), spr)
                        img = np.clip(img.astype(np.float32) * spec["exposure"] + noise, 0, 255).astype(np.uint8)
                        n, box = info[0]
                        rec = dict(seed=seed, d=d, r=r, kind=kind, phase=phase, t=round(float(t), 3), n=n, box=box and [int(v) for v in box])
                        dets = {name: det(img, 0.0) for name, det in (("yolo", yolo), ("base", base))}
                        for name, ds in dets.items():
                            lab = mission.match(ds, info)
                            rec[name] = [(round(x.confidence, 3), int(lb >= 0), round(iou(x.bbox, box), 3) if box else 0.0) for x, lb in zip(ds, lab)]
                        recs.append(rec)
                        if seed - SEEDS[0] == r - 1 and d == 0.20 and k == 0:  # 1 to 5 m, five scenes, so each real clip once
                            examples.append((kind, r, img, box, [x.bbox for x in dets["yolo"]], [x.bbox for x in dets["base"]],
                                             [x.confidence for x in dets["yolo"]]))
            ren.close()
        print(f"seed {seed}: {len(recs)} images", flush=True)
    RUNS.mkdir(parents=True, exist_ok=True)
    (RUNS / "sprite_check.json").write_text(json.dumps(recs))
    return recs, examples


def hit(rec, name, conf=0.0, need_iou=0.0):
    return any(c >= conf and lab and i >= need_iou for c, lab, i in rec[name])


def report(recs, examples):
    vis = [r for r in recs if r["n"] >= 10]
    L = ["Detection rate of one visible full-strength fire (10 flame pixels or more), by distance and tray diameter.",
         "YOLO 0.28: vision/detector.py at its own threshold. YOLO 0.5: only boxes that aim/ uses (min_conf 0.5).",
         "Baseline: sim/sim2sim/baseline_detector.py. Overlap: any overlap with the true flame box (campaign rule). IoU: 0.25 or more.", "",
         "| Flames | Tray (cm) | Distance (m) | Images | True box width (px) | YOLO 0.28, overlap | YOLO 0.28, IoU 0.25 | YOLO 0.5, overlap "
         "| Baseline, overlap |", "|---|---|---|---|---|---|---|---|---|"]
    rows = {}
    for kind in ("procedural", "real"):
        for d in TRAYS:
            for r in DISTS:
                g = [x for x in vis if x["kind"] == kind and x["d"] == d and abs(x["r"] - r) < 1e-6]
                if not g:
                    continue
                rate = lambda name, conf=0.0, iu=0.0: sum(hit(x, name, conf, iu) for x in g) / len(g)
                row = dict(n=len(g), w=float(np.median([x["box"][2] - x["box"][0] for x in g])), yolo=rate("yolo"), yolo_iou=rate("yolo", 0, 0.25),
                           yolo_aim=rate("yolo", AIM_CONF), base=rate("base"))
                rows[kind, d, float(r)] = row
                L.append(f"| {kind} | {100 * d:.0f} | {r:.1f} | {row['n']} | {row['w']:.0f} | {100 * row['yolo']:.0f} % | {100 * row['yolo_iou']:.0f} % "
                         f"| {100 * row['yolo_aim']:.0f} % | {100 * row['base']:.0f} % |")
    # recall by true box width, all distances and trays: the same bins as the recall-by-fire-size table of vision/README.md
    edges = [0, 8, 12, 16, 24, 32, 48, 64, 96, 128, 1000]
    L += ["", "Real flames, YOLO detector at 0.28, by true flame box width (all trays and distances):", "",
          "| Box width (px) | Images | Detected, overlap | Detected, IoU 0.25 | YOLO on real photos, IoU 0.25 (vision/README.md) |", "|---|---|---|---|---|"]
    yolo_ref = [0.154, 0.317, 0.391, 0.577, 0.709, 0.793, 0.820, 0.905, 0.940, 0.961]
    for (lo, hi), ref in zip(zip(edges, edges[1:]), yolo_ref):
        g = [x for x in vis if x["kind"] == "real" and lo <= x["box"][2] - x["box"][0] < hi]
        if g:
            L.append(f"| {lo}-{hi if hi < 1000 else 'more'} | {len(g)} | {100 * np.mean([hit(x, 'yolo') for x in g]):.0f} % "
                     f"| {100 * np.mean([hit(x, 'yolo', 0, 0.25) for x in g]):.0f} % | {100 * ref:.0f} % |")
    L += ["", "False detections (boxes that overlap no flame), per image, all images:", "", "| Flames | YOLO 0.28 | YOLO 0.5 | Baseline |", "|---|---|---|---|"]
    for kind in ("procedural", "real"):
        g = [x for x in recs if x["kind"] == kind]
        fp = lambda name, conf=0.0: sum(1 for x in g for c, lab, _ in x[name] if not lab and c >= conf) / len(g)
        L.append(f"| {kind} | {fp('yolo'):.3f} | {fp('yolo', AIM_CONF):.3f} | {fp('base'):.3f} |")
    names = [p.stem for p in sorted(world.SPRITES.glob("*.webp"))]
    L += ["", "By real clip (phase % 5). Procedural: the same scenes, trays and placements with the procedural flames:", "",
          "| Real clip | Images | YOLO 0.28, real | YOLO 0.28, procedural | YOLO 0.5, real | Baseline, real | Baseline, procedural |",
          "|---|---|---|---|---|---|---|"]
    for c, name in enumerate(names):
        g = {k: [x for x in vis if x["kind"] == k and x["phase"] % len(names) == c] for k in ("real", "procedural")}
        rate = lambda k, name_, conf=0.0: 100 * np.mean([hit(x, name_, conf) for x in g[k]])
        L.append(f"| {name} | {len(g['real'])} | {rate('real', 'yolo'):.0f} % | {rate('procedural', 'yolo'):.0f} % | {rate('real', 'yolo', AIM_CONF):.0f} % "
                 f"| {rate('real', 'base'):.0f} % | {rate('procedural', 'base'):.0f} % |")
    L += ["", f"Images: {len(recs)} ({len(vis)} with a visible flame). Hidden flames (furniture between) are not in the rates."]
    OUT.mkdir(exist_ok=True)
    (OUT / "sprite_check.md").write_text("\n".join(L) + "\n")
    print("\n".join(L))

    sys.path.insert(0, str(ROOT / "sim" / "payload"))
    import report as style
    plt = style.plt
    fig, axs = plt.subplots(1, 4, figsize=(17, 4.2), sharey=True)
    for ax, d in zip(axs, TRAYS):
        for kind, ls in (("real", "-"), ("procedural", ":")):
            xs = [r for r in DISTS if (kind, d, float(r)) in rows]
            for key, col, lab in (("yolo", style.S1, "YOLO, 0.28"), ("yolo_aim", style.S3, "YOLO, 0.5 (aim/)"), ("base", style.S2, "baseline colour")):
                ax.plot(xs, [100 * rows[kind, d, float(r)][key] for r in xs], ls, marker="o", ms=3.5, color=col, label=f"{lab}, {kind}")
        x50 = world.K.fy * d / 16  # 50 % recall at a 16 px box width (vision/README.md, recall by fire size), flame width = tray diameter
        ax.axvline(x50, color=style.INK2, lw=0.8, ls="--")
        ax.text(x50 + 0.05, 3, "vision/ estimate: 50 %", fontsize=7, color=style.INK2, rotation=90, va="bottom")
        ax.set(title=f"tray {100 * d:.0f} cm", xlabel="distance to the fire (m)", ylim=(-2, 102), xlim=(0.8, 5.2))
        ax.grid(color=style.GRID)
    axs[0].set_ylabel("images with the fire detected (%)")
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="lower center", ncol=6, fontsize=8)
    fig.tight_layout(rect=(0, 0.08, 1, 1))
    fig.savefig(OUT / "fig_sprite_check.png", dpi=120)
    plt.close(fig)

    # example crops (200 x 150 px around the fire, 1.6 x, no smoothing): YOLO boxes green with the confidence, baseline orange
    tiles = []
    for kind in ("procedural", "real"):
        row = []
        for kind_, r, img, box, b_yolo, b_base, confs in examples:
            if kind_ != kind:
                continue
            cx, cy = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
            x0, y0 = int(np.clip(cx - 100, 0, world.W - 200)), int(np.clip(cy - 75, 0, world.H - 150))
            im = Image.fromarray(img).crop((x0, y0, x0 + 200, y0 + 150)).resize((320, 240), Image.NEAREST)
            dr = ImageDraw.Draw(im)
            z = lambda b: [1.6 * (b[0] - x0), 1.6 * (b[1] - y0), 1.6 * (b[2] - x0), 1.6 * (b[3] - y0)]
            for b in b_base:
                dr.rectangle(z(b), outline=(255, 140, 0), width=2)
            for b, c in zip(b_yolo, confs):
                dr.rectangle(z(b), outline=(0, 230, 0), width=2)
                dr.text((z(b)[0], max(z(b)[1] - 12, 0)), f"{c:.2f}", fill=(0, 255, 0))
            dr.rectangle([0, 0, 320, 16], fill=(0, 0, 0))
            dr.text((4, 2), f"{kind}, 20 cm tray, {r:.0f} m", fill=(255, 255, 255))
            row.append(np.asarray(im))
        tiles.append(np.concatenate(row, 1))
    Image.fromarray(np.concatenate(tiles, 0)).save(OUT / "fig_sprite_examples.jpg", quality=88)


if __name__ == "__main__":
    report(*run())
