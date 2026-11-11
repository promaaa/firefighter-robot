"""Run the ASMR 2026 baseline walking policy in MuJoCo.

    uv run python sim/sim2sim/run.py                 one ASMR model x commands -> results/results.md
    uv run python sim/sim2sim/run.py --noise          the same commands with training observation noise
    uv run python sim/sim2sim/run.py --check          self-check: stand for 5 s
    MUJOCO_GL=egl uv run python sim/sim2sim/run.py --video 'vx 0.3' out.mp4

The robot paths, joint order and standing pose come from
source/robot_models/robots/asmr2026_model.py. The MJCF defines joint limits,
per-joint position actuator gains and effort limits.
"""

import subprocess
import sys
from multiprocessing import Pool
from pathlib import Path

import mujoco
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from source.robot_models.robots import asmr2026_policy as policy_contract  # noqa: E402
from source.robot_models.robots.asmr2026_model import (  # noqa: E402
    LEG_JOINTS,
    MJCF_SCENE_PATH,
    ROBOT_ID,
    STANDING_ROOT_HEIGHT,
)

HERE = Path(__file__).resolve().parent
OUT = HERE / "results"
OUT.mkdir(exist_ok=True)

MODEL = ROBOT_ID
MODEL_PATH = MJCF_SCENE_PATH
POLICY = ROOT / "policies" / "baseline" / "policy.onnx"
JOINTS = list(LEG_JOINTS)
DEFAULT = policy_contract.DEFAULT
DEFAULT_BASE_POSITION = np.array([0.0, 0.0, STANDING_ROOT_HEIGHT], dtype=np.float32)
ACTION_SCALE = policy_contract.ACTION_SCALE
POLICY_DT, PHYSICS_DT = policy_contract.POLICY_DT, 0.0005
TILT_LIMIT = policy_contract.TILT_LIMIT

# MuJoCo model arrays are the source for joint-side gains and limits.
_REFERENCE_MODEL = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
KP = _REFERENCE_MODEL.actuator_gainprm[:12, 0].copy()
KD = -_REFERENCE_MODEL.actuator_biasprm[:12, 2].copy()
EFFORT_LIMITS = _REFERENCE_MODEL.actuator_forcerange[:12, 1].copy()
JOINT_LIMITS = _REFERENCE_MODEL.jnt_range[1:13].copy()
assert np.allclose(KP, policy_contract.KP) and np.allclose(KD, policy_contract.KD)
assert np.allclose(JOINT_LIMITS, policy_contract.JOINT_LIMITS)
assert np.allclose(EFFORT_LIMITS, policy_contract.EFFORT_LIMITS)

# Baseline task payload: 1 kg, centered on the upper torso plate.
PAYLOAD_MASS = 1.0
PAYLOAD_POSITION = (0.0, 0.0, 0.8838)
PAYLOAD_SIZE = (0.10, 0.10, 0.10)

COMMANDS = {
    "stand": (0.0, 0.0, 0.0),
    "vx 0.3": (0.3, 0.0, 0.0),
    "yaw 0.5": (0.0, 0.0, 0.5),
    "vx 0.2 + yaw 0.5": (0.2, 0.0, 0.5),
}
NOISE = np.r_[
    np.zeros(3), np.full(3, 0.3), np.full(3, 0.05),
    np.full(12, 0.05), np.full(12, 2.0), np.zeros(12),
]


def _box_inertia(mass, size):
    sx, sy, sz = size
    return [
        mass / 12 * (sy * sy + sz * sz),
        mass / 12 * (sx * sx + sz * sz),
        mass / 12 * (sx * sx + sy * sy),
    ]


def add_payload(spec):
    """Attach the baseline task's nominal payload to an ASMR MJCF spec."""
    payload = spec.body("base").add_body(name="payload_upper", pos=PAYLOAD_POSITION)
    payload.mass = PAYLOAD_MASS
    payload.ipos = [0.0, 0.0, 0.0]
    payload.inertia = _box_inertia(PAYLOAD_MASS, PAYLOAD_SIZE)
    payload.explicitinertial = True
    payload.add_geom(
        name="payload_upper_visual",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[v / 2 for v in PAYLOAD_SIZE],
        contype=0,
        conaffinity=0,
        mass=0,
        rgba=[0.85, 0.2, 0.1, 1.0],
    )


def configure_torque_actuators(model, torque_limits):
    """Turn the ASMR position actuators into direct torque actuators for drive emulation."""
    limits = np.broadcast_to(np.asarray(torque_limits, dtype=float), (model.nu,))
    model.actuator_gaintype[:] = mujoco.mjtGain.mjGAIN_FIXED
    model.actuator_biastype[:] = mujoco.mjtBias.mjBIAS_NONE
    model.actuator_gainprm[:] = 0.0
    model.actuator_gainprm[:, 0] = 1.0
    model.actuator_biasprm[:] = 0.0
    model.actuator_ctrllimited[:] = 0
    model.actuator_forcelimited[:] = 1
    model.actuator_forcerange[:, 0] = -limits
    model.actuator_forcerange[:, 1] = limits


def load_model(torque_limit=None):
    """Load the ASMR MJCF scene and attach the policy's nominal payload."""
    spec = mujoco.MjSpec.from_file(str(MODEL_PATH))
    add_payload(spec)
    model = spec.compile()
    model.opt.timestep = PHYSICS_DT
    if torque_limit is not None:
        limits = np.minimum(model.actuator_forcerange[:, 1], float(torque_limit))
        model.actuator_forcerange[:, 0] = -limits
        model.actuator_forcerange[:, 1] = limits
    return model


def quat_mul(a, b):
    """Multiply quaternions in [w,x,y,z] order."""
    return np.r_[a[0] * b[0] - np.dot(a[1:], b[1:]),
                 a[0] * b[1:] + b[0] * a[1:] + np.cross(a[1:], b[1:])]


quat_rotate_inverse = policy_contract.quat_rotate_inverse
observation = policy_contract.observation


def _imu_frame_transform(model, data):
    """Return fixed IMU-to-base rotation and inverse quaternion for the MJCF mount."""
    base_rotation = data.xmat[model.body("base").id].reshape(3, 3)
    imu_rotation = data.site("imu").xmat.reshape(3, 3)
    imu_rotation_in_base = base_rotation.T @ imu_rotation
    imu_quat_in_base = np.zeros(4)
    mujoco.mju_mat2Quat(imu_quat_in_base, imu_rotation_in_base.flatten())
    base_quat_in_imu = imu_quat_in_base * np.array([1.0, -1.0, -1.0, -1.0])
    return imu_rotation_in_base, base_quat_in_imu


def simulate(model, command, limit=None, seconds=20.0, frames=None, start="standing", noise=False):
    """Run the ASMR model and return tracking, fall and actuator saturation metrics."""
    if model not in (MODEL, "asmr2026", "asmr2026_humanoid"):
        raise ValueError(f"only {MODEL!r} is configured; got {model!r}")
    if start not in ("standing", "default"):
        raise ValueError("ASMR sim2sim supports start='standing' or 'default'")

    rng = np.random.default_rng(0)
    m = load_model(torque_limit=limit)
    d = mujoco.MjData(m)
    d.qpos[:3] = DEFAULT_BASE_POSITION
    d.qpos[7:19] = DEFAULT
    mujoco.mj_forward(m, d)

    assert [m.joint(j).qposadr[0] for j in JOINTS] == list(range(7, 19)), "joint order differs from the policy"
    assert [m.actuator(i).name for i in range(m.nu)] == JOINTS, "actuator order differs from the policy"
    if not POLICY.exists():
        raise FileNotFoundError(f"baseline network not found: {POLICY}")
    policy = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
    base = m.body("base").id
    imu_rotation_in_base, base_quat_in_imu = _imu_frame_transform(m, d)
    cmd = np.asarray(command, dtype=np.float32)
    action = np.zeros(12, dtype=np.float32)
    substeps = round(POLICY_DT / PHYSICS_DT)
    z0 = d.site("imu").xpos[2]
    effort_limits = m.actuator_forcerange[:, 1].copy()
    vx, wz, sat, peak, fall = [], [], 0, 0.0, None

    for k in range(round(seconds / POLICY_DT)):
        imu_quat = d.sensor("imu_quat").data.copy()
        quat = quat_mul(imu_quat, base_quat_in_imu)
        gyro = imu_rotation_in_base @ d.sensor("imu_gyro").data.copy()
        obs = observation(cmd, quat, gyro, d.qpos[7:19], d.qvel[6:18], action)
        if noise:
            obs += (rng.uniform(-1, 1, 45) * NOISE).astype(np.float32)
        action = policy.run(None, {"obs": obs})[0][0]
        target = np.clip(DEFAULT + ACTION_SCALE * action, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

        for _ in range(substeps):
            d.ctrl[:] = target
            mujoco.mj_step(m, d)
            tau = d.actuator_force.copy()
            sat += np.any(np.abs(tau) >= effort_limits * 0.999)
            peak = max(peak, float(np.abs(tau).max()))
            if frames and d.time * 30 >= frames.n:
                frames(m, d)

        t = d.time
        base_quat = quat_mul(d.sensor("imu_quat").data, base_quat_in_imu)
        tilt = np.arccos(np.clip(-quat_rotate_inverse(base_quat, np.array([0.0, 0.0, -1.0]))[2], -1, 1))
        if fall is None and (tilt > TILT_LIMIT or d.site("imu").xpos[2] < 0.6 * z0):
            fall = t
        if fall is not None and (not frames or t > fall + 2.0):
            break
        if t >= 5.0 and fall is None:
            rotation = d.xmat[base].reshape(3, 3)
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            vel = d.sensor("frame_vel").data
            vx.append(np.cos(yaw) * vel[0] + np.sin(yaw) * vel[1])
            wz.append((rotation @ d.qvel[3:6])[2])

    n = k + 1
    ok = bool(vx) and fall is None
    return dict(
        model=MODEL,
        command=command,
        limit="MJCF" if limit is None else float(limit),
        fall=fall,
        vx=float(np.mean(vx)) if ok else None,
        wz=float(np.mean(wz)) if ok else None,
        sat=100.0 * sat / (n * substeps),
        peak=peak,
    )


def row(result, name):
    outcome = f"falls at {result['fall']:.1f} s" if result["fall"] is not None else "no fall"
    fmt = lambda x: "-" if x is None else f"{x:+.2f}"
    limit = result["limit"] if isinstance(result["limit"], str) else f"{result['limit']:.1f}"
    return (f"| {result['model']} | {limit} | {name} | {outcome} | {fmt(result['vx'])} | {fmt(result['wz'])} | "
            f"{result['sat']:.1f} | {result['peak']:.1f} |")


def run_all(noise=False):
    names = {value: key for key, value in COMMANDS.items()}
    jobs = [(MODEL, COMMANDS[key], None, 20.0, None, "standing", noise) for key in COMMANDS]
    with Pool(min(4, len(jobs))) as pool:
        results = pool.starmap(simulate, jobs)
    lines = [
        "| Model | Effort limit | Command | Result in 20 s | Mean vx, 5-20 s (m/s) | Mean yaw rate (rad/s) | Saturated steps (%) | Peak torque (Nm) |",
        "|---|---:|---|---|---:|---:|---:|---:|",
    ]
    lines.extend(row(result, names[result["command"]]) for result in results)
    report = "\n".join(lines) + "\n"
    (OUT / ("results_noise.md" if noise else "results.md")).write_text(report)
    print(report)


def video(cmd_name, out, seconds=10.0, noise=False):
    """Render one ASMR run at 30 fps; requires ffmpeg and a supported MuJoCo renderer."""
    width, height = 640, 480
    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s",
         f"{width}x{height}", "-r", "30", "-i", "-", "-pix_fmt", "yuv420p", out],
        stdin=subprocess.PIPE,
    )
    state = {}

    def grab(m, d):
        if not state:
            state["renderer"] = mujoco.Renderer(m, height, width)
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            camera.trackbodyid = m.body("base").id
            camera.distance, camera.elevation, camera.azimuth = 2.2, -15.0, 135.0
            state["camera"] = camera
        state["renderer"].update_scene(d, state["camera"])
        ff.stdin.write(state["renderer"].render().tobytes())
        grab.n += 1
    grab.n = 0

    result = simulate(MODEL, COMMANDS[cmd_name], None, seconds, frames=grab, noise=noise)
    ff.stdin.close()
    ff.wait()
    state["renderer"].close()
    print(row(result, cmd_name))


def check():
    """Check the policy observation frame and a short standing rollout."""
    assert np.allclose(quat_rotate_inverse(np.array([1.0, 0, 0, 0]), np.array([0, 0, -1.0])), [0, 0, -1])
    result = simulate(MODEL, (0.0, 0.0, 0.0), seconds=5.0)
    assert result["fall"] is None, f"ASMR robot fell at {result['fall']:.2f} s"
    print(f"check passed: {MODEL} stands 5 s with a zero command")


if __name__ == "__main__":
    if "--check" in sys.argv:
        check()
    elif "--video" in sys.argv:
        index = sys.argv.index("--video")
        video(sys.argv[index + 1], sys.argv[index + 2], noise="--noise" in sys.argv)
    else:
        run_all(noise="--noise" in sys.argv)
