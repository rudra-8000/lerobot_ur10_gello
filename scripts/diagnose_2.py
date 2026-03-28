from lerobot_robot_ur10 import UR10Config
from lerobot_robot_ur10.lerobot_robot_ur10.ur10 import UR10
from lerobot_teleoperator_gello import GelloConfig
from lerobot_teleoperator_gello.lerobot_teleoperator_gello.gello import Gello
from lerobot.processor import make_default_processors
import time
import numpy as np

cfg = UR10Config(ip="192.168.100.3")
robot = UR10(cfg)
robot.connect()

# Warmup
for _ in range(10):
    robot.get_observation()

results = {}

# 1. RTDE joints only
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.rtde_rec.getActualQ()
    times.append(time.perf_counter() - t)
results["rtde_joints"] = times

# 2. Gripper read only
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.gripper.get_pos_normalized()
    times.append(time.perf_counter() - t)
results["gripper_read"] = times

# 3. Cameras only (both)
times = []
for _ in range(200):
    t = time.perf_counter()
    for name, cam in robot.cameras.items():
        cam.async_read()
    times.append(time.perf_counter() - t)
results["cameras_both"] = times

# 4. Full get_observation
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.get_observation()
    times.append(time.perf_counter() - t)
results["full_get_obs"] = times

# 5. send_action latency (send current pose back — no movement)
obs = robot.get_observation()
current_action = {k: obs[k] for k in robot.action_features}
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.send_action(current_action)
    times.append(time.perf_counter() - t)
results["send_action"] = times

robot.disconnect()

# 6. GELLO get_action latency
gello_cfg = GelloConfig(
    port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO528D-if00-port0",
    id="gello_teleop",
)
gello = Gello(gello_cfg)
gello.connect(calibrate=True)  # press Enter to reuse existing calibration

# Warmup
for _ in range(10):
    gello.get_action()

times = []
for _ in range(200):
    t = time.perf_counter()
    gello.get_action()
    times.append(time.perf_counter() - t)
results["gello_get_action"] = times

gello.disconnect()

# 7. Full pipeline: get_obs + get_action + processors + send_action
robot2 = UR10(cfg)
robot2.connect()
gello2 = Gello(gello_cfg)
gello2.connect(calibrate=True)

teleop_proc, robot_proc, obs_proc = make_default_processors()

for _ in range(10):
    obs = robot2.get_observation()
    act = gello2.get_action()

times = []
for _ in range(200):
    t = time.perf_counter()
    obs       = robot2.get_observation()
    obs_p     = obs_proc(obs)
    raw_act   = gello2.get_action()
    tel_act   = teleop_proc((raw_act, obs_p))
    robot_act = robot_proc((tel_act, obs_p))
    robot2.send_action(robot_act)
    times.append(time.perf_counter() - t)
results["full_pipeline"] = times

robot2.disconnect()
gello2.disconnect()

# ── Print results ──────────────────────────────────────────────────────────
print()
print(f"{'component':<25} {'mean':>8} {'p50':>8} {'p95':>8} {'max':>8}")
print("-" * 58)
for k, v in results.items():
    arr = np.array(v) * 1000
    print(f"{k:<25} {arr.mean():>7.1f}ms {np.percentile(arr,50):>7.1f}ms "
          f"{np.percentile(arr,95):>7.1f}ms {arr.max():>7.1f}ms")

print()
target_30hz = 1000 / 30
target_20hz = 1000 / 20
fp = np.array(results["full_pipeline"]) * 1000
print(f"Full pipeline mean: {fp.mean():.1f}ms")
print(f"  30 Hz budget: {target_30hz:.1f}ms  → {'✓ OK' if fp.mean() < target_30hz else '✗ TOO SLOW'}")
print(f"  20 Hz budget: {target_20hz:.1f}ms  → {'✓ OK' if fp.mean() < target_20hz else '✗ TOO SLOW'}")