from lerobot_robot_ur10 import UR10Config
from lerobot_robot_ur10.lerobot_robot_ur10.ur10 import UR10
import time, numpy as np

cfg = UR10Config(ip='192.168.100.3')
robot = UR10(cfg)
robot.connect()

# Warmup
for _ in range(10):
    robot.get_observation()

# Benchmark each component separately
results = {}

# 1. RTDE joints only
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.rtde_rec.getActualQ()
    times.append(time.perf_counter() - t)
results['rtde_joints'] = times

# 2. Gripper only
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.gripper.get_pos_normalized()
    times.append(time.perf_counter() - t)
results['gripper'] = times

# 3. Cameras only (both)
times = []
for _ in range(200):
    t = time.perf_counter()
    for name, cam in robot.cameras.items():
        cam.async_read()
    times.append(time.perf_counter() - t)
results['cameras_both'] = times

# 4. Full get_observation
times = []
for _ in range(200):
    t = time.perf_counter()
    robot.get_observation()
    times.append(time.perf_counter() - t)
results['full_get_obs'] = times

robot.disconnect()

print()
for k, v in results.items():
    arr = np.array(v) * 1000
    print(f'{k:20s}  mean={arr.mean():.1f}ms  p50={np.percentile(arr,50):.1f}ms  p95={np.percentile(arr,95):.1f}ms  max={arr.max():.1f}ms')
