# scripts/debug_gello.py
from lerobot_teleoperator_gello import GelloConfig
from lerobot_teleoperator_gello.lerobot_teleoperator_gello.gello import Gello
import time

g = Gello(GelloConfig())
g.connect(calibrate=False)

print("Move GELLO joints one at a time. Watch sign and magnitude. Ctrl+C to stop.")
try:
    while True:
        a = g.get_action()
        vals = " | ".join(f"j{i}={v:+.2f}" for i, v in enumerate(
            [a[k] for k in ["joint_0","joint_1","joint_2","joint_3","joint_4","joint_5"]]
        ))
        print(f"\r{vals}  grip={a['gripper']:.2f}", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    g.disconnect()
