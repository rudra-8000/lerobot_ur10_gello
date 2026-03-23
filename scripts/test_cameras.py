# scripts/test_cameras.py
"""Test RealSense cameras independently of the robot."""
import cv2
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode

cameras = {
    "wrist": RealSenseCamera(RealSenseCameraConfig(
        serial_number_or_name="923322071837",  # D435I
        fps=30,
        width=848,
        height=480,
        color_mode=ColorMode.RGB,
    )),
    "top": RealSenseCamera(RealSenseCameraConfig(
        serial_number_or_name="204322061013",  # D415
        fps=30,
        width=848,
        height=480,
        color_mode=ColorMode.RGB,
    )),
}

for name, cam in cameras.items():
    cam.connect()
    frame = cam.read()
    # Convert RGB → BGR for cv2.imwrite
    cv2.imwrite(f"/mnt/robot/rudra/lerobot_ur10_gello/tmp/cam_{name}.jpg", frame[:, :, ::-1])
    print(f"{name}: shape={frame.shape}, dtype={frame.dtype}")
    cam.disconnect()

print("Done. Open /mnt/robot/rudra/lerobot_ur10_gello/tmp/cam_wrist.jpg and /mnt/robot/rudra/lerobot_ur10_gello/tmp/cam_top.jpg in VS Code.")
