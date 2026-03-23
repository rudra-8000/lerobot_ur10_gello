# scripts/launch_wrist_cam.py
import logging
from lerobot.cameras.zmq.image_server import ImageServer
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.cameras.configs import ColorMode

logging.basicConfig(level=logging.INFO)

# Monkey-patch the config to include fourcc
# lerobot index 0 → /dev/video10 → D435I YUYV RGB stream
config = {
    "fps": 30,
    "cameras": {
        "wrist": {
            "device_id": 0,       # lerobot enum index 0 = /dev/video10
            "shape": [480, 848],
        }
    },
}
ImageServer(config, port=5000).run()
