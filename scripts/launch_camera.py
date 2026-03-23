#!/usr/bin/env python
"""
Launch ZMQ image servers for UR10 cameras.

Usage:
    python scripts/launch_cameras.py

Starts two ImageServer processes:
    port 5000 → wrist  (D435i, /dev/video0 or index 0)
    port 5001 → top    (D415,  /dev/video2 or index 2)

Find your camera indices first:
    v4l2-ctl --list-devices
    or: python -c "import cv2; [print(i, cv2.VideoCapture(i).isOpened()) for i in range(10)]"
"""

import logging
import multiprocessing
from lerobot.cameras.zmq.image_server import ImageServer

logging.basicConfig(level=logging.INFO)


WRIST_CONFIG = {
    "fps": 30,
    "cameras": {
        "wrist": {
            "device_id": 0,        # ← change to your D435i /dev/video index
            "shape": [480, 848],   # [height, width] — D435i native 848x480
        }
    },
}

TOP_CONFIG = {
    "fps": 30,
    "cameras": {
        "top": {
            "device_id": 2,        # ← change to your D415 /dev/video index
            "shape": [480, 848],   # D415 also supports 848x480
        }
    },
}


def run_server(config, port):
    ImageServer(config, port=port).run()


if __name__ == "__main__":
    p1 = multiprocessing.Process(target=run_server, args=(WRIST_CONFIG, 5000), name="wrist-cam")
    p2 = multiprocessing.Process(target=run_server, args=(TOP_CONFIG,   5001), name="top-cam")

    p1.start()
    p2.start()
    print("Camera servers started. Ctrl+C to stop.")

    try:
        p1.join()
        p2.join()
    except KeyboardInterrupt:
        p1.terminate()
        p2.terminate()
        p1.join()
        p2.join()
        print("Camera servers stopped.")
