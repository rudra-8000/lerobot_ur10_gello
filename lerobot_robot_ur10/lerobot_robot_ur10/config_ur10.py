# lerobotur5egello/lerobotrobotur5e/lerobotrobotur5e/config_ur10.py

"""Configuration dataclass for the UR10 CB3 robot with PincOpen gripper.

Cameras:
  - wrist:  Intel RealSense D435i (wrist-mounted), streamed via ZMQ on port 5000
  - top:    Intel RealSense D415  (top-mounted),   streamed via ZMQ on port 5001

ZMQ camera nodes are started by gello_software:
  python experiments/launch_camera_nodes.py
"""

from dataclasses import dataclass, field
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot_camera_zmq import ZMQCameraConfig


@RobotConfig.register_subclass("ur10")
@dataclass
class UR10Config(RobotConfig):
    # UR10 CB3 default IP — change if yours differs
    ip: str = "192.168.100.3"

    # gripper_port: str = "/dev/ttyUSB0"
    gripper_port: str = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4W75-if00-port0"
    gripper_baud: int = 1_000_000
    gripper_dxl_id: int = 0
    gripper_open_angle: float  = 285.0
    gripper_close_angle: float = 166.0
    gripper_default_speed: int = 0.8
    gripper_default_torque: int = 0.5


    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            # D435i wrist camera
            # D435i supports up to 1280x720@30fps RGB, but 640x480 is safer
            # for latency at 30fps during teleoperation
            "wrist": ZMQCameraConfig(
                tcp_address="tcp://127.0.0.1:5000",
                topic="wrist",
                fps=30,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
            ),
            # D415 top/overhead camera
            # D415 is a rolling shutter cam optimised for well-lit static scenes
            "top": ZMQCameraConfig(
                tcp_address="tcp://127.0.0.1:5001",
                topic="top",
                fps=30,
                width=640,
                height=480,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
            ),
        }
    )
