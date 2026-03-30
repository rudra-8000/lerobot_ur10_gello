"""Move GELLO to its calibration home position before teleoperation.

The home position is defined by config.calibration_position (radians).
Inverts _process_action() to find the target motor counts, enables torque,
drives each joint there, then disables torque so the arm is freely backdrivable.

Usage:
    python scripts/gello_home.py
    python scripts/gello_home.py --speed 0.3   # slower, safer
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from pathlib import Path

import numpy as np

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
from lerobot_teleoperator_gello import GelloConfig
from lerobot_teleoperator_gello.lerobot_teleoperator_gello.gello import GelloCalibration

logging.basicConfig(level=logging.INFO, format="%(message)s")
log = logging.getLogger(__name__)

RAD_PER_COUNT = 2 * np.pi / (4096 - 1)
JOINT_NAMES   = ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

GELLO_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO528D-if00-port0"


def load_calibration(cfg: GelloConfig) -> GelloCalibration:
    """Load saved calibration file — must exist (run calibrate_gello_teleop.py first)."""
    from lerobot.teleoperators.config import TeleoperatorConfig
    # Reconstruct calibration_fpath the same way Gello does
    calib_dir = Path("~/.cache/huggingface/lerobot/calibration/teleoperators/gello").expanduser()
    fpath = calib_dir / f"{cfg.id}.json"
    if not fpath.is_file():
        raise FileNotFoundError(
            f"No calibration file at {fpath}. Run calibrate_gello_teleop.py first."
        )
    with open(fpath) as f:
        data = json.load(f)
    log.info(f"Calibration loaded from {fpath}")
    return GelloCalibration(**data)


def rad_to_counts(
    target_rad: float,
    motor: str,
    joint_idx: int,
    calibration: GelloCalibration,
    cfg: GelloConfig,
) -> int:
    """
    Inverse of _process_action():
        angle_rad = sign * (counts - offset) * RAD_PER_COUNT + ref_pos_rad
        counts    = offset + (angle_rad - ref_pos_rad) / (sign * RAD_PER_COUNT)
    """
    offset   = calibration.joint_offsets[motor]
    sign     = cfg.joint_signs[joint_idx]
    ref_rad  = cfg.calibration_position[joint_idx]
    counts   = offset + (target_rad - ref_rad) / (sign * RAD_PER_COUNT)
    return int(round(counts))


def move_to_home(speed: float = 0.5) -> None:
    cfg = GelloConfig(port=GELLO_PORT, id="gello_teleop")
    calib = load_calibration(cfg)

    bus = DynamixelMotorsBus(
        port=cfg.port,
        motors={
            "joint_0": Motor(1, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "joint_1": Motor(2, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "joint_2": Motor(3, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "joint_3": Motor(4, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "joint_4": Motor(5, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "joint_5": Motor(6, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(7, "xl330-m077", MotorNormMode.RANGE_0_100),
        },
    )
    bus.connect(handshake=False)
    bus.set_baudrate(cfg.baudrate)
    bus._handshake()
    bus._assert_motors_exist()

    # ── Read current positions ─────────────────────────────────────────────
    current = bus.sync_read("Present_Position", normalize=False)
    log.info("\nCurrent positions (counts):")
    for name, val in current.items():
        log.info(f"  {name}: {val}")

    # ── Compute target counts for each joint ───────────────────────────────
    targets: dict[str, int] = {}
    for idx, motor in enumerate(JOINT_NAMES):
        home_rad = cfg.calibration_position[idx]
        targets[motor] = rad_to_counts(home_rad, motor, idx, calib, cfg)

    # Gripper home = open position
    targets["gripper"] = calib.gripper_open_position

    log.info("\nTarget home positions (counts):")
    for name, val in targets.items():
        log.info(f"  {name}: {val}")

    # ── Profile velocity — scale MAX_PROFILE_VEL by speed arg ─────────────
    # XL330 max velocity index ~200; we write it per-motor
    profile_vel = max(1, int(round(speed * 200)))

    # ── Enable torque and move all joints simultaneously ───────────────────
    log.info(f"\nMoving to home at speed={speed:.2f} (profile_vel={profile_vel})...")

    bus.disable_torque()
    bus.configure_motors()

    # Set extended position mode for arm joints, current-position for gripper
    for motor in JOINT_NAMES:
        bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)
    bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)

    # Set profile velocity for smooth motion
    for motor in bus.motors:
        bus.write("Profile_Velocity", motor, profile_vel)

    bus.enable_torque()

    # Write all goal positions at once
    bus.sync_write("Goal_Position", targets, normalize=False)

    # ── Wait until all joints settle ───────────────────────────────────────
    POSITION_TOL = 20  # counts (~1.8 deg for XL330)
    TIMEOUT_S    = 10.0
    POLL_S       = 0.05

    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        present = bus.sync_read("Present_Position", normalize=False)
        errors  = {m: abs(present[m] - targets[m]) for m in targets}
        max_err = max(errors.values())

        status = "  ".join(f"{m}:{e:4d}" for m, e in errors.items())
        print(f"\r  err(counts) [{status}]  max={max_err:4d}", end="", flush=True)

        if max_err < POSITION_TOL:
            print()
            log.info("✓ All joints at home position.")
            break
        time.sleep(POLL_S)
    else:
        print()
        log.warning("Timeout — some joints may not have reached home. Check for obstructions.")

    # ── Disable torque — arm is now freely backdrivable ────────────────────
    bus.disable_torque()
    log.info("Torque disabled — GELLO is ready for teleoperation.")

    bus.disconnect()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Move GELLO to calibration home position")
    p.add_argument(
        "--speed", type=float, default=0.4,
        help="Movement speed as fraction of max [0.1–1.0]. Default 0.4 (safe).",
    )
    args = p.parse_args()
    move_to_home(speed=args.speed)