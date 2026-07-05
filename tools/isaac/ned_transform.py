"""NED ↔ Isaac/Omniverse coordinate transforms for FlightSim integration."""

from __future__ import annotations

import math
from typing import Iterable


def json_float(value) -> float:
    """Coerce ROS/numpy scalars to native float for JSON export."""
    return float(value)


def json_float_list(values: Iterable[float]) -> list[float]:
    return [float(v) for v in values]


def ned_position_to_isaac(position_ned_m: Iterable[float]) -> tuple[float, float, float]:
    """Convert NED position (m) to Isaac world position (X=North, Y=East, Z=Up)."""
    north, east, down = position_ned_m
    return float(north), float(east), float(-down)


def ned_velocity_to_isaac(velocity_ned_mps: Iterable[float]) -> tuple[float, float, float]:
    """Convert NED velocity to Isaac world velocity."""
    vn, ve, vd = velocity_ned_mps
    return float(vn), float(ve), float(-vd)


def ned_quat_to_isaac_wxyz(attitude_wxyz: Iterable[float]) -> tuple[float, float, float, float]:
    """Convert body→NED quaternion to body→Isaac (Z-up) quaternion.

    Isaac frame is NED with flipped Z axis: q_isaac = q_flip * q_ned.
    q_flip rotates 180° about X (maps Down → Up).
    """
    w, x, y, z = (float(v) for v in attitude_wxyz)
    # 180° about X: (0, 1, 0, 0) * q_ned
    return (
        -x,
        w,
        z,
        -y,
    )


def camera_intrinsics(width_px: int, height_px: int, fov_rad: float) -> dict[str, float]:
    """Pinhole intrinsics matching FlightSim vision node defaults."""
    half = fov_rad * 0.5
    fx = (width_px * 0.5) / math.tan(half)
    fy = (height_px * 0.5) / math.tan(half)
    return {
        "width": float(width_px),
        "height": float(height_px),
        "fx": fx,
        "fy": fy,
        "cx": width_px * 0.5,
        "cy": height_px * 0.5,
        "fov_rad": fov_rad,
    }
