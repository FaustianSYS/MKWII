#!/usr/bin/env python3
"""Thread-safe ROS 2 telemetry client for the Qt tactical GUI."""

from __future__ import annotations

import math
import threading
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from flightsim_msgs.msg import EngagementStatus, MissileState, SceneState, TargetState


@dataclass
class TelemetrySnapshot:
    connected: bool = False
    missile: dict[str, Any] | None = None
    target: dict[str, Any] | None = None
    engagement: dict[str, Any] | None = None
    scene_entities: list[dict[str, Any]] = field(default_factory=list)
    missile_trail: list[list[float]] = field(default_factory=list)
    target_trail: list[list[float]] = field(default_factory=list)


class FlightSimRosClient(Node):
    """Subscribes to /flightsim/* truth topics used for visualization and tests."""

    def __init__(self, trail_len: int = 500) -> None:
        super().__init__("flightsim_qt_tactical_gui")
        self._lock = threading.Lock()
        self._trail_len = trail_len
        self._snap = TelemetrySnapshot(connected=True)

        self.create_subscription(SceneState, "/flightsim/scene_state", self._on_scene, qos_profile_sensor_data)
        self.create_subscription(MissileState, "/flightsim/missile_state", self._on_missile, 10)
        self.create_subscription(TargetState, "/flightsim/target_state", self._on_target, 10)
        self.create_subscription(EngagementStatus, "/flightsim/engagement_status", self._on_status, 10)
        self.get_logger().info("Qt tactical GUI subscribed to /flightsim/* topics")

    def snapshot(self) -> TelemetrySnapshot:
        with self._lock:
            return deepcopy(self._snap)

    def _push_trail(self, trail: list[list[float]], position: list[float]) -> None:
        trail.append([float(position[0]), float(position[1]), float(position[2])])
        if len(trail) > self._trail_len:
            del trail[: len(trail) - self._trail_len]

    def _on_missile(self, msg: MissileState) -> None:
        speed = math.sqrt(sum(float(v) * float(v) for v in msg.velocity_ned_mps))
        rate = math.sqrt(
            float(msg.pitch_rate_rps) ** 2 + float(msg.yaw_rate_rps) ** 2 + float(msg.roll_rate_rps) ** 2
        )
        data = {
            "position": [float(v) for v in msg.position_ned_m],
            "velocity": [float(v) for v in msg.velocity_ned_mps],
            "speed_mps": speed,
            "active": bool(msg.active),
            "hit": bool(msg.hit),
            "thrust_n": float(msg.thrust_n),
            "fin_pitch_deg": math.degrees(float(msg.fin_pitch_rad)),
            "fin_yaw_deg": math.degrees(float(msg.fin_yaw_rad)),
            "fin_roll_deg": math.degrees(float(msg.fin_roll_rad)),
            "seeker_locked": bool(msg.seeker_locked),
            "seeker_range_m": float(msg.seeker_range_m),
            "pitch_rate_rps": float(msg.pitch_rate_rps),
            "yaw_rate_rps": float(msg.yaw_rate_rps),
            "roll_rate_rps": float(msg.roll_rate_rps),
            "rate_mag_rps": rate,
        }
        with self._lock:
            self._snap.missile = data
            self._push_trail(self._snap.missile_trail, data["position"])

    def _on_target(self, msg: TargetState) -> None:
        speed = math.sqrt(sum(float(v) * float(v) for v in msg.velocity_ned_mps))
        data = {
            "id": msg.id,
            "model": msg.model,
            "position": [float(v) for v in msg.position_ned_m],
            "velocity": [float(v) for v in msg.velocity_ned_mps],
            "speed_mps": speed,
        }
        with self._lock:
            self._snap.target = data
            self._push_trail(self._snap.target_trail, data["position"])

    def _on_status(self, msg: EngagementStatus) -> None:
        with self._lock:
            self._snap.engagement = {
                "range_m": float(msg.range_m),
                "miss_distance_m": float(msg.miss_distance_m),
                "intercept": bool(msg.intercept),
                "complete": bool(msg.complete),
                "step_count": int(msg.step_count),
            }

    def _on_scene(self, msg: SceneState) -> None:
        entities = []
        for entity in msg.entities:
            entities.append(
                {
                    "id": entity.id,
                    "type": entity.type,
                    "model": entity.model,
                    "position": [float(v) for v in entity.position_ned_m],
                }
            )
        with self._lock:
            self._snap.scene_entities = entities


class RosSpinThread(threading.Thread):
    def __init__(self, node: FlightSimRosClient) -> None:
        super().__init__(daemon=True, name="flightsim-qt-ros-spin")
        self._node = node
        self._stop = threading.Event()

    def run(self) -> None:
        while rclpy.ok() and not self._stop.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.02)

    def stop(self) -> None:
        self._stop.set()
