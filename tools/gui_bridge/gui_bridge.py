#!/usr/bin/env python3
"""ROS 2 → WebSocket bridge for the FlightSim tactical GUI."""

from __future__ import annotations

import asyncio
import json
import threading
from typing import Any

import rclpy
from flightsim_msgs.msg import AircraftState, EngagementStatus, MissileState, SimStatus, TargetState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

try:
    import websockets
except ImportError as exc:
    raise SystemExit("Install websockets: pip install websockets") from exc


class GuiBridge(Node):
    def __init__(self) -> None:
        super().__init__("flightsim_gui_bridge")
        self.payload: dict[str, Any] = {}
        self.clients: set[Any] = set()

        self.create_subscription(TargetState, "target_state", self._on_target, qos_profile_sensor_data)
        self.create_subscription(MissileState, "missile_state", self._on_missile, qos_profile_sensor_data)
        self.create_subscription(
            EngagementStatus, "engagement_status", self._on_engagement, qos_profile_sensor_data
        )
        self.create_subscription(AircraftState, "aircraft_state", self._on_aircraft, qos_profile_sensor_data)
        self.create_subscription(SimStatus, "sim_status", self._on_sim_status, qos_profile_sensor_data)
        self.get_logger().info("GUI bridge listening on ws://0.0.0.0:8765")

    def _on_target(self, msg: TargetState) -> None:
        self.payload["target"] = {
            "position": list(msg.position_ned_m),
            "velocity": list(msg.velocity_ned_mps),
        }

    def _on_missile(self, msg: MissileState) -> None:
        self.payload["missile"] = {
            "position": list(msg.position_ned_m),
            "velocity": list(msg.velocity_ned_mps),
            "active": msg.active,
            "hit": msg.hit,
        }

    def _on_engagement(self, msg: EngagementStatus) -> None:
        self.payload["engagement"] = {
            "range_m": msg.range_m,
            "miss_distance_m": msg.miss_distance_m,
            "intercept": msg.intercept,
            "complete": msg.complete,
            "step_count": msg.step_count,
        }

    def _on_aircraft(self, msg: AircraftState) -> None:
        speed = sum(v * v for v in msg.velocity_ned_mps) ** 0.5
        self.payload["aircraft"] = {
            "altitude_m": -msg.position_ned_m[2],
            "speed_mps": speed,
            "throttle": msg.throttle,
            "safe_mode": 0,
        }

    def _on_sim_status(self, msg: SimStatus) -> None:
        if "aircraft" in self.payload:
            self.payload["aircraft"]["safe_mode"] = msg.safe_mode

    def snapshot(self) -> str:
        return json.dumps(self.payload)


async def ws_handler(websocket, bridge: GuiBridge) -> None:
    bridge.clients.add(websocket)
    try:
        async for _ in websocket:
            pass
    finally:
        bridge.clients.discard(websocket)


async def broadcast_loop(bridge: GuiBridge) -> None:
    while rclpy.ok():
        if bridge.clients:
            message = bridge.snapshot()
            stale = []
            for client in bridge.clients:
                try:
                    await client.send(message)
                except websockets.ConnectionClosed:
                    stale.append(client)
            for client in stale:
                bridge.clients.discard(client)
        await asyncio.sleep(0.05)


async def run_server(bridge: GuiBridge) -> None:
    async with websockets.serve(lambda ws: ws_handler(ws, bridge), "0.0.0.0", 8765):
        await broadcast_loop(bridge)


def spin_ros(bridge: GuiBridge) -> None:
    while rclpy.ok():
        rclpy.spin_once(bridge, timeout_sec=0.01)


def main() -> None:
    rclpy.init()
    bridge = GuiBridge()
    ros_thread = threading.Thread(target=spin_ros, args=(bridge,), daemon=True)
    ros_thread.start()
    try:
        asyncio.run(run_server(bridge))
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
