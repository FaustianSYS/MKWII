#!/usr/bin/env python3
"""Record FlightSim truth topics to per-step JSON frames for Isaac dataset ingest."""

from __future__ import annotations

import argparse
import json
import sys
import threading
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from flightsim_msgs.msg import EngagementStatus, MissileState, SceneState, TargetState

from ned_transform import json_float, json_float_list


class TruthLogger(Node):
    def __init__(self, output_dir: Path, wait_complete: bool) -> None:
        super().__init__("flightsim_truth_logger")
        self.output_dir = output_dir
        self.frames_dir = output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.wait_complete = wait_complete
        self.frame_count = 0
        self.latest_missile: dict | None = None
        self.latest_target: dict | None = None
        self.latest_status: dict | None = None
        self._done = threading.Event()

        self.create_subscription(SceneState, "/flightsim/scene_state", self._on_scene, qos_profile_sensor_data)
        self.create_subscription(MissileState, "/flightsim/missile_state", self._on_missile, 10)
        self.create_subscription(TargetState, "/flightsim/target_state", self._on_target, 10)
        self.create_subscription(EngagementStatus, "/flightsim/engagement_status", self._on_status, 10)

        self.get_logger().info(f"Truth logger writing to {output_dir}")

    def _stamp(self, msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _on_missile(self, msg: MissileState) -> None:
        self.latest_missile = {
            "position_ned_m": json_float_list(msg.position_ned_m),
            "velocity_ned_mps": json_float_list(msg.velocity_ned_mps),
            "attitude_wxyz": [
                json_float(msg.attitude_w),
                json_float(msg.attitude_x),
                json_float(msg.attitude_y),
                json_float(msg.attitude_z),
            ],
            "fin_pitch_rad": json_float(msg.fin_pitch_rad),
            "fin_yaw_rad": json_float(msg.fin_yaw_rad),
            "fin_roll_rad": json_float(msg.fin_roll_rad),
            "thrust_n": json_float(msg.thrust_n),
            "active": bool(msg.active),
            "seeker_locked": bool(msg.seeker_locked),
            "seeker_range_m": json_float(msg.seeker_range_m),
        }

    def _on_target(self, msg: TargetState) -> None:
        self.latest_target = {
            "id": msg.id,
            "model": msg.model,
            "position_ned_m": json_float_list(msg.position_ned_m),
            "velocity_ned_mps": json_float_list(msg.velocity_ned_mps),
            "attitude_wxyz": json_float_list(msg.attitude_wxyz),
        }

    def _on_status(self, msg: EngagementStatus) -> None:
        self.latest_status = {
            "range_m": json_float(msg.range_m),
            "miss_distance_m": json_float(msg.miss_distance_m),
            "intercept": bool(msg.intercept),
            "complete": bool(msg.complete),
            "step_count": int(msg.step_count),
        }
        if self.wait_complete and msg.complete:
            self._done.set()

    def _on_scene(self, msg: SceneState) -> None:
        entities = []
        for entity in msg.entities:
            entities.append({
                "id": entity.id,
                "type": entity.type,
                "model": entity.model,
                "position_ned_m": json_float_list(entity.position_ned_m),
                "velocity_ned_mps": json_float_list(entity.velocity_ned_mps),
                "attitude_wxyz": json_float_list(entity.attitude_wxyz),
            })

        frame = {
            "sim_step": int(msg.sim_step),
            "stamp_sec": self._stamp(msg),
            "entities": entities,
            "missile": self.latest_missile,
            "target": self.latest_target,
            "engagement": self.latest_status,
        }

        path = self.frames_dir / f"{self.frame_count:06d}.json"
        path.write_text(json.dumps(frame, indent=2))
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"Recorded {self.frame_count} frames (sim_step={int(msg.sim_step)})"
            )

    def write_metadata(self) -> None:
        metadata = {
            "mode": "dataset_sidecar_phase0_a",
            "recorded_at": datetime.now(timezone.utc).isoformat(),
            "frame_count": self.frame_count,
            "topics": [
                "/flightsim/scene_state",
                "/flightsim/missile_state",
                "/flightsim/target_state",
                "/flightsim/engagement_status",
            ],
            "coordinate_frame": "ned",
        }
        (self.output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

    def wait_until_done(self) -> None:
        if self.wait_complete:
            self._done.wait()


def main() -> int:
    parser = argparse.ArgumentParser(description="FlightSim truth logger for Isaac dataset sidecar")
    parser.add_argument("--output", type=Path, required=True, help="Dataset output directory")
    parser.add_argument(
        "--wait-complete",
        action="store_true",
        help="Exit after engagement_status.complete is true",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TruthLogger(args.output, args.wait_complete)
    try:
        if args.wait_complete:
            while rclpy.ok() and not node._done.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_metadata()
        node.get_logger().info(f"Wrote {node.frame_count} frames to {args.output}")
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
