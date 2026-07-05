#!/usr/bin/env python3
"""Isaac Sim dataset sidecar — Phase 0 Mode A.

Subscribes to FlightSim truth topics, syncs kinematic actors, renders seeker
frames, and exports PNG + label JSON. Falls back to dry-run logging when Isaac
Sim is not installed.

Usage:
  python3 tools/isaac/isaac_dataset_sidecar.py --dry-run
  python3 tools/isaac/isaac_dataset_sidecar.py --output datasets/isaac_001 --headless
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from flightsim_msgs.msg import EngagementStatus, MissileState, SceneState, TargetState

from ned_transform import (
    camera_intrinsics,
    json_float_list,
    ned_position_to_isaac,
    ned_quat_to_isaac_wxyz,
)


class IsaacDatasetSidecar(Node):
    """ROS 2 subscriber that drives Isaac rendering or dry-run export."""

    def __init__(self, output_dir: Path, dry_run: bool) -> None:
        super().__init__("flightsim_isaac_dataset_sidecar")
        self.output_dir = output_dir
        self.dry_run = dry_run
        self.images_dir = output_dir / "images"
        self.labels_dir = output_dir / "labels"
        self.images_dir.mkdir(parents=True, exist_ok=True)
        self.labels_dir.mkdir(parents=True, exist_ok=True)
        self.frame_count = 0
        self.intrinsics = camera_intrinsics(640, 480, 0.52)
        self._latest_missile = None
        self._latest_target = None
        self._done = threading.Event()
        self._sim_app = None
        self._isaac_world = None

        if not dry_run:
            self._init_isaac()

        self.create_subscription(SceneState, "/flightsim/scene_state", self._on_scene, qos_profile_sensor_data)
        self.create_subscription(MissileState, "/flightsim/missile_state", self._on_missile, 10)
        self.create_subscription(TargetState, "/flightsim/target_state", self._on_target, 10)
        self.create_subscription(EngagementStatus, "/flightsim/engagement_status", self._on_status, 10)

        mode = "dry-run" if dry_run else "isaac"
        self.get_logger().info(f"Isaac dataset sidecar ({mode}) → {output_dir}")

    def _init_isaac(self) -> None:
        try:
            from isaacsim import SimulationApp  # type: ignore[import-not-found]
        except ImportError as exc:
            raise SystemExit(
                "Isaac Sim not found. Install Isaac Sim 4.x and set ISAACSIM_PATH, "
                "or use --dry-run to verify ROS subscription only."
            ) from exc

        self._sim_app = SimulationApp({"headless": True})
        self.get_logger().info("Isaac SimulationApp started — load USD scene and wire prims here")

        # TODO Phase 1: import USD assets, create kinematic prims for missile/shahed/depot,
        # attach Replicator or RTX camera to missile nose (+X boresight).

    def _on_status(self, msg: EngagementStatus) -> None:
        if msg.complete:
            self._done.set()

    def _on_missile(self, msg: MissileState) -> None:
        self._latest_missile = msg

    def _on_target(self, msg: TargetState) -> None:
        self._latest_target = msg

    def _on_scene(self, msg: SceneState) -> None:
        label = {
            "sim_step": int(msg.sim_step),
            "intrinsics": self.intrinsics,
            "entities": [],
        }

        for entity in msg.entities:
            pos = ned_position_to_isaac(entity.position_ned_m)
            quat = ned_quat_to_isaac_wxyz(entity.attitude_wxyz)
            label["entities"].append({
                "id": entity.id,
                "type": entity.type,
                "model": entity.model,
                "position_isaac_m": list(pos),
                "attitude_wxyz_isaac": list(quat),
                "position_ned_m": json_float_list(entity.position_ned_m),
                "attitude_wxyz_ned": json_float_list(entity.attitude_wxyz),
            })

            if not self.dry_run and self._isaac_world is not None:
                # TODO: self._isaac_world.set_prim_pose(entity.id, pos, quat)
                pass

        label_path = self.labels_dir / f"{self.frame_count:06d}.json"
        label_path.write_text(json.dumps(label, indent=2))

        if not self.dry_run:
            # TODO: render camera → PNG at self.images_dir / f"{self.frame_count:06d}.png"
            pass

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f"Sidecar frame {self.frame_count} (sim_step={int(msg.sim_step)})"
            )

    def wait_until_done(self) -> None:
        self._done.wait()

    def shutdown_isaac(self) -> None:
        if self._sim_app is not None:
            self._sim_app.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Isaac Sim dataset sidecar (Phase 0 Mode A)")
    parser.add_argument("--output", type=Path, default=Path("datasets/isaac_run"))
    parser.add_argument("--dry-run", action="store_true", help="Subscribe only; no Isaac Sim required")
    parser.add_argument("--headless", action="store_true", help="Run Isaac headless (when Isaac installed)")
    args = parser.parse_args()

    rclpy.init()
    node = IsaacDatasetSidecar(args.output, dry_run=args.dry_run)
    try:
        while rclpy.ok() and not node._done.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        meta = {
            "mode": "isaac_dataset_sidecar_phase0_a",
            "dry_run": args.dry_run,
            "frame_count": node.frame_count,
            "intrinsics": node.intrinsics,
        }
        (args.output / "metadata.json").write_text(json.dumps(meta, indent=2))
        node.shutdown_isaac()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
