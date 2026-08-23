#!/usr/bin/env python3
"""FlightSim Qt tactical GUI — visualization and interactive testing.

Requires:
  - ROS 2 Jazzy overlay sourced
  - system Python with PyQt5 (`/usr/bin/python3` + python3-pyqt5)
  - engagement (or full stack) publishing /flightsim/* topics

Usage:
  ./scripts/launch_qt_gui.sh
  # or with engagement in another terminal:
  /usr/bin/python3 tools/qt_gui/tactical_gui.py
"""

from __future__ import annotations

import sys
from pathlib import Path

# Allow running as a script from repo root or tools/qt_gui.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
from PyQt5.QtWidgets import QApplication

from main_window import TacticalMainWindow
from ros_client import FlightSimRosClient, RosSpinThread


def main() -> int:
    rclpy.init()
    node = FlightSimRosClient()
    spin = RosSpinThread(node)
    spin.start()

    app = QApplication(sys.argv)
    app.setApplicationName("FlightSimQtGui")
    window = TacticalMainWindow(node, spin)
    window.show()

    code = app.exec_()
    spin.stop()
    spin.join(timeout=1.0)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return int(code)


if __name__ == "__main__":
    sys.exit(main())
