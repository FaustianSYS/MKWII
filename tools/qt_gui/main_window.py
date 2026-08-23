"""Main window for FlightSim Qt tactical visualization / testing."""

from __future__ import annotations

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from map_widget import NedMapWidget
from ros_client import FlightSimRosClient, RosSpinThread, TelemetrySnapshot


def _fmt(value: float | None, digits: int = 1) -> str:
    if value is None:
        return "—"
    return f"{value:.{digits}f}"


class MetricLabel(QLabel):
    def __init__(self, title: str) -> None:
        super().__init__()
        self._title = title
        self.setTextFormat(Qt.RichText)
        self.set_value("—")

    def set_value(self, value: str, accent: str | None = None) -> None:
        color = accent or "#e8eef7"
        self.setText(
            f"<div style='color:#8aa0b8;font-size:11px'>{self._title}</div>"
            f"<div style='color:{color};font-size:18px;font-family:monospace'>{value}</div>"
        )


class TacticalMainWindow(QMainWindow):
    def __init__(self, ros_client: FlightSimRosClient, spin_thread: RosSpinThread) -> None:
        super().__init__()
        self._ros = ros_client
        self._spin = spin_thread
        self._last_complete = False
        self._event_count = 0

        self.setWindowTitle("FlightSim · Qt Tactical Test GUI")
        self.resize(1280, 800)
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #101826; color: #e8eef7; }
            QFrame#panel {
                background: #162033;
                border: 1px solid #243247;
                border-radius: 8px;
            }
            QLabel#header { font-size: 16px; font-weight: 600; }
            QTextEdit {
                background: #0b1220;
                border: 1px solid #243247;
                color: #c9d6e5;
                font-family: monospace;
                font-size: 11px;
            }
            QPushButton {
                background: #243247;
                border: 1px solid #3a4d66;
                border-radius: 4px;
                padding: 6px 12px;
            }
            QPushButton:hover { background: #2e4058; }
            """
        )

        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        left = QFrame()
        left.setObjectName("panel")
        left_layout = QVBoxLayout(left)
        title = QLabel("Engagement / Missile Test")
        title.setObjectName("header")
        left_layout.addWidget(title)

        grid = QGridLayout()
        self.m_range = MetricLabel("RANGE (m)")
        self.m_miss = MetricLabel("MISS (m)")
        self.m_step = MetricLabel("STEP")
        self.m_status = MetricLabel("STATUS")
        self.m_mspeed = MetricLabel("MISSILE SPEED (m/s)")
        self.m_thrust = MetricLabel("THRUST (N)")
        self.m_fins = MetricLabel("FINS P/Y/R (deg)")
        self.m_rates = MetricLabel("BODY RATE (rad/s)")
        self.m_lock = MetricLabel("SEEKER")
        self.m_tspeed = MetricLabel("TARGET SPEED (m/s)")
        metrics = [
            self.m_range,
            self.m_miss,
            self.m_step,
            self.m_status,
            self.m_mspeed,
            self.m_thrust,
            self.m_fins,
            self.m_rates,
            self.m_lock,
            self.m_tspeed,
        ]
        for i, metric in enumerate(metrics):
            grid.addWidget(metric, i // 2, i % 2)
        left_layout.addLayout(grid)

        btn_row = QHBoxLayout()
        self.clear_btn = QPushButton("Clear trails")
        self.clear_btn.clicked.connect(self._clear_trails)
        self.mark_btn = QPushButton("Mark event")
        self.mark_btn.clicked.connect(lambda: self._log("Manual mark", "info"))
        btn_row.addWidget(self.clear_btn)
        btn_row.addWidget(self.mark_btn)
        left_layout.addLayout(btn_row)

        left_layout.addWidget(QLabel("Event log"))
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        left_layout.addWidget(self.log, stretch=1)

        right = QFrame()
        right.setObjectName("panel")
        right_layout = QVBoxLayout(right)
        map_title = QLabel("NED Tactical Map")
        map_title.setObjectName("header")
        right_layout.addWidget(map_title)
        self.map = NedMapWidget()
        right_layout.addWidget(self.map, stretch=1)
        hint = QLabel("For headless CI use unit tests; this GUI is for interactive visualization.")
        hint.setStyleSheet("color:#8aa0b8;font-size:11px")
        right_layout.addWidget(hint)

        layout.addWidget(left, 2)
        layout.addWidget(right, 3)

        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._log("Qt tactical GUI started — waiting for /flightsim/*", "info")

    def _clear_trails(self) -> None:
        with self._ros._lock:  # noqa: SLF001 — intentional for test GUI
            self._ros._snap.missile_trail.clear()
            self._ros._snap.target_trail.clear()
        self._log("Trails cleared", "info")

    def _log(self, message: str, level: str = "info") -> None:
        self._event_count += 1
        self.log.append(f"[{self._event_count:04d}] {message}")

    def _refresh(self) -> None:
        snap: TelemetrySnapshot = self._ros.snapshot()
        eng = snap.engagement or {}
        mis = snap.missile or {}
        tgt = snap.target or {}

        self.m_range.set_value(_fmt(eng.get("range_m")))
        self.m_miss.set_value(_fmt(eng.get("miss_distance_m")))
        self.m_step.set_value(str(eng.get("step_count", "—")))

        if eng.get("intercept"):
            self.m_status.set_value("INTERCEPT", "#3dd68c")
        elif eng.get("complete"):
            self.m_status.set_value("COMPLETE", "#f0c14b")
        elif mis.get("active"):
            self.m_status.set_value("ACTIVE", "#4c9aff")
        else:
            self.m_status.set_value("IDLE")

        self.m_mspeed.set_value(_fmt(mis.get("speed_mps")))
        self.m_thrust.set_value(_fmt(mis.get("thrust_n"), 0))
        if mis:
            self.m_fins.set_value(
                f"{mis.get('fin_pitch_deg', 0):.1f} / {mis.get('fin_yaw_deg', 0):.1f} / {mis.get('fin_roll_deg', 0):.1f}"
            )
            self.m_rates.set_value(_fmt(mis.get("rate_mag_rps"), 2))
            lock = "LOCKED" if mis.get("seeker_locked") else "SEARCH"
            self.m_lock.set_value(lock, "#3dd68c" if mis.get("seeker_locked") else "#8aa0b8")
        self.m_tspeed.set_value(_fmt(tgt.get("speed_mps")))

        depot = None
        for entity in snap.scene_entities:
            if entity.get("type") == "depot" or entity.get("id") == "depot":
                depot = entity.get("position")
                break

        status = "LIVE" if snap.missile or snap.target else "AWAITING TELEMETRY"
        self.map.set_tracks(
            mis.get("position"),
            tgt.get("position"),
            snap.missile_trail,
            snap.target_trail,
            depot,
            status,
        )

        complete = bool(eng.get("complete"))
        if complete and not self._last_complete:
            if eng.get("intercept"):
                self._log(
                    f"Intercept at step {eng.get('step_count')} · miss={eng.get('miss_distance_m'):.2f} m",
                    "ok",
                )
            else:
                self._log(
                    f"Engagement ended · miss={eng.get('miss_distance_m'):.2f} m",
                    "warn",
                )
        self._last_complete = complete

    def closeEvent(self, event) -> None:  # noqa: N802
        self._timer.stop()
        self._spin.stop()
        event.accept()
