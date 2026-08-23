"""NED horizontal map widget for FlightSim Qt tactical GUI."""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QWidget


class NedMapWidget(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumSize(480, 360)
        self._missile_pos: list[float] | None = None
        self._target_pos: list[float] | None = None
        self._missile_trail: list[list[float]] = []
        self._target_trail: list[list[float]] = []
        self._depot_pos: list[float] | None = None
        self._status = "AWAITING TELEMETRY"

    def set_tracks(
        self,
        missile_pos: list[float] | None,
        target_pos: list[float] | None,
        missile_trail: list[list[float]],
        target_trail: list[list[float]],
        depot_pos: list[float] | None = None,
        status: str = "",
    ) -> None:
        self._missile_pos = missile_pos
        self._target_pos = target_pos
        self._missile_trail = missile_trail
        self._target_trail = target_trail
        self._depot_pos = depot_pos
        if status:
            self._status = status
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        painter.fillRect(0, 0, w, h, QColor("#0b1220"))

        points: list[list[float]] = []
        points.extend(self._missile_trail)
        points.extend(self._target_trail)
        if self._missile_pos:
            points.append(self._missile_pos)
        if self._target_pos:
            points.append(self._target_pos)
        if self._depot_pos:
            points.append(self._depot_pos)

        if not points:
            painter.setPen(QColor("#6b7c93"))
            painter.setFont(QFont("monospace", 11))
            painter.drawText(self.rect(), Qt.AlignCenter, self._status)
            return

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        pad_m = 80.0
        min_x, max_x = min(xs) - pad_m, max(xs) + pad_m
        min_y, max_y = min(ys) - pad_m, max(ys) + pad_m
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)

        margin = 40.0

        def to_screen(north: float, east: float) -> tuple[float, float]:
            sx = margin + ((north - min_x) / span_x) * (w - 2 * margin)
            sy = h - (margin + ((east - min_y) / span_y) * (h - 2 * margin))
            return sx, sy

        # Grid
        painter.setPen(QPen(QColor(76, 154, 255, 28), 1))
        for i in range(11):
            gx = min_x + (i / 10.0) * span_x
            x1, y1 = to_screen(gx, min_y)
            x2, y2 = to_screen(gx, max_y)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            gy = min_y + (i / 10.0) * span_y
            x3, y3 = to_screen(min_x, gy)
            x4, y4 = to_screen(max_x, gy)
            painter.drawLine(int(x3), int(y3), int(x4), int(y4))

        def draw_trail(trail: list[list[float]], color: QColor) -> None:
            if len(trail) < 2:
                return
            painter.setPen(QPen(color, 2))
            for i in range(1, len(trail)):
                x1, y1 = to_screen(trail[i - 1][0], trail[i - 1][1])
                x2, y2 = to_screen(trail[i][0], trail[i][1])
                painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        draw_trail(self._target_trail, QColor("#f0c14b"))
        draw_trail(self._missile_trail, QColor("#4c9aff"))

        if self._depot_pos:
            dx, dy = to_screen(self._depot_pos[0], self._depot_pos[1])
            painter.setBrush(QColor("#8b9bb4"))
            painter.setPen(Qt.NoPen)
            painter.drawRect(int(dx - 5), int(dy - 5), 10, 10)

        if self._target_pos:
            tx, ty = to_screen(self._target_pos[0], self._target_pos[1])
            painter.setBrush(QColor("#f0c14b"))
            painter.setPen(QPen(QColor("#fff3c4"), 1))
            painter.drawEllipse(int(tx - 6), int(ty - 6), 12, 12)
            painter.setPen(QColor("#f0c14b"))
            painter.drawText(int(tx + 10), int(ty - 4), "TARGET")

        if self._missile_pos:
            mx, my = to_screen(self._missile_pos[0], self._missile_pos[1])
            painter.setBrush(QColor("#4c9aff"))
            painter.setPen(QPen(QColor("#cfe6ff"), 1))
            painter.drawEllipse(int(mx - 6), int(my - 6), 12, 12)
            painter.setPen(QColor("#4c9aff"))
            painter.drawText(int(mx + 10), int(my - 4), "MISSILE")

        # Axes label
        painter.setPen(QColor("#6b7c93"))
        painter.setFont(QFont("monospace", 9))
        painter.drawText(12, 18, "NED map · +X North · +Y East")
