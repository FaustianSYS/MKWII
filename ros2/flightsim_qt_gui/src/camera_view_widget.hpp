#pragma once

#include "telemetry_bridge.hpp"

#include <QWidget>

class CameraViewWidget : public QWidget {
  Q_OBJECT
 public:
  enum class Mode { Seeker, Drone };

  explicit CameraViewWidget(Mode mode, QWidget* parent = nullptr);

 public slots:
  void setSnapshot(const TelemetrySnapshot& snap);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  bool projectPoint(const QVector3D& eye, const QVector3D& forward, const QVector3D& world_up,
                    float fov_rad, const QVector3D& point_ned, QPointF* out_uv) const;
  QVector3D safeForward(const QVector3D& vel, const QVector3D& fallback) const;

  Mode mode_;
  TelemetrySnapshot snap_;
};
