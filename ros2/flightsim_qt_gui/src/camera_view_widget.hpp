#pragma once

#include "obj_mesh.hpp"
#include "telemetry_bridge.hpp"

#include <QColor>
#include <QPolygonF>
#include <QWidget>

#include <functional>

class QPainter;

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
  struct ProjectedTriangle {
    float depth{0.0F};
    QPolygonF polygon;
    QColor fill;
  };

  bool projectPoint(const QVector3D& eye, const QVector3D& forward, const QVector3D& world_up, float fov_rad,
                    const QVector3D& point_ned, QPointF* out_uv) const;
  QVector3D safeForward(const QVector3D& vel, const QVector3D& fallback) const;
  QVector3D seekerForward() const;
  void ensureShahedMesh();
  void drawShahedSeeker(QPainter& painter, const QRectF& viewport, const QVector3D& eye, const QVector3D& forward,
                        const QVector3D& world_up, float fov_rad,
                        const std::function<QPointF(const QPointF&)>& to_pixel);

  Mode mode_;
  TelemetrySnapshot snap_;
  ObjMesh shahed_mesh_;
  bool shahed_mesh_load_attempted_{false};
  bool shahed_mesh_ok_{false};
};
