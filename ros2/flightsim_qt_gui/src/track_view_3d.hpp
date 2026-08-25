#pragma once

#include "telemetry_bridge.hpp"

#include <QMatrix4x4>
#include <QOpenGLFunctions_2_1>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector>
#include <QVector3D>
#include <array>
#include <cstdint>

class TrackView3D : public QOpenGLWidget, protected QOpenGLFunctions_2_1 {
  Q_OBJECT
 public:
  explicit TrackView3D(QWidget* parent = nullptr);

 public slots:
  void setSnapshot(const TelemetrySnapshot& snap);
  void resetCamera();
  void followMissile(bool enabled);
  void randomizeGround();

 protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  struct TerrainWave {
    float amp_m{0.0F};
    float freq_n{0.0F};
    float freq_e{0.0F};
    float phase{0.0F};
  };

  QVector3D nedToDisplay(const QVector3D& ned) const;
  void regenerateTerrain(std::uint32_t seed);
  float sampleGroundUp_m(float north_m, float east_m) const;
  void drawGroundPlane();
  void drawGrid();
  void drawAxes();
  void drawTrail(const QVector<TrackSample>& trail, float r, float g, float b, float width);
  void drawMarker(const QVector3D& display_pos, float r, float g, float b, float size);
  void drawLos();
  void updateAutoFrame();
  QPointF projectToScreen(const QVector3D& display_pos, const QMatrix4x4& mvp) const;
  void drawCoordOverlays(const QMatrix4x4& mvp);

  TelemetrySnapshot snap_;
  bool follow_missile_{true};

  float yaw_deg_{35.0F};
  float pitch_deg_{25.0F};
  float distance_m_{2500.0F};
  QVector3D look_at_{0.0F, 0.0F, 800.0F};

  QPoint last_mouse_;
  bool dragging_{false};
  bool panning_{false};

  static constexpr float kGroundHalf_m = 500.0F;
  static constexpr int kTerrainRes = 48;  // quads per side → (res+1)^2 verts
  std::array<TerrainWave, 6> terrain_waves_{};
  float terrain_base_up_m_{0.0F};
  std::uint32_t terrain_seed_{1U};
  quint64 last_step_count_{0};
};
