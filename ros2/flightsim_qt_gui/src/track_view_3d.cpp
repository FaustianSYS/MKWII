#include "track_view_3d.hpp"

#include <QMouseEvent>
#include <QPainter>
#include <QVector4D>
#include <QWheelEvent>
#include <QtMath>
#include <algorithm>
#include <cmath>

TrackView3D::TrackView3D(QWidget* parent) : QOpenGLWidget(parent) {
  setMinimumSize(640, 480);
  setFocusPolicy(Qt::StrongFocus);
}

void TrackView3D::setSnapshot(const TelemetrySnapshot& snap) {
  snap_ = snap;
  if (follow_missile_ && snap_.has_missile) {
    look_at_ = nedToDisplay(snap_.missile_pos_ned);
  }
  update();
}

void TrackView3D::resetCamera() {
  yaw_deg_ = 35.0F;
  pitch_deg_ = 25.0F;
  distance_m_ = 2500.0F;
  look_at_ = QVector3D(0.0F, 0.0F, 800.0F);
  updateAutoFrame();
  update();
}

void TrackView3D::followMissile(bool enabled) {
  follow_missile_ = enabled;
}

QVector3D TrackView3D::nedToDisplay(const QVector3D& ned) const {
  // Display frame: +X North, +Y East, +Z Up
  return QVector3D(ned.x(), ned.y(), -ned.z());
}

void TrackView3D::initializeGL() {
  initializeOpenGLFunctions();
  glClearColor(0.04F, 0.07F, 0.10F, 1.0F);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);
}

void TrackView3D::resizeGL(int w, int h) {
  glViewport(0, 0, w, h);
}

void TrackView3D::updateAutoFrame() {
  if (!snap_.has_missile && !snap_.has_target) {
    return;
  }
  QVector3D min_p(1e9F, 1e9F, 1e9F);
  QVector3D max_p(-1e9F, -1e9F, -1e9F);
  auto expand = [&](const QVector3D& ned) {
    const QVector3D p = nedToDisplay(ned);
    min_p.setX(std::min(min_p.x(), p.x()));
    min_p.setY(std::min(min_p.y(), p.y()));
    min_p.setZ(std::min(min_p.z(), p.z()));
    max_p.setX(std::max(max_p.x(), p.x()));
    max_p.setY(std::max(max_p.y(), p.y()));
    max_p.setZ(std::max(max_p.z(), p.z()));
  };
  if (snap_.has_missile) {
    expand(snap_.missile_pos_ned);
  }
  if (snap_.has_target) {
    expand(snap_.target_pos_ned);
  }
  for (const auto& s : snap_.missile_trail) {
    expand(s.position_ned);
  }
  for (const auto& s : snap_.target_trail) {
    expand(s.position_ned);
  }
  look_at_ = (min_p + max_p) * 0.5F;
  const float span = std::max({max_p.x() - min_p.x(), max_p.y() - min_p.y(), max_p.z() - min_p.z(), 200.0F});
  distance_m_ = std::clamp(span * 1.8F, 400.0F, 20000.0F);
}

void TrackView3D::paintGL() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  QMatrix4x4 projection;
  projection.perspective(45.0F, width() > 0 ? float(width()) / float(height()) : 1.0F, 1.0F, 100000.0F);

  const float yaw = qDegreesToRadians(yaw_deg_);
  const float pitch = qDegreesToRadians(pitch_deg_);
  const QVector3D eye = look_at_ + QVector3D(distance_m_ * std::cos(pitch) * std::cos(yaw),
                                             distance_m_ * std::cos(pitch) * std::sin(yaw),
                                             distance_m_ * std::sin(pitch));

  QMatrix4x4 view;
  view.lookAt(eye, look_at_, QVector3D(0.0F, 0.0F, 1.0F));

  QMatrix4x4 mvp = projection * view;
  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(mvp.constData());
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  drawGrid();
  drawAxes();
  drawTrail(snap_.target_trail, 0.95F, 0.55F, 0.20F, 2.0F);
  drawTrail(snap_.missile_trail, 0.25F, 0.85F, 1.0F, 2.5F);
  drawLos();

  if (snap_.has_target) {
    drawMarker(nedToDisplay(snap_.target_pos_ned), 1.0F, 0.45F, 0.15F, 18.0F);
  }
  if (snap_.has_missile) {
    drawMarker(nedToDisplay(snap_.missile_pos_ned), 0.2F, 0.9F, 1.0F, 14.0F);
  }
  for (const auto& entity : snap_.entities) {
    if (entity.type == QStringLiteral("missile") || entity.type == QStringLiteral("target")) {
      continue;
    }
    drawMarker(nedToDisplay(entity.position_ned), 0.6F, 0.8F, 0.5F, 10.0F);
  }

  drawCoordOverlays(mvp);
}

QPointF TrackView3D::projectToScreen(const QVector3D& display_pos, const QMatrix4x4& mvp) const {
  const QVector4D clip = mvp * QVector4D(display_pos, 1.0F);
  if (std::abs(clip.w()) < 1e-6F) {
    return QPointF(-1.0, -1.0);
  }
  const float ndc_x = clip.x() / clip.w();
  const float ndc_y = clip.y() / clip.w();
  const float ndc_z = clip.z() / clip.w();
  if (ndc_z < -1.0F || ndc_z > 1.0F) {
    return QPointF(-1.0, -1.0);
  }
  return QPointF((ndc_x * 0.5F + 0.5F) * width(), (1.0F - (ndc_y * 0.5F + 0.5F)) * height());
}

void TrackView3D::drawCoordOverlays(const QMatrix4x4& mvp) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  QFont font = painter.font();
  font.setFamily(QStringLiteral("monospace"));
  font.setPointSize(10);
  painter.setFont(font);

  auto drawLabel = [&](const QVector3D& ned, const QString& name, const QColor& color) {
    const QPointF screen = projectToScreen(nedToDisplay(ned), mvp);
    if (screen.x() < 0.0) {
      return;
    }
    const QString text = QStringLiteral("%1\nN %2  E %3  D %4")
                             .arg(name)
                             .arg(ned.x(), 0, 'f', 1)
                             .arg(ned.y(), 0, 'f', 1)
                             .arg(ned.z(), 0, 'f', 1);
    const QRectF box(screen.x() + 12.0, screen.y() - 34.0, 210.0, 40.0);
    painter.fillRect(box.adjusted(-6, -4, 6, 4), QColor(8, 14, 22, 180));
    painter.setPen(color);
    painter.drawText(box, Qt::AlignLeft | Qt::AlignVCenter, text);
  };

  if (snap_.has_missile) {
    drawLabel(snap_.missile_pos_ned, QStringLiteral("MISSILE"), QColor(64, 220, 240));
  }
  if (snap_.has_target) {
    drawLabel(snap_.target_pos_ned, QStringLiteral("TARGET"), QColor(255, 160, 70));
  }
  painter.end();
}

void TrackView3D::drawGrid() {
  glLineWidth(1.0F);
  glBegin(GL_LINES);
  glColor4f(0.18F, 0.28F, 0.34F, 0.7F);
  const float step = 200.0F;
  const float extent = 4000.0F;
  for (float x = -extent; x <= extent + 0.1F; x += step) {
    glVertex3f(x, -extent, 0.0F);
    glVertex3f(x, extent, 0.0F);
  }
  for (float y = -extent; y <= extent + 0.1F; y += step) {
    glVertex3f(-extent, y, 0.0F);
    glVertex3f(extent, y, 0.0F);
  }
  glEnd();
}

void TrackView3D::drawAxes() {
  glLineWidth(2.5F);
  glBegin(GL_LINES);
  glColor3f(0.9F, 0.3F, 0.3F);
  glVertex3f(0, 0, 0);
  glVertex3f(400, 0, 0);  // North
  glColor3f(0.3F, 0.9F, 0.3F);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 400, 0);  // East
  glColor3f(0.3F, 0.5F, 1.0F);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 400);  // Up
  glEnd();
}

void TrackView3D::drawTrail(const QVector<TrackSample>& trail, float r, float g, float b, float width) {
  if (trail.size() < 2) {
    return;
  }
  glLineWidth(width);
  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < trail.size(); ++i) {
    const float a = 0.25F + 0.75F * float(i) / float(trail.size() - 1);
    glColor4f(r, g, b, a);
    const QVector3D p = nedToDisplay(trail[i].position_ned);
    glVertex3f(p.x(), p.y(), p.z());
  }
  glEnd();
}

void TrackView3D::drawMarker(const QVector3D& display_pos, float r, float g, float b, float size) {
  glPointSize(size);
  glBegin(GL_POINTS);
  glColor3f(r, g, b);
  glVertex3f(display_pos.x(), display_pos.y(), display_pos.z());
  glEnd();
}

void TrackView3D::drawLos() {
  if (!snap_.has_missile || !snap_.has_target) {
    return;
  }
  const QVector3D a = nedToDisplay(snap_.missile_pos_ned);
  const QVector3D b = nedToDisplay(snap_.target_pos_ned);
  glEnable(GL_LINE_STIPPLE);
  glLineStipple(1, 0x0F0F);
  glLineWidth(1.5F);
  glBegin(GL_LINES);
  glColor4f(0.85F, 0.85F, 0.4F, 0.8F);
  glVertex3f(a.x(), a.y(), a.z());
  glVertex3f(b.x(), b.y(), b.z());
  glEnd();
  glDisable(GL_LINE_STIPPLE);
}

void TrackView3D::mousePressEvent(QMouseEvent* event) {
  last_mouse_ = event->pos();
  dragging_ = (event->buttons() & Qt::LeftButton);
  panning_ = (event->buttons() & Qt::RightButton) || (event->buttons() & Qt::MiddleButton);
}

void TrackView3D::mouseMoveEvent(QMouseEvent* event) {
  const QPoint delta = event->pos() - last_mouse_;
  last_mouse_ = event->pos();
  if (dragging_) {
    yaw_deg_ += delta.x() * 0.35F;
    pitch_deg_ = std::clamp(pitch_deg_ - delta.y() * 0.25F, -89.0F, 89.0F);
    update();
  } else if (panning_) {
    const float scale = distance_m_ * 0.0015F;
    const float yaw = qDegreesToRadians(yaw_deg_);
    const QVector3D right(std::sin(yaw), -std::cos(yaw), 0.0F);
    const QVector3D up(0.0F, 0.0F, 1.0F);
    look_at_ -= right * (delta.x() * scale);
    look_at_ += up * (delta.y() * scale);
    follow_missile_ = false;
    update();
  }
}

void TrackView3D::wheelEvent(QWheelEvent* event) {
  const float steps = event->angleDelta().y() / 120.0F;
  distance_m_ = std::clamp(distance_m_ * std::pow(0.9F, steps), 50.0F, 50000.0F);
  update();
}
