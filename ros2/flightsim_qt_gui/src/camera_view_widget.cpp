#include "camera_view_widget.hpp"

#include "shahed_model.hpp"

#include <QLinearGradient>
#include <QPainter>
#include <QPaintEvent>
#include <QtMath>
#include <algorithm>
#include <cmath>

namespace {

constexpr int kSeekerTriangleStride = 8;

}  // namespace

CameraViewWidget::CameraViewWidget(Mode mode, QWidget* parent) : QWidget(parent), mode_(mode) {
  setFixedSize(260, 180);
  setAttribute(Qt::WA_OpaquePaintEvent, true);
}

void CameraViewWidget::setSnapshot(const TelemetrySnapshot& snap) {
  snap_ = snap;
  update();
}

void CameraViewWidget::ensureShahedMesh() {
  if (shahed_mesh_load_attempted_) {
    return;
  }
  shahed_mesh_load_attempted_ = true;
  shahed_mesh_ok_ = shahed::prepareMesh(shahed_mesh_);
}

QVector3D CameraViewWidget::safeForward(const QVector3D& vel, const QVector3D& fallback) const {
  if (vel.lengthSquared() > 1.0F) {
    return vel.normalized();
  }
  if (fallback.lengthSquared() > 1.0e-6F) {
    return fallback.normalized();
  }
  return QVector3D(1.0F, 0.0F, 0.0F);
}

QVector3D CameraViewWidget::seekerForward() const {
  if (snap_.has_missile_attitude) {
    return snap_.missile_attitude.rotatedVector(QVector3D(1.0F, 0.0F, 0.0F)).normalized();
  }
  return safeForward(snap_.missile_vel_ned,
                    snap_.has_target ? (snap_.target_pos_ned - snap_.missile_pos_ned) : QVector3D(1, 0, 0));
}

bool CameraViewWidget::projectPoint(const QVector3D& eye, const QVector3D& forward, const QVector3D& world_up,
                                    float fov_rad, const QVector3D& point_ned, QPointF* out_uv) const {
  QVector3D f = forward.normalized();
  QVector3D r = QVector3D::crossProduct(f, world_up);
  if (r.lengthSquared() < 1.0e-8F) {
    r = QVector3D::crossProduct(f, QVector3D(0.0F, 1.0F, 0.0F));
  }
  r.normalize();
  const QVector3D u = QVector3D::crossProduct(r, f).normalized();

  const QVector3D rel = point_ned - eye;
  const float depth = QVector3D::dotProduct(rel, f);
  if (depth < 2.0F) {
    return false;
  }
  const float x = QVector3D::dotProduct(rel, r) / depth;
  const float y = QVector3D::dotProduct(rel, u) / depth;
  const float half = std::tan(0.5F * fov_rad);
  if (half < 1.0e-4F) {
    return false;
  }
  *out_uv = QPointF(x / half, y / half);
  return true;
}

void CameraViewWidget::drawShahedSeeker(QPainter& painter, const QRectF& viewport, const QVector3D& eye,
                                        const QVector3D& forward, const QVector3D& world_up, float fov_rad,
                                        const std::function<QPointF(const QPointF&)>& to_pixel) {
  if (!snap_.has_target || !shahed_mesh_ok_) {
    return;
  }

  const QQuaternion attitude = shahed::targetOrientation(snap_);
  const QVector3D target_pos = snap_.target_pos_ned;
  const QVector3D light_dir = (forward + QVector3D(0.0F, 0.0F, -0.35F)).normalized();
  const QVector3D f_hat = forward.normalized();

  QVector<ProjectedTriangle> projected;
  projected.reserve(shahed_mesh_.triangles().size() / kSeekerTriangleStride + 16);

  const auto& vertices = shahed_mesh_.vertices();
  for (int i = 0; i < shahed_mesh_.triangles().size(); i += kSeekerTriangleStride) {
    const auto& tri = shahed_mesh_.triangles()[i];
    const QVector3D& a_body = vertices[tri.i0];
    const QVector3D& b_body = vertices[tri.i1];
    const QVector3D& c_body = vertices[tri.i2];

    const QVector3D a_ned = target_pos + (attitude.isNull() ? a_body : attitude.rotatedVector(a_body));
    const QVector3D b_ned = target_pos + (attitude.isNull() ? b_body : attitude.rotatedVector(b_body));
    const QVector3D c_ned = target_pos + (attitude.isNull() ? c_body : attitude.rotatedVector(c_body));

    const QVector3D normal_ned =
        QVector3D::crossProduct(b_ned - a_ned, c_ned - a_ned).normalized();
    const float shade = std::clamp(0.30F + 0.70F * std::max(0.0F, QVector3D::dotProduct(normal_ned, light_dir)),
                                   0.15F, 1.0F);

    QPolygonF polygon;
    float depth_sum = 0.0F;
    bool visible = true;
    for (const QVector3D& point_ned : {a_ned, b_ned, c_ned}) {
      QPointF uv;
      if (!projectPoint(eye, forward, world_up, fov_rad, point_ned, &uv)) {
        visible = false;
        break;
      }
      depth_sum += QVector3D::dotProduct(point_ned - eye, f_hat);
      polygon << to_pixel(uv);
    }
    if (!visible || depth_sum < 6.0F) {
      continue;
    }

    const int base = static_cast<int>(190 * shade);
    projected.push_back(
        ProjectedTriangle{depth_sum / 3.0F, polygon, QColor(base + 10, base / 2 + 40, 20, 235)});
  }

  std::sort(projected.begin(), projected.end(),
            [](const ProjectedTriangle& lhs, const ProjectedTriangle& rhs) { return lhs.depth > rhs.depth; });

  painter.setPen(Qt::NoPen);
  for (const auto& tri : projected) {
    painter.setBrush(tri.fill);
    painter.drawPolygon(tri.polygon);
  }
}

void CameraViewWidget::paintEvent(QPaintEvent* /*event*/) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const QRectF frame = rect().adjusted(1, 1, -1, -1);
  const bool seeker = (mode_ == Mode::Seeker);
  const QColor accent = seeker ? QColor(64, 220, 240) : QColor(255, 160, 70);
  const QColor bg = seeker ? QColor(6, 14, 12) : QColor(10, 14, 22);

  p.fillRect(rect(), bg);
  p.setPen(QPen(accent.darker(140), 1.5));
  p.drawRect(frame);

  p.fillRect(QRectF(frame.left(), frame.top(), frame.width(), 22), QColor(0, 0, 0, 140));
  p.setPen(accent);
  QFont title = p.font();
  title.setBold(true);
  title.setPointSize(9);
  p.setFont(title);
  p.drawText(QRectF(frame.left() + 8, frame.top(), frame.width() - 16, 22), Qt::AlignVCenter | Qt::AlignLeft,
             seeker ? QStringLiteral("SEEKER VIEW") : QStringLiteral("DRONE VIEW"));

  const QRectF viewport(frame.left() + 6, frame.top() + 26, frame.width() - 12, frame.height() - 32);
  p.setClipRect(viewport);
  p.fillRect(viewport, seeker ? QColor(4, 18, 14) : QColor(8, 16, 28));

  const QVector3D world_up(0.0F, 0.0F, -1.0F);

  QVector3D eye;
  QVector3D forward;
  float fov = 0.70F;
  QVector3D primary;
  QVector3D secondary;
  bool have_primary = false;
  bool have_secondary = false;
  QColor primary_color;
  QColor secondary_color;

  if (seeker) {
    if (!snap_.has_missile) {
      p.setClipping(false);
      p.setPen(QColor(120, 140, 130));
      p.drawText(viewport, Qt::AlignCenter, QStringLiteral("NO MISSILE"));
      return;
    }
    eye = snap_.missile_pos_ned;
    forward = seekerForward();
    fov = snap_.seeker_fov_rad > 0.05F ? snap_.seeker_fov_rad : 0.52F;
    if (snap_.has_target) {
      primary = snap_.target_pos_ned;
      have_primary = true;
      primary_color = QColor(255, 170, 70);
    }
  } else {
    if (!snap_.has_target) {
      p.setClipping(false);
      p.setPen(QColor(140, 130, 120));
      p.drawText(viewport, Qt::AlignCenter, QStringLiteral("NO DRONE"));
      return;
    }
    eye = snap_.target_pos_ned;
    forward = safeForward(snap_.target_vel_ned, QVector3D(1, 0, 0));
    fov = 0.85F;
    if (snap_.has_missile) {
      primary = snap_.missile_pos_ned;
      have_primary = true;
      primary_color = QColor(64, 220, 240);
    }
    secondary = QVector3D(eye.x() + forward.x() * 80.0F, eye.y() + forward.y() * 80.0F, 0.0F);
    have_secondary = true;
    secondary_color = QColor(80, 120, 90);
  }

  {
    const float pitch = std::asin(std::clamp(-forward.z(), -1.0F, 1.0F));
    const float half = std::tan(0.5F * fov);
    const float v = (-std::tan(pitch)) / half;
    const float cy = viewport.center().y() - v * (viewport.height() * 0.5);
    QLinearGradient grad(viewport.topLeft(), viewport.bottomLeft());
    if (seeker) {
      grad.setColorAt(0.0, QColor(10, 30, 22));
      grad.setColorAt(1.0, QColor(4, 12, 10));
    } else {
      grad.setColorAt(0.0, QColor(18, 28, 48));
      const qreal band = std::clamp(static_cast<qreal>((cy - viewport.top()) / viewport.height()), 0.05, 0.95);
      grad.setColorAt(band, QColor(22, 36, 28));
      grad.setColorAt(1.0, QColor(28, 44, 30));
    }
    p.fillRect(viewport, grad);
    p.setPen(QPen(QColor(accent.red(), accent.green(), accent.blue(), 90), 1.0));
    p.drawLine(QPointF(viewport.left(), cy), QPointF(viewport.right(), cy));
  }

  auto to_pixel = [&](const QPointF& uv) {
    return QPointF(viewport.center().x() + uv.x() * viewport.width() * 0.5,
                   viewport.center().y() - uv.y() * viewport.height() * 0.5);
  };

  if (seeker && snap_.has_target) {
    ensureShahedMesh();
    drawShahedSeeker(p, viewport, eye, forward, world_up, fov, to_pixel);
  }

  if (have_secondary) {
    QPointF uv;
    if (projectPoint(eye, forward, world_up, fov, secondary, &uv) && std::abs(uv.x()) < 1.4 &&
        std::abs(uv.y()) < 1.4) {
      const QPointF px = to_pixel(uv);
      p.setPen(Qt::NoPen);
      p.setBrush(secondary_color);
      p.drawEllipse(px, 5.0, 3.0);
    }
  }

  if (have_primary) {
    QPointF uv;
    if (projectPoint(eye, forward, world_up, fov, primary, &uv)) {
      const bool in_fov = std::abs(uv.x()) <= 1.0 && std::abs(uv.y()) <= 1.0;
      const QPointF px = to_pixel(uv);
      if (!seeker && (in_fov || (std::abs(uv.x()) < 1.6 && std::abs(uv.y()) < 1.6))) {
        p.setPen(QPen(primary_color, 1.5));
        p.setBrush(QColor(primary_color.red(), primary_color.green(), primary_color.blue(), in_fov ? 210 : 90));
        p.drawEllipse(px, 6.0, 6.0);
      }
      if (seeker && in_fov && snap_.seeker_locked) {
        p.setBrush(Qt::NoBrush);
        p.setPen(QPen(QColor(80, 255, 140), 1.4));
        p.drawRect(QRectF(px.x() - 14, px.y() - 14, 28, 28));
      }
    } else if (seeker) {
      p.setPen(QColor(180, 80, 60));
      p.drawText(viewport.adjusted(8, 8, -8, -8), Qt::AlignTop | Qt::AlignRight, QStringLiteral("NO TRACK"));
    }
  }

  p.setClipping(false);
  p.setPen(QPen(accent, 1.2));
  const QPointF c = viewport.center();
  if (seeker) {
    p.drawLine(QPointF(c.x() - 18, c.y()), QPointF(c.x() - 6, c.y()));
    p.drawLine(QPointF(c.x() + 6, c.y()), QPointF(c.x() + 18, c.y()));
    p.drawLine(QPointF(c.x(), c.y() - 18), QPointF(c.x(), c.y() - 6));
    p.drawLine(QPointF(c.x(), c.y() + 6), QPointF(c.x(), c.y() + 18));
    p.drawRect(viewport.adjusted(10, 10, -10, -10));
    p.setPen(snap_.seeker_locked ? QColor(80, 255, 140) : QColor(200, 160, 60));
    p.drawText(viewport.adjusted(8, -2, -8, -6), Qt::AlignBottom | Qt::AlignLeft,
               snap_.seeker_locked ? QStringLiteral("LOCK") : QStringLiteral("SEARCH"));
  } else {
    p.drawEllipse(c, 10, 10);
    p.drawLine(QPointF(c.x() - 22, c.y()), QPointF(c.x() - 12, c.y()));
    p.drawLine(QPointF(c.x() + 12, c.y()), QPointF(c.x() + 22, c.y()));
  }

  p.setPen(QPen(accent.darker(140), 1.5));
  p.setBrush(Qt::NoBrush);
  p.drawRect(frame);
}
