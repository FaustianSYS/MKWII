#include "track_view_3d.hpp"

#include "shahed_model.hpp"

#include <QFile>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QVector4D>
#include <QWheelEvent>
#include <QtMath>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

std::uint32_t lcg_next(std::uint32_t& state) {
  state = state * 1664525U + 1013904223U;
  return state;
}

float lcg_uniform(std::uint32_t& state, float lo, float hi) {
  const float u = static_cast<float>(lcg_next(state) & 0x00FFFFFFU) / static_cast<float>(0x00FFFFFFU);
  return lo + (hi - lo) * u;
}

}  // namespace

TrackView3D::TrackView3D(QWidget* parent) : QOpenGLWidget(parent) {
  setMinimumSize(640, 480);
  setFocusPolicy(Qt::StrongFocus);
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  regenerateTerrain(static_cast<std::uint32_t>(ticks) ^ 0xA5A5A5A5U);
}

TrackView3D::~TrackView3D() {
  makeCurrent();
  shahed_mesh_.destroy(this);
  doneCurrent();
}

void TrackView3D::setSnapshot(const TelemetrySnapshot& snap) {
  // New engagement run → reshuffle terrain curvature.
  if (snap.has_engagement && snap.step_count < last_step_count_) {
    randomizeGround();
  }
  if (snap.has_engagement) {
    last_step_count_ = snap.step_count;
  }
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

void TrackView3D::randomizeGround() {
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  regenerateTerrain(static_cast<std::uint32_t>(ticks) ^ (terrain_seed_ * 2654435761U));
  update();
}

void TrackView3D::regenerateTerrain(std::uint32_t seed) {
  if (seed == 0U) {
    seed = 1U;
  }
  terrain_seed_ = seed;
  std::uint32_t rng = seed;
  terrain_base_up_m_ = lcg_uniform(rng, -8.0F, 12.0F);

  // Layered sines → smooth randomized curved surface over the 1 km square.
  for (auto& wave : terrain_waves_) {
    wave.amp_m = lcg_uniform(rng, 4.0F, 28.0F);
    // Wavelength roughly 180–700 m across the play area.
    const float waves_n = lcg_uniform(rng, 1.2F, 5.5F);
    const float waves_e = lcg_uniform(rng, 1.2F, 5.5F);
    constexpr float kPi = 3.14159265F;
    wave.freq_n = (waves_n * 2.0F * kPi) / (2.0F * kGroundHalf_m);
    wave.freq_e = (waves_e * 2.0F * kPi) / (2.0F * kGroundHalf_m);
    wave.phase = lcg_uniform(rng, 0.0F, 2.0F * kPi);
    if (lcg_uniform(rng, 0.0F, 1.0F) < 0.35F) {
      wave.amp_m *= -1.0F;  // allow valleys
    }
  }
}

float TrackView3D::sampleGroundUp_m(float north_m, float east_m) const {
  float h = terrain_base_up_m_;
  for (const auto& wave : terrain_waves_) {
    h += wave.amp_m * std::sin(wave.freq_n * north_m + wave.freq_e * east_m + wave.phase);
  }
  // Soft falloff near edges so the border stays readable.
  const float nx = std::clamp(north_m / kGroundHalf_m, -1.0F, 1.0F);
  const float ey = std::clamp(east_m / kGroundHalf_m, -1.0F, 1.0F);
  const float edge = (1.0F - nx * nx) * (1.0F - ey * ey);
  return h * (0.35F + 0.65F * edge);
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
  ensureShahedMesh();
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

  const QMatrix4x4 mvp = projection * view;

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projection.constData());
  glMatrixMode(GL_MODELVIEW);
  glLoadMatrixf(view.constData());

  drawGroundPlane();
  drawGrid();
  drawAxes();
  drawTrail(snap_.target_trail, 0.95F, 0.55F, 0.20F, 2.0F);
  drawTrail(snap_.missile_trail, 0.25F, 0.85F, 1.0F, 2.5F);
  drawLos();

  if (snap_.has_target) {
    if (shahed_mesh_ok_) {
      drawShahedModel(snap_.target_pos_ned);
    } else {
      drawMarker(nedToDisplay(snap_.target_pos_ned), 1.0F, 0.45F, 0.15F, 18.0F);
    }
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

void TrackView3D::ensureShahedMesh() {
  if (shahed_mesh_load_attempted_) {
    return;
  }
  shahed_mesh_load_attempted_ = true;

  if (!shahed::prepareMesh(shahed_mesh_)) {
    shahed_mesh_ok_ = false;
    return;
  }
  shahed_mesh_.compileDisplayList(this);
  shahed_mesh_ok_ = shahed_mesh_.isCompiled();
}

QQuaternion TrackView3D::targetOrientation() const {
  return shahed::targetOrientation(snap_);
}

QMatrix4x4 TrackView3D::bodyToDisplayRotation(const QQuaternion& attitude_wxyz) const {
  QMatrix4x4 rotation;
  rotation.rotate(attitude_wxyz.normalized());
  // NED (+Z down) -> display (+Z up): proper rotation is S * R * S, not S * R (which mirrors).
  rotation(0, 2) = -rotation(0, 2);
  rotation(1, 2) = -rotation(1, 2);
  rotation(2, 0) = -rotation(2, 0);
  rotation(2, 1) = -rotation(2, 1);
  return rotation;
}

void TrackView3D::drawShahedModel(const QVector3D& pos_ned) {
  if (!shahed_mesh_ok_) {
    return;
  }

  const QVector3D origin = nedToDisplay(pos_ned);
  const QQuaternion attitude = targetOrientation();

  GLboolean lighting_was_enabled = GL_FALSE;
  glGetBooleanv(GL_LIGHTING, &lighting_was_enabled);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_NORMALIZE);

  const GLfloat light_pos[4] = {-0.35F, 0.55F, 0.75F, 0.0F};
  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  const GLfloat ambient[4] = {0.18F, 0.18F, 0.18F, 1.0F};
  const GLfloat diffuse[4] = {0.95F, 0.95F, 0.95F, 1.0F};
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);

  glPushMatrix();
  glTranslatef(origin.x(), origin.y(), origin.z());
  if (!attitude.isNull()) {
    glMultMatrixf(bodyToDisplayRotation(attitude).constData());
  }
  glColor4f(0.78F, 0.42F, 0.16F, 1.0F);
  shahed_mesh_.draw(this);
  glPopMatrix();

  if (!lighting_was_enabled) {
    glDisable(GL_LIGHTING);
  }
  glDisable(GL_COLOR_MATERIAL);
  glDisable(GL_NORMALIZE);
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

  auto nearestCorner = [](const QRectF& box, const QPointF& target) -> QPointF {
    const QPointF corners[4] = {
        box.topLeft(),
        box.topRight(),
        box.bottomLeft(),
        box.bottomRight(),
    };
    QPointF best = corners[0];
    qreal best_d = QLineF(best, target).length();
    for (int i = 1; i < 4; ++i) {
      const qreal d = QLineF(corners[i], target).length();
      if (d < best_d) {
        best_d = d;
        best = corners[i];
      }
    }
    return best;
  };

  auto drawLabel = [&](const QVector3D& ned, const QString& name, const QColor& color, const QPointF& offset) {
    const QPointF anchor = projectToScreen(nedToDisplay(ned), mvp);
    if (anchor.x() < 0.0) {
      return;
    }

    const QString text = QStringLiteral("%1\nN %2  E %3  D %4")
                             .arg(name)
                             .arg(ned.x(), 0, 'f', 1)
                             .arg(ned.y(), 0, 'f', 1)
                             .arg(ned.z(), 0, 'f', 1);

    constexpr qreal box_w = 210.0;
    constexpr qreal box_h = 40.0;
    QRectF box(anchor.x() + offset.x(), anchor.y() + offset.y(), box_w, box_h);

    // Keep the callout on-screen.
    box.moveLeft(std::clamp(box.left(), 8.0, static_cast<qreal>(width()) - box_w - 8.0));
    box.moveTop(std::clamp(box.top(), 8.0, static_cast<qreal>(height()) - box_h - 8.0));

    const QRectF padded = box.adjusted(-6, -4, 6, 4);
    const QPointF corner = nearestCorner(padded, anchor);

    // Leader line: box corner → object
    QPen leader(color, 1.4);
    leader.setCosmetic(true);
    painter.setPen(leader);
    painter.drawLine(corner, anchor);

    // Anchor tick on the object
    painter.setBrush(color);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(anchor, 3.5, 3.5);

    // Callout box
    painter.setBrush(QColor(8, 14, 22, 200));
    painter.setPen(QPen(color, 1.0));
    painter.drawRoundedRect(padded, 4, 4);
    painter.setPen(color);
    painter.drawText(box, Qt::AlignLeft | Qt::AlignVCenter, text);
  };

  // Offset boxes away from markers; different corners for missile vs target.
  if (snap_.has_missile) {
    drawLabel(snap_.missile_pos_ned, QStringLiteral("MISSILE"), QColor(64, 220, 240), QPointF(70.0, -78.0));
  }
  if (snap_.has_target) {
    drawLabel(snap_.target_pos_ned, QStringLiteral("TARGET"), QColor(255, 160, 70), QPointF(-250.0, 28.0));
  }
  painter.end();
}

void TrackView3D::drawGroundPlane() {
  // Randomized curved 1 km × 1 km heightfield (display Z = Up).
  constexpr float half = kGroundHalf_m;
  constexpr int res = kTerrainRes;
  const float step = (2.0F * half) / static_cast<float>(res);

  glDisable(GL_CULL_FACE);
  glBegin(GL_TRIANGLES);
  for (int i = 0; i < res; ++i) {
    for (int j = 0; j < res; ++j) {
      const float n0 = -half + static_cast<float>(i) * step;
      const float n1 = n0 + step;
      const float e0 = -half + static_cast<float>(j) * step;
      const float e1 = e0 + step;

      const float z00 = sampleGroundUp_m(n0, e0);
      const float z10 = sampleGroundUp_m(n1, e0);
      const float z11 = sampleGroundUp_m(n1, e1);
      const float z01 = sampleGroundUp_m(n0, e1);

      auto shade = [](float z) {
        const float t = std::clamp((z + 25.0F) / 60.0F, 0.0F, 1.0F);
        return QVector3D(0.10F + 0.08F * t, 0.18F + 0.22F * t, 0.12F + 0.10F * t);
      };

      const QVector3D c00 = shade(z00);
      const QVector3D c10 = shade(z10);
      const QVector3D c11 = shade(z11);
      const QVector3D c01 = shade(z01);

      glColor4f(c00.x(), c00.y(), c00.z(), 0.94F);
      glVertex3f(n0, e0, z00);
      glColor4f(c10.x(), c10.y(), c10.z(), 0.94F);
      glVertex3f(n1, e0, z10);
      glColor4f(c11.x(), c11.y(), c11.z(), 0.94F);
      glVertex3f(n1, e1, z11);

      glColor4f(c00.x(), c00.y(), c00.z(), 0.94F);
      glVertex3f(n0, e0, z00);
      glColor4f(c11.x(), c11.y(), c11.z(), 0.94F);
      glVertex3f(n1, e1, z11);
      glColor4f(c01.x(), c01.y(), c01.z(), 0.94F);
      glVertex3f(n0, e1, z01);
    }
  }
  glEnd();

  // Border outline following the curved edge
  glLineWidth(2.0F);
  glBegin(GL_LINE_LOOP);
  glColor4f(0.40F, 0.62F, 0.45F, 1.0F);
  const int edge = res;
  for (int i = 0; i <= edge; ++i) {
    const float n = -half + static_cast<float>(i) * step;
    glVertex3f(n, -half, sampleGroundUp_m(n, -half) + 0.4F);
  }
  for (int j = 1; j <= edge; ++j) {
    const float e = -half + static_cast<float>(j) * step;
    glVertex3f(half, e, sampleGroundUp_m(half, e) + 0.4F);
  }
  for (int i = edge - 1; i >= 0; --i) {
    const float n = -half + static_cast<float>(i) * step;
    glVertex3f(n, half, sampleGroundUp_m(n, half) + 0.4F);
  }
  for (int j = edge - 1; j >= 1; --j) {
    const float e = -half + static_cast<float>(j) * step;
    glVertex3f(-half, e, sampleGroundUp_m(-half, e) + 0.4F);
  }
  glEnd();
}

void TrackView3D::drawGrid() {
  // Contour-style grid draped on the curved ground
  constexpr float half = kGroundHalf_m;
  constexpr float step = 100.0F;
  constexpr int samples = 40;
  const float ds = (2.0F * half) / static_cast<float>(samples);

  glLineWidth(1.0F);
  glBegin(GL_LINES);
  glColor4f(0.24F, 0.40F, 0.30F, 0.80F);
  for (float x = -half; x <= half + 0.1F; x += step) {
    for (int k = 0; k < samples; ++k) {
      const float y0 = -half + static_cast<float>(k) * ds;
      const float y1 = y0 + ds;
      glVertex3f(x, y0, sampleGroundUp_m(x, y0) + 0.35F);
      glVertex3f(x, y1, sampleGroundUp_m(x, y1) + 0.35F);
    }
  }
  for (float y = -half; y <= half + 0.1F; y += step) {
    for (int k = 0; k < samples; ++k) {
      const float x0 = -half + static_cast<float>(k) * ds;
      const float x1 = x0 + ds;
      glVertex3f(x0, y, sampleGroundUp_m(x0, y) + 0.35F);
      glVertex3f(x1, y, sampleGroundUp_m(x1, y) + 0.35F);
    }
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
