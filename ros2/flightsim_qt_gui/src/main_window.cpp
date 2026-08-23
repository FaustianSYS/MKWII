#include "main_window.hpp"

#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVector3D>
#include <QVBoxLayout>

namespace {

void setMetricValue(QLabel* label, const QString& title, const QString& value, const QString& accent = QString()) {
  const QString color = accent.isEmpty() ? QStringLiteral("#e8eef7") : accent;
  label->setText(QStringLiteral("<div style='color:#8aa0b8;font-size:11px'>%1</div>"
                                "<div style='color:%2;font-size:18px;font-family:monospace'>%3</div>")
                     .arg(title, color, value));
}

QString fmt(float value, int digits = 1) {
  return QString::number(static_cast<double>(value), 'f', digits);
}

QString fmtNed(const QVector3D& ned) {
  return QStringLiteral("N %1  E %2  D %3")
      .arg(fmt(ned.x(), 1), fmt(ned.y(), 1), fmt(ned.z(), 1));
}

}  // namespace

MainWindow::MainWindow(TelemetryBridge* bridge, QWidget* parent)
    : QMainWindow(parent), bridge_(bridge) {
  setWindowTitle(QStringLiteral("FlightSim · Qt 3D Tactical GUI"));
  resize(1440, 900);
  setStyleSheet(QStringLiteral(
      "QMainWindow, QWidget { background: #101826; color: #e8eef7; }"
      "QFrame#panel {"
      "  background: #162033;"
      "  border: 1px solid #243247;"
      "  border-radius: 8px;"
      "}"
      "QLabel#header { font-size: 16px; font-weight: 600; }"
      "QTextEdit {"
      "  background: #0b1220;"
      "  border: 1px solid #243247;"
      "  color: #c9d6e5;"
      "  font-family: monospace;"
      "  font-size: 11px;"
      "}"
      "QPushButton {"
      "  background: #243247;"
      "  border: 1px solid #3a4d66;"
      "  border-radius: 4px;"
      "  padding: 6px 12px;"
      "}"
      "QPushButton:hover { background: #2e4058; }"
      "QCheckBox { spacing: 8px; }"));

  auto* root = new QWidget(this);
  setCentralWidget(root);
  auto* layout = new QHBoxLayout(root);
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(12);

  auto* left = new QFrame(root);
  left->setObjectName(QStringLiteral("panel"));
  left->setFixedWidth(380);
  auto* left_layout = new QVBoxLayout(left);

  auto* title = new QLabel(QStringLiteral("3D Track / Missile Test"), left);
  title->setObjectName(QStringLiteral("header"));
  left_layout->addWidget(title);

  auto* hint = new QLabel(
      QStringLiteral("LMB orbit · RMB pan · wheel zoom\nAxes: red=N green=E blue=Up"), left);
  hint->setStyleSheet(QStringLiteral("color:#8aa0b8;font-size:11px;"));
  left_layout->addWidget(hint);

  auto* grid = new QGridLayout();
  m_range_ = makeMetric(QStringLiteral("RANGE (m)"));
  m_miss_ = makeMetric(QStringLiteral("MISS (m)"));
  m_step_ = makeMetric(QStringLiteral("STEP"));
  m_status_ = makeMetric(QStringLiteral("STATUS"));
  m_mspeed_ = makeMetric(QStringLiteral("MISSILE SPEED (m/s)"));
  m_thrust_ = makeMetric(QStringLiteral("THRUST (N)"));
  m_fins_ = makeMetric(QStringLiteral("FINS P/Y/R (deg)"));
  m_rates_ = makeMetric(QStringLiteral("BODY RATE (rad/s)"));
  m_lock_ = makeMetric(QStringLiteral("SEEKER"));
  m_tspeed_ = makeMetric(QStringLiteral("TARGET SPEED (m/s)"));
  m_mpos_ = makeMetric(QStringLiteral("MISSILE NED (m)"));
  m_tpos_ = makeMetric(QStringLiteral("TARGET NED (m)"));

  const QList<QLabel*> metrics = {m_range_, m_miss_,  m_step_,  m_status_, m_mspeed_,
                                  m_thrust_, m_fins_, m_rates_, m_lock_,   m_tspeed_};
  for (int i = 0; i < metrics.size(); ++i) {
    grid->addWidget(metrics[i], i / 2, i % 2);
  }
  const int coord_row = (metrics.size() + 1) / 2;
  grid->addWidget(m_mpos_, coord_row, 0, 1, 2);
  grid->addWidget(m_tpos_, coord_row + 1, 0, 1, 2);
  left_layout->addLayout(grid);

  follow_check_ = new QCheckBox(QStringLiteral("Follow missile"), left);
  follow_check_->setChecked(true);
  left_layout->addWidget(follow_check_);

  auto* btn_row = new QHBoxLayout();
  auto* clear_btn = new QPushButton(QStringLiteral("Clear trails"), left);
  auto* reset_btn = new QPushButton(QStringLiteral("Reset camera"), left);
  auto* mark_btn = new QPushButton(QStringLiteral("Mark event"), left);
  btn_row->addWidget(clear_btn);
  btn_row->addWidget(reset_btn);
  btn_row->addWidget(mark_btn);
  left_layout->addLayout(btn_row);

  log_ = new QTextEdit(left);
  log_->setReadOnly(true);
  left_layout->addWidget(log_, 1);

  view3d_ = new TrackView3D(root);

  layout->addWidget(left);
  layout->addWidget(view3d_, 1);

  connect(bridge_, &TelemetryBridge::telemetryUpdated, this, &MainWindow::onTelemetryUpdated);
  connect(bridge_, &TelemetryBridge::engagementCompleted, this, &MainWindow::onEngagementCompleted);
  connect(clear_btn, &QPushButton::clicked, this, &MainWindow::onClearTrails);
  connect(reset_btn, &QPushButton::clicked, this, &MainWindow::onResetCamera);
  connect(mark_btn, &QPushButton::clicked, this, &MainWindow::onMarkEvent);
  connect(follow_check_, &QCheckBox::toggled, this, &MainWindow::onFollowToggled);

  appendLog(QStringLiteral("Listening on /flightsim/* truth topics"));
}

QLabel* MainWindow::makeMetric(const QString& title) {
  auto* label = new QLabel(this);
  label->setTextFormat(Qt::RichText);
  setMetricValue(label, title, QStringLiteral("—"));
  return label;
}

void MainWindow::appendLog(const QString& line) {
  const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss.zzz"));
  log_->append(QStringLiteral("[%1] %2").arg(stamp, line));
}

void MainWindow::onTelemetryUpdated() {
  const TelemetrySnapshot snap = bridge_->snapshot();
  view3d_->setSnapshot(snap);

  if (snap.has_engagement) {
    setMetricValue(m_range_, QStringLiteral("RANGE (m)"), fmt(snap.range_m));
    setMetricValue(m_miss_, QStringLiteral("MISS (m)"), fmt(snap.miss_distance_m));
    setMetricValue(m_step_, QStringLiteral("STEP"), QString::number(snap.step_count));
    QString status = QStringLiteral("RUNNING");
    QString accent;
    if (snap.complete) {
      status = snap.intercept ? QStringLiteral("INTERCEPT") : QStringLiteral("MISS");
      accent = snap.intercept ? QStringLiteral("#3dd68c") : QStringLiteral("#ff6b6b");
    }
    setMetricValue(m_status_, QStringLiteral("STATUS"), status, accent);
  }
  if (snap.has_missile) {
    setMetricValue(m_mspeed_, QStringLiteral("MISSILE SPEED (m/s)"), fmt(snap.missile_speed_mps));
    setMetricValue(m_thrust_, QStringLiteral("THRUST (N)"), fmt(snap.thrust_n, 0));
    setMetricValue(m_fins_, QStringLiteral("FINS P/Y/R (deg)"),
                   QStringLiteral("%1 / %2 / %3")
                       .arg(fmt(snap.fin_pitch_deg), fmt(snap.fin_yaw_deg), fmt(snap.fin_roll_deg)));
    setMetricValue(m_rates_, QStringLiteral("BODY RATE (rad/s)"), fmt(snap.rate_mag_rps, 3));
    const QString lock = snap.seeker_locked ? QStringLiteral("LOCKED") : QStringLiteral("SEARCH");
    setMetricValue(m_lock_, QStringLiteral("SEEKER"), lock,
                   snap.seeker_locked ? QStringLiteral("#3dd68c") : QStringLiteral("#e8b84a"));
    setMetricValue(m_mpos_, QStringLiteral("MISSILE NED (m)"), fmtNed(snap.missile_pos_ned),
                   QStringLiteral("#4ecdc4"));
  }
  if (snap.has_target) {
    setMetricValue(m_tspeed_, QStringLiteral("TARGET SPEED (m/s)"), fmt(snap.target_speed_mps));
    setMetricValue(m_tpos_, QStringLiteral("TARGET NED (m)"), fmtNed(snap.target_pos_ned),
                   QStringLiteral("#f0a05a"));
  }
}

void MainWindow::onEngagementCompleted(bool intercept, float miss_distance_m, quint64 step_count) {
  appendLog(QStringLiteral("Engagement complete: %1  miss=%2 m  steps=%3")
                .arg(intercept ? QStringLiteral("INTERCEPT") : QStringLiteral("MISS"),
                     fmt(miss_distance_m), QString::number(step_count)));
}

void MainWindow::onClearTrails() {
  bridge_->clearTrails();
  view3d_->setSnapshot(bridge_->snapshot());
  appendLog(QStringLiteral("Cleared track trails"));
}

void MainWindow::onResetCamera() {
  view3d_->resetCamera();
  appendLog(QStringLiteral("Camera reset"));
}

void MainWindow::onMarkEvent() {
  ++event_count_;
  const TelemetrySnapshot snap = bridge_->snapshot();
  appendLog(QStringLiteral("MARK #%1  range=%2  step=%3  M[%4]  T[%5]")
                .arg(event_count_)
                .arg(snap.has_engagement ? fmt(snap.range_m) : QStringLiteral("—"))
                .arg(snap.has_engagement ? QString::number(snap.step_count) : QStringLiteral("—"))
                .arg(snap.has_missile ? fmtNed(snap.missile_pos_ned) : QStringLiteral("—"))
                .arg(snap.has_target ? fmtNed(snap.target_pos_ned) : QStringLiteral("—")));
}

void MainWindow::onFollowToggled(bool checked) {
  view3d_->followMissile(checked);
}
