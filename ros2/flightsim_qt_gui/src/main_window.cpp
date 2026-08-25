#include "main_window.hpp"

#include <QDateTime>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSplitter>
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
      "QPushButton:checked {"
      "  background: #1f4d3a;"
      "  border: 1px solid #3dd68c;"
      "  color: #b8f5d4;"
      "}"
      "QCheckBox { spacing: 8px; }"
      "QSplitter::handle:horizontal {"
      "  background: #243247;"
      "  width: 5px;"
      "  margin: 0 2px;"
      "  border-radius: 2px;"
      "}"
      "QSplitter::handle:horizontal:hover { background: #3a4d66; }"
      "QScrollArea { border: none; background: transparent; }"));

  auto* root = new QWidget(this);
  setCentralWidget(root);
  auto* layout = new QHBoxLayout(root);
  layout->setContentsMargins(12, 12, 12, 12);
  layout->setSpacing(0);

  auto* splitter = new QSplitter(Qt::Horizontal, root);
  splitter->setObjectName(QStringLiteral("mainSplitter"));
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(6);

  auto* left_scroll = new QScrollArea(splitter);
  left_scroll->setWidgetResizable(true);
  left_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  left_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  left_scroll->setFrameShape(QFrame::NoFrame);
  left_scroll->setMinimumWidth(240);
  left_scroll->setMaximumWidth(720);
  left_scroll->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

  auto* left = new QFrame(left_scroll);
  left->setObjectName(QStringLiteral("panel"));
  left->setMinimumWidth(220);
  auto* left_layout = new QVBoxLayout(left);

  auto* title = new QLabel(QStringLiteral("3D Track / Missile Test"), left);
  title->setObjectName(QStringLiteral("header"));
  left_layout->addWidget(title);

  auto* hint = new QLabel(
      QStringLiteral("LMB orbit · RMB pan · wheel zoom\n"
                     "Ground: randomized curved 1 km × 1 km · Axes: red=N green=E blue=Up"),
      left);
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

  continuous_btn_ = new QPushButton(QStringLiteral("Continuous run"), left);
  continuous_btn_->setCheckable(true);
  continuous_btn_->setToolTip(
      QStringLiteral("When on, auto reset/initialize and rerun after each engagement completes"));
  left_layout->addWidget(continuous_btn_);

  continuous_timer_ = new QTimer(this);
  continuous_timer_->setSingleShot(true);
  continuous_timer_->setInterval(750);

  auto* btn_row = new QHBoxLayout();
  auto* rerun_btn = new QPushButton(QStringLiteral("Initialize / Rerun"), left);
  auto* clear_btn = new QPushButton(QStringLiteral("Clear trails"), left);
  auto* reset_btn = new QPushButton(QStringLiteral("Reset camera"), left);
  auto* mark_btn = new QPushButton(QStringLiteral("Mark event"), left);
  btn_row->addWidget(rerun_btn);
  btn_row->addWidget(clear_btn);
  left_layout->addLayout(btn_row);

  auto* btn_row2 = new QHBoxLayout();
  btn_row2->addWidget(reset_btn);
  btn_row2->addWidget(mark_btn);
  left_layout->addLayout(btn_row2);

  log_ = new QTextEdit(left);
  log_->setReadOnly(true);
  left_layout->addWidget(log_, 1);
  left_scroll->setWidget(left);

  auto* center = new QWidget(splitter);
  center->setMinimumWidth(400);
  center->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  auto* center_layout = new QGridLayout(center);
  center_layout->setContentsMargins(0, 0, 0, 0);
  center_layout->setSpacing(0);

  view3d_ = new TrackView3D(center);
  center_layout->addWidget(view3d_, 0, 0);

  auto* pip_host = new QWidget(center);
  pip_host->setAttribute(Qt::WA_TransparentForMouseEvents, false);
  auto* pip_layout = new QVBoxLayout(pip_host);
  pip_layout->setContentsMargins(0, 0, 0, 0);
  pip_layout->setSpacing(8);
  seeker_view_ = new CameraViewWidget(CameraViewWidget::Mode::Seeker, pip_host);
  drone_view_ = new CameraViewWidget(CameraViewWidget::Mode::Drone, pip_host);
  seeker_view_->setStyleSheet(QStringLiteral("background: transparent;"));
  drone_view_->setStyleSheet(QStringLiteral("background: transparent;"));
  pip_layout->addWidget(seeker_view_);
  pip_layout->addWidget(drone_view_);
  pip_layout->addStretch(1);
  center_layout->addWidget(pip_host, 0, 0, Qt::AlignTop | Qt::AlignRight);
  center_layout->setContentsMargins(0, 10, 10, 0);
  pip_host->raise();

  splitter->addWidget(left_scroll);
  splitter->addWidget(center);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);
  splitter->setSizes({380, 1060});

  layout->addWidget(splitter);

  connect(bridge_, &TelemetryBridge::telemetryUpdated, this, &MainWindow::onTelemetryUpdated);
  connect(bridge_, &TelemetryBridge::engagementCompleted, this, &MainWindow::onEngagementCompleted);
  connect(clear_btn, &QPushButton::clicked, this, &MainWindow::onClearTrails);
  connect(reset_btn, &QPushButton::clicked, this, &MainWindow::onResetCamera);
  connect(mark_btn, &QPushButton::clicked, this, &MainWindow::onMarkEvent);
  connect(rerun_btn, &QPushButton::clicked, this, &MainWindow::onRerun);
  connect(follow_check_, &QCheckBox::toggled, this, &MainWindow::onFollowToggled);
  connect(continuous_btn_, &QPushButton::toggled, this, &MainWindow::onContinuousToggled);
  connect(continuous_timer_, &QTimer::timeout, this, &MainWindow::onContinuousRestart);

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
  seeker_view_->setSnapshot(snap);
  drone_view_->setSnapshot(snap);

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
  if (continuous_btn_->isChecked()) {
    appendLog(QStringLiteral("Continuous run: restarting in 0.75 s…"));
    continuous_timer_->start();
  }
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

void MainWindow::triggerRerun(const QString& reason) {
  bridge_->requestReinitialize();
  view3d_->randomizeGround();
  view3d_->setSnapshot(bridge_->snapshot());
  appendLog(reason);
}

void MainWindow::onRerun() {
  continuous_timer_->stop();
  triggerRerun(QStringLiteral("Initialize / Rerun requested (new spawn + curved ground)"));
}

void MainWindow::onContinuousToggled(bool checked) {
  if (!checked) {
    continuous_timer_->stop();
    appendLog(QStringLiteral("Continuous run OFF"));
    return;
  }
  appendLog(QStringLiteral("Continuous run ON — will auto reset/initialize after each finish"));
  const TelemetrySnapshot snap = bridge_->snapshot();
  if (!snap.has_engagement || snap.complete) {
    continuous_timer_->start();
  }
}

void MainWindow::onContinuousRestart() {
  if (!continuous_btn_->isChecked()) {
    return;
  }
  ++continuous_run_count_;
  triggerRerun(QStringLiteral("Continuous run #%1 — reset + initialize").arg(continuous_run_count_));
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
