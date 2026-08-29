#include "telemetry_bridge.hpp"

#include <QTimer>
#include <cmath>

namespace {

float magnitude(const QVector3D& v) {
  return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
}

}  // namespace

TelemetryBridge::TelemetryBridge(QObject* parent) : QObject(parent) {
  node_ = std::make_shared<rclcpp::Node>("flightsim_qt_tactical_gui");

  auto sensor_qos = rclcpp::SensorDataQoS();
  missile_sub_ = node_->create_subscription<flightsim_msgs::msg::MissileState>(
      "/flightsim/missile_state", 10,
      [this](const flightsim_msgs::msg::MissileState::SharedPtr msg) { onMissile(msg); });
  target_sub_ = node_->create_subscription<flightsim_msgs::msg::TargetState>(
      "/flightsim/target_state", 10,
      [this](const flightsim_msgs::msg::TargetState::SharedPtr msg) { onTarget(msg); });
  status_sub_ = node_->create_subscription<flightsim_msgs::msg::EngagementStatus>(
      "/flightsim/engagement_status", 10,
      [this](const flightsim_msgs::msg::EngagementStatus::SharedPtr msg) { onStatus(msg); });
  scene_sub_ = node_->create_subscription<flightsim_msgs::msg::SceneState>(
      "/flightsim/scene_state", sensor_qos,
      [this](const flightsim_msgs::msg::SceneState::SharedPtr msg) { onScene(msg); });
  reinit_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/flightsim/reinitialize", 10);
}

TelemetryBridge::~TelemetryBridge() = default;

void TelemetryBridge::start() {
  auto* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &TelemetryBridge::onSpinTimer);
  timer->start(10);
}

void TelemetryBridge::clearTrails() {
  QMutexLocker lock(&mutex_);
  snap_.missile_trail.clear();
  snap_.target_trail.clear();
}

void TelemetryBridge::requestReinitialize() {
  {
    QMutexLocker lock(&mutex_);
    snap_.missile_trail.clear();
    snap_.target_trail.clear();
    last_complete_ = false;
    snap_.complete = false;
    snap_.intercept = false;
    snap_.step_count = 0;
  }
  std_msgs::msg::Empty msg;
  reinit_pub_->publish(msg);
  emit telemetryUpdated();
}

TelemetrySnapshot TelemetryBridge::snapshot() const {
  QMutexLocker lock(&mutex_);
  return snap_;
}

void TelemetryBridge::onSpinTimer() {
  if (rclcpp::ok() && node_) {
    rclcpp::spin_some(node_);
  }
}

void TelemetryBridge::pushTrail(QVector<TrackSample>& trail, const QVector3D& pos) {
  TrackSample sample;
  sample.position_ned = pos;
  trail.push_back(sample);
  while (trail.size() > kTrailLen) {
    trail.remove(0);
  }
}

void TelemetryBridge::onMissile(const flightsim_msgs::msg::MissileState::SharedPtr msg) {
  const QVector3D pos(msg->position_ned_m[0], msg->position_ned_m[1], msg->position_ned_m[2]);
  const QVector3D vel(msg->velocity_ned_mps[0], msg->velocity_ned_mps[1], msg->velocity_ned_mps[2]);
  const float rate = std::sqrt(msg->pitch_rate_rps * msg->pitch_rate_rps + msg->yaw_rate_rps * msg->yaw_rate_rps +
                               msg->roll_rate_rps * msg->roll_rate_rps);

  {
    QMutexLocker lock(&mutex_);
    snap_.has_missile = true;
    snap_.missile_pos_ned = pos;
    snap_.missile_vel_ned = vel;
    snap_.missile_speed_mps = magnitude(vel);
    snap_.thrust_n = msg->thrust_n;
    snap_.fin_pitch_deg = msg->fin_pitch_rad * 57.2957795F;
    snap_.fin_yaw_deg = msg->fin_yaw_rad * 57.2957795F;
    snap_.fin_roll_deg = msg->fin_roll_rad * 57.2957795F;
    snap_.rate_mag_rps = rate;
    snap_.seeker_locked = msg->seeker_locked;
    snap_.seeker_fov_rad = msg->seeker_fov_azimuth_rad > 0.05F ? msg->seeker_fov_azimuth_rad : 0.52F;
    snap_.missile_active = msg->active;
    snap_.missile_hit = msg->hit;
    snap_.missile_attitude = QQuaternion(msg->attitude_w, msg->attitude_x, msg->attitude_y, msg->attitude_z);
    snap_.has_missile_attitude = snap_.missile_attitude.length() > 1.0e-6F;
    pushTrail(snap_.missile_trail, pos);
  }
  emit telemetryUpdated();
}

void TelemetryBridge::onTarget(const flightsim_msgs::msg::TargetState::SharedPtr msg) {
  const QVector3D pos(msg->position_ned_m[0], msg->position_ned_m[1], msg->position_ned_m[2]);
  const QVector3D vel(msg->velocity_ned_mps[0], msg->velocity_ned_mps[1], msg->velocity_ned_mps[2]);
  {
    QMutexLocker lock(&mutex_);
    snap_.has_target = true;
    snap_.target_pos_ned = pos;
    snap_.target_vel_ned = vel;
    snap_.target_speed_mps = magnitude(vel);
    if (msg->attitude_wxyz.size() >= 4U) {
      snap_.target_attitude = QQuaternion(msg->attitude_wxyz[0], msg->attitude_wxyz[1], msg->attitude_wxyz[2],
                                          msg->attitude_wxyz[3]);
      snap_.has_target_attitude = snap_.target_attitude.length() > 1.0e-6F;
    } else {
      snap_.has_target_attitude = false;
    }
    pushTrail(snap_.target_trail, pos);
  }
  emit telemetryUpdated();
}

void TelemetryBridge::onStatus(const flightsim_msgs::msg::EngagementStatus::SharedPtr msg) {
  bool fire_complete = false;
  bool intercept = false;
  float miss = 0.0F;
  quint64 step = 0;
  {
    QMutexLocker lock(&mutex_);
    snap_.has_engagement = true;
    snap_.range_m = msg->range_m;
    snap_.miss_distance_m = msg->miss_distance_m;
    snap_.intercept = msg->intercept;
    snap_.complete = msg->complete;
    snap_.step_count = msg->step_count;
    if (msg->complete && !last_complete_) {
      fire_complete = true;
      intercept = msg->intercept;
      miss = msg->miss_distance_m;
      step = msg->step_count;
    }
    last_complete_ = msg->complete;
  }
  emit telemetryUpdated();
  if (fire_complete) {
    emit engagementCompleted(intercept, miss, step);
  }
}

void TelemetryBridge::onScene(const flightsim_msgs::msg::SceneState::SharedPtr msg) {
  QVector<EntitySample> entities;
  entities.reserve(static_cast<int>(msg->entities.size()));
  for (const auto& entity : msg->entities) {
    EntitySample sample;
    sample.id = QString::fromStdString(entity.id);
    sample.type = QString::fromStdString(entity.type);
    sample.position_ned = QVector3D(entity.position_ned_m[0], entity.position_ned_m[1], entity.position_ned_m[2]);
    entities.push_back(sample);
  }
  {
    QMutexLocker lock(&mutex_);
    snap_.entities = entities;
  }
  emit telemetryUpdated();
}
