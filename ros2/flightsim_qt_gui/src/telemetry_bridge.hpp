#pragma once

#include <QMutex>
#include <QObject>
#include <QVector3D>
#include <QVector>

#include <rclcpp/rclcpp.hpp>
#include <flightsim_msgs/msg/engagement_status.hpp>
#include <flightsim_msgs/msg/missile_state.hpp>
#include <flightsim_msgs/msg/scene_state.hpp>
#include <flightsim_msgs/msg/target_state.hpp>
#include <std_msgs/msg/empty.hpp>

struct TrackSample {
  QVector3D position_ned{};
};

struct EntitySample {
  QString id;
  QString type;
  QVector3D position_ned{};
};

struct TelemetrySnapshot {
  bool has_missile{false};
  bool has_target{false};
  bool has_engagement{false};

  QVector3D missile_pos_ned{};
  QVector3D missile_vel_ned{};
  float missile_speed_mps{0.0F};
  float thrust_n{0.0F};
  float fin_pitch_deg{0.0F};
  float fin_yaw_deg{0.0F};
  float fin_roll_deg{0.0F};
  float rate_mag_rps{0.0F};
  bool seeker_locked{false};
  float seeker_fov_rad{0.52F};
  bool missile_active{false};
  bool missile_hit{false};

  QVector3D target_pos_ned{};
  QVector3D target_vel_ned{};
  float target_speed_mps{0.0F};

  float range_m{0.0F};
  float miss_distance_m{0.0F};
  bool intercept{false};
  bool complete{false};
  quint64 step_count{0};

  QVector<TrackSample> missile_trail;
  QVector<TrackSample> target_trail;
  QVector<EntitySample> entities;
};

class TelemetryBridge : public QObject {
  Q_OBJECT
 public:
  explicit TelemetryBridge(QObject* parent = nullptr);
  ~TelemetryBridge() override;

  TelemetrySnapshot snapshot() const;

 public slots:
  void start();
  void clearTrails();
  void requestReinitialize();

 signals:
  void telemetryUpdated();
  void engagementCompleted(bool intercept, float miss_distance_m, quint64 step_count);

 private slots:
  void onSpinTimer();

 private:
  void onMissile(const flightsim_msgs::msg::MissileState::SharedPtr msg);
  void onTarget(const flightsim_msgs::msg::TargetState::SharedPtr msg);
  void onStatus(const flightsim_msgs::msg::EngagementStatus::SharedPtr msg);
  void onScene(const flightsim_msgs::msg::SceneState::SharedPtr msg);
  void pushTrail(QVector<TrackSample>& trail, const QVector3D& pos);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<flightsim_msgs::msg::MissileState>::SharedPtr missile_sub_;
  rclcpp::Subscription<flightsim_msgs::msg::TargetState>::SharedPtr target_sub_;
  rclcpp::Subscription<flightsim_msgs::msg::EngagementStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<flightsim_msgs::msg::SceneState>::SharedPtr scene_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reinit_pub_;

  mutable QMutex mutex_;
  TelemetrySnapshot snap_;
  bool last_complete_{false};
  static constexpr int kTrailLen = 800;
};
