#pragma once

#include "telemetry_bridge.hpp"
#include "track_view_3d.hpp"

#include <QCheckBox>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTextEdit>

class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  explicit MainWindow(TelemetryBridge* bridge, QWidget* parent = nullptr);

 private slots:
  void onTelemetryUpdated();
  void onEngagementCompleted(bool intercept, float miss_distance_m, quint64 step_count);
  void onClearTrails();
  void onResetCamera();
  void onMarkEvent();
  void onFollowToggled(bool checked);

 private:
  void appendLog(const QString& line);
  QLabel* makeMetric(const QString& title);

  TelemetryBridge* bridge_;
  TrackView3D* view3d_;
  QLabel* m_range_;
  QLabel* m_miss_;
  QLabel* m_step_;
  QLabel* m_status_;
  QLabel* m_mspeed_;
  QLabel* m_thrust_;
  QLabel* m_fins_;
  QLabel* m_rates_;
  QLabel* m_lock_;
  QLabel* m_tspeed_;
  QLabel* m_mpos_;
  QLabel* m_tpos_;
  QTextEdit* log_;
  QCheckBox* follow_check_;
  int event_count_{0};
};
