#pragma once

#include "camera_view_widget.hpp"
#include "telemetry_bridge.hpp"
#include "track_view_3d.hpp"

#include <QCheckBox>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>

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
  void onRerun();
  void onFollowToggled(bool checked);
  void onContinuousToggled(bool checked);
  void onContinuousRestart();

 private:
  void appendLog(const QString& line);
  QLabel* makeMetric(const QString& title);
  void triggerRerun(const QString& reason);

  TelemetryBridge* bridge_;
  TrackView3D* view3d_;
  CameraViewWidget* seeker_view_;
  CameraViewWidget* drone_view_;
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
  QPushButton* continuous_btn_;
  QTimer* continuous_timer_;
  int event_count_{0};
  int continuous_run_count_{0};
};
