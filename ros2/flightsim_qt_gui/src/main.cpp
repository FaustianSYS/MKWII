#include "main_window.hpp"
#include "telemetry_bridge.hpp"

#include <QApplication>
#include <QSurfaceFormat>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  format.setVersion(2, 1);
  format.setProfile(QSurfaceFormat::CompatibilityProfile);
  format.setSamples(4);
  QSurfaceFormat::setDefaultFormat(format);

  QApplication app(argc, argv);
  QApplication::setApplicationName(QStringLiteral("FlightSim Qt Tactical GUI"));

  TelemetryBridge bridge;
  MainWindow window(&bridge);
  bridge.start();
  window.show();

  const int code = app.exec();
  rclcpp::shutdown();
  return code;
}
