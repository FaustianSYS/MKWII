#include "shahed_model.hpp"

#include <QFile>
#include <QVector3D>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>

namespace shahed {

QString resolveModelPath() {
  try {
    const std::string share = ament_index_cpp::get_package_share_directory("flightsim_qt_gui");
    const QString installed = QString::fromStdString(share) + QStringLiteral("/models/shahed_136.obj");
    if (QFile::exists(installed)) {
      return installed;
    }
  } catch (const std::exception&) {
  }

  if (const char* root = std::getenv("FLIGHTSIM_ROOT")) {
    const QString dev = QString::fromUtf8(root) +
                        QStringLiteral("/assets/ue5/models/shahed_136/source/shahed_136.obj");
    if (QFile::exists(dev)) {
      return dev;
    }
  }

  return QString();
}

bool prepareMesh(ObjMesh& mesh) {
  const QString path = resolveModelPath();
  if (path.isEmpty() || !mesh.loadFromFile(path)) {
    return false;
  }
  mesh.mirrorX();
  mesh.rotateXMinus90();
  return true;
}

QQuaternion targetOrientation(const TelemetrySnapshot& snap) {
  if (snap.has_target_attitude) {
    return snap.target_attitude.normalized();
  }
  if (snap.target_vel_ned.lengthSquared() > 1.0F) {
    return QQuaternion::rotationTo(QVector3D(1.0F, 0.0F, 0.0F), snap.target_vel_ned.normalized());
  }
  return QQuaternion();
}

}  // namespace shahed
