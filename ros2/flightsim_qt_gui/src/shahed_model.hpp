#pragma once

#include "obj_mesh.hpp"
#include "telemetry_bridge.hpp"

#include <QQuaternion>
#include <QString>

namespace shahed {

QString resolveModelPath();
bool prepareMesh(ObjMesh& mesh);
QQuaternion targetOrientation(const TelemetrySnapshot& snap);

}  // namespace shahed
