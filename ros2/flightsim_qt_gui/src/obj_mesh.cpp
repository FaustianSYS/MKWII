#include "obj_mesh.hpp"

#include <QFile>
#include <QTextStream>
#include <QtMath>
#include <cmath>

namespace {

int parseFaceIndex(const QString& token) {
  const int slash = token.indexOf('/');
  const QString idx = slash >= 0 ? token.left(slash) : token;
  return idx.toInt() - 1;
}

}  // namespace

ObjMesh::~ObjMesh() = default;

bool ObjMesh::loadFromFile(const QString& path) {
  vertices_.clear();
  triangles_.clear();

  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }

  QTextStream stream(&file);
  while (!stream.atEnd()) {
    const QString line = stream.readLine().trimmed();
    if (line.isEmpty() || line.startsWith('#')) {
      continue;
    }
    if (line.startsWith(QStringLiteral("v "))) {
      const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 4) {
        vertices_.append(QVector3D(parts[1].toFloat(), parts[2].toFloat(), parts[3].toFloat()));
      }
      continue;
    }
    if (line.startsWith(QStringLiteral("f "))) {
      const QStringList parts = line.split(' ', Qt::SkipEmptyParts);
      if (parts.size() < 4) {
        continue;
      }
      const int i0 = parseFaceIndex(parts[1]);
      const int i1 = parseFaceIndex(parts[2]);
      const int i2 = parseFaceIndex(parts[3]);
      if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= vertices_.size() || i1 >= vertices_.size() ||
          i2 >= vertices_.size()) {
        continue;
      }
      triangles_.append(Triangle{i0, i1, i2});
      if (parts.size() >= 5) {
        const int i3 = parseFaceIndex(parts[4]);
        if (i3 >= 0 && i3 < vertices_.size()) {
          triangles_.append(Triangle{i0, i2, i3});
        }
      }
    }
  }

  return !vertices_.isEmpty() && !triangles_.isEmpty();
}

void ObjMesh::mirrorX() {
  for (auto& vertex : vertices_) {
    vertex.setX(-vertex.x());
  }
  for (auto& tri : triangles_) {
    std::swap(tri.i1, tri.i2);
  }
}

void ObjMesh::rotateXMinus90() {
  for (auto& vertex : vertices_) {
    const float y = vertex.y();
    const float z = vertex.z();
    vertex.setY(z);
    vertex.setZ(-y);
  }
}

void ObjMesh::compileDisplayList(QOpenGLFunctions_2_1* gl) {
  if (!gl || !isLoaded() || display_list_ != 0) {
    return;
  }

  display_list_ = gl->glGenLists(1);
  gl->glNewList(display_list_, GL_COMPILE);
  gl->glBegin(GL_TRIANGLES);
  for (const auto& tri : triangles_) {
    const QVector3D& a = vertices_[tri.i0];
    const QVector3D& b = vertices_[tri.i1];
    const QVector3D& c = vertices_[tri.i2];
    const QVector3D n = QVector3D::crossProduct(b - a, c - a);
    const float len = n.length();
    if (len > 1.0e-8F) {
      const QVector3D norm = n / len;
      gl->glNormal3f(norm.x(), norm.y(), norm.z());
    }
    gl->glVertex3f(a.x(), a.y(), a.z());
    gl->glVertex3f(b.x(), b.y(), b.z());
    gl->glVertex3f(c.x(), c.y(), c.z());
  }
  gl->glEnd();
  gl->glEndList();
}

void ObjMesh::draw(QOpenGLFunctions_2_1* gl) const {
  if (gl && display_list_ != 0) {
    gl->glCallList(display_list_);
  }
}

void ObjMesh::destroy(QOpenGLFunctions_2_1* gl) {
  if (gl && display_list_ != 0) {
    gl->glDeleteLists(display_list_, 1);
    display_list_ = 0;
  }
}
