#pragma once

#include <QOpenGLFunctions_2_1>
#include <QVector3D>
#include <QString>
#include <QVector>

class ObjMesh {
 public:
  struct Triangle {
    int i0{0};
    int i1{0};
    int i2{0};
  };

  ObjMesh() = default;
  ~ObjMesh();

  ObjMesh(const ObjMesh&) = delete;
  ObjMesh& operator=(const ObjMesh&) = delete;

  bool loadFromFile(const QString& path);
  void mirrorX();
  void rotateXMinus90();
  void compileDisplayList(QOpenGLFunctions_2_1* gl);
  void draw(QOpenGLFunctions_2_1* gl) const;
  void destroy(QOpenGLFunctions_2_1* gl);

  bool isLoaded() const { return !vertices_.isEmpty(); }
  bool isCompiled() const { return display_list_ != 0; }

  const QVector<QVector3D>& vertices() const { return vertices_; }
  const QVector<Triangle>& triangles() const { return triangles_; }

 private:
  QVector<QVector3D> vertices_;
  QVector<Triangle> triangles_;
  unsigned int display_list_{0};
};
