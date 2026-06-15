#pragma once

#include "../core/vec3.h"

#include <vector>

struct CoordinateFrame {
  Vec3 origin = {0.0, 0.0, 0.0};
  Vec3 scale = {1.0, 1.0, 1.0};
};

struct PointCloud {
  std::vector<Vec3> points;
  CoordinateFrame frame;
};

struct OrientedPoint {
  Vec3 position;
  Vec3 normal;
  double lambda0;
  double lambda1;
  double lambda2;

  OrientedPoint()
      : position{0.0, 0.0, 0.0}, normal{0.0, 0.0, 1.0},
        lambda0(0.0), lambda1(0.0), lambda2(0.0) {}

  OrientedPoint(const Vec3& point_position, const Vec3& point_normal,
                double point_lambda0 = 0.0, double point_lambda1 = 0.0,
                double point_lambda2 = 0.0)
      : position(point_position), normal(point_normal),
        lambda0(point_lambda0), lambda1(point_lambda1), lambda2(point_lambda2) {}
};

struct OrientedPointCloud {
  std::vector<OrientedPoint> points;
  CoordinateFrame frame;
};

struct Vertex {
  Vec3 position;
  Vec3 normal;
};

struct Mesh {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  CoordinateFrame frame;
};

struct ScalarField {
  unsigned int width = 0;
  unsigned int depth = 0;
  unsigned int height = 0;
  Vec3 origin = {0.0, 0.0, 0.0};
  Vec3 spacing = {1.0, 1.0, 1.0};
  std::vector<double> values;
  CoordinateFrame frame;
};
