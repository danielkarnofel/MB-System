#pragma once

#include "../data_types/geometry_data.h"

#include <cmath>

enum class SyntheticPointCloudShape {
  TiltedPlane,
  RippledSurface,
  Ridge,
  VerticalWall,
  Overhang,
};

struct SyntheticPointCloudOptions {
  SyntheticPointCloudShape shape;
  int width;
  int height;
  double spacing;
  double noise_stddev;
  double outlier_fraction;

  SyntheticPointCloudOptions()
      : shape(SyntheticPointCloudShape::RippledSurface),
        width(60),
        height(45),
        spacing(1.0),
        noise_stddev(0.0),
        outlier_fraction(0.0) {}
};

inline double synthetic_surface_z(SyntheticPointCloudShape shape, double x, double y) {
  switch (shape) {
  case SyntheticPointCloudShape::TiltedPlane:
    return 0.20 * x - 0.10 * y;
  case SyntheticPointCloudShape::Ridge:
    return 0.20 * x - 0.10 * y + 2.0 * std::exp(-0.05 * y * y);
  case SyntheticPointCloudShape::VerticalWall:
    return x < 0.0 ? -2.0 : 2.0;
  case SyntheticPointCloudShape::Overhang:
    return 0.20 * x + std::sin(y * 0.25);
  case SyntheticPointCloudShape::RippledSurface:
  default:
    return 0.20 * x - 0.10 * y + 0.25 * std::sin(x * 0.35) * std::cos(y * 0.25);
  }
}

inline PointCloud generate_synthetic_pointcloud(const SyntheticPointCloudOptions& options) {
  PointCloud point_cloud;
  point_cloud.points.reserve(options.width * options.height);

  for (int y = 0; y < options.height; y++) {
    for (int x = 0; x < options.width; x++) {
      const double px = (x - options.width / 2) * options.spacing;
      const double py = (y - options.height / 2) * options.spacing;
      const double pz = synthetic_surface_z(options.shape, px, py);
      point_cloud.points.push_back({px, py, pz});
    }
  }

  // Stub: noise and outlier controls are declared but not applied yet.
  (void)options.noise_stddev;
  (void)options.outlier_fraction;
  return point_cloud;
}

inline PointCloud generate_synthetic_pointcloud() {
  return generate_synthetic_pointcloud(SyntheticPointCloudOptions());
}
