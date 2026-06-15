#pragma once

#include "../data_types/geometry_data.h"

enum class MeshGenerationMethod {
  SwathTopology,
  XYDelaunay,
  AlphaShape,
  BallPivoting,
  Poisson,
};

inline Mesh generate_mesh_swath_topology(const PointCloud& point_cloud) {
  Mesh mesh;
  return mesh;
}

inline Mesh generate_mesh_xy_delaunay(const PointCloud& point_cloud) {
  Mesh mesh;
  return mesh;
}

inline Mesh generate_mesh_poisson(const OrientedPointCloud& oriented_point_cloud) {
  Mesh mesh;
  return mesh;
}

inline Mesh marching_cubes(const ScalarField& field) {
  Mesh mesh;
  return mesh;
}
