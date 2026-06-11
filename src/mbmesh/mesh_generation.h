#pragma once

#include "core/geometry_types.h"

#include <vector>

enum class MeshGenerationMethod {
    SwathTopology,
    XYDelaunay,
    AlphaShape,
    BallPivoting,
    Poisson,
};

inline Mesh generate_mesh_swath_topology(const PointCloud& point_cloud) {
    Mesh mesh;
    mesh.frame = point_cloud.frame;
    return mesh;
}

inline Mesh generate_mesh_xy_delaunay(const PointCloud& point_cloud) {
    Mesh mesh;
    mesh.frame = point_cloud.frame;
    return mesh;
}

inline Mesh generate_mesh_poisson(const OrientedPointCloud& oriented_points) {
    Mesh mesh;
    mesh.frame = oriented_points.frame;
    return mesh;
}

inline Mesh marching_cubes(const ScalarField& field) {
    Mesh mesh;
    mesh.frame = field.frame;
    return mesh;
}
