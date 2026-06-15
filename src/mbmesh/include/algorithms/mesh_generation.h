
#pragma once

#include "../data_types/geometry.h"

enum class MeshGenerationMethod {
    SwathTopology,
    XYDelaunay,
    AlphaShape,
    BallPivoting,
    Poisson,
};

[[nodiscard]] inline Mesh generate_mesh(const OrientedPointCloud &oriented_pointcloud, MeshGenerationMethod method) {
    Mesh mesh;
    return mesh;
}
