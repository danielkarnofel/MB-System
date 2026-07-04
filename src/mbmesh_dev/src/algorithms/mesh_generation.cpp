#include "algorithms/mesh_generation.h"

namespace {

constexpr Grid2DOptions default_grid_2D_options{0.1, 5.0, 0.0};
constexpr Grid3DOptions default_grid_3D_options{0.5, 1.0, 0.5, 0.0};
constexpr ScreenedPoissonOptions default_screened_poisson_options{};

} // namespace

Mesh generate_mesh(const PointCloud &pointcloud, MeshGenerationMethod method) {
    switch (method) {
    case MeshGenerationMethod::Grid3D:
        return generate_mesh_grid_3D(pointcloud, default_grid_3D_options);
    case MeshGenerationMethod::Grid2D:
        return generate_mesh_grid_2D(pointcloud, default_grid_2D_options);
    default:
        return Mesh();
    }
}

Mesh generate_mesh(const OrientedPointCloud &oriented_pointcloud, MeshGenerationMethod method) {
    switch (method) {
    case MeshGenerationMethod::ScreenedPoisson:
        return generate_mesh_screened_poisson(oriented_pointcloud, default_screened_poisson_options);
    case MeshGenerationMethod::Grid2D:
        return generate_mesh_grid_2D(oriented_pointcloud, default_grid_2D_options);
    default:
        return Mesh();
    }
}
