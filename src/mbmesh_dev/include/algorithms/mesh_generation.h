#pragma once

#include <map>
#include <utility>

#include "../data_types/geometry.h"

enum class MeshGenerationMethod {
    Poisson,
    Grid2D,
    Grid3D
};

Mesh generate_mesh(const PointCloud &pointcloud, MeshGenerationMethod method);

Mesh generate_mesh(const OrientedPointCloud &oriented_pointcloud, MeshGenerationMethod method);

// ====================================================================================================

struct ScreenedPoissonOptions {
    double cell_size = 0.1;
    double padding = 0.3;
    double normal_splat_radius = 0.15;
    double screening_weight = 4.0;
    int solver_iterations = 200;
    bool use_screening = true;
    bool estimate_iso_value_from_samples = true;
    double iso_value = 0.0;
};

ScalarGrid3D screened_poisson(const OrientedPointCloud &oriented_pointcloud, ScreenedPoissonOptions options);

struct MarchingCubesOptions {
    double iso_value = 0.0;
};

Mesh marching_cubes(const ScalarGrid3D &scalar_grid, MarchingCubesOptions options);

struct PoissonOptions {
    int normal_neighbors = 20;
    ScreenedPoissonOptions screened_poisson;
    MarchingCubesOptions marching_cubes;
};

// Generates a mesh from a point cloud using PCA normal estimation, Screened Poisson surface reconstruction, and marching cubes
Mesh generate_mesh_poisson(const PointCloud &pointcloud, PoissonOptions options);

Mesh generate_mesh_poisson(const OrientedPointCloud &oriented_pointcloud, PoissonOptions options);

// ====================================================================================================

struct Grid2DOptions {
    double cell_size;
    double max_edge_length;
    double min_triangle_area;
};

Mesh generate_mesh_grid_2D(const PointCloud &pointcloud, Grid2DOptions options);

Mesh generate_mesh_grid_2D(const OrientedPointCloud &oriented_pointcloud, Grid2DOptions options);

// ====================================================================================================

struct Grid3DOptions {
    double cell_size;
    double padding;
    double point_radius;
    double iso_value;
};

Mesh generate_mesh_grid_3D(const PointCloud &pointcloud, const Grid3DOptions &options);

Mesh generate_mesh_grid_3D(const OrientedPointCloud &oriented_pointcloud, const Grid3DOptions &options);

// ====================================================================================================
