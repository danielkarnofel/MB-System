#pragma once

#include <map>
#include <utility>

#include "../data_types/geometry.h"

enum class MeshGenerationMethod {
    ScreenedPoisson,
    Grid2D,
    Grid3D
};

Mesh generate_mesh(const PointCloud &pointcloud, MeshGenerationMethod method);

Mesh generate_mesh(const OrientedPointCloud &oriented_pointcloud, MeshGenerationMethod method);

// ====================================================================================================

struct ScreenedPoissonOptions {
    
};

Mesh generate_mesh_screened_poisson(const OrientedPointCloud &oriented_pointcloud, ScreenedPoissonOptions options);

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
