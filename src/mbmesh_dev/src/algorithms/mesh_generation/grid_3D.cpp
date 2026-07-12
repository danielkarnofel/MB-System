#include "algorithms/mesh_generation.h"

#include <algorithm>
#include <cmath>

namespace {

Bounds3D pointcloud_bounds(const PointCloud &pointcloud) {
    Bounds3D bounds(pointcloud.points[0], pointcloud.points[0]);
    for (const Point &point : pointcloud.points) {
        bounds.include(point);
    }
    return bounds;
}

int grid_node_count(double extent, double cell_size) {
    return static_cast<int>(std::ceil(extent / cell_size)) + 1;
}

void mark_point_neighborhood(ScalarGrid3D &grid, const Point &point, double radius) {
    const int center_x = cell_index(point.x, grid.origin.x, grid.cell_size);
    const int center_y = cell_index(point.y, grid.origin.y, grid.cell_size);
    const int center_z = cell_index(point.z, grid.origin.z, grid.cell_size);
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(radius / grid.cell_size)));
    const double radius_squared = radius * radius;

    for (int z = center_z - radius_cells; z <= center_z + radius_cells; z++) {
        for (int y = center_y - radius_cells; y <= center_y + radius_cells; y++) {
            for (int x = center_x - radius_cells; x <= center_x + radius_cells; x++) {
                if (!grid.contains(x, y, z)) {
                    continue;
                }

                const Vec3 node_position = grid.position(x, y, z);
                if ((node_position - point).length_squared() <= radius_squared) {
                    grid.at(x, y, z) = -1.0;
                }
            }
        }
    }
}

PointCloud oriented_to_pointcloud(const OrientedPointCloud &oriented_pointcloud) {
    PointCloud pointcloud;
    pointcloud.points.reserve(oriented_pointcloud.oriented_points.size());
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        pointcloud.points.push_back(oriented_point.point);
    }
    return pointcloud;
}

} // namespace

ScalarGrid3D pointcloud_to_grid_3D(const PointCloud &pointcloud, const Grid3DOptions &options) {
    if (pointcloud.points.empty() || options.cell_size <= 0.0 || options.point_radius <= 0.0) {
        return ScalarGrid3D();
    }

    Bounds3D bounds = pointcloud_bounds(pointcloud);
    bounds.expand(options.padding);

    const Vec3 size = bounds.size();
    ScalarGrid3D grid(grid_node_count(size.x, options.cell_size),
                      grid_node_count(size.y, options.cell_size),
                      grid_node_count(size.z, options.cell_size),
                      options.cell_size,
                      bounds.min);

    std::fill(grid.values.begin(), grid.values.end(), 1.0);
    for (const Point &point : pointcloud.points) {
        mark_point_neighborhood(grid, point, options.point_radius);
    }

    return grid;
}

Mesh generate_mesh_grid_3D(const PointCloud &pointcloud, const Grid3DOptions &options) {
    const ScalarGrid3D scalar_grid = pointcloud_to_grid_3D(pointcloud, options);
    MarchingCubesOptions marching_cubes_options;
    marching_cubes_options.iso_value = options.iso_value;
    return marching_cubes(scalar_grid, marching_cubes_options);
}

Mesh generate_mesh_grid_3D(const OrientedPointCloud &oriented_pointcloud, const Grid3DOptions &options) {
    return generate_mesh_grid_3D(oriented_to_pointcloud(oriented_pointcloud), options);
}
