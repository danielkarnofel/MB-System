
#pragma once

#include "../data_types/geometry.h"

constexpr double default_cell_size = 0.01; // In centimeters

[[nodiscard]] PointCloud decimate_pointcloud_2d(const PointCloud &pointcloud, double size = default_cell_size);

[[nodiscard]] inline PointCloud decimate_pointcloud_3d(const PointCloud &pointcloud, double size = default_cell_size) {
    PointCloud decimated_pointcloud = pointcloud;
    return decimated_pointcloud;
}

[[nodiscard]] inline OrientedPointCloud decimate_oriented_pointcloud_2d(const OrientedPointCloud &oriented_pointcloud, double size = default_cell_size) {
    OrientedPointCloud decimated_oriented_pointcloud = oriented_pointcloud;
    return decimated_oriented_pointcloud;
}

[[nodiscard]] inline OrientedPointCloud decimate_oriented_pointcloud_3d(const OrientedPointCloud &oriented_pointcloud, double size = default_cell_size) {
    OrientedPointCloud decimated_oriented_pointcloud = oriented_pointcloud;
    return decimated_oriented_pointcloud;
}
