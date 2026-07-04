
#pragma once

#include "../data_types/geometry.h"

constexpr double default_cell_size = 0.01;

PointCloud decimate_pointcloud_2d(const PointCloud &pointcloud, double size = default_cell_size);

OrientedPointCloud decimate_oriented_pointcloud_2d(const OrientedPointCloud &oriented_pointcloud, double size = default_cell_size);

PointCloud decimate_pointcloud_3d(const PointCloud &pointcloud, double size = default_cell_size);

OrientedPointCloud decimate_oriented_pointcloud_3d(const OrientedPointCloud &oriented_pointcloud, double size);
