
#pragma once

#include "../data_types/geometry.h"

#include <vector>

enum class NormalEstimationMethod {
    PCA,
};

[[nodiscard]] std::vector<OrientedPoint> estimate_normals_pca(const std::vector<Vec3> &points, int k = 20);

[[nodiscard]] std::vector<OrientedPoint> estimate_normals(const std::vector<Vec3> &points,
                                                          int k = 20,
                                                          NormalEstimationMethod method = NormalEstimationMethod::PCA);

[[nodiscard]] OrientedPointCloud estimate_oriented_points_pca(const PointCloud &pointcloud, int k = 20);

[[nodiscard]] OrientedPointCloud estimate_oriented_points(const PointCloud &pointcloud,
                                                          int k = 20,
                                                          NormalEstimationMethod method = NormalEstimationMethod::PCA);
