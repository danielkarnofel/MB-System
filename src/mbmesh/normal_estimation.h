#pragma once

#include "core/geometry_types.h"
#include "core/mat3.h"
#include "core/kd_tree.h"

#include <cstddef>
#include <vector>

enum class NormalEstimationMethod {
    PCA,
};

// Function declarations
inline std::vector<OrientedPoint> estimate_normals_PCA(const std::vector<Vec3>& points, int k = 20);
inline OrientedPointCloud estimate_oriented_points_PCA(const PointCloud& point_cloud, int k = 20);

// Driver function
inline std::vector<OrientedPoint> estimate_normals(const std::vector<Vec3>& points, NormalEstimationMethod method = NormalEstimationMethod::PCA) {
    std::vector<OrientedPoint> oriented_points;
    switch (method) {
        case NormalEstimationMethod::PCA:
            oriented_points = estimate_normals_PCA(points);
            break;
        default: 
            oriented_points = estimate_normals_PCA(points);
    }
    return oriented_points;
}

inline OrientedPointCloud estimate_oriented_points(const PointCloud& point_cloud, NormalEstimationMethod method = NormalEstimationMethod::PCA, int k = 20) {
    OrientedPointCloud oriented_points;
    oriented_points.frame = point_cloud.frame;

    switch (method) {
        case NormalEstimationMethod::PCA:
            oriented_points.points = estimate_normals_PCA(point_cloud.points, k);
            break;
        default:
            oriented_points.points = estimate_normals_PCA(point_cloud.points, k);
    }

    return oriented_points;
}

// Estimation method implementations:

inline std::vector<OrientedPoint> estimate_normals_PCA(const std::vector<Vec3>& points, int k) {
    std::vector<OrientedPoint> oriented_points;
    oriented_points.reserve(points.size());

    KDTree tree(points);

    for (std::size_t i = 0; i < points.size(); i++) {
        auto neighbors = tree.k_nearest(points[i], k, i);

        if (neighbors.size() < 3) {
            oriented_points.push_back({points[i], {0.0, 0.0, 1.0}, 0.0, 0.0, 0.0});
            continue;
        }

        // Calculate centroid
        Vec3 local_center = {0.0, 0.0, 0.0};
        for (const auto &neighbor : neighbors) {
            local_center = add(local_center, points[neighbor.index]);
        }
        local_center = multiply(local_center, 1.0 / neighbors.size());

        // Calculate covariance
        Mat3 covariance;
        for (const auto &neighbor : neighbors) {
            Vec3 d = subtract(points[neighbor.index], local_center);
            covariance.add_outer_product(d);
        }
        covariance = covariance * (1.0 / neighbors.size());

        // Eigen decomposition
        EigenDecomposition eig = covariance.eigen_decomposition_symmetric();
        const int normal_index = eig.index_of_smallest_value();
        Vec3 normal = normalize(eig.vectors[normal_index]);

        // Orient normals
        // Keep normals consistently upward for bathymetry-style point clouds.
        if (normal.z < 0.0) {
            normal = multiply(normal, -1.0);
        }
        
        oriented_points.push_back({
            points[i],
            normal,
            eig.values.x,
            eig.values.y,
            eig.values.z
        });
    }

    return oriented_points;
}

inline OrientedPointCloud estimate_oriented_points_PCA(const PointCloud& point_cloud, int k) {
    OrientedPointCloud oriented_points;
    oriented_points.frame = point_cloud.frame;
    oriented_points.points = estimate_normals_PCA(point_cloud.points, k);
    return oriented_points;
}
