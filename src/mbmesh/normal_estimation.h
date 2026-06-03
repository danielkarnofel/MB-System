#pragma once

#include "vec3.h"
#include "mat3.h"
#include "kd_tree.h"

#include <cstddef>
#include <vector>

struct OrientedPoint {
    Vec3 p;
    Vec3 n;

    // We save the lambda values from the eigendecomposition because they are useful for later processing
    double lambda0;
    double lambda1;
    double lambda2;
};

enum class NormalEstimationMethod {
    PCA,
};

// Function declarations
inline std::vector<OrientedPoint> estimate_normals_PCA(const std::vector<Vec3>& points, int k = 20);

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
