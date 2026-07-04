#include "algorithms/normal_estimation.h"

#include "math/eigen.h"
#include "math/kdtree.h"
#include "math/mat3.h"

#include <cstddef>
#include <vector>

std::vector<OrientedPoint> estimate_normals_pca(const std::vector<Vec3> &points, int k) {
    std::vector<OrientedPoint> oriented_points;
    oriented_points.reserve(points.size());

    const std::size_t neighbor_count = k > 0 ? static_cast<std::size_t>(k) : 0;
    KDTree tree(points);

    for (std::size_t i = 0; i < points.size(); i++) {
        const std::vector<KDTree::Neighbor> neighbors = tree.k_nearest(points[i], neighbor_count, i);

        if (neighbors.size() < 3) {
            OrientedPoint oriented_point(points[i], Vec3(0.0, 0.0, 1.0));
            oriented_points.push_back(oriented_point);
            continue;
        }

        Vec3 local_center(0.0);
        for (std::size_t j = 0; j < neighbors.size(); j++) {
            local_center += points[neighbors[j].index];
        }
        local_center /= static_cast<double>(neighbors.size());

        Mat3 covariance;
        for (std::size_t j = 0; j < neighbors.size(); j++) {
            const Vec3 delta = points[neighbors[j].index] - local_center;
            covariance.add_outer_product(delta);
        }
        covariance /= static_cast<double>(neighbors.size());

        const EigenDecomposition eig = eigen_decomposition_symmetric(covariance);
        const int normal_index = eig.index_of_smallest_value();
        Vec3 normal = normalize(eig.vectors[normal_index]);

        if (normal.z < 0.0) {
            normal = -normal;
        }

        OrientedPoint oriented_point(points[i], normal);
        oriented_point.lambdas = eig.values;
        oriented_points.push_back(oriented_point);
    }

    return oriented_points;
}

std::vector<OrientedPoint> estimate_normals(const std::vector<Vec3> &points, int k, NormalEstimationMethod method) {
    switch (method) {
    case NormalEstimationMethod::PCA:
        return estimate_normals_pca(points, k);
        break;
    default:
        return estimate_normals_pca(points, k);
    }
}

OrientedPointCloud estimate_oriented_points_pca(const PointCloud &pointcloud, int k) {
    OrientedPointCloud oriented_pointcloud;
    oriented_pointcloud.oriented_points = estimate_normals_pca(pointcloud.points, k);
    return oriented_pointcloud;
}

OrientedPointCloud estimate_oriented_points(const PointCloud &pointcloud, int k, NormalEstimationMethod method) {
    switch (method) {
    case NormalEstimationMethod::PCA:
        return estimate_oriented_points_pca(pointcloud, k);
        break;
    default:
        return estimate_oriented_points_pca(pointcloud, k);
    }
}
