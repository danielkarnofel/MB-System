#include "synthetic_data/sampling.h"

#include <cmath>

namespace {

constexpr double normal_step = 1e-3;

[[nodiscard]] Vec3 heightfield_normal(HeightFunction height, double x, double y) {
    const double dz_dx = (height(x + normal_step, y) - height(x - normal_step, y)) / (2.0 * normal_step);
    const double dz_dy = (height(x, y + normal_step) - height(x, y - normal_step)) / (2.0 * normal_step);
    return normalize(Vec3(-dz_dx, -dz_dy, 1.0));
}

[[nodiscard]] Vec3 sdf_normal(SdfFunction sdf, const Vec3 &p) {
    const double dx = sdf(p.x + normal_step, p.y, p.z) - sdf(p.x - normal_step, p.y, p.z);
    const double dy = sdf(p.x, p.y + normal_step, p.z) - sdf(p.x, p.y - normal_step, p.z);
    const double dz = sdf(p.x, p.y, p.z + normal_step) - sdf(p.x, p.y, p.z - normal_step);
    return normalize(Vec3(dx, dy, dz));
}

} // namespace

PointCloud sample_heightfield_points(HeightFunction height,
                                     double min_x,
                                     double max_x,
                                     double min_y,
                                     double max_y,
                                     double interval) {
    PointCloud pointcloud;
    if (height == nullptr || interval <= 0.0 || min_x > max_x || min_y > max_y) {
        return pointcloud;
    }

    for (double y = min_y; y <= max_y; y += interval) {
        for (double x = min_x; x <= max_x; x += interval) {
            pointcloud.points.push_back(Vec3(x, y, height(x, y)));
        }
    }

    return pointcloud;
}

OrientedPointCloud sample_oriented_heightfield_points(HeightFunction height,
                                                      double min_x,
                                                      double max_x,
                                                      double min_y,
                                                      double max_y,
                                                      double interval) {
    OrientedPointCloud pointcloud;
    if (height == nullptr || interval <= 0.0 || min_x > max_x || min_y > max_y) {
        return pointcloud;
    }

    for (double y = min_y; y <= max_y; y += interval) {
        for (double x = min_x; x <= max_x; x += interval) {
            pointcloud.oriented_points.push_back(OrientedPoint(Vec3(x, y, height(x, y)),
                                                               heightfield_normal(height, x, y)));
        }
    }

    return pointcloud;
}

PointCloud sample_sdf_surface_points(SdfFunction sdf,
                                     const Vec3 &min_corner,
                                     const Vec3 &max_corner,
                                     double interval,
                                     double surface_threshold) {
    PointCloud pointcloud;
    if (sdf == nullptr || interval <= 0.0 || surface_threshold < 0.0) {
        return pointcloud;
    }

    for (double z = min_corner.z; z <= max_corner.z; z += interval) {
        for (double y = min_corner.y; y <= max_corner.y; y += interval) {
            for (double x = min_corner.x; x <= max_corner.x; x += interval) {
                const Vec3 p(x, y, z);
                if (std::fabs(sdf(x, y, z)) <= surface_threshold) {
                    pointcloud.points.push_back(p);
                }
            }
        }
    }

    return pointcloud;
}

OrientedPointCloud sample_oriented_sdf_surface_points(SdfFunction sdf,
                                                      const Vec3 &min_corner,
                                                      const Vec3 &max_corner,
                                                      double interval,
                                                      double surface_threshold) {
    OrientedPointCloud pointcloud;
    if (sdf == nullptr || interval <= 0.0 || surface_threshold < 0.0) {
        return pointcloud;
    }

    for (double z = min_corner.z; z <= max_corner.z; z += interval) {
        for (double y = min_corner.y; y <= max_corner.y; y += interval) {
            for (double x = min_corner.x; x <= max_corner.x; x += interval) {
                const Vec3 p(x, y, z);
                if (std::fabs(sdf(x, y, z)) <= surface_threshold) {
                    pointcloud.oriented_points.push_back(OrientedPoint(p, sdf_normal(sdf, p)));
                }
            }
        }
    }

    return pointcloud;
}

