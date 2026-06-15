#include "synthetic_data/pointcloud_generator.h"

#include <cmath>

PointCloud generate_synthetic_pointcloud(SdfFunction sdf,
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
                const double value = sdf(x, y, z);
                if (std::fabs(value) <= surface_threshold) {
                    pointcloud.points.push_back(Vec3(x, y, z));
                }
            }
        }
    }

    return pointcloud;
}
