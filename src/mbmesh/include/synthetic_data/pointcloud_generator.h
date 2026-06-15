
#pragma once

#include "../data_types/geometry.h"

using SdfFunction = double (*)(double x, double y, double z);

[[nodiscard]] PointCloud generate_synthetic_pointcloud(SdfFunction sdf,
                                                       const Vec3 &min_corner,
                                                       const Vec3 &max_corner,
                                                       double interval,
                                                       double surface_threshold);
