#pragma once

#include "../data_types/geometry.h"
#include "heightfield.h"
#include "sdfs.h"

[[nodiscard]] PointCloud sample_heightfield_points(HeightFunction height,
                                                   double min_x,
                                                   double max_x,
                                                   double min_y,
                                                   double max_y,
                                                   double interval);

[[nodiscard]] OrientedPointCloud sample_oriented_heightfield_points(HeightFunction height,
                                                                    double min_x,
                                                                    double max_x,
                                                                    double min_y,
                                                                    double max_y,
                                                                    double interval);

[[nodiscard]] PointCloud sample_sdf_surface_points(SdfFunction sdf,
                                                   const Vec3 &min_corner,
                                                   const Vec3 &max_corner,
                                                   double interval,
                                                   double surface_threshold);

[[nodiscard]] OrientedPointCloud sample_oriented_sdf_surface_points(SdfFunction sdf,
                                                                    const Vec3 &min_corner,
                                                                    const Vec3 &max_corner,
                                                                    double interval,
                                                                    double surface_threshold);

