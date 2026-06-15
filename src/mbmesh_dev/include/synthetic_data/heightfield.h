#pragma once

#include "../data_types/geometry.h"

using HeightFunction = double (*)(double x, double y);

[[nodiscard]] double tilted_plane_height(double x, double y);

[[nodiscard]] double rippled_heightfield(double x, double y);

[[nodiscard]] double ridge_heightfield(double x, double y);

[[nodiscard]] double trench_heightfield(double x, double y);

[[nodiscard]] double mound_heightfield(double x, double y);

[[nodiscard]] double cliff_heightfield(double x, double y);

[[nodiscard]] double noisy_heightfield(double x, double y);

[[nodiscard]] PointCloud generate_heightfield_pointcloud(HeightFunction height,
                                                         double min_x,
                                                         double max_x,
                                                         double min_y,
                                                         double max_y,
                                                         double interval);

