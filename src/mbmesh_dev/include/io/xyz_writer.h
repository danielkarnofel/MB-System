#pragma once

#include <string>

#include "../data_types/geometry.h"

[[nodiscard]] bool write_xyz_pointcloud(const PointCloud &pointcloud,
                                        const std::string &path,
                                        std::string *error_message = nullptr);
