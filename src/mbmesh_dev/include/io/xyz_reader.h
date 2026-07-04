#pragma once

#include <string>

#include "../data_types/geometry.h"

bool read_xyz_file(const std::string &path, PointCloud *pointcloud, std::string *error_message);

PointCloud read_xyz_file(const std::string &path);
