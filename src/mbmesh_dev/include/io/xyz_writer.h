#pragma once

#include <string>

#include "../data_types/geometry.h"

bool write_xyz_pointcloud(const PointCloud &pointcloud, const std::string &path);
