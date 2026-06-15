#pragma once

#include "../data_types/geometry_data.h"

#include <string>

struct XyzPointCloudReaderOptions {
  CoordinateFrame frame;
  bool skip_comments;

  XyzPointCloudReaderOptions()
      : frame(), skip_comments(true) {}
};

bool read_xyz_pointcloud_file(const char* filename,
                              const XyzPointCloudReaderOptions& options,
                              PointCloud* point_cloud,
                              std::string* error_message);

inline bool read_xyz_pointcloud_file(const char* filename,
                                     PointCloud* point_cloud,
                                     std::string* error_message = nullptr) {
  return read_xyz_pointcloud_file(filename, XyzPointCloudReaderOptions(),
                                  point_cloud, error_message);
}
