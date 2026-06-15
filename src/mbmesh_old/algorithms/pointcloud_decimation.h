#pragma once

#include "../data_types/geometry_data.h"

#include <cstddef>

enum class PointCloudDecimationMethod {
  XYCellCenter,
  XYCellMean,
  XYCellMinZ,
  XYCellMaxZ,
};

struct PointCloudDecimationOptions {
  double xy_cell_size;
  PointCloudDecimationMethod method;
  bool preserve_frame;

  PointCloudDecimationOptions()
      : xy_cell_size(0.01),
        method(PointCloudDecimationMethod::XYCellCenter),
        preserve_frame(true) {}
};

struct PointCloudDecimationStats {
  std::size_t input_points;
  std::size_t output_points;
  std::size_t occupied_cells;

  PointCloudDecimationStats()
      : input_points(0), output_points(0), occupied_cells(0) {}
};

inline PointCloud decimate_pointcloud_xy(const PointCloud& point_cloud,
                                         const PointCloudDecimationOptions& options,
                                         PointCloudDecimationStats* stats = nullptr) {
  PointCloud decimated;
  if (options.preserve_frame) {
    decimated.frame = point_cloud.frame;
  }

  // Stub: pass-through until the XY cell accumulator is implemented.
  decimated.points = point_cloud.points;

  if (stats != nullptr) {
    stats->input_points = point_cloud.points.size();
    stats->output_points = decimated.points.size();
    stats->occupied_cells = decimated.points.size();
  }

  return decimated;
}

inline PointCloud decimate_pointcloud_xy(const PointCloud& point_cloud,
                                         double xy_cell_size,
                                         PointCloudDecimationStats* stats = nullptr) {
  PointCloudDecimationOptions options;
  options.xy_cell_size = xy_cell_size;
  return decimate_pointcloud_xy(point_cloud, options, stats);
}
