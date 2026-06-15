#include "xyz_pointcloud_reader.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace {

void set_error(std::string* error_message, const std::string& message) {
  if (error_message != nullptr) {
    *error_message = message;
  }
}

bool is_blank_or_comment(const std::string& line, bool skip_comments) {
  const std::size_t first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return true;
  }
  return skip_comments && line[first] == '#';
}

}  // namespace

bool read_xyz_pointcloud_file(const char* filename,
                              const XyzPointCloudReaderOptions& options,
                              PointCloud* point_cloud,
                              std::string* error_message) {
  if (error_message != nullptr) {
    error_message->clear();
  }

  if (filename == nullptr || filename[0] == '\0') {
    set_error(error_message, "XYZ point cloud filename is empty");
    return false;
  }

  if (point_cloud == nullptr) {
    set_error(error_message, "PointCloud output pointer is null");
    return false;
  }

  std::ifstream input(filename);
  if (!input) {
    set_error(error_message, std::string("Unable to open XYZ point cloud file: ") + filename);
    return false;
  }

  PointCloud parsed;
  parsed.frame = options.frame;

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    line_number++;
    if (is_blank_or_comment(line, options.skip_comments)) {
      continue;
    }

    std::istringstream stream(line);
    Vec3 point = {0.0, 0.0, 0.0};
    if (!(stream >> point.x >> point.y >> point.z)) {
      std::ostringstream message;
      message << "Invalid XYZ point at line " << line_number << " in " << filename;
      set_error(error_message, message.str());
      return false;
    }

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      std::ostringstream message;
      message << "Non-finite XYZ point at line " << line_number << " in " << filename;
      set_error(error_message, message.str());
      return false;
    }

    parsed.points.push_back(point);
  }

  if (parsed.points.empty()) {
    set_error(error_message, std::string("XYZ point cloud file contains no points: ") + filename);
    return false;
  }

  *point_cloud = parsed;
  return true;
}
