#include "algorithms/normal_estimation.h"
#include "io/x3dom_writer.h"
#include "io/xyz_pointcloud_reader.h"
#include "synthetic/synthetic_pointcloud.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

namespace {

bool ensure_directory_exists(const char* path) {
  struct stat status;
  if (stat(path, &status) == 0) {
    return S_ISDIR(status.st_mode);
  }

  if (mkdir(path, 0755) == 0) {
    return true;
  }

  return errno == EEXIST;
}

std::string join_path(const std::string& directory, const char* filename) {
  if (directory.empty() || directory[directory.size() - 1] == '/') {
    return directory + filename;
  }
  return directory + "/" + filename;
}

void print_usage(const char* program_name) {
  std::fprintf(stderr,
               "usage: %s [input.xyz] [output_dir] [normal_k] [max_x3dom_points]\n"
               "       %s --synthetic [output_dir] [normal_k] [max_x3dom_points]\n"
               "\n"
               "If input.xyz is omitted, a synthetic tilted surface is used.\n"
               "Outputs:\n"
               "  pointcloud.html\n"
               "  oriented_pointcloud.html\n",
               program_name, program_name);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1 &&
      (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0)) {
    print_usage(argv[0]);
    return 0;
  }

  const bool use_synthetic =
      argc <= 1 || (argc > 1 && std::strcmp(argv[1], "--synthetic") == 0);
  const char* input_file = use_synthetic ? nullptr : argv[1];
  const std::string output_dir = argc > 2 ? argv[2] : "mbmesh_pipeline_output";
  const int normal_k = argc > 3 ? std::max(3, std::atoi(argv[3])) : 20;
  const std::size_t max_x3dom_points =
      argc > 4 ? static_cast<std::size_t>(std::max(1, std::atoi(argv[4])))
               : X3DomWriterOptions().max_points;

  if (!ensure_directory_exists(output_dir.c_str())) {
    std::fprintf(stderr, "Error: unable to create output directory: %s\n",
                 output_dir.c_str());
    return 1;
  }

  PointCloud point_cloud;
  std::string error_message;
  if (input_file != nullptr) {
    if (!read_xyz_pointcloud_file(input_file, &point_cloud, &error_message)) {
      std::fprintf(stderr, "Error: %s\n", error_message.c_str());
      return 1;
    }
    std::fprintf(stderr, "Loaded %zu points from %s\n", point_cloud.points.size(),
                 input_file);
  } else {
    point_cloud = generate_synthetic_pointcloud();
    std::fprintf(stderr, "Generated %zu synthetic test points\n",
                 point_cloud.points.size());
  }

  X3DomWriterOptions point_options;
  point_options.title = "mbmesh experimental pipeline: point cloud";
  point_options.point_size = 3.0;
  point_options.max_points = max_x3dom_points;

  const std::string pointcloud_html = join_path(output_dir, "pointcloud.html");
  if (!write_pointcloud_x3dom_file(pointcloud_html.c_str(), point_cloud,
                                   point_options, &error_message)) {
    std::fprintf(stderr, "Error: %s\n", error_message.c_str());
    return 1;
  }

  OrientedPointCloud oriented_points = estimate_oriented_points_PCA(point_cloud, normal_k);

  X3DomWriterOptions oriented_options;
  oriented_options.title = "mbmesh experimental pipeline: oriented point cloud";
  oriented_options.point_size = 3.0;
  oriented_options.normal_scale = 2.0;
  oriented_options.max_points = max_x3dom_points;
  oriented_options.max_normal_vectors = 2000;

  const std::string oriented_html = join_path(output_dir, "oriented_pointcloud.html");
  if (!write_oriented_pointcloud_x3dom_file(oriented_html.c_str(), oriented_points,
                                            oriented_options, &error_message)) {
    std::fprintf(stderr, "Error: %s\n", error_message.c_str());
    return 1;
  }

  std::fprintf(stderr, "Estimated normals using k = %d\n", normal_k);
  std::fprintf(stderr, "Wrote %s\n", pointcloud_html.c_str());
  std::fprintf(stderr, "Wrote %s\n", oriented_html.c_str());
  return 0;
}
