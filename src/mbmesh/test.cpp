#include "algorithms/normal_estimation.h"
#include "algorithms/decimation.h"
#include "io/x3dom_writer.h"
#include "synthetic_data/heightfield.h"
#include "data_types/geometry.h"

#include <cstdio>
#include <string>

int main() {
    const double interval = 0.25;
    const int normal_k = 20;

    const PointCloud pointcloud = generate_heightfield_pointcloud(noisy_heightfield,
                                                                  -5.0,
                                                                  5.0,
                                                                  -5.0,
                                                                  5.0,
                                                                  interval);

    const PointCloud decimated_pointcloud = decimate_pointcloud_2d(pointcloud, 0.5);

    const OrientedPointCloud oriented_pointcloud = estimate_oriented_points(pointcloud, normal_k);
    const char *visualization_path = "src/mbmesh/oriented_pointcloud.html";

    const OrientedPointCloud decimated_oriented_pointcloud = estimate_oriented_points(decimated_pointcloud, normal_k);
    const char *decimated_visualization_path = "src/mbmesh/decimated_oriented_pointcloud.html";

    std::printf("Generated %zu synthetic points\n", pointcloud.points.size());
    std::printf("Estimated %zu oriented points\n", oriented_pointcloud.oriented_points.size());

    std::printf("Generated %zu decimated points\n", decimated_pointcloud.points.size());
    std::printf("Estimated %zu decimated oriented points\n", decimated_oriented_pointcloud.oriented_points.size());

    if (!oriented_pointcloud.oriented_points.empty()) {
        const OrientedPoint &sample = oriented_pointcloud.oriented_points[0];
        std::printf("Sample normal: (%f, %f, %f)\n", sample.normal.x, sample.normal.y, sample.normal.z);
    }

    if (pointcloud.points.empty()) {
        std::fprintf(stderr, "Error: synthetic point cloud is empty\n");
        return 1;
    }

    if (oriented_pointcloud.oriented_points.size() != pointcloud.points.size()) {
        std::fprintf(stderr, "Error: oriented point count does not match point count\n");
        return 1;
    }

    if (!decimated_oriented_pointcloud.oriented_points.empty()) {
        const OrientedPoint &sample = decimated_oriented_pointcloud.oriented_points[0];
        std::printf("Sample normal: (%f, %f, %f)\n", sample.normal.x, sample.normal.y, sample.normal.z);
    }

    if (decimated_pointcloud.points.empty()) {
        std::fprintf(stderr, "Error: decimated point cloud is empty\n");
        return 1;
    }

    if (decimated_oriented_pointcloud.oriented_points.size() != decimated_pointcloud.points.size()) {
        std::fprintf(stderr, "Error: decimated oriented point count does not match decimated point count\n");
        return 1;
    }

    X3DomWriterOptions options;
    options.title = "mbmesh2 synthetic oriented point cloud";
    options.point_size = 3.0;
    options.normal_scale = 0.35;
    options.max_normal_vectors = 2000;

    std::string error_message;
    if (!write_oriented_pointcloud_x3dom_file(visualization_path, oriented_pointcloud, options, &error_message)) {
        std::fprintf(stderr, "Error: %s\n", error_message.c_str());
        return 1;
    }

    std::printf("Wrote %s\n", visualization_path);

    if (!write_oriented_pointcloud_x3dom_file(decimated_visualization_path, decimated_oriented_pointcloud, options, &error_message)) {
        std::fprintf(stderr, "Error: %s\n", error_message.c_str());
        return 1;
    }

    std::printf("Wrote %s\n", decimated_visualization_path);

    return 0;
}
