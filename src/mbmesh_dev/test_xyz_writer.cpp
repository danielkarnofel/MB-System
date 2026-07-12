#include "io/xyz_reader.h"
#include "io/xyz_writer.h"

#include <cstddef>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

namespace {

bool close_enough(double actual, double expected) {
    return std::fabs(actual - expected) < 1.0e-9;
}

int fail(const char *message) {
    std::fprintf(stderr, "Error: %s\n", message);
    return 1;
}

bool same_point(const Point &actual, const Point &expected) {
    return close_enough(actual.x, expected.x) &&
           close_enough(actual.y, expected.y) &&
           close_enough(actual.z, expected.z);
}

} // namespace

int main() {
    PointCloud pointcloud;
    pointcloud.points.push_back(Point(1.0, 2.0, 3.0));
    pointcloud.points.push_back(Point(-4.5, 0.25, 9.75));
    pointcloud.points.push_back(Point(123456.789, -98765.4321, 0.000123));

    const std::string output_path = "/tmp/mbmesh_xyz_writer_roundtrip.xyz";
    std::string error_message;
    if (!write_xyz_pointcloud(pointcloud, output_path, &error_message)) {
        std::fprintf(stderr, "Error: %s\n", error_message.c_str());
        return 1;
    }

    PointCloud parsed_pointcloud;
    if (!read_xyz_file(output_path, &parsed_pointcloud, &error_message)) {
        std::fprintf(stderr, "Error: %s\n", error_message.c_str());
        return 1;
    }

    if (parsed_pointcloud.points.size() != pointcloud.points.size()) {
        return fail("round-trip point count does not match");
    }

    for (std::size_t i = 0; i < pointcloud.points.size(); i++) {
        if (!same_point(parsed_pointcloud.points[i], pointcloud.points[i])) {
            return fail("round-trip point value does not match");
        }
    }

    PointCloud invalid_pointcloud;
    invalid_pointcloud.points.push_back(Point(0.0, 0.0, 0.0));
    invalid_pointcloud.points.push_back(Point(std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0));

    error_message.clear();
    if (write_xyz_pointcloud(invalid_pointcloud, "/tmp/mbmesh_xyz_writer_invalid.xyz", &error_message)) {
        return fail("invalid point cloud unexpectedly wrote successfully");
    }

    if (error_message.find("index 1") == std::string::npos) {
        return fail("invalid point cloud did not report the failing point index");
    }

    error_message.clear();
    if (write_xyz_pointcloud(pointcloud, "", &error_message)) {
        return fail("empty output path unexpectedly wrote successfully");
    }

    std::printf("XYZ writer round-tripped %zu points and rejected invalid output\n", pointcloud.points.size());
    return 0;
}
