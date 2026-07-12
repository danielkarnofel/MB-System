#include "io/xyz_reader.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

bool close_enough(double actual, double expected) {
    return std::fabs(actual - expected) < 1.0e-9;
}

int fail(const char *message) {
    std::fprintf(stderr, "Error: %s\n", message);
    return 1;
}

} // namespace

int main() {
    const std::string valid_path = "/tmp/mbmesh_xyz_reader_valid.xyz";
    {
        std::ofstream output(valid_path);
        output << "# comment line\n";
        output << "1.0 2.0 3.0\n";
        output << "4.0,5.0,6.0\n";
        output << "\n";
        output << "-1.5 0.25 9.75 100.0\n";
    }

    PointCloud pointcloud;
    std::string error_message;
    if (!read_xyz_file(valid_path, &pointcloud, &error_message)) {
        std::fprintf(stderr, "Error: %s\n", error_message.c_str());
        return 1;
    }

    if (pointcloud.points.size() != 3) {
        return fail("expected three parsed XYZ points");
    }

    if (!close_enough(pointcloud.points[0].x, 1.0) ||
        !close_enough(pointcloud.points[0].y, 2.0) ||
        !close_enough(pointcloud.points[0].z, 3.0)) {
        return fail("first parsed point did not match expected values");
    }

    if (!close_enough(pointcloud.points[1].x, 4.0) ||
        !close_enough(pointcloud.points[1].y, 5.0) ||
        !close_enough(pointcloud.points[1].z, 6.0)) {
        return fail("comma-separated point did not match expected values");
    }

    const PointCloud convenience_pointcloud = read_xyz_file(valid_path);
    if (convenience_pointcloud.points.size() != pointcloud.points.size()) {
        return fail("convenience reader returned an unexpected point count");
    }

    const std::string invalid_path = "/tmp/mbmesh_xyz_reader_invalid.xyz";
    {
        std::ofstream output(invalid_path);
        output << "1.0 2.0\n";
    }

    PointCloud invalid_pointcloud;
    error_message.clear();
    if (read_xyz_file(invalid_path, &invalid_pointcloud, &error_message)) {
        return fail("invalid XYZ file unexpectedly parsed successfully");
    }

    if (error_message.find("line 1") == std::string::npos) {
        return fail("invalid XYZ file did not report the failing line number");
    }

    std::printf("XYZ reader parsed %zu points and rejected malformed input\n", pointcloud.points.size());
    return 0;
}
