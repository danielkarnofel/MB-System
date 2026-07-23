#include "io/swath_reader.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

bool nearly_equal(double a, double b, double tolerance = 1.0e-6) {
    return std::fabs(a - b) <= tolerance;
}

bool write_fixture(const std::string &path) {
    std::ofstream output(path);
    if (!output) {
        return false;
    }

    output << "# sensor_lon sensor_lat sensor_elev sounding_lon sounding_lat sounding_elev beam_flag beam_index timestamp\n";
    output << "-122.0000 36.0000 5.0 -122.0000 36.0000 -50.0 0 0 1000.0\n";
    output << "-122.0000 36.0000 5.0 -121.9990 36.0010 -51.0 0 1 1000.0\n";
    output << "-122.0000 36.0000 5.0 -121.9980 36.0020 -52.0 1 2 1000.0\n";
    output << "-122.0000 36.0000 5.0 -121.9000 36.1000 -53.0 0 3 1000.0\n";
    return true;
}

} // namespace

int main() {
    const std::string fixture_path = "/tmp/mbmesh_dev_swath_fixture.txt";
    if (!write_fixture(fixture_path)) {
        std::fprintf(stderr, "Failed to write swath reader fixture\n");
        return 1;
    }

    SwathReaderOptions options;
    options.frame = coordinate_frame_from_origin(GeodeticPosition{-122.0, 36.0, 5.0});
    options.has_frame = true;
    options.has_bounds = true;
    options.bounds.degrees_W = -122.01;
    options.bounds.degrees_E = -121.99;
    options.bounds.degrees_S = 35.99;
    options.bounds.degrees_N = 36.01;

    CollectedPointCloud pointcloud;
    SwathReaderStats stats;
    std::string error;
    if (!read_swath_file(fixture_path, options, &pointcloud, &stats, &error)) {
        std::fprintf(stderr, "Failed to read swath fixture: %s\n", error.c_str());
        return 1;
    }

    if (stats.records_read != 4 ||
        stats.soundings_read != 4 ||
        stats.soundings_accepted != 2 ||
        stats.soundings_flagged != 1 ||
        stats.soundings_out_of_bounds != 1 ||
        pointcloud.collected_points.size() != 2) {
        std::fprintf(stderr,
                     "Unexpected swath stats: records=%zu read=%zu accepted=%zu flagged=%zu out_of_bounds=%zu points=%zu\n",
                     stats.records_read,
                     stats.soundings_read,
                     stats.soundings_accepted,
                     stats.soundings_flagged,
                     stats.soundings_out_of_bounds,
                     pointcloud.collected_points.size());
        return 1;
    }

    const CollectedPoint &first = pointcloud.collected_points[0];
    if (!nearly_equal(first.point.x, 0.0) ||
        !nearly_equal(first.point.y, 0.0) ||
        !nearly_equal(first.point.z, -55.0) ||
        !nearly_equal(first.origin.x, 0.0) ||
        !nearly_equal(first.origin.y, 0.0) ||
        !nearly_equal(first.origin.z, 0.0)) {
        std::fprintf(stderr, "First collected point/origin was not transformed as expected\n");
        return 1;
    }

    const Vec3 expected_second =
        geodetic_to_local(GeodeticPosition{-121.9990, 36.0010, -51.0}, options.frame);
    const CollectedPoint &second = pointcloud.collected_points[1];
    if (!nearly_equal(second.point.x, expected_second.x) ||
        !nearly_equal(second.point.y, expected_second.y) ||
        !nearly_equal(second.point.z, expected_second.z)) {
        std::fprintf(stderr, "Second collected point was not transformed as expected\n");
        return 1;
    }

    const SwathFile metadata = read_swath_file(fixture_path);
    if (metadata.path != fixture_path) {
        std::fprintf(stderr, "Swath metadata path was not retained\n");
        return 1;
    }

    const std::string malformed_path = "/tmp/mbmesh_dev_swath_malformed.txt";
    {
        std::ofstream malformed(malformed_path);
        malformed << "-122.0 36.0 5.0 -122.0 36.0\n";
    }

    CollectedPointCloud malformed_pointcloud;
    if (read_swath_file(malformed_path, options, &malformed_pointcloud, nullptr, &error)) {
        std::fprintf(stderr, "Malformed swath input unexpectedly succeeded\n");
        return 1;
    }

    std::printf("Swath reader converted %zu soundings and rejected flagged/out-of-bounds input\n",
                pointcloud.collected_points.size());
    return 0;
}
