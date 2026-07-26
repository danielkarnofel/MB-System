#include "io/swath_reader.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr unsigned char mb_flag_flag = 0x01;

[[nodiscard]] bool is_finite(GeodeticPosition position) {
    return std::isfinite(position.longitude) &&
           std::isfinite(position.latitude) &&
           std::isfinite(position.elevation);
}

[[nodiscard]] bool is_valid_record(const Ping &ping, const Sounding &sounding) {
    return std::isfinite(ping.timestamp) &&
           is_finite(ping.sensor_position) &&
           is_finite(sounding.position);
}

void set_error(std::string *error, const std::string &message) {
    if (error != nullptr) {
        *error = message;
    }
}

[[nodiscard]] std::string trim_copy(const std::string &text) {
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }

    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

[[nodiscard]] bool parse_record_line(const std::string &line,
                                     Ping *ping,
                                     Sounding *sounding,
                                     std::string *error) {
    if (ping == nullptr || sounding == nullptr) {
        set_error(error, "internal error: null output record");
        return false;
    }

    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');

    int beam_flag = 0;
    std::size_t beam_index = 0;
    std::istringstream input(normalized);
    if (!(input >> ping->sensor_position.longitude >>
          ping->sensor_position.latitude >>
          ping->sensor_position.elevation >>
          sounding->position.longitude >>
          sounding->position.latitude >>
          sounding->position.elevation >>
          beam_flag >>
          beam_index >>
          ping->timestamp)) {
        set_error(error,
                  "expected 9 values: sensor_lon sensor_lat sensor_elev sounding_lon sounding_lat sounding_elev beam_flag beam_index timestamp");
        return false;
    }

    if (beam_flag < 0 || beam_flag > std::numeric_limits<unsigned char>::max()) {
        set_error(error, "beam_flag is outside the supported byte range");
        return false;
    }

    sounding->beam_flag = static_cast<char>(beam_flag);
    sounding->beam_index = beam_index;

    if (!is_valid_record(*ping, *sounding)) {
        set_error(error, "record contains a non-finite coordinate or timestamp");
        return false;
    }

    return true;
}

} // namespace

bool GeographicBounds::contains(double lon, double lat) const {
    const bool lat_in_bounds = lat >= degrees_S && lat <= degrees_N;
    const bool lon_in_bounds = degrees_W <= degrees_E
                                   ? lon >= degrees_W && lon <= degrees_E
                                   : lon >= degrees_W || lon <= degrees_E;
    return lat_in_bounds && lon_in_bounds;
}

CoordinateFrame coordinate_frame_from_origin(GeodeticPosition origin) {
    constexpr double earth_radius_meters = 6378137.0;
    constexpr double degrees_to_radians = pi / 180.0;

    CoordinateFrame frame;
    frame.origin = origin;
    frame.meters_per_degree_lat = earth_radius_meters * degrees_to_radians;
    frame.meters_per_degree_lon =
        earth_radius_meters * std::cos(origin.latitude * degrees_to_radians) * degrees_to_radians;
    return frame;
}

LocalPosition geodetic_to_local(GeodeticPosition position, CoordinateFrame frame) {
    return LocalPosition((position.longitude - frame.origin.longitude) * frame.meters_per_degree_lon,
                         (position.latitude - frame.origin.latitude) * frame.meters_per_degree_lat,
                         position.elevation - frame.origin.elevation);
}

bool is_beam_flag_valid(char beam_flag) {
    return (static_cast<unsigned char>(beam_flag) & mb_flag_flag) == 0;
}

CollectedPoint make_collected_point(const Ping &ping, const Sounding &sounding, CoordinateFrame frame) {
    CollectedPoint collected_point;
    collected_point.point = geodetic_to_local(sounding.position, frame);
    collected_point.origin = geodetic_to_local(ping.sensor_position, frame);
    return collected_point;
}

bool append_sounding_to_pointcloud(const Ping &ping,
                                   const Sounding &sounding,
                                   const SwathReaderOptions &options,
                                   CollectedPointCloud *pointcloud,
                                   SwathReaderStats *stats,
                                   std::string *error) {
    if (pointcloud == nullptr) {
        set_error(error, "pointcloud output must not be null");
        return false;
    }

    if (!is_valid_record(ping, sounding)) {
        set_error(error, "sounding record contains a non-finite coordinate or timestamp");
        return false;
    }

    if (stats != nullptr) {
        stats->soundings_read++;
    }

    if (options.reject_flagged_beams && !is_beam_flag_valid(sounding.beam_flag)) {
        if (stats != nullptr) {
            stats->soundings_flagged++;
        }
        return true;
    }

    if (options.has_bounds && !options.bounds.contains(sounding.position.longitude, sounding.position.latitude)) {
        if (stats != nullptr) {
            stats->soundings_out_of_bounds++;
        }
        return true;
    }

    CoordinateFrame frame = options.has_frame
                                ? options.frame
                                : coordinate_frame_from_origin(ping.sensor_position);
    pointcloud->collected_points.push_back(make_collected_point(ping, sounding, frame));

    if (stats != nullptr) {
        stats->soundings_accepted++;
    }

    return true;
}

bool read_swath_file(const std::string &path,
                     const SwathReaderOptions &options,
                     CollectedPointCloud *pointcloud,
                     SwathReaderStats *stats,
                     std::string *error) {
    if (pointcloud == nullptr) {
        set_error(error, "pointcloud output must not be null");
        return false;
    }

    std::ifstream input(path);
    if (!input) {
        set_error(error, "failed to open swath fixture file: " + path);
        return false;
    }

    SwathReaderOptions effective_options = options;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        line_number++;

        const std::string trimmed = trim_copy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        Ping ping;
        Sounding sounding;
        std::string parse_error;
        if (!parse_record_line(trimmed, &ping, &sounding, &parse_error)) {
            set_error(error, path + ":" + std::to_string(line_number) + ": " + parse_error);
            return false;
        }

        if (!effective_options.has_frame) {
            effective_options.frame = coordinate_frame_from_origin(ping.sensor_position);
            effective_options.has_frame = true;
        }

        if (stats != nullptr) {
            stats->records_read++;
        }

        if (!append_sounding_to_pointcloud(ping, sounding, effective_options, pointcloud, stats, error)) {
            return false;
        }
    }

    return true;
}

SwathFile read_swath_file(const std::string &path) {
    SwathFile swath_file;
    swath_file.path = path;
    return swath_file;
}
