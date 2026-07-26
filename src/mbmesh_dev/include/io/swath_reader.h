#pragma once

#include <string>

#include "../data_types/swathfile.h"

struct SwathReaderOptions {
    CoordinateFrame frame;
    bool has_frame = false;
    GeographicBounds bounds;
    bool has_bounds = false;
    bool reject_flagged_beams = true;
};

struct SwathReaderStats {
    std::size_t records_read = 0;
    std::size_t soundings_read = 0;
    std::size_t soundings_accepted = 0;
    std::size_t soundings_flagged = 0;
    std::size_t soundings_out_of_bounds = 0;
};

[[nodiscard]] bool is_beam_flag_valid(char beam_flag);
[[nodiscard]] CollectedPoint make_collected_point(const Ping &ping,
                                                  const Sounding &sounding,
                                                  CoordinateFrame frame);

bool append_sounding_to_pointcloud(const Ping &ping,
                                   const Sounding &sounding,
                                   const SwathReaderOptions &options,
                                   CollectedPointCloud *pointcloud,
                                   SwathReaderStats *stats,
                                   std::string *error);

/*
 * Standalone development reader for text swath fixtures. Each non-comment line
 * is one sounding record:
 *
 * sensor_lon sensor_lat sensor_elev sounding_lon sounding_lat sounding_elev
 * beam_flag beam_index timestamp
 *
 * The MBIO-backed reader can feed the same Ping/Sounding conversion path after
 * it pulls real beams from MB-System buffers.
 */
bool read_swath_file(const std::string &path,
                     const SwathReaderOptions &options,
                     CollectedPointCloud *pointcloud,
                     SwathReaderStats *stats,
                     std::string *error);

SwathFile read_swath_file(const std::string &path);
