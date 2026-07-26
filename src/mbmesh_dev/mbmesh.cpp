#include <vector>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <string>
#include <utility>

#include "io/datalist_reader.h"
#include "io/glb_writer.h"
#include "io/x3dom_writer.h"

#include "data_types/swathfile.h"
#include "data_types/geometry.h"

#include "algorithms/point_decimation.h"
#include "algorithms/normal_estimation.h"
#include "algorithms/screened_poisson.h"
#include "algorithms/marching_cubes.h"
#include "algorithms/support_trimming.h"

struct Options {
    std::filesystem::path input_datalist;
    std::filesystem::path output_directory = "tileset";

    GeographicBounds bounds;
    bool has_bounds = false;

    bool write_x3dom = false;
    int verbose = 0;

    PointDecimationOptions decimation;
    NormalEstimationOptions normals;
    ScreenedPoissonOptions poisson;
    MarchingCubesOptions marching_cubes;
    SupportTrimmingOptions trimming;
};

Options parse_options(int argc, char **argv);

bool write_outputs(const Mesh &mesh, const Options &options, std::string *error);

int main(int argc, char **argv) {

    // Parse and validate configuration:
    Options options = parse_options(argc, argv);

    if (options.input_datalist.empty()) {
        std::cerr << "mbmesh: Input datalist path is empty\n";
        return 1;
    }

    // Read the datalist incrementally and collect valid soundings:
    DatalistReaderOptions reader_options;
    reader_options.bounds = options.bounds;
    reader_options.has_bounds = options.has_bounds;
    reader_options.verbose = options.verbose;

    DatalistReadResult read_result;
    std::string error;
    if (!read_datalist(
            options.input_datalist,
            reader_options,
            &read_result,
            &error)) {
        std::cerr << "mbmesh: " << error << '\n';
        return 1;
    }
    CollectedPointCloud collected_points = std::move(read_result.points);

    // ================================================================================

    // Estimate PCA normals and orient them using the sensor-origin vector:
    OrientedPointCloud oriented_points = normal_estimation(collected_points, {});
    
    // Reconstruct implicit surface using Screened Poisson:
    ScalarGrid3D poisson_surface = screened_poisson(oriented_points, {});

    // Extract isosurface using Marching Cubes:
    Mesh raw_mesh = marching_cubes(poisson_surface, {});

    // Post-process mesh using oriented-sample support trimming:
    Mesh clean_mesh = support_trimming(raw_mesh, oriented_points, {});

    // ================================================================================

    // Write desired output files:
    if (!write_outputs(clean_mesh, options, &error)) {
        std::cerr << "mbmesh: " << error << '\n';
        return 1;
    }

    return 0;
}

Options parse_options(int argc, char **argv) {
    Options options;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "-html") {
            argv[i] = const_cast<char*>("--html");
        }
    }

    static const struct option long_options[] = {
        {"input", required_argument, nullptr, 'I'},
        {"output", required_argument, nullptr, 'O'},
        {"bounds", required_argument, nullptr, 'R'},
        {"html", no_argument, nullptr, 'H'},
        {"verbose", no_argument, nullptr, 'V'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0},
    };

    int c = 0;
    int option_index = 0;
    while ((c = getopt_long(argc, argv, "I:O:R:VHh", long_options, &option_index)) != -1) {
        switch (c) {
        case 'I':
            options.input_datalist = optarg;
            break;
        case 'O':
            options.output_directory = optarg;
            break;
        case 'R': {
            double west = 0.0, east = 0.0, south = 0.0, north = 0.0;
            if (std::sscanf(optarg, "%lf/%lf/%lf/%lf", &west, &east, &south, &north) == 4) {
                options.bounds.degrees_W = west;
                options.bounds.degrees_E = east;
                options.bounds.degrees_S = south;
                options.bounds.degrees_N = north;
                options.has_bounds = true;
            } else {
                std::cerr << "mbmesh: invalid bounds argument: " << optarg << '\n';
            }
            break;
        }
        case 'V':
            ++options.verbose;
            break;
        case 'H':
            options.write_x3dom = true;
            break;
        case 'h':
            std::cout << "Usage: mbmesh [-I datalist] [-O outputdir] [-R w/e/s/n] [-html] [-V]\n";
            break;
        default:
            break;
        }
    }

    if (options.input_datalist.empty() && optind < argc) {
        options.input_datalist = argv[optind];
    }

    return options;
}

bool write_outputs(
    const Mesh &mesh,
    const Options &options,
    std::string *error)
{
    const auto glb_path = options.output_directory / "mesh.glb";

    if (!write_mesh_glb_file(glb_path, mesh, error))
        return false;

    if (options.write_x3dom) {
        return write_glb_x3dom_file(
            options.output_directory / "mesh.html",
            "mesh.glb",
            {},
            error);
    }

    return true;
}
