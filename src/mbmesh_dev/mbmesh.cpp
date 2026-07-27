#include <cstdio>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

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

    bool help_requested = false;
    int verbose = 0;

    std::filesystem::path input_datalist;
    std::filesystem::path output_directory = "mbmesh_output";

    GeographicBounds bounds;
    bool use_bounds = false;
    bool write_html = false;

    // Algorithm-specific defaults are documented with their option structs.
    PointDecimationOptions decimation;
    NormalEstimationOptions normals;
    ScreenedPoissonOptions poisson;
    MarchingCubesOptions marching_cubes;
    SupportTrimmingOptions trimming;
};

Options parse_options(int argc, char **argv);

bool write_outputs(const Mesh &mesh, const Options &options, std::string *error);

int main(int argc, char **argv) {

    Options options = parse_options(argc, argv);
    if (options.help_requested) {
        // TODO
        return 0;
    }

    if (options.input_datalist.empty()) {
        std::cerr << "mbmesh: input datalist path is empty; use -I <path>\n";
        return 1;
    }

    // Read the datalist incrementally and collect valid soundings:
    if (options.verbose > 0) {
        std::cerr << "mbmesh: Reading datalist: " << options.input_datalist << '\n';
    }

    DatalistReaderOptions reader_options;
    reader_options.bounds = options.bounds;
    reader_options.use_bounds = options.use_bounds;
    reader_options.verbose = options.verbose;

    DatalistReadResult read_result;
    std::string error;
    if (!read_datalist(options.input_datalist, reader_options, &read_result, &error)) {
        std::cerr << "mbmesh: " << error << '\n';
        return 1;
    }
    CollectedPointCloud collected_points = std::move(read_result.points);
    if (collected_points.empty()) {
        std::cerr << "mbmesh: input contains no accepted soundings\n";
        return 1;
    }

    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [input] completed: accepted "
            << collected_points.size()
            << " of " << read_result.stats.soundings_read
            << " soundings from " << read_result.stats.files_read
            << " files\n";
    }

    // ================================================================================

    // Decimate points to desired level of detail
    const std::size_t input_sample_count = collected_points.size();
    CollectedPointCloud decimated_points;
    if (options.decimation.decimate) {
        if (options.verbose > 0) {
            std::cerr
                << "mbmesh: [decimation] starting: cell_size="
                << options.decimation.cell_size << '\n';
        }
        decimated_points =
            point_decimation(std::move(collected_points), options.decimation);
    } else {
        if (options.verbose > 0) {
            std::cerr
                << "mbmesh: [decimation] disabled; using "
                << input_sample_count << " input samples\n";
        }
        decimated_points = std::move(collected_points);
    }
    if (decimated_points.empty()) {
        std::cerr << "mbmesh: point decimation produced no samples\n";
        return 1;
    }
    if (options.verbose > 0 && options.decimation.decimate) {
        std::cerr
            << "mbmesh: [decimation] completed: retained "
            << decimated_points.size() << " of " << input_sample_count
            << " samples\n";
    }

    // Estimate PCA normals and orient them using the sensor-origin vector:
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [normal estimation] starting: neighbors="
            << options.normals.k << '\n';
    }
    OrientedPointCloud oriented_points =
        normal_estimation(decimated_points, options.normals);
    if (oriented_points.empty()) {
        std::cerr << "mbmesh: normal estimation produced no oriented samples\n";
        return 1;
    }
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [normal estimation] completed: oriented "
            << oriented_points.size() << " samples\n";
    }
    
    // Reconstruct implicit surface using Screened Poisson:
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [screened Poisson] starting: cell_size="
            << options.poisson.cell_size
            << ", padding=" << options.poisson.padding
            << ", splat_radius=" << options.poisson.normal_splat_radius
            << ", iterations=" << options.poisson.solver_iterations
            << ", screening=";
        if (options.poisson.use_screening) {
            std::cerr << "enabled (weight="
                      << options.poisson.screening_weight << ")\n";
        } else {
            std::cerr << "disabled\n";
        }
    }
    ScalarGrid3D poisson_surface =
        screened_poisson(oriented_points, options.poisson);
    if (poisson_surface.values.empty()) {
        std::cerr << "mbmesh: screened Poisson reconstruction produced an empty field\n";
        return 1;
    }
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [screened Poisson] completed: grid="
            << poisson_surface.nx << 'x'
            << poisson_surface.ny << 'x'
            << poisson_surface.nz << '\n';
    }

    // Extract isosurface using Marching Cubes:
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [marching cubes] starting: iso_value="
            << options.marching_cubes.iso_value << '\n';
    }
    Mesh raw_mesh =
        marching_cubes(poisson_surface, options.marching_cubes);
    if (raw_mesh.vertices.empty() || raw_mesh.indices.empty()) {
        std::cerr << "mbmesh: marching cubes produced an empty mesh\n";
        return 1;
    }
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [marching cubes] completed: "
            << raw_mesh.vertices.size() << " vertices, "
            << raw_mesh.indices.size() / 3 << " triangles\n";
    }

    // Post-process mesh using oriented-sample support trimming:
    Mesh clean_mesh;
    SupportTrimmingDiagnostics trimming_diagnostics;
    if (options.trimming.enabled) {
        if (options.verbose > 0) {
            std::cerr
                << "mbmesh: [support trimming] starting: radius=";
            if (options.trimming.support_radius > 0.0) {
                std::cerr << options.trimming.support_radius;
            } else {
                std::cerr << "automatic";
            }
            std::cerr << ", normal_offset=";
            if (options.trimming.max_normal_offset > 0.0) {
                std::cerr << options.trimming.max_normal_offset;
            } else {
                std::cerr << "automatic";
            }
            std::cerr
                << ", minimum_neighbors="
                << options.trimming.minimum_neighbors
                << ", normal_alignment=";
            if (options.trimming.minimum_normal_alignment > 0.0) {
                std::cerr << options.trimming.minimum_normal_alignment
                          << '\n';
            } else {
                std::cerr << "disabled\n";
            }
        }
        clean_mesh =
            support_trimming(raw_mesh,
                             oriented_points,
                             options.trimming,
                             &trimming_diagnostics);
    } else {
        if (options.verbose > 0) {
            std::cerr
                << "mbmesh: [support trimming] disabled; using raw mesh\n";
        }
        clean_mesh = std::move(raw_mesh);
    }
    if (clean_mesh.vertices.empty() || clean_mesh.indices.empty()) {
        std::cerr
            << "mbmesh: support trimming removed the entire mesh; "
               "adjust the trimming thresholds or disable trimming\n";
        return 1;
    }
    if (options.verbose > 0 && options.trimming.enabled) {
        std::cerr
            << "mbmesh: [support trimming] completed: spacing="
            << trimming_diagnostics.estimated_point_spacing
            << ", resolved_radius="
            << trimming_diagnostics.resolved_support_radius
            << ", resolved_normal_offset="
            << trimming_diagnostics.resolved_max_normal_offset
            << ", supported_vertices="
            << trimming_diagnostics.supported_vertices
            << '/' << trimming_diagnostics.input_vertices
            << ", rejected(neighbors="
            << trimming_diagnostics.rejected_for_neighbors
            << ", offset="
            << trimming_diagnostics.rejected_for_normal_offset
            << ", alignment="
            << trimming_diagnostics.rejected_for_normal_alignment
            << "), output=" << clean_mesh.vertices.size()
            << " vertices, " << clean_mesh.indices.size() / 3
            << " triangles\n";
    }

    // ================================================================================

    // Write desired output files:
    if (options.verbose > 0) {
        std::cerr
            << "mbmesh: [output] starting: directory="
            << options.output_directory
            << ", glb=enabled, html="
            << (options.write_html ? "enabled" : "disabled")
            << '\n';
    }
    if (!write_outputs(clean_mesh, options, &error)) {
        std::cerr << "mbmesh: " << error << '\n';
        return 1;
    }
    if (options.verbose > 0) {
        std::cerr << "mbmesh: [output] completed\n";
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
                options.use_bounds = true;
            } else {
                std::cerr << "mbmesh: invalid bounds argument: " << optarg << '\n';
            }
            break;
        }
        case 'V':
            ++options.verbose;
            break;
        case 'H':
            options.write_html = true;
            break;
        case 'h':
            std::cout << "Usage: mbmesh [-I datalist] [-O outputdir] [-R w/e/s/n] [-html] [-V]\n";
            options.help_requested = true;
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
    if (options.output_directory.empty()) {
        if (error != nullptr) {
            *error = "Output directory path is empty";
        }
        return false;
    }

    std::error_code filesystem_error;
    std::filesystem::create_directories(
        options.output_directory,
        filesystem_error);
    if (filesystem_error) {
        if (error != nullptr) {
            *error =
                "Unable to create output directory '" +
                options.output_directory.string() +
                "': " + filesystem_error.message();
        }
        return false;
    }

    const auto glb_path = options.output_directory / "mesh.glb";

    if (!write_mesh_glb_file(glb_path, mesh, error)) {
        return false;
    }

    if (options.write_html) {
        return write_glb_x3dom_file(
            options.output_directory / "mesh.html",
            "mesh.glb",
            {},
            error);
    }

    return true;
}
