#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "settings.h"

#include "io/xyz_writer.h"
#include "io/glb_writer.h"
#include "io/x3dom_writer.h"

#include "algorithms/point_decimation.h"
#include "algorithms/normal_estimation.h"
#include "algorithms/screened_poisson.h"
#include "algorithms/marching_cubes.h"
#include "algorithms/support_trimming.h"

// ========================================================================================================

Options parse_options(int argc, char **argv);

void print_usage();

bool write_outputs(
    const Mesh &mesh,
    const Mesh &raw_mesh,
    const PointCloud &points,
    const OrientedPointCloud &oriented_points,
    const CollectedPointCloud &collected_points,
    const CoordinateFrame &frame,
    const Options &options,
    std::string *error);

bool launch_html_viewer_server(
    const std::filesystem::path &directory,
    const std::string &html_filename);

// ========================================================================================================

int main(int argc, char **argv) {

    Options options = parse_options(argc, argv);

    if (options.help_requested) {
        print_usage();
        return 0;
    }

    if (options.input_datalist.empty()) {
        std::cerr << "mbmesh: input datalist path is empty; use -I <path>\n";
        return 1;
    }

    if (options.verbose) {
        std::cerr << "mbmesh: Reading datalist: " << options.input_datalist << '\n';
    }

    std::string error;
    PreprocessedDatalist preprocessed;
    if (!preprocess_datalist(options, &preprocessed, &error)) {
        std::cerr << "mbmesh: " << error << '\n';
        return 1;
    }

    if (options.verbose) {
        std::cerr << "mbmesh: [input] completed: accepted "
                  << preprocessed.read_result.points.size()
                  << " of " << preprocessed.read_result.stats.soundings_read
                  << " soundings from " << preprocessed.read_result.stats.files_read
                  << " files\n";
    }

    if (options.metadata_requested) {
        print_datalist_metadata(preprocessed, options);
        return 0;
    }

    CollectedPointCloud collected_points = std::move(preprocessed.read_result.points);

    // ====================================================================================================
    // Point Decimation
    // ====================================================================================================

    const std::size_t input_sample_count = collected_points.size();

    CollectedPointCloud decimated_points;

    if (options.decimation.decimate) {

        if (options.verbose) {
            std::cerr << "mbmesh: [decimation] starting: cell_size=" << options.decimation.cell_size << '\n';
        }

        decimated_points = point_decimation(std::move(collected_points), options.decimation);
        
        if (decimated_points.empty()) {
            std::cerr << "mbmesh: point decimation produced no samples\n";
            return 1;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [decimation] completed: retained " << decimated_points.size() << " of " << input_sample_count << " samples\n";
        }

    } else {

        if (options.verbose) {
            std::cerr << "mbmesh: [decimation] disabled; using " << input_sample_count << " input samples\n";
        }
        decimated_points = std::move(collected_points);
    }

    // ====================================================================================================
    // Normal Estimation
    // ====================================================================================================

    if (options.verbose) {
        const bool using_search_radius = options.normals.search_radius > 0.0;
        std::cerr << "mbmesh: [normal estimation] starting: "
                  << "search_radius=" << options.normals.search_radius
                  << ", radius_mode=" << (using_search_radius ? "enabled" : "disabled")
                  << ", k_nearest=" << options.normals.k
                  << ", fallback_min_neighbors=" << options.normals.minimum_neighbors
                  << ", fallback_to_knearest=" << (using_search_radius ? "when radius neighborhood < minimum_neighbors" : "always")
                  << '\n';
    }

    OrientedPointCloud oriented_points = normal_estimation(decimated_points, options.normals);

    if (oriented_points.empty()) {
        std::cerr << "mbmesh: normal estimation produced no oriented samples\n";
        return 1;
    }
    if (options.verbose) {
        std::cerr << "mbmesh: [normal estimation] completed: oriented " << oriented_points.size() << " samples\n";
    }

    // ====================================================================================================
    // Screened Poisson
    // ====================================================================================================

    if (options.verbose) {
        std::cerr << "mbmesh: [screened Poisson] starting... "
                  << "\n\tcell_size: " << options.poisson.cell_size
                  << "\n\tpadding: " << options.poisson.padding
                  << "\n\tsplat_radius: " << options.poisson.normal_splat_radius
                  << "\n\titerations: " << options.poisson.solver_iterations
                  << "\n\tscreening: " << (options.poisson.use_screening ? "enabled" : "disabled")
                  << "\n\tscreening_weight: " << options.poisson.screening_weight << "\n";
    }

    ScalarGrid3D poisson_surface = screened_poisson(oriented_points, options.poisson);

    if (poisson_surface.values.empty()) {
        std::cerr << "mbmesh: screened Poisson reconstruction produced an empty field\n";
        return 1;
    }

    if (options.verbose) {
        std::cerr << "mbmesh: [screened Poisson] completed: grid = " << poisson_surface.nx << 'x' << poisson_surface.ny << 'x' << poisson_surface.nz << '\n';
    }

    // ====================================================================================================
    // Marching Cubes
    // ====================================================================================================

    if (options.verbose) {
        std::cerr << "mbmesh: [marching cubes] starting: iso_value=" << options.marching_cubes.iso_value << '\n';
    }

    Mesh raw_mesh = marching_cubes(poisson_surface, options.marching_cubes);

    if (raw_mesh.vertices.empty() || raw_mesh.indices.empty()) {
        std::cerr << "mbmesh: marching cubes produced an empty mesh\n";
        return 1;
    }

    if (options.verbose) {
        std::cerr << "mbmesh: [marching cubes] completed: " << raw_mesh.vertices.size() << " vertices, " << raw_mesh.indices.size() / 3 << " triangles\n";
    }

    // ====================================================================================================
    // Support Trimming
    // ====================================================================================================

    Mesh clean_mesh;

    SupportTrimmingDiagnostics trimming_diagnostics;

    if (options.trimming.enabled) {

        if (options.verbose) {
            std::cerr << "mbmesh: [support trimming] starting... "
                      << "\n\tradius: " << ((options.trimming.support_radius > 0.0) ? std::to_string(options.trimming.support_radius) : "auto")
                      << "\n\tnormal_offset: " << ((options.trimming.max_normal_offset > 0.0) ? std::to_string(options.trimming.max_normal_offset) : "auto")
                      << "\n\tminimum_neighbors: " << options.trimming.minimum_neighbors
                      << "\n\tminimum_normal_alignment: " << ((options.trimming.minimum_normal_alignment > 0.0) ? std::to_string(options.trimming.minimum_normal_alignment) : "disabled");
        }
        clean_mesh = support_trimming(raw_mesh, oriented_points, options.trimming, &trimming_diagnostics);

    } else {

        if (options.verbose) {
            std::cerr << "mbmesh: [support trimming] disabled; using raw mesh\n";
        }
        clean_mesh = raw_mesh;
    }

    if (clean_mesh.vertices.empty() || clean_mesh.indices.empty()) {
        std::cerr << "mbmesh: support trimming removed the entire mesh. Adjust the trimming thresholds or disable trimming\n";
        return 1;
    }

    if (options.verbose && options.trimming.enabled) {
        std::cerr << "mbmesh: [support trimming] completed."
                  << "\n\tspacing: " << trimming_diagnostics.estimated_point_spacing
                  << "\n\tresolved_radius: " << trimming_diagnostics.resolved_support_radius
                  << "\n\tresolved_normal_offset: " << trimming_diagnostics.resolved_max_normal_offset
                  << "\n\tsupported_vertices: " << trimming_diagnostics.supported_vertices << '/' << trimming_diagnostics.input_vertices
                  << "\n\trejected: (neighbors = " << trimming_diagnostics.rejected_for_neighbors << ", offset = " << trimming_diagnostics.rejected_for_normal_offset << ", alignment = " << trimming_diagnostics.rejected_for_normal_alignment << ")"
                  << "\n\toutput: " << clean_mesh.vertices.size() << " vertices, " << clean_mesh.indices.size() / 3 << " triangles\n";
    }

    // ====================================================================================================
    // Write Outputs
    // ====================================================================================================

    // Write desired output files:
    if (options.verbose) {
        std::cerr << "mbmesh: [output] starting..."
                  << "\n\tdirectory: " << options.output_directory
                  << "\n\tglb: enabled"
                  << "\n\thtml: " << (options.write_html ? "enabled" : "disabled")
                  << "\n\tlocal_xyz: " << (options.write_local_xyz ? "enabled" : "disabled")
                  << "\n\tecef_xyz: " << (options.write_ecef_xyz ? "enabled" : "disabled")
                  << "\n\toriented_ply: " << (options.write_oriented_ply ? "enabled" : "disabled")
                  << "\n\tpointcloud_glb: " << (options.write_pointcloud_glb ? "enabled" : "disabled")
                  << "\n\tnormal_glb: " << (options.write_normal_glb ? "enabled" : "disabled")
                  << "\n\torigin_glb: " << (options.write_origin_glb ? "enabled" : "disabled")
                  << "\n\traw_mesh_glb: " << (options.write_raw_mesh_glb ? "enabled" : "disabled")
                  << '\n';
    }

    PointCloud output_points;
    output_points.reserve(decimated_points.size());
    for (const CollectedPoint &collected_point : decimated_points) {
        output_points.push_back(collected_point.point);
    }

    if (!write_outputs(clean_mesh, raw_mesh, output_points, oriented_points, decimated_points, preprocessed.read_result.frame, options, &error)) {
        std::cerr << "mbmesh: " << error << '\n';
        return 1;
    }

    if (options.verbose) {
        std::cerr << "mbmesh: [output] completed\n";
    }

    // ====================================================================================================
    // Launch Local Server for X3DOM Viewer
    // ====================================================================================================

    if (options.write_html && !launch_html_viewer_server(options.output_directory, "mesh.html")) {
        std::cerr << "mbmesh: warning: failed to auto-launch Python web server/viewer\n";
    }

    return 0;
}

// ========================================================================================================

Options parse_options(int argc, char **argv) {

    Options options;

    struct SingleDashLongOption {
        const char *single_dash;
        const char *double_dash;
    };

    static const SingleDashLongOption single_dash_long_options[] = {
        {"-html", "--html"},
        {"-local-xyz", "--local-xyz"},
        {"-ecef-xyz", "--ecef-xyz"},
        {"-xyz", "--xyz"},
        {"-oriented-ply", "--oriented-ply"},
        {"-pointcloud-glb", "--pointcloud-glb"},
        {"-normal-glb", "--normal-glb"},
        {"-origin-glb", "--origin-glb"},
        {"-raw-mesh-glb", "--raw-mesh-glb"},
        {"-diagnostics", "--diagnostics"},
        {"-all-outputs", "--all-outputs"},
    };

    for (int i = 1; i < argc; i++) {
        for (const SingleDashLongOption &option_alias : single_dash_long_options) {
            if (std::strcmp(argv[i], option_alias.single_dash) == 0) {
                argv[i] = const_cast<char *>(option_alias.double_dash);
                break;
            }
        }
    }

    static const struct option long_options[] = {
        {"input", required_argument, nullptr, 'I'},
        {"output", required_argument, nullptr, 'O'},
        {"bounds", required_argument, nullptr, 'R'},
        {"lod", required_argument, nullptr, 'L'},
        {"level-of-detail", required_argument, nullptr, 'L'},
        {"metadata", no_argument, nullptr, 1000},
        {"info", no_argument, nullptr, 1000},
        {"html", no_argument, nullptr, 1001},
        {"local-xyz", no_argument, nullptr, 1002},
        {"ecef-xyz", no_argument, nullptr, 1003},
        {"xyz", no_argument, nullptr, 1004},
        {"oriented-ply", no_argument, nullptr, 1005},
        {"pointcloud-glb", no_argument, nullptr, 1006},
        {"normal-glb", no_argument, nullptr, 1007},
        {"origin-glb", no_argument, nullptr, 1008},
        {"raw-mesh-glb", no_argument, nullptr, 1009},
        {"diagnostics", no_argument, nullptr, 1010},
        {"all-outputs", no_argument, nullptr, 1011},
        {"verbose", no_argument, nullptr, 'V'},
        {"help", no_argument, nullptr, 'H'},
        {nullptr, 0, nullptr, 0},
    };

    int c = 0;
    int option_index = 0;
    while ((c = getopt_long(argc, argv, "I:O:R:L:VHh", long_options, &option_index)) != -1) {

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

        case 'L':
            options.level_of_detail = std::strtod(optarg, nullptr);
            break;

        case 'V':
            options.verbose = true;
            break;

        case 1000:
            options.metadata_requested = true;
            break;

        case 1001:
            options.write_html = true;
            break;

        case 1002:
            options.write_local_xyz = true;
            break;

        case 1003:
            options.write_ecef_xyz = true;
            break;

        case 1004:
            options.write_local_xyz = true;
            options.write_ecef_xyz = true;
            break;

        case 1005:
            options.write_oriented_ply = true;
            break;

        case 1006:
            options.write_pointcloud_glb = true;
            break;

        case 1007:
            options.write_normal_glb = true;
            break;

        case 1008:
            options.write_origin_glb = true;
            break;

        case 1009:
            options.write_raw_mesh_glb = true;
            break;

        case 1010:
            options.write_pointcloud_glb = true;
            options.write_normal_glb = true;
            options.write_origin_glb = true;
            break;

        case 1011:
            options.write_html = true;
            options.write_local_xyz = true;
            options.write_ecef_xyz = true;
            options.write_oriented_ply = true;
            options.write_pointcloud_glb = true;
            options.write_normal_glb = true;
            options.write_origin_glb = true;
            options.write_raw_mesh_glb = true;
            break;

        case 'H':
        case 'h':
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

void print_usage() {
    std::cout << "mbmesh generates 3D meshes from swath sonar bathymetry data.\n\n";
    std::cout << "usage: mbmesh -I datalist [-R west/east/south/north] [-O outputdir] [-L meters] [output options] [-V]\n\n";
    std::cout << "Required:\n";
    std::cout << "  -I, --input <datalist>       Input MB-System datalist file\n\n";
    std::cout << "Optional:\n";
    std::cout << "  -O, --output <outputdir>     Output directory [output]\n";
    std::cout << "  -R, --bounds <w/e/s/n>       Geographic bounds in degrees\n";
    std::cout << "  -L, --lod <meters>           Requested smallest feature size [0.1]\n";
    std::cout << "      --metadata, --info       Print dataset metadata and exit\n";
    std::cout << "  -V, --verbose                Enable progress and diagnostic logging\n";
    std::cout << "  -H, -h, --help               Print this help message\n\n";
    std::cout << "Output options:\n";
    std::cout << "      --html                   Also write mesh.html and launch local X3DOM viewer\n";
    std::cout << "      --xyz                    Also write local and ECEF XYZ point clouds\n";
    std::cout << "      --local-xyz              Also write pointcloud-local.xyz\n";
    std::cout << "      --ecef-xyz               Also write pointcloud-ecef.xyz\n";
    std::cout << "      --oriented-ply           Also write oriented-pointcloud-ecef.ply\n";
    std::cout << "      --pointcloud-glb         Also write pointcloud.glb\n";
    std::cout << "      --normal-glb             Also write normals.glb\n";
    std::cout << "      --origin-glb             Also write origins.glb\n";
    std::cout << "      --raw-mesh-glb           Also write raw_mesh.glb before support trimming\n";
    std::cout << "      --diagnostics            Also write pointcloud, normal, and origin GLBs\n";
    std::cout << "      --all-outputs            Write all optional outputs\n\n";
    std::cout << "The default output is mesh.glb.\n";
    std::cout << "Single-dash long output options such as -html are accepted for legacy compatibility.\n";
}

namespace {

constexpr std::size_t maximum_diagnostic_glb_points = 2'000'000;

std::string shell_quote(const std::string &value) {
    std::string quoted = "'";
    for (const char character : value) {
        if (character == '\'') {
            quoted += "'\\''";
        } else {
            quoted += character;
        }
    }
    quoted += "'";
    return quoted;
}

} // namespace

// ========================================================================================================

bool write_outputs(
    const Mesh &mesh,
    const Mesh &raw_mesh,
    const PointCloud &points,
    const OrientedPointCloud &oriented_points,
    const CollectedPointCloud &collected_points,
    const CoordinateFrame &frame,
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
    std::filesystem::create_directories(options.output_directory, filesystem_error);

    if (filesystem_error) {
        if (error != nullptr) {
            *error = "Error creating output directory '" + options.output_directory.string() + "': " + filesystem_error.message();
        }
        return false;
    }

    if (options.write_ecef_xyz) {
        const auto ecef_xyz_path = options.output_directory / "pointcloud-ecef.xyz";
        if (!write_ecef_xyz_pointcloud(points, frame, ecef_xyz_path.string(), error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << ecef_xyz_path << '\n';
        }
    }

    if (options.write_local_xyz) {
        const auto local_xyz_path = options.output_directory / "pointcloud-local.xyz";
        if (!write_local_xyz_pointcloud(points, local_xyz_path.string(), error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << local_xyz_path << '\n';
        }
    }

    if (options.write_oriented_ply) {
        const auto oriented_ply_path = options.output_directory / "oriented-pointcloud-ecef.ply";
        if (!write_ply_oriented_pointcloud(oriented_points, frame, oriented_ply_path.string(), error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << oriented_ply_path << '\n';
        }
    }

    if (options.write_pointcloud_glb) {
        const auto pointcloud_glb_path = options.output_directory / "pointcloud.glb";
        if (!write_pointcloud_glb_file(pointcloud_glb_path, points, error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << pointcloud_glb_path << '\n';
        }
    }

    if (options.write_normal_glb) {
        const auto normals_glb_path = options.output_directory / "normals.glb";
        if (!write_normal_lines_glb_file(normals_glb_path, oriented_points, 0.25, error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << normals_glb_path << '\n';
        }
    }

    if (options.write_origin_glb) {
        const auto origins_glb_path = options.output_directory / "origins.glb";
        if (!write_origin_ray_lines_glb_file(origins_glb_path, collected_points, 0.25, error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << origins_glb_path << '\n';
        }
    }

    if (options.write_raw_mesh_glb) {
        const auto raw_mesh_glb_path = options.output_directory / "raw_mesh.glb";
        if (!write_mesh_glb_file(raw_mesh_glb_path, raw_mesh, error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << raw_mesh_glb_path << '\n';
        }
    }

    const auto clean_mesh_glb_path = options.output_directory / "mesh.glb";
    if (!write_mesh_glb_file(clean_mesh_glb_path, mesh, error)) {
        return false;
    }
    if (options.verbose) {
        std::cerr << "mbmesh: [output] wrote " << clean_mesh_glb_path << '\n';
    }

    if (options.write_html) {
        const auto html_path = options.output_directory / "mesh.html";
        if (!write_glb_x3dom_file(html_path, "mesh.glb", {}, error)) {
            return false;
        }
        if (options.verbose) {
            std::cerr << "mbmesh: [output] wrote " << html_path << '\n';
        }
    }

    return true;
}

// ========================================================================================================

bool launch_html_viewer_server(
    const std::filesystem::path &directory,
    const std::string &html_filename)
{
    if (directory.empty() || html_filename.empty()) {
        return false;
    }

    const int port = 8000;
    const std::string directory_string = directory.string();

    std::string server_command =
        "python3 -m http.server " + std::to_string(port) +
        " --bind 127.0.0.1 --directory " + shell_quote(directory_string) +
        " >/tmp/mbmesh_http.log 2>&1 &";

    const int server_status = std::system(server_command.c_str());
    if (server_status != 0) {
        return false;
    }

    const std::string url =
        "http://127.0.0.1:" + std::to_string(port) + "/" + html_filename;
    const std::string open_command =
        "python3 -c \"import webbrowser; webbrowser.open('" + url + "')\"";

    const int open_status = std::system(open_command.c_str());
    if (open_status != 0) {
        return false;
    }

    std::cerr << "Started Python web server at " << url << '\n';
    std::cerr << "Server logs: /tmp/mbmesh_http.log\n";
    return true;
}
