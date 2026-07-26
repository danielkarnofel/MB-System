#include <vector>
#include <filesystem>

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

CollectedPointCloud read_datalist(Options options);

bool write_outputs(const Mesh &mesh, const Options &options, std::string *error);

int main(int argc, char **argv) {

    // Parse and validate configuration:
    Options options = parse_options(argc, argv);

    // Read the datalist incrementally and collect valid soundings:
    CollectedPointCloud collected_points = read_datalist(options);

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
    std::string error_message = "Error writing outputs.";
    write_outputs(clean_mesh, options, &error_message);
}

Options parse_options(int argc, char **argv) {
    Options options;
    return options;
}

CollectedPointCloud read_datalist(Options options) {
    CollectedPointCloud collected_points;
    return collected_points;
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
