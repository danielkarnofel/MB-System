#include <vector>
#include <filesystem>

struct OutputOptions {

    // Includes point, sensor-origin, and estimated normal
    // px, py, pz, ox, oy, oz, nx, ny, nz
    bool write_point_xyz = false;
    bool write_point_glb = true;
    bool write_point_html = false;

    bool write_mesh_glb = true;
    bool write_mesh_html = false; 

    std::filesystem::path directory;
};

int main(int argc, char **argv) {

    // Each data structure should be freed as soon as it is no longer needed
    // Output files should be written as soon as possible so memory can be freed

    // Parse and validate configuration:
    // Options options = parse_options(argc, argv);

    // Each sounding is transformed into the selected local coordinate frame and paired with its sensor origin as it is read.
    // for (Swathfile in Datalist)
    //   for (Ping in Swathfile)
    //     for (Beam in Ping)
    //       Beam -> Sounding 
    //       Sounding -> raw CollectedPoint
    //       raw CollectedPoint -> transformed CollectedPoint
    //       discard out-of-bounds points

    // Read the datalist incrementally and collect valid soundings:
    // CollectedPointCloud collected_points = read_datalist(options);

    // Estimate PCA normals and orient them using the sensor-origin vector:
    // OrientedPointCloud oriented_points = estimate_oriented_points(collected_points, options);
    
    // Reconstruct implicit surface using Screened Poisson:
    // ScalarGrid3D poisson_surface = screened_poisson(collected_points, options);

    // Extract isosurface using Marching Cubes:
    // Mesh raw_mesh = marching_cubes(poisson_surface, options);

    // Post-process mesh using oriented-sample support trimming:
    // Mesh clean_mesh = postprocess_mesh(raw_mesh, oriented_points, options);

    // Write desired output files:
    // write_outputs(collected_points, oriented_points, clean_mesh, options);
}
