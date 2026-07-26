#include "algorithms/decimation.h"
#include "algorithms/mesh_generation.h"
#include "algorithms/normal_estimation.h"
#include "io/x3dom_writer.h"
#include "io/xyz_reader.h"

#include <cstdio>
#include <filesystem>
#include <string>

int main() {
    const std::string input_path = "src/mbmesh_dev/test_data/adjustedPointcloud.xyz";
    const std::string output_dir = "src/mbmesh_dev/output";
    std::filesystem::create_directories(output_dir);

    std::string error;
    PointCloud pointcloud;
    if (!read_xyz_file(input_path, &pointcloud, &error)) {
        std::fprintf(stderr, "Failed to read XYZ file '%s': %s\n", input_path.c_str(), error.c_str());
        return 1;
    }

    constexpr double decimation_cell_size = 0.35;
    PointCloud decimated_pointcloud = decimate_pointcloud_2d(pointcloud, decimation_cell_size);
    std::printf("Loaded %zu XYZ points from %s\n", pointcloud.points.size(), input_path.c_str());
    std::printf("Decimated to %zu points using %.3f XY cells\n",
                decimated_pointcloud.points.size(),
                decimation_cell_size);

    pointcloud.points.clear();
    pointcloud.points.shrink_to_fit();

    if (decimated_pointcloud.points.empty()) {
        std::fprintf(stderr, "Decimated point cloud is empty\n");
        return 1;
    }

    X3DomWriterOptions point_options;
    point_options.title = "MB-System adjusted XYZ decimated point cloud";
    point_options.max_points = 250000;

    const std::string decimated_output = output_dir + "/adjusted_poisson_decimated_points.html";
    if (!write_pointcloud_x3dom_file(decimated_output.c_str(), decimated_pointcloud, point_options, &error)) {
        std::fprintf(stderr, "Failed to write decimated point cloud '%s': %s\n",
                     decimated_output.c_str(),
                     error.c_str());
        return 1;
    }

    PoissonOptions poisson_options;
    poisson_options.normal_neighbors = 16;
    poisson_options.screened_poisson.cell_size = 0.35;
    poisson_options.screened_poisson.padding = 0.70;
    poisson_options.screened_poisson.normal_splat_radius = 0.70;
    poisson_options.screened_poisson.screening_weight = 4.0;
    poisson_options.screened_poisson.solver_iterations = 150;
    poisson_options.screened_poisson.use_screening = true;
    poisson_options.screened_poisson.estimate_iso_value_from_samples = true;

    const OrientedPointCloud oriented_pointcloud =
        estimate_oriented_points(decimated_pointcloud, poisson_options.normal_neighbors);

    X3DomWriterOptions oriented_options;
    oriented_options.title = "MB-System adjusted XYZ estimated normals";
    oriented_options.normal_scale = 0.35;
    oriented_options.max_points = 250000;
    oriented_options.max_normal_vectors = 25000;

    const std::string oriented_output = output_dir + "/adjusted_poisson_oriented_points.html";
    if (!write_oriented_pointcloud_x3dom_file(oriented_output.c_str(), oriented_pointcloud, oriented_options, &error)) {
        std::fprintf(stderr, "Failed to write oriented point cloud '%s': %s\n",
                     oriented_output.c_str(),
                     error.c_str());
        return 1;
    }

    const Mesh poisson_mesh = generate_mesh_poisson(oriented_pointcloud, poisson_options);
    std::printf("Generated screened Poisson mesh: %zu vertices, %zu triangles\n",
                poisson_mesh.vertices.size(),
                poisson_mesh.indices.size() / 3);

    if (poisson_mesh.vertices.empty() || poisson_mesh.indices.empty()) {
        std::fprintf(stderr, "Screened Poisson mesh is empty\n");
        return 1;
    }

    X3DomWriterOptions mesh_options;
    mesh_options.title = "MB-System adjusted XYZ screened Poisson mesh";
    mesh_options.max_triangles = 500000;

    const std::string mesh_output = output_dir + "/adjusted_poisson_mesh.html";
    if (!write_mesh_x3dom_file(mesh_output.c_str(), poisson_mesh, mesh_options, &error)) {
        std::fprintf(stderr, "Failed to write Poisson mesh '%s': %s\n",
                     mesh_output.c_str(),
                     error.c_str());
        return 1;
    }

    std::printf("Wrote %s\n", decimated_output.c_str());
    std::printf("Wrote %s\n", oriented_output.c_str());
    std::printf("Wrote %s\n", mesh_output.c_str());
    return 0;
}
