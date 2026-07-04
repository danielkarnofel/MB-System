#include "algorithms/mesh_generation.h"
#include "data_types/geometry.h"
#include "io/x3dom_writer.h"
#include "io/xyz_reader.h"

#include <cstdio>
#include <filesystem>
#include <string>

int main() {
    const std::string input_path = "src/mbmesh_dev/test_data/adjustedPointcloud.xyz";
    const std::string output_dir = "src/mbmesh_dev/output";
    std::filesystem::create_directories(output_dir);

    PointCloud pointcloud;
    std::string error;
    if (!read_xyz_file(input_path, &pointcloud, &error)) {
        std::fprintf(stderr, "Failed to read XYZ file '%s': %s\n", input_path.c_str(), error.c_str());
        return 1;
    }

    X3DomWriterOptions pointcloud_options;
    pointcloud_options.title = "MB-System adjusted XYZ point cloud";
    pointcloud_options.max_points = 250000;

    const std::string pointcloud_output = output_dir + "/adjusted_pointcloud.html";
    if (!write_pointcloud_x3dom_file(pointcloud_output.c_str(), pointcloud, pointcloud_options, &error)) {
        std::fprintf(stderr, "Failed to write point cloud HTML '%s': %s\n", pointcloud_output.c_str(), error.c_str());
        return 1;
    }

    Grid2DOptions mesh_options{0.1, 5.0, 0.0};
    Mesh mesh = generate_mesh_grid_2D(pointcloud, mesh_options);
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::fprintf(stderr,
                     "Generated mesh is empty: %zu vertices, %zu triangles\n",
                     mesh.vertices.size(),
                     mesh.indices.size() / 3);
        return 1;
    }

    X3DomWriterOptions mesh_options_x3dom;
    mesh_options_x3dom.title = "MB-System adjusted XYZ Grid2D mesh";
    mesh_options_x3dom.max_triangles = 500000;

    const std::string mesh_output = output_dir + "/adjusted_grid_2D_mesh.html";
    if (!write_mesh_x3dom_file(mesh_output.c_str(), mesh, mesh_options_x3dom, &error)) {
        std::fprintf(stderr, "Failed to write mesh HTML '%s': %s\n", mesh_output.c_str(), error.c_str());
        return 1;
    }

    std::printf("Read %zu XYZ points\n", pointcloud.points.size());
    std::printf("Wrote %s\n", pointcloud_output.c_str());
    std::printf("Generated %zu mesh vertices and %zu triangles\n", mesh.vertices.size(), mesh.indices.size() / 3);
    std::printf("Wrote %s\n", mesh_output.c_str());

    return 0;
}
