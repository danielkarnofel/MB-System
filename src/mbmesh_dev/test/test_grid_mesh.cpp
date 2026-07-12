#include "algorithms/decimation.h"
#include "algorithms/mesh_generation.h"
#include "data_types/geometry.h"
#include "io/x3dom_writer.h"
#include "synthetic_data/datasets.h"

#include <cstdio>
#include <filesystem>
#include <string>

namespace {

bool write_pointcloud_preview(const PointCloud &pointcloud,
                              const std::string &output_dir,
                              std::string *error) {
    X3DomWriterOptions pointcloud_options;
    pointcloud_options.title = "MB-System synthetic terrain arch point cloud";
    pointcloud_options.max_points = 250000;

    const std::string pointcloud_output = output_dir + "/synthetic_terrain_arch_pointcloud.html";
    if (!write_pointcloud_x3dom_file(pointcloud_output.c_str(), pointcloud, pointcloud_options, error)) {
        std::fprintf(stderr, "Failed to write point cloud HTML '%s': %s\n", pointcloud_output.c_str(), error->c_str());
        return false;
    }

    std::printf("Wrote %s\n", pointcloud_output.c_str());
    return true;
}

bool run_grid_2d_test(const PointCloud &pointcloud,
                      const std::string &output_dir,
                      std::string *error) {
    Grid2DOptions mesh_options{0.08, 5.0, 0.0};
    Mesh mesh = generate_mesh_grid_2D(pointcloud, mesh_options);
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::fprintf(stderr,
                     "Generated Grid2D mesh is empty: %zu vertices, %zu triangles\n",
                     mesh.vertices.size(),
                     mesh.indices.size() / 3);
        return false;
    }

    X3DomWriterOptions mesh_options_x3dom;
    mesh_options_x3dom.title = "MB-System synthetic terrain arch Grid2D mesh";
    mesh_options_x3dom.max_triangles = 500000;

    const std::string mesh_output = output_dir + "/synthetic_terrain_arch_grid_2D_mesh.html";
    if (!write_mesh_x3dom_file(mesh_output.c_str(), mesh, mesh_options_x3dom, error)) {
        std::fprintf(stderr, "Failed to write Grid2D mesh HTML '%s': %s\n", mesh_output.c_str(), error->c_str());
        return false;
    }

    std::printf("Generated Grid2D mesh: %zu vertices, %zu triangles\n",
                mesh.vertices.size(),
                mesh.indices.size() / 3);
    std::printf("Wrote %s\n", mesh_output.c_str());
    return true;
}

bool run_decimated_grid_3d_test(const PointCloud &pointcloud,
                                const std::string &output_dir,
                                std::string *error) {
    constexpr double decimation_cell_size = 0.12;
    const PointCloud decimated_pointcloud = decimate_pointcloud_3d(pointcloud, decimation_cell_size);
    if (decimated_pointcloud.points.empty()) {
        std::fprintf(stderr, "Decimated Grid3D input is empty\n");
        return false;
    }

    X3DomWriterOptions decimated_options;
    decimated_options.title = "MB-System synthetic terrain arch decimated point cloud";
    decimated_options.max_points = 250000;

    const std::string decimated_output = output_dir + "/synthetic_terrain_arch_decimated_pointcloud.html";
    if (!write_pointcloud_x3dom_file(decimated_output.c_str(), decimated_pointcloud, decimated_options, error)) {
        std::fprintf(stderr, "Failed to write decimated point cloud HTML '%s': %s\n", decimated_output.c_str(), error->c_str());
        return false;
    }

    Grid3DOptions grid_options{0.12, 0.24, 0.18, 0.0};
    const Mesh mesh = generate_mesh_grid_3D(decimated_pointcloud, grid_options);

    std::printf("Decimated point cloud: %zu -> %zu points using cell size %.3f\n",
                pointcloud.points.size(),
                decimated_pointcloud.points.size(),
                decimation_cell_size);
    std::printf("Wrote %s\n", decimated_output.c_str());
    std::printf("Generated Grid3D mesh: %zu vertices, %zu triangles\n",
                mesh.vertices.size(),
                mesh.indices.size() / 3);

    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::fprintf(stderr, "Generated Grid3D mesh is empty\n");
        return false;
    }

    X3DomWriterOptions mesh_options_x3dom;
    mesh_options_x3dom.title = "MB-System synthetic terrain arch Grid3D mesh";
    mesh_options_x3dom.max_triangles = 500000;

    const std::string mesh_output = output_dir + "/synthetic_terrain_arch_grid_3D_mesh.html";
    if (!write_mesh_x3dom_file(mesh_output.c_str(), mesh, mesh_options_x3dom, error)) {
        std::fprintf(stderr, "Failed to write Grid3D mesh HTML '%s': %s\n", mesh_output.c_str(), error->c_str());
        return false;
    }

    std::printf("Wrote %s\n", mesh_output.c_str());
    return true;
}

} // namespace

int main() {
    const std::string output_dir = "src/mbmesh_dev/output";
    std::filesystem::create_directories(output_dir);

    std::string error;
    const PointCloud pointcloud = generate_terrain_half_torus_arch_pointcloud(
        5.0,
        5.0,
        96,
        96,
        72,
        20);

    std::printf("Generated %zu synthetic terrain arch points\n", pointcloud.points.size());

    if (!write_pointcloud_preview(pointcloud, output_dir, &error)) {
        return 1;
    }

    if (!run_grid_2d_test(pointcloud, output_dir, &error)) {
        return 1;
    }

    if (!run_decimated_grid_3d_test(pointcloud, output_dir, &error)) {
        return 1;
    }

    return 0;
}
