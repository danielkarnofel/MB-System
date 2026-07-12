#include "algorithms/mesh_generation.h"
#include "io/x3dom_writer.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

namespace {

ScalarGrid3D make_sphere_sdf_grid(int resolution, double cell_size, double radius) {
    const double extent = static_cast<double>(resolution - 1) * cell_size;
    const Vec3 origin(-0.5 * extent, -0.5 * extent, -0.5 * extent);
    ScalarGrid3D grid(resolution, resolution, resolution, cell_size, origin);

    for (int z = 0; z < grid.nz; z++) {
        for (int y = 0; y < grid.ny; y++) {
            for (int x = 0; x < grid.nx; x++) {
                const Vec3 position = grid.position(x, y, z);
                grid.at(x, y, z) = position.length() - radius;
            }
        }
    }

    return grid;
}

bool validate_mesh_indices(const Mesh &mesh) {
    if (mesh.indices.empty() || mesh.indices.size() % 3 != 0) {
        return false;
    }

    for (const unsigned int index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            return false;
        }
    }

    return true;
}

bool validate_sphere_radius(const Mesh &mesh, double radius, double tolerance) {
    for (const Vec3 &vertex : mesh.vertices) {
        if (std::fabs(vertex.length() - radius) > tolerance) {
            return false;
        }
    }

    return true;
}

bool validate_normals(const Mesh &mesh) {
    if (mesh.normals.size() != mesh.vertices.size()) {
        return false;
    }

    for (const Vec3 &normal : mesh.normals) {
        const double length = normal.length();
        if (std::fabs(length - 1.0) > 1.0e-6) {
            return false;
        }
    }

    return true;
}

} // namespace

int main() {
    constexpr int resolution = 36;
    constexpr double cell_size = 0.08;
    constexpr double radius = 1.0;
    const std::string output_dir = "src/mbmesh_dev/output";
    const std::string output_path = output_dir + "/sphere_sdf_marching_cubes.html";

    std::filesystem::create_directories(output_dir);

    const ScalarGrid3D grid = make_sphere_sdf_grid(resolution, cell_size, radius);

    MarchingCubesOptions options;
    options.iso_value = 0.0;
    const Mesh mesh = marching_cubes(grid, options);

    std::printf("Sphere SDF marching cubes: %zu vertices, %zu triangles\n",
                mesh.vertices.size(),
                mesh.indices.size() / 3);

    if (mesh.vertices.empty()) {
        std::fprintf(stderr, "Expected non-empty sphere mesh\n");
        return 1;
    }

    if (!validate_mesh_indices(mesh)) {
        std::fprintf(stderr, "Sphere mesh has invalid indices\n");
        return 1;
    }

    if (!validate_normals(mesh)) {
        std::fprintf(stderr, "Sphere mesh has invalid normals\n");
        return 1;
    }

    if (!validate_sphere_radius(mesh, radius, cell_size * 0.75)) {
        std::fprintf(stderr, "Sphere mesh vertices deviate too far from radius %.3f\n", radius);
        return 1;
    }

    X3DomWriterOptions writer_options;
    writer_options.title = "MB-System sphere SDF marching cubes";
    writer_options.max_triangles = 500000;
    writer_options.mesh_transparency = 0.0;

    std::string error;
    if (!write_mesh_x3dom_file(output_path.c_str(), mesh, writer_options, &error)) {
        std::fprintf(stderr, "Failed to write sphere mesh HTML '%s': %s\n", output_path.c_str(), error.c_str());
        return 1;
    }

    std::printf("Wrote %s\n", output_path.c_str());
    return 0;
}
