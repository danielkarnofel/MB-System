#include "algorithms/mesh_generation.h"
#include "io/x3dom_writer.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <string>

namespace {

constexpr double pi = 3.14159265358979323846;

OrientedPointCloud generate_hemisphere_oriented_pointcloud(double radius,
                                                           int latitude_steps,
                                                           int longitude_steps) {
    OrientedPointCloud pointcloud;
    if (radius <= 0.0 || latitude_steps < 2 || longitude_steps < 3) {
        return pointcloud;
    }

    pointcloud.oriented_points.reserve(static_cast<std::size_t>(latitude_steps) *
                                       static_cast<std::size_t>(longitude_steps));

    for (int latitude = 0; latitude < latitude_steps; latitude++) {
        const double theta = 0.5 * pi * static_cast<double>(latitude) /
                             static_cast<double>(latitude_steps - 1);
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);

        for (int longitude = 0; longitude < longitude_steps; longitude++) {
            const double phi = 2.0 * pi * static_cast<double>(longitude) /
                               static_cast<double>(longitude_steps);
            const Vec3 normal(sin_theta * std::cos(phi),
                              sin_theta * std::sin(phi),
                              cos_theta);
            pointcloud.oriented_points.push_back(OrientedPoint(normal * radius, normal));
        }
    }

    return pointcloud;
}

} // namespace

int main() {
    const std::string output_dir = "src/mbmesh_dev/output";
    std::filesystem::create_directories(output_dir);

    const OrientedPointCloud hemisphere = generate_hemisphere_oriented_pointcloud(1.0, 36, 96);
    std::printf("Generated %zu oriented hemisphere points\n", hemisphere.oriented_points.size());

    X3DomWriterOptions point_options;
    point_options.title = "MB-System Poisson hemisphere oriented point cloud";
    point_options.normal_scale = 0.08;
    point_options.max_normal_vectors = 5000;

    std::string error;
    const std::string pointcloud_output = output_dir + "/poisson_hemisphere_oriented_points.html";
    if (!write_oriented_pointcloud_x3dom_file(pointcloud_output.c_str(), hemisphere, point_options, &error)) {
        std::fprintf(stderr, "Failed to write hemisphere oriented point cloud '%s': %s\n",
                     pointcloud_output.c_str(),
                     error.c_str());
        return 1;
    }

    PoissonOptions options;
    options.screened_poisson.cell_size = 0.06;
    options.screened_poisson.padding = 0.24;
    options.screened_poisson.normal_splat_radius = 0.12;
    options.screened_poisson.screening_weight = 4.0;
    options.screened_poisson.solver_iterations = 200;
    options.screened_poisson.use_screening = true;
    options.screened_poisson.estimate_iso_value_from_samples = true;

    const Mesh mesh = generate_mesh_poisson(hemisphere, options);
    std::printf("Generated Poisson mesh: %zu vertices, %zu triangles\n",
                mesh.vertices.size(),
                mesh.indices.size() / 3);

    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::printf("Poisson mesh is empty while screened_poisson is still a stub; this is expected for now.\n");
        std::printf("Wrote %s\n", pointcloud_output.c_str());
        return 0;
    }

    X3DomWriterOptions mesh_options;
    mesh_options.title = "MB-System Poisson hemisphere mesh";
    mesh_options.max_triangles = 500000;

    const std::string mesh_output = output_dir + "/poisson_hemisphere_mesh.html";
    if (!write_mesh_x3dom_file(mesh_output.c_str(), mesh, mesh_options, &error)) {
        std::fprintf(stderr, "Failed to write hemisphere Poisson mesh '%s': %s\n",
                     mesh_output.c_str(),
                     error.c_str());
        return 1;
    }

    std::printf("Wrote %s\n", pointcloud_output.c_str());
    std::printf("Wrote %s\n", mesh_output.c_str());
    return 0;
}
