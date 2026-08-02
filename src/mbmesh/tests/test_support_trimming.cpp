#include "synthetic_data.h"
#include "algorithms/marching_cubes.h"
#include "algorithms/screened_poisson.h"
#include "algorithms/support_trimming.h"
#include "io/glb_writer.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

constexpr double pi = 3.14159265358979323846;

bool indices_are_valid(const Mesh &mesh) {
    if (mesh.indices.size() % 3 != 0) {
        return false;
    }
    for (const unsigned int index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            return false;
        }
    }
    return true;
}

OrientedPointCloud make_plane_with_half_torus_arch_samples() {
    OrientedPointCloud samples;

    constexpr double plane_extent = 2.5;
    constexpr double plane_step = 0.12;
    constexpr double major_radius = 1.35;
    constexpr double minor_radius = 0.30;
    constexpr double torus_theta_step = 0.12;
    constexpr double torus_phi_step = 0.12;
    constexpr double arch_height_offset = 0.18;

    for (double y = -plane_extent; y <= plane_extent + plane_step * 0.5; y += plane_step) {
        for (double x = -plane_extent; x <= plane_extent + plane_step * 0.5; x += plane_step) {
            samples.push_back(OrientedPoint(Vec3(x, y, 0.0), Vec3(0.0, 0.0, 1.0)));
        }
    }

    // A half-torus ridge oriented across the plane: the major arc lies in the X/Z
    // plane and rises above the ground, while the tube cross-section runs along Y.
    for (double theta = 0.0; theta <= pi + 0.5 * torus_theta_step; theta += torus_theta_step) {
        for (double phi = -pi; phi <= pi + 0.5 * torus_phi_step; phi += torus_phi_step) {
            const double radial = major_radius + minor_radius * std::cos(phi);
            const Vec3 point(
                radial * std::cos(theta),
                minor_radius * std::sin(phi),
                radial * std::sin(theta) + arch_height_offset);

            const Vec3 normal(
                std::cos(theta) * std::cos(phi),
                std::sin(phi),
                std::sin(theta) * std::cos(phi));

            if (point.z < 0.0) {
                continue;
            }

            samples.push_back(OrientedPoint(point, normalize(normal)));
        }
    }

    return samples;
}

} // namespace

int main() {
    const std::filesystem::path output_dir = "src/mbmesh/output";
    const std::filesystem::path raw_glb_path = output_dir / "st-test-raw-mesh.glb";
    const std::filesystem::path trimmed_glb_path = output_dir / "st-test-trimmed-mesh.glb";

    std::filesystem::create_directories(output_dir);

    const OrientedPointCloud samples = make_plane_with_half_torus_arch_samples();
    if (samples.empty()) {
        std::fprintf(stderr, "Plane + half-torus sample generation produced no oriented points\n");
        return 1;
    }

    ScreenedPoissonOptions poisson_options;
    poisson_options.cell_size = 0.04;
    poisson_options.padding = 0.30;
    poisson_options.normal_splat_radius = 0.18;
    poisson_options.screening_weight = 8.0;
    poisson_options.solver_iterations = 400;
    poisson_options.use_screening = true;
    poisson_options.iso_value = 0.0;

    const ScalarGrid3D poisson_surface = screened_poisson(samples, poisson_options);
    if (poisson_surface.values.empty()) {
        std::fprintf(stderr, "Screened Poisson reconstruction produced no scalar grid\n");
        return 1;
    }

    MarchingCubesOptions marching_options;
    marching_options.iso_value = 0.0;
    const Mesh raw_mesh = marching_cubes(poisson_surface, marching_options);
    if (raw_mesh.vertices.empty() || raw_mesh.indices.empty()) {
        std::fprintf(stderr, "Marching cubes produced an empty mesh\n");
        return 1;
    }

    SupportTrimmingOptions trimming_options;
    trimming_options.enabled = true;
    trimming_options.minimum_neighbors = 2;             // keep the neighborhood check
    trimming_options.minimum_normal_alignment = 0.0;  // or 0.0 if you want to preserve topology
    trimming_options.support_radius = 0.8;
    trimming_options.max_normal_offset = 0.25;

    SupportTrimmingDiagnostics diagnostics;
    const Mesh trimmed_mesh =
        support_trimming(raw_mesh, samples, trimming_options, &diagnostics);
    if (trimmed_mesh.vertices.empty() ||
        trimmed_mesh.indices.empty() ||
        trimmed_mesh.normals.size() != trimmed_mesh.vertices.size() ||
        !indices_are_valid(trimmed_mesh)) {
        std::fprintf(stderr, "Unexpected trimmed mesh structure\n");
        return 1;
    }
    if (diagnostics.supported_vertices == 0 ||
        diagnostics.output_triangles == 0) {
        std::fprintf(stderr, "Support trimming diagnostic output was empty\n");
        return 1;
    }

    std::printf("Plane + half-torus screened Poisson: raw=%zu vertices / %zu triangles, "
                "trimmed=%zu vertices / %zu triangles\n",
                raw_mesh.vertices.size(),
                raw_mesh.indices.size() / 3,
                trimmed_mesh.vertices.size(),
                trimmed_mesh.indices.size() / 3);

    std::string error;
    if (!write_mesh_glb_file(raw_glb_path, raw_mesh, &error)) {
        std::fprintf(stderr, "Failed to write raw GLB: %s\n", error.c_str());
        return 1;
    }

    if (!write_mesh_glb_file(trimmed_glb_path, trimmed_mesh, &error)) {
        std::fprintf(stderr, "Failed to write trimmed GLB: %s\n", error.c_str());
        return 1;
    }

    return 0;
}

/*
g++ -std=c++17 -I src/mbmesh/include -I src/mbgrd2gltf \
  src/mbmesh/tests/test_support_trimming.cpp \
  src/mbmesh/src/algorithms/marching_cubes.cpp \
  src/mbmesh/src/algorithms/screened_poisson.cpp \
  src/mbmesh/src/algorithms/support_trimming.cpp \
  src/mbmesh/src/io/glb_writer.cpp \
  -o /tmp/mbmesh_test_support_trimming

/tmp/mbmesh_test_support_trimming
*/
