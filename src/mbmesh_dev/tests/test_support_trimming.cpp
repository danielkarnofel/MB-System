#include "algorithms/support_trimming.h"

#include <cmath>
#include <cstddef>
#include <iostream>

namespace {

OrientedPointCloud make_plane_samples() {
    OrientedPointCloud samples;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            samples.push_back(
                OrientedPoint(Vec3(static_cast<double>(x),
                                   static_cast<double>(y),
                                   0.0),
                              Vec3(0.0, 0.0, -1.0)));
        }
    }
    return samples;
}

Mesh make_surface_and_closure_mesh() {
    Mesh mesh;
    mesh.vertices = {
        Vec3(-0.5, -0.5, 0.0),
        Vec3(0.5, -0.5, 0.0),
        Vec3(0.5, 0.5, 0.0),
        Vec3(-0.5, 0.5, 0.0),
        Vec3(-0.5, -0.5, -2.0),
        Vec3(0.5, -0.5, -2.0),
        Vec3(0.5, 0.5, -2.0),
        Vec3(-0.5, 0.5, -2.0),
    };
    mesh.indices = {
        0, 1, 2,
        0, 2, 3,
        4, 6, 5,
        4, 7, 6,
    };
    return mesh;
}

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

} // namespace

int main() {
    const OrientedPointCloud samples = make_plane_samples();
    const Mesh raw_mesh = make_surface_and_closure_mesh();

    SupportTrimmingOptions options;
    options.minimum_normal_alignment = 0.9;

    SupportTrimmingDiagnostics diagnostics;
    const Mesh trimmed_mesh =
        support_trimming(raw_mesh, samples, options, &diagnostics);
    if (trimmed_mesh.vertices.size() != 4 ||
        trimmed_mesh.indices.size() != 6 ||
        trimmed_mesh.normals.size() != trimmed_mesh.vertices.size() ||
        !indices_are_valid(trimmed_mesh)) {
        std::cerr << "Unexpected trimmed mesh structure\n";
        return 1;
    }
    if (diagnostics.supported_vertices != 4 ||
        diagnostics.rejected_for_normal_offset != 4 ||
        diagnostics.output_triangles != 2) {
        std::cerr << "Unexpected trimming diagnostics\n";
        return 1;
    }

    for (const Vec3 &vertex : trimmed_mesh.vertices) {
        if (std::fabs(vertex.z) > vec3_epsilon) {
            std::cerr << "Unsupported closure vertex survived trimming\n";
            return 1;
        }
    }

    SupportTrimmingOptions disabled_options;
    disabled_options.enabled = false;
    const Mesh unchanged_mesh =
        support_trimming(raw_mesh, samples, disabled_options);
    if (unchanged_mesh.vertices.size() != raw_mesh.vertices.size() ||
        unchanged_mesh.indices != raw_mesh.indices) {
        std::cerr << "Disabled trimming changed the mesh\n";
        return 1;
    }

    return 0;
}
