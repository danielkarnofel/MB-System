#include "algorithms/mesh_generation.h"
#include "algorithms/normal_estimation.h"
#include "data_types/geometry.h"
#include "io/x3dom_writer.h"
#include "synthetic_data/noise.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

namespace {

constexpr double pi = 3.14159265358979323846;
constexpr double normal_step = 1.0e-3;

[[nodiscard]] double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

[[nodiscard]] double step_fraction(int index, int count) {
    return count <= 1 ? 0.0 : static_cast<double>(index) / static_cast<double>(count - 1);
}

[[nodiscard]] double terrain_height(double x, double y) {
    const double slope = 0.10 * x - 0.06 * y;
    const double ridge = 0.32 * std::exp(-0.75 * (y + 0.9) * (y + 0.9));
    const double trench = -0.22 * std::exp(-1.2 * (x - 1.1) * (x - 1.1));
    const double ripple = 0.16 * std::sin(2.8 * x + 0.7 * y) * std::cos(1.6 * y);
    const double roughness = 0.07 * perlin_noise(1.7 * x, 1.7 * y, 4.0);
    return slope + ridge + trench + ripple + roughness;
}

[[nodiscard]] Vec3 terrain_normal(double x, double y) {
    const double dz_dx = (terrain_height(x + normal_step, y) - terrain_height(x - normal_step, y)) /
                         (2.0 * normal_step);
    const double dz_dy = (terrain_height(x, y + normal_step) - terrain_height(x, y - normal_step)) /
                         (2.0 * normal_step);
    return normalize(Vec3(-dz_dx, -dz_dy, 1.0));
}

void append_oriented_terrain(OrientedPointCloud &oriented_pointcloud,
                             double width,
                             double length,
                             int width_steps,
                             int length_steps) {
    for (int iy = 0; iy < length_steps; iy++) {
        const double y = lerp(-0.5 * length, 0.5 * length, step_fraction(iy, length_steps));
        for (int ix = 0; ix < width_steps; ix++) {
            const double x = lerp(-0.5 * width, 0.5 * width, step_fraction(ix, width_steps));
            oriented_pointcloud.oriented_points.push_back(OrientedPoint(Vec3(x, y, terrain_height(x, y)),
                                                                        terrain_normal(x, y)));
        }
    }
}

void append_oriented_half_torus_arch(OrientedPointCloud &oriented_pointcloud,
                                     int arch_steps,
                                     int tube_steps) {
    constexpr double major_radius = 1.45;
    constexpr double tube_radius = 0.22;
    constexpr double y_center = 0.15;
    constexpr double base_clearance = 0.18;

    for (int iu = 0; iu < arch_steps; iu++) {
        const double u = pi * step_fraction(iu, arch_steps);
        const double cos_u = std::cos(u);
        const double sin_u = std::sin(u);
        const double center_x = major_radius * cos_u;
        const double center_z = terrain_height(center_x, y_center) + base_clearance + major_radius * sin_u;

        const Vec3 radial(cos_u, 0.0, sin_u);
        const Vec3 radial_derivative(-sin_u, 0.0, cos_u);
        const Vec3 center(center_x, y_center, center_z);

        const double terrain_slope_x =
            (terrain_height(center_x + normal_step, y_center) -
             terrain_height(center_x - normal_step, y_center)) /
            (2.0 * normal_step);
        const Vec3 center_derivative(-major_radius * sin_u,
                                     0.0,
                                     terrain_slope_x * (-major_radius * sin_u) + major_radius * cos_u);

        for (int iv = 0; iv < tube_steps; iv++) {
            const double v = 2.0 * pi * static_cast<double>(iv) / static_cast<double>(tube_steps);
            const double cos_v = std::cos(v);
            const double sin_v = std::sin(v);

            const Vec3 point = center +
                               radial * (tube_radius * cos_v) +
                               Vec3(0.0, tube_radius * sin_v, 0.0);

            const Vec3 tangent_u = center_derivative + radial_derivative * (tube_radius * cos_v);
            const Vec3 tangent_v = radial * (-tube_radius * sin_v) + Vec3(0.0, tube_radius * cos_v, 0.0);
            Vec3 normal = cross(tangent_v, tangent_u);
            if (normal.length_squared() <= 1.0e-20) {
                normal = radial * cos_v + Vec3(0.0, sin_v, 0.0);
            }

            oriented_pointcloud.oriented_points.push_back(OrientedPoint(point, normalize(normal)));
        }
    }
}

[[nodiscard]] OrientedPointCloud generate_exact_oriented_terrain_arch() {
    OrientedPointCloud oriented_pointcloud;
    oriented_pointcloud.oriented_points.reserve(96u * 96u + 72u * 20u);

    append_oriented_terrain(oriented_pointcloud, 5.0, 5.0, 96, 96);
    append_oriented_half_torus_arch(oriented_pointcloud, 72, 20);
    return oriented_pointcloud;
}

[[nodiscard]] PointCloud strip_normals(const OrientedPointCloud &oriented_pointcloud) {
    PointCloud pointcloud;
    pointcloud.points.reserve(oriented_pointcloud.oriented_points.size());
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        pointcloud.points.push_back(oriented_point.point);
    }
    return pointcloud;
}

void print_normal_comparison(const OrientedPointCloud &exact_oriented_pointcloud,
                             const OrientedPointCloud &estimated_oriented_pointcloud) {
    if (exact_oriented_pointcloud.oriented_points.size() != estimated_oriented_pointcloud.oriented_points.size() ||
        exact_oriented_pointcloud.oriented_points.empty()) {
        return;
    }

    std::size_t opposing_count = 0;
    std::size_t y_sign_mismatch_count = 0;
    double dot_sum = 0.0;
    double y_dot_sum = 0.0;

    for (std::size_t i = 0; i < exact_oriented_pointcloud.oriented_points.size(); i++) {
        const Vec3 &exact = exact_oriented_pointcloud.oriented_points[i].normal;
        const Vec3 &estimated = estimated_oriented_pointcloud.oriented_points[i].normal;
        const double normal_dot = dot(exact, estimated);
        dot_sum += normal_dot;
        y_dot_sum += exact.y * estimated.y;

        if (normal_dot < 0.0) {
            opposing_count++;
        }

        if (std::fabs(exact.y) > 0.25 && std::fabs(estimated.y) > 0.25 && exact.y * estimated.y < 0.0) {
            y_sign_mismatch_count++;
        }
    }

    const double count = static_cast<double>(exact_oriented_pointcloud.oriented_points.size());
    std::printf("PCA normal comparison: average dot %.3f, opposing %zu/%zu, Y-sign mismatches %zu/%zu\n",
                dot_sum / count,
                opposing_count,
                exact_oriented_pointcloud.oriented_points.size(),
                y_sign_mismatch_count,
                exact_oriented_pointcloud.oriented_points.size());
    std::printf("Average exact/PCA normal Y product: %.3f\n", y_dot_sum / count);
}

void print_mesh_bounds(const char *label, const Mesh &mesh) {
    if (mesh.vertices.empty()) {
        return;
    }

    Bounds3D bounds(mesh.vertices[0], mesh.vertices[0]);
    for (const Vec3 &vertex : mesh.vertices) {
        bounds.include(vertex);
    }

    const Vec3 center = bounds.center();
    std::printf("%s bounds: min (%.3f, %.3f, %.3f), max (%.3f, %.3f, %.3f), center (%.3f, %.3f, %.3f)\n",
                label,
                bounds.min.x,
                bounds.min.y,
                bounds.min.z,
                bounds.max.x,
                bounds.max.y,
                bounds.max.z,
                center.x,
                center.y,
                center.z);
}

[[nodiscard]] PoissonOptions terrain_arch_poisson_options() {
    PoissonOptions options;
    options.normal_neighbors = 18;
    options.screened_poisson.cell_size = 0.12;
    options.screened_poisson.padding = 0.36;
    options.screened_poisson.normal_splat_radius = 0.24;
    options.screened_poisson.screening_weight = 4.0;
    options.screened_poisson.solver_iterations = 200;
    options.screened_poisson.use_screening = true;
    options.screened_poisson.estimate_iso_value_from_samples = true;
    return options;
}

bool write_mesh_preview(const Mesh &mesh,
                        const std::string &path,
                        const std::string &title,
                        std::string *error) {
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        std::fprintf(stderr, "Mesh '%s' is empty: %zu vertices, %zu triangles\n",
                     path.c_str(),
                     mesh.vertices.size(),
                     mesh.indices.size() / 3);
        return false;
    }

    X3DomWriterOptions options;
    options.title = title;
    options.max_triangles = 500000;
    if (!write_mesh_x3dom_file(path.c_str(), mesh, options, error)) {
        std::fprintf(stderr, "Failed to write mesh '%s': %s\n", path.c_str(), error->c_str());
        return false;
    }

    std::printf("Wrote %s\n", path.c_str());
    return true;
}

} // namespace

int main() {
    const std::string output_dir = "src/mbmesh_dev/output";
    std::filesystem::create_directories(output_dir);

    const OrientedPointCloud exact_oriented_pointcloud = generate_exact_oriented_terrain_arch();
    const PointCloud pure_pointcloud = strip_normals(exact_oriented_pointcloud);
    const PoissonOptions options = terrain_arch_poisson_options();

    std::printf("Generated synthetic terrain arch with %zu exact oriented points\n",
                exact_oriented_pointcloud.oriented_points.size());

    const Mesh exact_normal_mesh = generate_mesh_poisson(exact_oriented_pointcloud, options);
    std::printf("Exact-normal screened Poisson mesh: %zu vertices, %zu triangles\n",
                exact_normal_mesh.vertices.size(),
                exact_normal_mesh.indices.size() / 3);
    print_mesh_bounds("Exact-normal mesh", exact_normal_mesh);

    const OrientedPointCloud pca_positive_z_oriented_pointcloud =
        estimate_oriented_points(pure_pointcloud, options.normal_neighbors);
    print_normal_comparison(exact_oriented_pointcloud, pca_positive_z_oriented_pointcloud);

    const Mesh pca_positive_z_mesh = generate_mesh_poisson(pca_positive_z_oriented_pointcloud, options);
    std::printf("PCA positive-Z screened Poisson mesh: %zu vertices, %zu triangles\n",
                pca_positive_z_mesh.vertices.size(),
                pca_positive_z_mesh.indices.size() / 3);
    print_mesh_bounds("PCA positive-Z mesh", pca_positive_z_mesh);

    std::string error;
    const std::string exact_output = output_dir + "/synthetic_terrain_arch_poisson_exact_normals.html";
    const std::string pca_output = output_dir + "/synthetic_terrain_arch_poisson_pca_positive_z.html";

    if (!write_mesh_preview(exact_normal_mesh,
                            exact_output,
                            "MB-System synthetic terrain arch Poisson mesh with exact normals",
                            &error)) {
        return 1;
    }

    if (!write_mesh_preview(pca_positive_z_mesh,
                            pca_output,
                            "MB-System synthetic terrain arch Poisson mesh with PCA positive-Z normals",
                            &error)) {
        return 1;
    }

    return 0;
}
