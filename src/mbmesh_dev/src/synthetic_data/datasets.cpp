#include "synthetic_data/datasets.h"

#include "synthetic_data/noise.h"

#include <cmath>

namespace {

constexpr double pi = 3.14159265358979323846;

[[nodiscard]] bool invalid_steps(int first, int second) {
    return first < 2 || second < 2;
}

[[nodiscard]] double lerp(double a, double b, double t) {
    return a + t * (b - a);
}

[[nodiscard]] double step_fraction(int index, int count) {
    return count <= 1 ? 0.0 : static_cast<double>(index) / static_cast<double>(count - 1);
}

void append_cave_wall(PointCloud &pointcloud,
                      double width,
                      double height,
                      int width_steps,
                      int height_steps,
                      double y_offset) {
    for (int iz = 0; iz < height_steps; iz++) {
        const double z_fraction = step_fraction(iz, height_steps);
        const double z = lerp(-0.5 * height, 0.5 * height, z_fraction);
        for (int ix = 0; ix < width_steps; ix++) {
            const double x_fraction = step_fraction(ix, width_steps);
            const double x = lerp(-0.5 * width, 0.5 * width, x_fraction);
            const double broad = 0.45 * std::sin(1.4 * x + 0.7 * z);
            const double ledge = 0.30 * std::tanh(4.0 * (z - 0.15 * std::sin(2.0 * x)));
            const double roughness = 0.12 * perlin_noise(0.8 * x, 1.1 * z, 3.0);
            pointcloud.points.push_back(Vec3(x, y_offset + broad + ledge + roughness, z));
        }
    }
}

[[nodiscard]] double terrain_height(double x, double y) {
    const double slope = 0.10 * x - 0.06 * y;
    const double ridge = 0.32 * std::exp(-0.75 * (y + 0.9) * (y + 0.9));
    const double trench = -0.22 * std::exp(-1.2 * (x - 1.1) * (x - 1.1));
    const double ripple = 0.16 * std::sin(2.8 * x + 0.7 * y) * std::cos(1.6 * y);
    const double roughness = 0.07 * perlin_noise(1.7 * x, 1.7 * y, 4.0);
    return slope + ridge + trench + ripple + roughness;
}

} // namespace

PointCloud generate_sphere_pointcloud(double radius, int latitude_steps, int longitude_steps) {
    PointCloud pointcloud;
    if (radius <= 0.0 || invalid_steps(latitude_steps, longitude_steps)) {
        return pointcloud;
    }

    pointcloud.points.reserve(static_cast<std::size_t>(latitude_steps) * static_cast<std::size_t>(longitude_steps));
    for (int i = 0; i < latitude_steps; i++) {
        const double theta = pi * step_fraction(i, latitude_steps);
        const double sin_theta = std::sin(theta);
        const double cos_theta = std::cos(theta);
        for (int j = 0; j < longitude_steps; j++) {
            const double phi = 2.0 * pi * static_cast<double>(j) / static_cast<double>(longitude_steps);
            pointcloud.points.push_back(Vec3(radius * sin_theta * std::cos(phi),
                                             radius * sin_theta * std::sin(phi),
                                             radius * cos_theta));
        }
    }

    return pointcloud;
}

PointCloud generate_torus_pointcloud(double major_radius,
                                     double minor_radius,
                                     int major_steps,
                                     int minor_steps) {
    PointCloud pointcloud;
    if (major_radius <= 0.0 || minor_radius <= 0.0 || invalid_steps(major_steps, minor_steps)) {
        return pointcloud;
    }

    pointcloud.points.reserve(static_cast<std::size_t>(major_steps) * static_cast<std::size_t>(minor_steps));
    for (int i = 0; i < major_steps; i++) {
        const double u = 2.0 * pi * static_cast<double>(i) / static_cast<double>(major_steps);
        for (int j = 0; j < minor_steps; j++) {
            const double v = 2.0 * pi * static_cast<double>(j) / static_cast<double>(minor_steps);
            const double radial = major_radius + minor_radius * std::cos(v);
            pointcloud.points.push_back(Vec3(radial * std::cos(u),
                                             radial * std::sin(u),
                                             minor_radius * std::sin(v)));
        }
    }

    return pointcloud;
}

PointCloud generate_arch_pointcloud(double radius, double length, int arch_steps, int length_steps) {
    PointCloud pointcloud;
    if (radius <= 0.0 || length <= 0.0 || invalid_steps(arch_steps, length_steps)) {
        return pointcloud;
    }

    const double wall_height = 0.8 * radius;
    pointcloud.points.reserve(static_cast<std::size_t>(length_steps) *
                              static_cast<std::size_t>(arch_steps + 2 * arch_steps));

    for (int iy = 0; iy < length_steps; iy++) {
        const double y = lerp(-0.5 * length, 0.5 * length, step_fraction(iy, length_steps));

        for (int i = 0; i < arch_steps; i++) {
            const double theta = pi * step_fraction(i, arch_steps);
            pointcloud.points.push_back(Vec3(radius * std::cos(theta), y, radius * std::sin(theta)));
        }

        for (int iz = 0; iz < arch_steps; iz++) {
            const double z = lerp(-wall_height, 0.0, step_fraction(iz, arch_steps));
            pointcloud.points.push_back(Vec3(-radius, y, z));
            pointcloud.points.push_back(Vec3(radius, y, z));
        }
    }

    return pointcloud;
}

PointCloud generate_cave_wall_pointcloud(double width, double height, int width_steps, int height_steps) {
    PointCloud pointcloud;
    if (width <= 0.0 || height <= 0.0 || invalid_steps(width_steps, height_steps)) {
        return pointcloud;
    }

    pointcloud.points.reserve(static_cast<std::size_t>(width_steps) * static_cast<std::size_t>(height_steps));
    append_cave_wall(pointcloud, width, height, width_steps, height_steps, 0.0);
    return pointcloud;
}

PointCloud generate_two_sheet_overhang_pointcloud(double width, double length, int width_steps, int length_steps) {
    PointCloud pointcloud;
    if (width <= 0.0 || length <= 0.0 || invalid_steps(width_steps, length_steps)) {
        return pointcloud;
    }

    pointcloud.points.reserve(static_cast<std::size_t>(width_steps) * static_cast<std::size_t>(length_steps) * 2);
    for (int iy = 0; iy < length_steps; iy++) {
        const double y = lerp(-0.5 * length, 0.5 * length, step_fraction(iy, length_steps));
        for (int ix = 0; ix < width_steps; ix++) {
            const double x = lerp(-0.5 * width, 0.5 * width, step_fraction(ix, width_steps));
            const double fold = 0.25 * std::sin(2.2 * x) + 0.12 * std::cos(1.7 * y);
            const double lower_z = -0.35 + fold;
            const double upper_z = 0.45 + 0.75 * fold + 0.18 * std::sin(2.0 * y);
            pointcloud.points.push_back(Vec3(x, y, lower_z));
            pointcloud.points.push_back(Vec3(x + 0.35 * std::sin(1.5 * y), y, upper_z));
        }
    }

    return pointcloud;
}

PointCloud generate_terrain_half_torus_arch_pointcloud(double width,
                                                       double length,
                                                       int terrain_width_steps,
                                                       int terrain_length_steps,
                                                       int arch_steps,
                                                       int tube_steps) {
    PointCloud pointcloud;
    if (width <= 0.0 || length <= 0.0 ||
        invalid_steps(terrain_width_steps, terrain_length_steps) ||
        invalid_steps(arch_steps, tube_steps)) {
        return pointcloud;
    }

    pointcloud.points.reserve(static_cast<std::size_t>(terrain_width_steps) *
                                  static_cast<std::size_t>(terrain_length_steps) +
                              static_cast<std::size_t>(arch_steps) *
                                  static_cast<std::size_t>(tube_steps));

    for (int iy = 0; iy < terrain_length_steps; iy++) {
        const double y = lerp(-0.5 * length, 0.5 * length, step_fraction(iy, terrain_length_steps));
        for (int ix = 0; ix < terrain_width_steps; ix++) {
            const double x = lerp(-0.5 * width, 0.5 * width, step_fraction(ix, terrain_width_steps));
            pointcloud.points.push_back(Vec3(x, y, terrain_height(x, y)));
        }
    }

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
        const Vec3 center(center_x, y_center, center_z);

        for (int iv = 0; iv < tube_steps; iv++) {
            const double v = 2.0 * pi * static_cast<double>(iv) / static_cast<double>(tube_steps);
            const Vec3 point = center +
                               radial * (tube_radius * std::cos(v)) +
                               Vec3(0.0, tube_radius * std::sin(v), 0.0);
            pointcloud.points.push_back(point);
        }
    }

    return pointcloud;
}
