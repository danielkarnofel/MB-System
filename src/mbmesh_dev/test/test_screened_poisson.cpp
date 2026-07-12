#include "algorithms/mesh_generation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace {

OrientedPointCloud generate_flat_plane_oriented_pointcloud(double width, int steps) {
    OrientedPointCloud pointcloud;
    if (width <= 0.0 || steps < 2) {
        return pointcloud;
    }

    pointcloud.oriented_points.reserve(static_cast<std::size_t>(steps) *
                                       static_cast<std::size_t>(steps));

    for (int y = 0; y < steps; y++) {
        const double fy = static_cast<double>(y) / static_cast<double>(steps - 1);
        const double py = width * (fy - 0.5);
        for (int x = 0; x < steps; x++) {
            const double fx = static_cast<double>(x) / static_cast<double>(steps - 1);
            const double px = width * (fx - 0.5);
            pointcloud.oriented_points.push_back(OrientedPoint(Vec3(px, py, 0.0), Vec3(0.0, 0.0, 1.0)));
        }
    }

    return pointcloud;
}

double trilinear_sample_scalar_grid(const ScalarGrid3D &grid, const Vec3 &position) {
    if (grid.nx < 2 || grid.ny < 2 || grid.nz < 2) {
        return 0.0;
    }

    const Vec3 grid_coordinate = (position - grid.origin) / grid.cell_size;
    const int x0 = std::clamp(static_cast<int>(std::floor(grid_coordinate.x)), 0, grid.nx - 2);
    const int y0 = std::clamp(static_cast<int>(std::floor(grid_coordinate.y)), 0, grid.ny - 2);
    const int z0 = std::clamp(static_cast<int>(std::floor(grid_coordinate.z)), 0, grid.nz - 2);
    const int x1 = x0 + 1;
    const int y1 = y0 + 1;
    const int z1 = z0 + 1;

    const double tx = std::clamp(grid_coordinate.x - static_cast<double>(x0), 0.0, 1.0);
    const double ty = std::clamp(grid_coordinate.y - static_cast<double>(y0), 0.0, 1.0);
    const double tz = std::clamp(grid_coordinate.z - static_cast<double>(z0), 0.0, 1.0);

    const double c00 = grid.at(x0, y0, z0) * (1.0 - tx) + grid.at(x1, y0, z0) * tx;
    const double c10 = grid.at(x0, y1, z0) * (1.0 - tx) + grid.at(x1, y1, z0) * tx;
    const double c01 = grid.at(x0, y0, z1) * (1.0 - tx) + grid.at(x1, y0, z1) * tx;
    const double c11 = grid.at(x0, y1, z1) * (1.0 - tx) + grid.at(x1, y1, z1) * tx;
    const double c0 = c00 * (1.0 - ty) + c10 * ty;
    const double c1 = c01 * (1.0 - ty) + c11 * ty;
    return c0 * (1.0 - tz) + c1 * tz;
}

double average_sample_value(const ScalarGrid3D &grid, const OrientedPointCloud &pointcloud) {
    double sum = 0.0;
    for (const OrientedPoint &oriented_point : pointcloud.oriented_points) {
        sum += trilinear_sample_scalar_grid(grid, oriented_point.point);
    }
    return sum / static_cast<double>(pointcloud.oriented_points.size());
}

bool scalar_grid_has_finite_values(const ScalarGrid3D &grid) {
    for (const double value : grid.values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

double scalar_grid_value_range(const ScalarGrid3D &grid) {
    if (grid.values.empty()) {
        return 0.0;
    }

    double min_value = grid.values[0];
    double max_value = grid.values[0];
    for (const double value : grid.values) {
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
    }
    return max_value - min_value;
}

} // namespace

int main() {
    const OrientedPointCloud plane = generate_flat_plane_oriented_pointcloud(1.0, 13);

    ScreenedPoissonOptions options;
    options.cell_size = 0.08;
    options.padding = 0.24;
    options.normal_splat_radius = 0.12;
    options.screening_weight = 4.0;
    options.solver_iterations = 200;
    options.use_screening = true;
    options.estimate_iso_value_from_samples = true;

    const ScalarGrid3D scalar_field = screened_poisson(plane, options);
    std::printf("Screened Poisson plane field: %d x %d x %d nodes, %zu values\n",
                scalar_field.nx,
                scalar_field.ny,
                scalar_field.nz,
                scalar_field.values.size());

    if (scalar_field.nx < 2 || scalar_field.ny < 2 || scalar_field.nz < 2 || scalar_field.values.empty()) {
        std::fprintf(stderr, "Expected a non-empty reconstruction grid\n");
        return 1;
    }

    if (!scalar_grid_has_finite_values(scalar_field)) {
        std::fprintf(stderr, "Scalar field contains non-finite values\n");
        return 1;
    }

    const double value_range = scalar_grid_value_range(scalar_field);
    std::printf("Scalar field value range: %.6f\n", value_range);
    if (value_range <= 1.0e-5) {
        std::fprintf(stderr, "Expected scalar field to vary after Poisson solve\n");
        return 1;
    }

    const double average_surface_value = average_sample_value(scalar_field, plane);
    std::printf("Average scalar value at plane samples: %.6f\n", average_surface_value);
    if (std::fabs(average_surface_value) > 1.0e-6) {
        std::fprintf(stderr, "Expected sample-centered iso value near zero\n");
        return 1;
    }

    const double above_value = trilinear_sample_scalar_grid(scalar_field, Vec3(0.0, 0.0, 0.16));
    const double below_value = trilinear_sample_scalar_grid(scalar_field, Vec3(0.0, 0.0, -0.16));
    std::printf("Scalar probe above plane: %.6f\n", above_value);
    std::printf("Scalar probe below plane: %.6f\n", below_value);
    if (std::fabs(above_value - below_value) <= 1.0e-5) {
        std::fprintf(stderr, "Expected scalar field to distinguish the two sides of the oriented plane\n");
        return 1;
    }

    return 0;
}
