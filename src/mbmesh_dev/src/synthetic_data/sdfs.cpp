#include "synthetic_data/sdfs.h"

#include "data_types/geometry.h"
#include "synthetic_data/noise.h"

#include <cmath>

namespace {

[[nodiscard]] double length_xy(double x, double y) {
    return std::sqrt(x * x + y * y);
}

} // namespace

double sphere_sdf(double x, double y, double z) {
    constexpr double radius = 1.0;
    return Vec3(x, y, z).length() - radius;
}

double plane_sdf(double x, double y, double z) {
    (void)x;
    (void)y;
    return z;
}

double tilted_plane_sdf(double x, double y, double z) {
    return z - (0.20 * x - 0.10 * y);
}

double perturbed_plane_sdf(double x, double y, double z) {
    const double ripple = 0.2 * perlin_noise(x * 0.35, y * 0.35, 0.0);
    const double wave = 0.1 * std::sin(x * 0.5) * std::cos(y * 0.35);
    return z - ripple - wave;
}

double torus_sdf(double x, double y, double z) {
    constexpr double major_radius = 1.0;
    constexpr double minor_radius = 0.25;
    const double radial_distance = length_xy(x, y) - major_radius;
    return std::sqrt(radial_distance * radial_distance + z * z) - minor_radius;
}
