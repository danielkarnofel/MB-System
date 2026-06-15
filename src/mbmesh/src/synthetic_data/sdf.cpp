#include "synthetic_data/sdf.h"

#include <cmath>

double sphere_sdf(double x, double y, double z, double radius) {
    return std::sqrt(x * x + y * y + z * z) - radius;
}

double sphere_sdf(double x, double y, double z) {
    return sphere_sdf(x, y, z, 1.0);
}

double perturbed_plane_sdf(double x, double y, double z) {
    const double ripple = 0.2 * perlin_noise(x * 0.35, y * 0.35, 0.0);
    const double wave = 0.1 * std::sin(x * 0.5) * std::cos(y * 0.35);
    return z - ripple - wave;
}
