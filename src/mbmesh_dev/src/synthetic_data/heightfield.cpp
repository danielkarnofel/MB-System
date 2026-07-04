#include "synthetic_data/heightfield.h"

#include "synthetic_data/noise.h"

#include <cmath>

double tilted_plane_height(double x, double y) {
    return 0.20 * x - 0.10 * y;
}

double rippled_heightfield(double x, double y) {
    return tilted_plane_height(x, y) + 0.25 * std::sin(x * 0.35) * std::cos(y * 0.25);
}

double ridge_heightfield(double x, double y) {
    return tilted_plane_height(x, y) + 2.0 * std::exp(-0.05 * y * y);
}

double trench_heightfield(double x, double y) {
    return tilted_plane_height(x, y) - 1.5 * std::exp(-0.08 * y * y);
}

double mound_heightfield(double x, double y) {
    return tilted_plane_height(x, y) + 1.8 * std::exp(-0.08 * (x * x + y * y));
}

double cliff_heightfield(double x, double y) {
    const double scarp = x < 0.0 ? -1.0 : 1.0;
    return 0.05 * y + scarp + 0.12 * std::sin(y * 0.4);
}

double noisy_heightfield(double x, double y) {
    const double broad_noise = 0.45 * perlin_noise(x * 0.12, y * 0.12, 0.0);
    const double fine_noise = 0.12 * perlin_noise(x * 0.55, y * 0.55, 7.0);
    return rippled_heightfield(x, y) + broad_noise + fine_noise;
}
