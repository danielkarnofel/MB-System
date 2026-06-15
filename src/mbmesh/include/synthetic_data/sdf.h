#pragma once

#include "noise.h"

[[nodiscard]] double sphere_sdf(double x, double y, double z);

[[nodiscard]] double sphere_sdf(double x, double y, double z, double radius);

[[nodiscard]] double perturbed_plane_sdf(double x, double y, double z);
