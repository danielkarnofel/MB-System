#pragma once

using SdfFunction = double (*)(double x, double y, double z);

[[nodiscard]] double sphere_sdf(double x, double y, double z);
[[nodiscard]] double plane_sdf(double x, double y, double z);
[[nodiscard]] double tilted_plane_sdf(double x, double y, double z);
[[nodiscard]] double perturbed_plane_sdf(double x, double y, double z);
[[nodiscard]] double torus_sdf(double x, double y, double z);
