#pragma once

using HeightFunction = double (*)(double x, double y);

[[nodiscard]] double tilted_plane_height(double x, double y);

[[nodiscard]] double rippled_heightfield(double x, double y);

[[nodiscard]] double ridge_heightfield(double x, double y);

[[nodiscard]] double trench_heightfield(double x, double y);

[[nodiscard]] double mound_heightfield(double x, double y);

[[nodiscard]] double cliff_heightfield(double x, double y);

[[nodiscard]] double noisy_heightfield(double x, double y);
