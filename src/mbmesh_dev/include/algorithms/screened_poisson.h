#pragma once

#include <algorithm>

#include "../data_types/geometry.h"

struct ScreenedPoissonOptions {
	double cell_size = 0.1;
	double padding = 0.3;
	double normal_splat_radius = 0.15;
	double screening_weight = 4.0;
	int solver_iterations = 200;
	bool use_screening = true;
	double iso_value = 0.0;
};

/******************************************************************************/

/******************************************************************************
 * Extracts an implicit surface from an OrientedPoint set.
 ******************************************************************************/

ScalarGrid3D screened_poisson(const OrientedPointCloud &oriented_points, ScreenedPoissonOptions options);
