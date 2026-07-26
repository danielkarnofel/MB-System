#pragma once

#include "../data_types/geometry.h"

#include "../include/math/eigen.h"
#include "../include/math/kdtree.h"
#include "../include/math/mat3.h"

struct NormalEstimationOptions {
    std::size_t k = 10;
};

/*******************************************************************************
 * Converts a CollectedPoint set to an OrientedPoint set using PCA (Principal 
 * Component Analysis) normal estimation.
 ******************************************************************************/
OrientedPointCloud normal_estimation(
    const CollectedPointCloud &collected_points, 
    NormalEstimationOptions options
);