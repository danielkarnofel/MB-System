#pragma once

#include <cstddef>

#include "../data_types/geometry.h"

#include "../include/math/eigen.h"
#include "../include/math/kdtree.h"
#include "../include/math/mat3.h"

struct NormalEstimationOptions {
    // Number of neighboring samples used with the query point for the local
    // PCA plane fit. Larger values suppress noise but smooth small features.
    std::size_t k = 12;
};

/*******************************************************************************
 * Converts a CollectedPoint set to an OrientedPoint set using PCA (Principal 
 * Component Analysis) normal estimation.
 ******************************************************************************/
OrientedPointCloud normal_estimation(
    const CollectedPointCloud &collected_points, 
    NormalEstimationOptions options
);
