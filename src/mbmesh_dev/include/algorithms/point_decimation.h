#pragma once

#include "../data_types/geometry.h"

struct PointDecimationOptions {

};

CollectedPointCloud point_decimation(CollectedPointCloud collected_points, PointDecimationOptions options);