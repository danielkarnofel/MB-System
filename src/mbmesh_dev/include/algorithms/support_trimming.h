#pragma once

#include "../data_types/geometry.h"

struct SupportTrimmingOptions {

};

Mesh support_trimming(Mesh raw_mesh, OrientedPointCloud oriented_points, SupportTrimmingOptions options);