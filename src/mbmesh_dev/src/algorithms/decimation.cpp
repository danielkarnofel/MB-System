#include "algorithms/decimation.h"

#include <cmath>
#include <map>
#include <utility>

namespace {

struct CellAccumulator {
    Vec3 sum;
    unsigned int count = 0;
};

}

PointCloud decimate_pointcloud_2d(const PointCloud &pointcloud, double cell_size) {
    if (cell_size <= 0.0) {
        return pointcloud;
    }

    // Calculate origin from bounds (using min point)
    double min_x = pointcloud.points[0].x;
    for (const Point &point : pointcloud.points) {
        min_x = std::min(min_x, point.x);
    }
    double min_y = pointcloud.points[0].y;
    for (const Point &point : pointcloud.points) {
        min_y = std::min(min_y, point.y);
    }
    const Vec3 origin(min_x, min_y, 0.0);

    std::map<std::pair<long long, long long>, CellAccumulator> cells;
    for (const Point &point : pointcloud.points) {
        const std::pair<long long, long long> key(cell_index(point.x, origin.x, cell_size), 
                                                   cell_index(point.y, origin.y, cell_size));
        CellAccumulator &cell = cells[key];
        cell.sum += point;
        cell.count++;
    }

    PointCloud decimated_pointcloud;
    decimated_pointcloud.points.reserve(cells.size());
    for (const auto &entry : cells) {
        const CellAccumulator &cell = entry.second;
        if (cell.count > 0) {
            decimated_pointcloud.points.push_back(cell.sum / static_cast<double>(cell.count));
        }
    }

    return decimated_pointcloud;
}

// Stub, needs to be implemented
OrientedPointCloud decimate_oriented_pointcloud_2d(const OrientedPointCloud &oriented_pointcloud, double cell_size) {
    OrientedPointCloud decimated_oriented_pointcloud = oriented_pointcloud;
    return decimated_oriented_pointcloud;
}

// Stub, needs to be implemented
PointCloud decimate_pointcloud_3d(const PointCloud &pointcloud, double cell_size) {
    PointCloud decimated_pointcloud = pointcloud;
    return decimated_pointcloud;
}

// Stub, needs to be implemented
OrientedPointCloud decimate_oriented_pointcloud_3d(const OrientedPointCloud &oriented_pointcloud, double cell_size) {
    OrientedPointCloud decimated_oriented_pointcloud = oriented_pointcloud;
    return decimated_oriented_pointcloud;
}
