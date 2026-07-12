#include "algorithms/decimation.h"

#include <cmath>
#include <map>
#include <utility>

namespace {

using Grid2DKey = std::pair<long long, long long>;
using Grid3DKey = std::pair<Grid2DKey, long long>;

struct CellAccumulator {
    Vec3 sum;
    unsigned int count = 0;
};

Bounds3D pointcloud_bounds(const PointCloud &pointcloud) {
    Bounds3D bounds(pointcloud.points[0], pointcloud.points[0]);
    for (const Point &point : pointcloud.points) {
        bounds.include(point);
    }
    return bounds;
}

Grid3DKey grid_3d_key(const Vec3 &point, const Vec3 &origin, double cell_size) {
    return Grid3DKey(
        Grid2DKey(cell_index(point.x, origin.x, cell_size),
                  cell_index(point.y, origin.y, cell_size)),
        cell_index(point.z, origin.z, cell_size));
}

}

PointCloud decimate_pointcloud_2d(const PointCloud &pointcloud, double cell_size) {
    if (pointcloud.points.empty() || cell_size <= 0.0) {
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

    std::map<Grid2DKey, CellAccumulator> cells;
    for (const Point &point : pointcloud.points) {
        const Grid2DKey key(cell_index(point.x, origin.x, cell_size),
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

PointCloud decimate_pointcloud_3d(const PointCloud &pointcloud, double cell_size) {
    if (pointcloud.points.empty() || cell_size <= 0.0) {
        return pointcloud;
    }

    const Bounds3D bounds = pointcloud_bounds(pointcloud);
    const Vec3 origin = bounds.min;
    std::map<Grid3DKey, CellAccumulator> cells;
    for (const Point &point : pointcloud.points) {
        CellAccumulator &cell = cells[grid_3d_key(point, origin, cell_size)];
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
OrientedPointCloud decimate_oriented_pointcloud_3d(const OrientedPointCloud &oriented_pointcloud, double cell_size) {
    OrientedPointCloud decimated_oriented_pointcloud = oriented_pointcloud;
    return decimated_oriented_pointcloud;
}
