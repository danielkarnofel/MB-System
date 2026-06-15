#include "algorithms/decimation.h"

#include <cmath>
#include <map>
#include <utility>

namespace {

struct CellAccumulator {
    Vec3 sum;
    unsigned int count = 0;
};

[[nodiscard]] long long cell_index(double value, double size) {
    return static_cast<long long>(std::floor(value / size));
}

} // namespace

PointCloud decimate_pointcloud_2d(const PointCloud &pointcloud, double size) {
    if (size <= 0.0) {
        return pointcloud;
    }

    std::map<std::pair<long long, long long>, CellAccumulator> cells;
    for (const Point &point : pointcloud.points) {
        const std::pair<long long, long long> key(cell_index(point.x, size), cell_index(point.y, size));
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
