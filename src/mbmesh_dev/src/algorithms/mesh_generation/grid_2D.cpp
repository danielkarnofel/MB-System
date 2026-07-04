#include "algorithms/mesh_generation.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace {

using GridKey = std::pair<long long, long long>;

struct CellAccumulator {
    Vec3 point_sum;
    Vec3 normal_sum;
    unsigned int count = 0;
};

double triangle_area_xy(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    const double ab_x = b.x - a.x;
    const double ab_y = b.y - a.y;
    const double ac_x = c.x - a.x;
    const double ac_y = c.y - a.y;
    return 0.5 * std::fabs(ab_x * ac_y - ab_y * ac_x);
}

double positive_min_spacing(std::vector<double> values) {
    constexpr double epsilon = 1e-9;
    if (values.size() < 2) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());

    double spacing = std::numeric_limits<double>::max();
    double previous = values[0];
    for (std::size_t i = 1; i < values.size(); i++) {
        const double delta = values[i] - previous;
        if (delta > epsilon) {
            spacing = std::min(spacing, delta);
            previous = values[i];
        }
    }
    return spacing == std::numeric_limits<double>::max() ? 0.0 : spacing;
}

bool triangle_edges_within_limit(const Mesh &mesh, unsigned int a, unsigned int b, unsigned int c, double max_edge_length) {
    if (max_edge_length <= 0.0) {
        return true;
    }
    const double max_edge_length_squared = max_edge_length * max_edge_length;
    return distance_squared(mesh.vertices[a], mesh.vertices[b]) <= max_edge_length_squared &&
           distance_squared(mesh.vertices[b], mesh.vertices[c]) <= max_edge_length_squared &&
           distance_squared(mesh.vertices[c], mesh.vertices[a]) <= max_edge_length_squared;
}

void add_triangle_if_valid(Mesh &mesh, unsigned int a, unsigned int b, unsigned int c, Grid2DOptions options) {
    const Vec3 &pa = mesh.vertices[a];
    const Vec3 &pb = mesh.vertices[b];
    const Vec3 &pc = mesh.vertices[c];

    if (triangle_area_xy(pa, pb, pc) <= options.min_triangle_area) return;

    if (!triangle_edges_within_limit(mesh, a, b, c, options.max_edge_length)) return;

    const double winding = (pb.x - pa.x) * (pc.y - pa.y) - (pb.y - pa.y) * (pc.x - pa.x);
    if (winding >= 0.0) {
        mesh.indices.push_back(a);
        mesh.indices.push_back(b);
        mesh.indices.push_back(c);
    } else {
        mesh.indices.push_back(a);
        mesh.indices.push_back(c);
        mesh.indices.push_back(b);
    }
}

void add_grid_triangles(Mesh &mesh, const Grid2D &grid, Grid2DOptions options) {
    mesh.indices.reserve(grid.cells.size() * 6);
    for (const auto &entry : grid.cells) {
        const long long x = entry.first.first;
        const long long y = entry.first.second;

        const auto p00 = grid.cells.find(GridKey(x, y));
        const auto p10 = grid.cells.find(GridKey(x + 1, y));
        const auto p01 = grid.cells.find(GridKey(x, y + 1));
        const auto p11 = grid.cells.find(GridKey(x + 1, y + 1));

        if (p00 == grid.cells.end() || p10 == grid.cells.end() ||
            p01 == grid.cells.end() || p11 == grid.cells.end()) {
            continue;
        }

        const unsigned int i00 = p00->second;
        const unsigned int i10 = p10->second;
        const unsigned int i01 = p01->second;
        const unsigned int i11 = p11->second;

        const double diagonal_00_11 = distance_squared(mesh.vertices[i00], mesh.vertices[i11]);
        const double diagonal_10_01 = distance_squared(mesh.vertices[i10], mesh.vertices[i01]);

        if (diagonal_00_11 <= diagonal_10_01) {
            add_triangle_if_valid(mesh, i00, i10, i11, options);
            add_triangle_if_valid(mesh, i00, i11, i01, options);
        } else {
            add_triangle_if_valid(mesh, i00, i10, i01, options);
            add_triangle_if_valid(mesh, i10, i11, i01, options);
        }
    }
}

PointCloud oriented_to_pointcloud(const OrientedPointCloud &oriented_pointcloud) {
    PointCloud pointcloud;
    pointcloud.points.reserve(oriented_pointcloud.oriented_points.size());
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        pointcloud.points.push_back(oriented_point.point);
    }
    return pointcloud;
}

} // namespace

// ====================================================================================================

Grid2D pointcloud_to_grid_2D(const PointCloud &pointcloud, Mesh &mesh, Grid2DOptions options) {

    Grid2D grid;
    mesh = Mesh();
    const std::size_t point_count = pointcloud.points.size();
    if (point_count < 3) {
        return grid;
    }

    double min_x = pointcloud.points[0].x;
    double min_y = pointcloud.points[0].y;
    for (const Point &point : pointcloud.points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
    }

    if (options.cell_size <= 0.0) {
        return grid;
    }

    grid.origin = Vec3(min_x, min_y, 0.0);
    grid.cell_size = options.cell_size;

    std::map<GridKey, CellAccumulator> accumulators;
    for (const Point &point : pointcloud.points) {
        const GridKey key(cell_index(point.x, min_x, options.cell_size), cell_index(point.y, min_y, options.cell_size));
        CellAccumulator &cell = accumulators[key];
        cell.point_sum += point;
        cell.count++;
    }

    mesh.vertices.reserve(accumulators.size());
    for (const auto &entry : accumulators) {
        const CellAccumulator &cell = entry.second;
        if (cell.count == 0) {
            continue;
        }

        const unsigned int vertex_index = static_cast<unsigned int>(mesh.vertices.size());
        mesh.vertices.push_back(cell.point_sum / static_cast<double>(cell.count));
        grid.cells.emplace(entry.first, vertex_index);
    }

    return grid;
}

// ====================================================================================================

Mesh generate_mesh_grid_2D(const PointCloud &pointcloud, Grid2DOptions options) {

    Mesh mesh;
    const Grid2D grid = pointcloud_to_grid_2D(pointcloud, mesh, options);
    if (grid.cells.size() < 3) {
        return mesh;
    }

    add_grid_triangles(mesh, grid, options);
    return mesh;
}

Mesh generate_mesh_grid_2D(const OrientedPointCloud &oriented_pointcloud, Grid2DOptions options) {

    const PointCloud pointcloud = oriented_to_pointcloud(oriented_pointcloud);

    Mesh mesh;
    Grid2D grid = pointcloud_to_grid_2D(pointcloud, mesh, options);
    if (grid.cells.size() < 3) {
        return mesh;
    }

    std::map<GridKey, CellAccumulator> accumulators;
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        const GridKey key(cell_index(oriented_point.point.x, grid.origin.x, grid.cell_size),
                          cell_index(oriented_point.point.y, grid.origin.y, grid.cell_size));
        CellAccumulator &cell = accumulators[key];
        cell.normal_sum += oriented_point.normal;
        cell.count++;
    }

    mesh.normals.resize(mesh.vertices.size(), Vec3(0.0, 0.0, 1.0));
    for (const auto &entry : grid.cells) {
        const auto accumulator = accumulators.find(entry.first);
        if (accumulator != accumulators.end() && accumulator->second.count > 0) {
            mesh.normals[entry.second] = normalize(accumulator->second.normal_sum);
        }
    }

    add_grid_triangles(mesh, grid, options);
    return mesh;
}
