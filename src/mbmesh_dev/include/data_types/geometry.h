
#pragma once

#include <cassert>
#include <cstddef>
#include <vector>
#include <map>

#include "../math/vec3.h"

// ====================================================================================================

using Point = Vec3;

struct PointCloud {
    std::vector<Point> points;
};

// ====================================================================================================

struct CollectedPoint {
    Vec3 point;
    Vec3 origin;
};

struct CollectedPointCloud {
    std::vector<CollectedPoint> collected_points;
};

// ====================================================================================================

struct OrientedPoint {
    Vec3 point;
    Vec3 normal;
    Vec3 lambdas;

    OrientedPoint(const Vec3 &point) : point(point), normal(Vec3(0.0, 0.0, 1.0)), lambdas(Vec3(0.0)) {}
    OrientedPoint(const Vec3 &point, const Vec3 &normal) : point(point), normal(normal), lambdas(Vec3(0.0)) {}
};

struct OrientedPointCloud {
    std::vector<OrientedPoint> oriented_points;
};

// ====================================================================================================

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    std::vector<unsigned int> indices;
};

// ====================================================================================================

struct GLB {
    // TODO: 
};

// ====================================================================================================

// ====================================================================================================

struct Grid2D {
    Vec3 origin;
    double cell_size = 0.0;
    std::map<std::pair<long long, long long>, unsigned int> cells;
};

// ====================================================================================================

struct Bounds3D {
    Vec3 min;
    Vec3 max;

    Bounds3D() : min(Vec3(0.0)), max(Vec3(0.0)) {}
    Bounds3D(const Vec3 &min_value, const Vec3 &max_value) : min(min_value), max(max_value) {}

    [[nodiscard]] Vec3 size() const { return max - min; }
    [[nodiscard]] Vec3 center() const { return (min + max) * 0.5; }

    void expand(double padding) {
        const Vec3 offset(padding);
        min -= offset;
        max += offset;
    }

    void include(const Vec3 &p) {
        if (p.x < min.x) {
            min.x = p.x;
        }
        if (p.y < min.y) {
            min.y = p.y;
        }
        if (p.z < min.z) {
            min.z = p.z;
        }
        if (p.x > max.x) {
            max.x = p.x;
        }
        if (p.y > max.y) {
            max.y = p.y;
        }
        if (p.z > max.z) {
            max.z = p.z;
        }
    }
};

// ====================================================================================================

struct ScalarGrid3D {
    Vec3 origin;
    double cell_size = 1.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<double> values;

    ScalarGrid3D() = default;

    ScalarGrid3D(int x_count, int y_count, int z_count, double grid_cell_size, const Vec3 &grid_origin = Vec3(0.0))
        : origin(grid_origin),
          cell_size(grid_cell_size),
          nx(x_count),
          ny(y_count),
          nz(z_count),
          values(value_count(x_count, y_count, z_count), 0.0) {}

    [[nodiscard]] static std::size_t value_count(int x_count, int y_count, int z_count) {
        assert(x_count >= 0 && y_count >= 0 && z_count >= 0);
        return static_cast<std::size_t>(x_count) *
               static_cast<std::size_t>(y_count) *
               static_cast<std::size_t>(z_count);
    }

    [[nodiscard]] bool contains(int x, int y, int z) const {
        return x >= 0 && x < nx &&
               y >= 0 && y < ny &&
               z >= 0 && z < nz;
    }

    [[nodiscard]] std::size_t index(int x, int y, int z) const {
        assert(contains(x, y, z));
        return static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nx) +
               static_cast<std::size_t>(y) * static_cast<std::size_t>(nx) +
               static_cast<std::size_t>(x);
    }

    double &at(int x, int y, int z) { return values[index(x, y, z)]; }
    [[nodiscard]] double at(int x, int y, int z) const { return values[index(x, y, z)]; }

    [[nodiscard]] Vec3 position(int x, int y, int z) const {
        return origin + Vec3(static_cast<double>(x),
                             static_cast<double>(y),
                             static_cast<double>(z)) * cell_size;
    }
};

struct VectorGrid3D {
    Vec3 origin;
    double cell_size = 1.0;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<Vec3> values;

    VectorGrid3D() = default;

    VectorGrid3D(int x_count, int y_count, int z_count, double grid_cell_size, const Vec3 &grid_origin = Vec3(0.0))
        : origin(grid_origin),
          cell_size(grid_cell_size),
          nx(x_count),
          ny(y_count),
          nz(z_count),
          values(ScalarGrid3D::value_count(x_count, y_count, z_count), Vec3(0.0)) {}

    [[nodiscard]] bool contains(int x, int y, int z) const {
        return x >= 0 && x < nx &&
               y >= 0 && y < ny &&
               z >= 0 && z < nz;
    }

    [[nodiscard]] std::size_t index(int x, int y, int z) const {
        assert(contains(x, y, z));
        return static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nx) +
               static_cast<std::size_t>(y) * static_cast<std::size_t>(nx) +
               static_cast<std::size_t>(x);
    }

    Vec3 &at(int x, int y, int z) { return values[index(x, y, z)]; }
    [[nodiscard]] Vec3 at(int x, int y, int z) const { return values[index(x, y, z)]; }

    [[nodiscard]] Vec3 position(int x, int y, int z) const {
        return origin + Vec3(static_cast<double>(x),
                             static_cast<double>(y),
                             static_cast<double>(z)) * cell_size;
    }
};

[[nodiscard]] inline long long cell_index(double value, double origin, double cell_size) {
    return static_cast<long long>(std::floor((value - origin) / cell_size));
}
