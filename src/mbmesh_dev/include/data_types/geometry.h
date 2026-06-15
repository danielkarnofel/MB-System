
#pragma once

#include <vector>

#include "../math/vec3.h"

using Point = Vec3;

struct PointCloud {
    std::vector<Point> points;
};

struct OrientedPoint {
    Vec3 point;
    Vec3 normal;
    Vec3 lambdas;

    OrientedPoint() : point(Vec3(0.0)), normal(Vec3(0.0, 0.0, 1.0)), lambdas(Vec3(0.0)) {}

    OrientedPoint(const Vec3 &point, const Vec3 &normal) : point(point), normal(normal), lambdas(Vec3(0.0)) {}
};

struct OrientedPointCloud {
    std::vector<OrientedPoint> oriented_points;
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<Vec3> normals;
    std::vector<unsigned int> indices;
};
