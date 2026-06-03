#include <vector>
#include "vec3.h"

struct Vertex {
    Vec3 position;
    Vec3 normal;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

struct OrientedPoint {
    Vec3 p;
    Vec3 n;
};

struct ScalarField {
    unsigned int width;
    unsigned int depth;
    unsigned int height;
    std::vector<std::vector<std::vector<double>>> values;

    // TODO: getters/setters
};

enum class MeshGenerationMethod {
    Delauney,
    AlphaShape,
    BallPivoting,
    Poisson,
};

Mesh generate_mesh_Delauney(std::vector<Vec3>& points) {
    // TODO
}

Mesh generate_mesh_Poisson(std::vector<OrientedPoint>& oriented_points) {
    // TODO
}

Mesh marching_cubes(std::vector<double>, unsigned int width, unsigned int depth, unsigned int height) {
    // TODO
}