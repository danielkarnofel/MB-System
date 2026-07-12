#include "algorithms/mesh_generation.h"

#include "algorithms/marching_cubes_tables.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

constexpr int cube_offsets[8][3] = {
    {0, 0, 0},
    {1, 0, 0},
    {1, 1, 0},
    {0, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {1, 1, 1},
    {0, 1, 1},
};

constexpr int edge_corners[12][2] = {
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 4},
    {0, 4},
    {1, 5},
    {2, 6},
    {3, 7},
};

struct EdgeCache {
    int nx = 0;
    int ny = 0;
    int nz = 0;
    std::vector<int> x_edges;
    std::vector<int> y_edges;
    std::vector<int> z_edges;

    explicit EdgeCache(const ScalarGrid3D &grid)
        : nx(grid.nx),
          ny(grid.ny),
          nz(grid.nz),
          x_edges(static_cast<std::size_t>(std::max(0, grid.nx - 1)) *
                      static_cast<std::size_t>(grid.ny) *
                      static_cast<std::size_t>(grid.nz),
                  -1),
          y_edges(static_cast<std::size_t>(grid.nx) *
                      static_cast<std::size_t>(std::max(0, grid.ny - 1)) *
                      static_cast<std::size_t>(grid.nz),
                  -1),
          z_edges(static_cast<std::size_t>(grid.nx) *
                      static_cast<std::size_t>(grid.ny) *
                      static_cast<std::size_t>(std::max(0, grid.nz - 1)),
                  -1) {}

    int &edge_vertex(int x, int y, int z, int edge) {
        switch (edge) {
        case 0:
            return x_edge(x, y, z);
        case 1:
            return y_edge(x + 1, y, z);
        case 2:
            return x_edge(x, y + 1, z);
        case 3:
            return y_edge(x, y, z);
        case 4:
            return x_edge(x, y, z + 1);
        case 5:
            return y_edge(x + 1, y, z + 1);
        case 6:
            return x_edge(x, y + 1, z + 1);
        case 7:
            return y_edge(x, y, z + 1);
        case 8:
            return z_edge(x, y, z);
        case 9:
            return z_edge(x + 1, y, z);
        case 10:
            return z_edge(x + 1, y + 1, z);
        case 11:
            return z_edge(x, y + 1, z);
        default:
            assert(false);
            return x_edges[0];
        }
    }

private:
    int &x_edge(int x, int y, int z) {
        assert(x >= 0 && x < nx - 1);
        assert(y >= 0 && y < ny);
        assert(z >= 0 && z < nz);
        return x_edges[(static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) +
                        static_cast<std::size_t>(y)) *
                           static_cast<std::size_t>(nx - 1) +
                       static_cast<std::size_t>(x)];
    }

    int &y_edge(int x, int y, int z) {
        assert(x >= 0 && x < nx);
        assert(y >= 0 && y < ny - 1);
        assert(z >= 0 && z < nz);
        return y_edges[(static_cast<std::size_t>(z) * static_cast<std::size_t>(ny - 1) +
                        static_cast<std::size_t>(y)) *
                           static_cast<std::size_t>(nx) +
                       static_cast<std::size_t>(x)];
    }

    int &z_edge(int x, int y, int z) {
        assert(x >= 0 && x < nx);
        assert(y >= 0 && y < ny);
        assert(z >= 0 && z < nz - 1);
        return z_edges[(static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) +
                        static_cast<std::size_t>(y)) *
                           static_cast<std::size_t>(nx) +
                       static_cast<std::size_t>(x)];
    }
};

bool grid_is_valid(const ScalarGrid3D &grid) {
    return grid.nx >= 2 &&
           grid.ny >= 2 &&
           grid.nz >= 2 &&
           grid.cell_size > 0.0 &&
           grid.values.size() == ScalarGrid3D::value_count(grid.nx, grid.ny, grid.nz);
}

Vec3 interpolate_edge(const Vec3 &a_position,
                      const Vec3 &b_position,
                      double a_value,
                      double b_value,
                      double iso_value) {
    const double denominator = b_value - a_value;
    if (std::fabs(denominator) <= 1.0e-12) {
        return (a_position + b_position) * 0.5;
    }

    const double t = std::clamp((iso_value - a_value) / denominator, 0.0, 1.0);
    return a_position + (b_position - a_position) * t;
}

int cube_case_index(const double values[8], double iso_value) {
    int cube_index = 0;
    for (int i = 0; i < 8; i++) {
        if (values[i] < iso_value) {
            cube_index |= (1 << i);
        }
    }
    return cube_index;
}

unsigned int vertex_for_edge(Mesh &mesh,
                             EdgeCache &cache,
                             const Vec3 positions[8],
                             const double values[8],
                             int x,
                             int y,
                             int z,
                             int edge,
                             double iso_value) {
    int &cached_vertex = cache.edge_vertex(x, y, z, edge);
    if (cached_vertex >= 0) {
        return static_cast<unsigned int>(cached_vertex);
    }

    const int a = edge_corners[edge][0];
    const int b = edge_corners[edge][1];
    cached_vertex = static_cast<int>(mesh.vertices.size());
    mesh.vertices.push_back(interpolate_edge(positions[a], positions[b], values[a], values[b], iso_value));
    return static_cast<unsigned int>(cached_vertex);
}

void accumulate_vertex_normals(Mesh &mesh) {
    mesh.normals.assign(mesh.vertices.size(), Vec3(0.0));

    for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const unsigned int a = mesh.indices[i];
        const unsigned int b = mesh.indices[i + 1];
        const unsigned int c = mesh.indices[i + 2];
        const Vec3 face_normal = cross(mesh.vertices[b] - mesh.vertices[a],
                                       mesh.vertices[c] - mesh.vertices[a]);

        if (face_normal.length_squared() <= 1.0e-20) {
            continue;
        }

        mesh.normals[a] += face_normal;
        mesh.normals[b] += face_normal;
        mesh.normals[c] += face_normal;
    }

    for (Vec3 &normal : mesh.normals) {
        if (normal.length_squared() > 1.0e-20) {
            normal = normalize(normal);
        } else {
            normal = Vec3(0.0, 0.0, 1.0);
        }
    }
}

} // namespace

Mesh marching_cubes(const ScalarGrid3D &scalar_grid, MarchingCubesOptions options) {
    Mesh mesh;
    if (!grid_is_valid(scalar_grid)) {
        return mesh;
    }

    EdgeCache cache(scalar_grid);

    for (int z = 0; z < scalar_grid.nz - 1; z++) {
        for (int y = 0; y < scalar_grid.ny - 1; y++) {
            for (int x = 0; x < scalar_grid.nx - 1; x++) {
                Vec3 positions[8];
                double values[8];
                for (int i = 0; i < 8; i++) {
                    const int gx = x + cube_offsets[i][0];
                    const int gy = y + cube_offsets[i][1];
                    const int gz = z + cube_offsets[i][2];
                    positions[i] = scalar_grid.position(gx, gy, gz);
                    values[i] = scalar_grid.at(gx, gy, gz);
                }

                const int cube_index = cube_case_index(values, options.iso_value);
                if (edgeTable[cube_index] == 0) {
                    continue;
                }

                unsigned int edge_vertices[12] = {};
                for (int edge = 0; edge < 12; edge++) {
                    if ((edgeTable[cube_index] & (1 << edge)) != 0) {
                        edge_vertices[edge] = vertex_for_edge(mesh,
                                                              cache,
                                                              positions,
                                                              values,
                                                              x,
                                                              y,
                                                              z,
                                                              edge,
                                                              options.iso_value);
                    }
                }

                for (int i = 0; triangleTable[cube_index][i] != -1; i += 3) {
                    mesh.indices.push_back(edge_vertices[triangleTable[cube_index][i]]);
                    mesh.indices.push_back(edge_vertices[triangleTable[cube_index][i + 1]]);
                    mesh.indices.push_back(edge_vertices[triangleTable[cube_index][i + 2]]);
                }
            }
        }
    }

    accumulate_vertex_normals(mesh);
    return mesh;
}
