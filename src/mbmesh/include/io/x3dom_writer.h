#pragma once

#include "../data_types/geometry.h"

#include <cstddef>
#include <string>

struct X3DomColor {
    double r;
    double g;
    double b;

    X3DomColor() : r(0.129), g(0.267), b(0.431) {}
    X3DomColor(double red, double green, double blue) : r(red), g(green), b(blue) {}
};

struct X3DomWriterOptions {
    std::string title;
    X3DomColor point_color;
    X3DomColor normal_color;
    X3DomColor mesh_color;
    double point_size;
    double normal_scale;
    std::size_t max_points;
    std::size_t max_normal_vectors;

    X3DomWriterOptions()
        : title("MB-System mbmesh2 debug view"),
          point_color(0.129, 0.267, 0.431),
          normal_color(0.95, 0.35, 0.12),
          mesh_color(0.35, 0.55, 0.72),
          point_size(2.0),
          normal_scale(0.5),
          max_points(250000),
          max_normal_vectors(10000) {}
};

[[nodiscard]] bool write_pointcloud_x3dom_file(const char *filename,
                                               const PointCloud &pointcloud,
                                               const X3DomWriterOptions &options,
                                               std::string *error_message);

[[nodiscard]] bool write_oriented_pointcloud_x3dom_file(const char *filename,
                                                        const OrientedPointCloud &oriented_pointcloud,
                                                        const X3DomWriterOptions &options,
                                                        std::string *error_message);

inline bool write_pointcloud_x3dom_file(const char *filename,
                                        const PointCloud &pointcloud,
                                        std::string *error_message = nullptr) {
    return write_pointcloud_x3dom_file(filename, pointcloud, X3DomWriterOptions(), error_message);
}

inline bool write_oriented_pointcloud_x3dom_file(const char *filename,
                                                 const OrientedPointCloud &oriented_pointcloud,
                                                 std::string *error_message = nullptr) {
    return write_oriented_pointcloud_x3dom_file(filename, oriented_pointcloud, X3DomWriterOptions(), error_message);
}
