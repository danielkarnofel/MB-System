#include "io/x3dom_writer.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <ostream>
#include <string>
#include <vector>

namespace {

void set_error(std::string *error_message, const std::string &message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

std::string escape_html(const std::string &text) {
    std::string escaped;
    escaped.reserve(text.size());
    for (char c : text) {
        switch (c) {
        case '&':
            escaped += "&amp;";
            break;
        case '<':
            escaped += "&lt;";
            break;
        case '>':
            escaped += "&gt;";
            break;
        case '"':
            escaped += "&quot;";
            break;
        default:
            escaped += c;
            break;
        }
    }
    return escaped;
}

void write_color(std::ostream &output, const X3DomColor &color) {
    output << color.r << " " << color.g << " " << color.b;
}

X3DomColor normal_direction_color(const Vec3 &normal) {
    if (normal.length_squared() <= vec3_epsilon * vec3_epsilon) {
        return X3DomColor(0.5, 0.5, 0.5);
    }

    const Vec3 unit_normal = normalize(normal);
    return X3DomColor(0.5 * (unit_normal.x + 1.0),
                      0.5 * (unit_normal.y + 1.0),
                      0.5 * (unit_normal.z + 1.0));
}

X3DomColor triangle_normal_direction_color(const Mesh &mesh, std::size_t triangle_index) {
    const std::size_t index = triangle_index * 3;
    const Vec3 &a = mesh.vertices[mesh.indices[index]];
    const Vec3 &b = mesh.vertices[mesh.indices[index + 1]];
    const Vec3 &c = mesh.vertices[mesh.indices[index + 2]];
    return normal_direction_color(cross(b - a, c - a));
}

bool open_output(const char *filename, std::ofstream &output, std::string *error_message) {
    if (filename == nullptr || filename[0] == '\0') {
        set_error(error_message, "X3DOM output filename is empty");
        return false;
    }

    output.open(filename);
    if (!output) {
        set_error(error_message, std::string("Unable to create X3DOM file: ") + filename);
        return false;
    }

    output << std::fixed << std::setprecision(6);
    return true;
}

void write_page_begin(std::ostream &output, const std::string &title) {
    output << "<!doctype html>\n"
           << "<html>\n"
           << "<head>\n"
           << "  <meta charset=\"utf-8\">\n"
           << "  <title>" << escape_html(title) << "</title>\n"
           << "  <script src=\"https://www.x3dom.org/download/x3dom.js\"></script>\n"
           << "  <link rel=\"stylesheet\" href=\"https://www.x3dom.org/download/x3dom.css\">\n"
           << "  <style>\n"
           << "    html, body { margin: 0; width: 100%; height: 100%; overflow: hidden; }\n"
           << "    x3d { width: 100vw; height: 100vh; display: block; background: #f6f7f8; }\n"
           << "  </style>\n"
           << "</head>\n"
           << "<body>\n"
           << "<x3d>\n"
           << "  <scene>\n"
           << "    <navigationInfo type='\"EXAMINE\" \"ANY\"'></navigationInfo>\n"
           << "    <viewpoint position=\"0 -15 8\" orientation=\"1 0 0 1.05\"></viewpoint>\n";
}

void write_page_end(std::ostream &output) {
    output << "  </scene>\n"
           << "</x3d>\n"
           << "</body>\n"
           << "</html>\n";
}

void write_point_set(std::ostream &output,
                     const std::vector<Vec3> &points,
                     const X3DomColor &color,
                     double point_size,
                     std::size_t max_points) {
    const std::size_t count = std::min(max_points, points.size());
    if (count == 0) {
        return;
    }

    const std::size_t step = std::max<std::size_t>(1, (points.size() + count - 1) / count);

    output << "    <shape>\n"
           << "      <appearance>\n"
           << "        <material diffuseColor=\"";
    write_color(output, color);
    output << "\" emissiveColor=\"";
    write_color(output, color);
    output << "\"></material>\n"
           << "        <pointProperties pointSizeScaleFactor=\"" << point_size << "\"></pointProperties>\n"
           << "      </appearance>\n"
           << "      <pointSet>\n"
           << "        <coordinate point=\"";

    std::size_t point_index = 0;
    for (std::size_t i = 0; i < points.size() && point_index < count; i += step) {
        output << points[i].x << " " << points[i].y << " " << points[i].z;
        point_index++;
        if (point_index < count) {
            output << ", ";
        }
    }

    output << "\"></coordinate>\n"
           << "      </pointSet>\n"
           << "    </shape>\n";
}

void write_oriented_point_set(std::ostream &output,
                              const OrientedPointCloud &oriented_pointcloud,
                              const X3DomWriterOptions &options) {
    std::vector<Vec3> points;
    points.reserve(oriented_pointcloud.oriented_points.size());
    for (const OrientedPoint &oriented_point : oriented_pointcloud.oriented_points) {
        points.push_back(oriented_point.point);
    }

    write_point_set(output, points, options.point_color, options.point_size, options.max_points);
}

void write_normal_lines(std::ostream &output,
                        const OrientedPointCloud &oriented_pointcloud,
                        const X3DomWriterOptions &options) {
    const std::size_t count = std::min(options.max_normal_vectors, oriented_pointcloud.oriented_points.size());
    if (count == 0 || options.normal_scale <= 0.0) {
        return;
    }

    const std::size_t step = std::max<std::size_t>(1, (oriented_pointcloud.oriented_points.size() + count - 1) / count);

    output << "    <shape>\n"
           << "      <appearance>\n"
           << "        <material diffuseColor=\"";
    write_color(output, options.normal_color);
    output << "\" emissiveColor=\"";
    write_color(output, options.normal_color);
    output << "\"></material>\n"
           << "      </appearance>\n"
           << "      <indexedLineSet coordIndex=\"";

    std::size_t line_index = 0;
    for (std::size_t i = 0; i < oriented_pointcloud.oriented_points.size() && line_index < count; i += step) {
        output << line_index * 2 << " " << line_index * 2 + 1 << " -1";
        line_index++;
        if (line_index < count) {
            output << " ";
        }
    }

    output << "\">\n"
           << "        <coordinate point=\"";

    line_index = 0;
    for (std::size_t i = 0; i < oriented_pointcloud.oriented_points.size() && line_index < count; i += step) {
        const OrientedPoint &oriented_point = oriented_pointcloud.oriented_points[i];
        const Vec3 start = oriented_point.point;
        const Vec3 end = oriented_point.point + oriented_point.normal * options.normal_scale;
        output << start.x << " " << start.y << " " << start.z << ", "
               << end.x << " " << end.y << " " << end.z;
        line_index++;
        if (line_index < count) {
            output << ", ";
        }
    }

    output << "\"></coordinate>\n"
           << "      </indexedLineSet>\n"
           << "    </shape>\n";
}

bool validate_mesh(const Mesh &mesh, std::string *error_message) {
    if (mesh.vertices.empty()) {
        set_error(error_message, "Mesh has no vertices");
        return false;
    }

    if (mesh.indices.empty()) {
        set_error(error_message, "Mesh has no triangle indices");
        return false;
    }

    if (mesh.indices.size() % 3 != 0) {
        set_error(error_message, "Mesh index count is not divisible by 3");
        return false;
    }

    for (const unsigned int index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            set_error(error_message, "Mesh contains an out-of-range vertex index");
            return false;
        }
    }

    return true;
}

void write_mesh_shape(std::ostream &output,
                      const Mesh &mesh,
                      const X3DomWriterOptions &options) {
    const std::size_t triangle_count = mesh.indices.size() / 3;
    const std::size_t count = std::min(options.max_triangles, triangle_count);
    if (count == 0) {
        return;
    }

    output << "    <shape>\n"
           << "      <appearance>\n"
           << "        <material diffuseColor=\"1 1 1\" transparency=\"0.0\"></material>\n"
           << "      </appearance>\n"
           << "      <indexedFaceSet solid=\"false\" colorPerVertex=\"false\" coordIndex=\"";

    for (std::size_t i = 0; i < count; i++) {
        const std::size_t index = i * 3;
        output << mesh.indices[index] << " "
               << mesh.indices[index + 1] << " "
               << mesh.indices[index + 2] << " -1";
        if (i + 1 < count) {
            output << " ";
        }
    }

    output << "\">\n"
           << "        <coordinate point=\"";

    for (std::size_t i = 0; i < mesh.vertices.size(); i++) {
        const Vec3 &vertex = mesh.vertices[i];
        output << vertex.x << " " << vertex.y << " " << vertex.z;
        if (i + 1 < mesh.vertices.size()) {
            output << ", ";
        }
    }

    output << "\"></coordinate>\n";

    output << "        <color color=\"";
    for (std::size_t i = 0; i < count; i++) {
        const X3DomColor color = triangle_normal_direction_color(mesh, i);
        write_color(output, color);
        if (i + 1 < count) {
            output << ", ";
        }
    }
    output << "\"></color>\n";

    if (mesh.normals.size() == mesh.vertices.size()) {
        output << "        <normal vector=\"";
        for (std::size_t i = 0; i < mesh.normals.size(); i++) {
            const Vec3 &normal = mesh.normals[i];
            output << normal.x << " " << normal.y << " " << normal.z;
            if (i + 1 < mesh.normals.size()) {
                output << ", ";
            }
        }
        output << "\"></normal>\n";
    }

    output << "      </indexedFaceSet>\n"
           << "    </shape>\n";
}

} // namespace

bool write_pointcloud_x3dom_file(const char *filename,
                                 const PointCloud &pointcloud,
                                 const X3DomWriterOptions &options,
                                 std::string *error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    std::ofstream output;
    if (!open_output(filename, output, error_message)) {
        return false;
    }

    write_page_begin(output, options.title);
    write_point_set(output, pointcloud.points, options.point_color, options.point_size, options.max_points);
    write_page_end(output);
    return true;
}

bool write_oriented_pointcloud_x3dom_file(const char *filename,
                                          const OrientedPointCloud &oriented_pointcloud,
                                          const X3DomWriterOptions &options,
                                          std::string *error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    std::ofstream output;
    if (!open_output(filename, output, error_message)) {
        return false;
    }

    write_page_begin(output, options.title);
    write_oriented_point_set(output, oriented_pointcloud, options);
    write_normal_lines(output, oriented_pointcloud, options);
    write_page_end(output);
    return true;
}

bool write_mesh_x3dom_file(const char *filename,
                           const Mesh &mesh,
                           const X3DomWriterOptions &options,
                           std::string *error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    if (!validate_mesh(mesh, error_message)) {
        return false;
    }

    std::ofstream output;
    if (!open_output(filename, output, error_message)) {
        return false;
    }

    write_page_begin(output, options.title);
    write_mesh_shape(output, mesh, options);
    write_page_end(output);
    return true;
}

bool write_oriented_pointcloud_mesh_x3dom_file(const char *filename,
                                               const OrientedPointCloud &oriented_pointcloud,
                                               const Mesh &mesh,
                                               const X3DomWriterOptions &options,
                                               std::string *error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    if (!validate_mesh(mesh, error_message)) {
        return false;
    }

    std::ofstream output;
    if (!open_output(filename, output, error_message)) {
        return false;
    }

    write_page_begin(output, options.title);
    write_mesh_shape(output, mesh, options);
    write_oriented_point_set(output, oriented_pointcloud, options);
    write_normal_lines(output, oriented_pointcloud, options);
    write_page_end(output);
    return true;
}
