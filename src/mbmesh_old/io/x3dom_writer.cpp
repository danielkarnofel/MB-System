#include "x3dom_writer.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {

void set_error(std::string* error_message, const std::string& message) {
  if (error_message != nullptr) {
    *error_message = message;
  }
}

std::string escape_html(const std::string& text) {
  std::string escaped;
  escaped.reserve(text.size());
  for (std::size_t i = 0; i < text.size(); i++) {
    switch (text[i]) {
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
      escaped += text[i];
      break;
    }
  }
  return escaped;
}

Vec3 apply_frame(const Vec3& point, const CoordinateFrame& frame) {
  return {
      frame.origin.x + point.x * frame.scale.x,
      frame.origin.y + point.y * frame.scale.y,
      frame.origin.z + point.z * frame.scale.z};
}

void write_page_begin(std::ostream& output, const std::string& title) {
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
         << "    <viewpoint position=\"0 0 120\" orientation=\"0 0 0 0\"></viewpoint>\n";
}

void write_page_end(std::ostream& output) {
  output << "  </scene>\n"
         << "</x3d>\n"
         << "</body>\n"
         << "</html>\n";
}

void write_color(std::ostream& output, const X3DomColor& color) {
  output << color.r << " " << color.g << " " << color.b;
}

bool open_output(const char* filename, std::ofstream* output, std::string* error_message) {
  if (filename == nullptr || filename[0] == '\0') {
    set_error(error_message, "X3DOM output filename is empty");
    return false;
  }

  output->open(filename);
  if (!*output) {
    set_error(error_message, std::string("Unable to create X3DOM file: ") + filename);
    return false;
  }

  (*output) << std::fixed << std::setprecision(6);
  return true;
}

void write_point_set(std::ostream& output,
                     const std::vector<Vec3>& points,
                     const CoordinateFrame& frame,
                     const X3DomColor& color,
                     double point_size,
                     std::size_t max_points) {
  const std::size_t count = std::min(max_points, points.size());
  if (count == 0) {
    return;
  }

  const std::size_t step =
      std::max<std::size_t>(1, (points.size() + count - 1) / count);

  output << "    <shape>\n"
         << "      <appearance>\n"
         << "        <material diffuseColor=\"";
  write_color(output, color);
  output << "\" emissiveColor=\"";
  write_color(output, color);
  output << "\"></material>\n"
         << "        <pointProperties pointSizeScaleFactor=\"" << point_size
         << "\"></pointProperties>\n"
         << "      </appearance>\n"
         << "      <pointSet>\n"
         << "        <coordinate point=\"";

  std::size_t point_index = 0;
  for (std::size_t i = 0; i < points.size() && point_index < count; i += step) {
    const Vec3 p = apply_frame(points[i], frame);
    output << p.x << " " << p.y << " " << p.z;
    point_index++;
    if (point_index < count) {
      output << ", ";
    }
  }

  output << "\"></coordinate>\n"
         << "      </pointSet>\n"
         << "    </shape>\n";
}

void write_oriented_point_set(std::ostream& output,
                              const OrientedPointCloud& oriented_points,
                              const X3DomWriterOptions& options) {
  std::vector<Vec3> positions;
  positions.reserve(oriented_points.points.size());
  for (std::size_t i = 0; i < oriented_points.points.size(); i++) {
    positions.push_back(oriented_points.points[i].position);
  }

  write_point_set(output, positions, oriented_points.frame, options.point_color,
                  options.point_size, options.max_points);
}

void write_normal_lines(std::ostream& output,
                        const OrientedPointCloud& oriented_points,
                        const X3DomWriterOptions& options) {
  const std::size_t count = std::min(options.max_normal_vectors,
                                     oriented_points.points.size());
  if (count == 0 || options.normal_scale <= 0.0) {
    return;
  }

  const std::size_t step =
      std::max<std::size_t>(1, (oriented_points.points.size() + count - 1) / count);

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
  for (std::size_t i = 0; i < oriented_points.points.size() && line_index < count; i += step) {
    output << line_index * 2 << " " << line_index * 2 + 1 << " -1";
    line_index++;
    if (line_index < count) {
      output << " ";
    }
  }

  output << "\">\n"
         << "        <coordinate point=\"";

  line_index = 0;
  for (std::size_t i = 0; i < oriented_points.points.size() && line_index < count; i += step) {
    const OrientedPoint& point = oriented_points.points[i];
    const Vec3 start = apply_frame(point.position, oriented_points.frame);
    const Vec3 end = apply_frame(add(point.position, multiply(point.normal, options.normal_scale)),
                                 oriented_points.frame);
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

void warn_if_pointcloud_capped(const char* label,
                               std::size_t original_count,
                               std::size_t max_points) {
  if (original_count > max_points) {
    std::fprintf(stderr,
                 "Warning: %s has %zu points; writing %zu sampled points to X3DOM.\n",
                 label, original_count, max_points);
  }
}

}  // namespace

bool write_pointcloud_x3dom_file(const char* filename,
                                 const PointCloud& point_cloud,
                                 const X3DomWriterOptions& options,
                                 std::string* error_message) {
  if (error_message != nullptr) {
    error_message->clear();
  }

  std::ofstream output;
  if (!open_output(filename, &output, error_message)) {
    return false;
  }

  warn_if_pointcloud_capped("PointCloud", point_cloud.points.size(), options.max_points);
  write_page_begin(output, options.title);
  write_point_set(output, point_cloud.points, point_cloud.frame, options.point_color,
                  options.point_size, options.max_points);
  write_page_end(output);
  return true;
}

bool write_oriented_pointcloud_x3dom_file(const char* filename,
                                          const OrientedPointCloud& oriented_points,
                                          const X3DomWriterOptions& options,
                                          std::string* error_message) {
  if (error_message != nullptr) {
    error_message->clear();
  }

  std::ofstream output;
  if (!open_output(filename, &output, error_message)) {
    return false;
  }

  warn_if_pointcloud_capped("OrientedPointCloud", oriented_points.points.size(),
                            options.max_points);
  write_page_begin(output, options.title);
  write_oriented_point_set(output, oriented_points, options);
  write_normal_lines(output, oriented_points, options);
  write_page_end(output);
  return true;
}

bool write_mesh_x3dom_file(const char* filename,
                           const Mesh& mesh,
                           const X3DomWriterOptions& options,
                           std::string* error_message) {
  if (error_message != nullptr) {
    error_message->clear();
  }

  std::ofstream output;
  if (!open_output(filename, &output, error_message)) {
    return false;
  }

  write_page_begin(output, options.title);
  output << "    <shape>\n"
         << "      <appearance><material diffuseColor=\"";
  write_color(output, options.mesh_color);
  output << "\"></material></appearance>\n"
         << "      <indexedFaceSet solid=\"false\" coordIndex=\"";

  for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
    output << mesh.indices[i] << " " << mesh.indices[i + 1] << " "
           << mesh.indices[i + 2] << " -1";
    if (i + 3 < mesh.indices.size()) {
      output << " ";
    }
  }

  output << "\">\n"
         << "        <coordinate point=\"";

  for (std::size_t i = 0; i < mesh.vertices.size(); i++) {
    const Vec3 p = apply_frame(mesh.vertices[i].position, mesh.frame);
    output << p.x << " " << p.y << " " << p.z;
    if (i + 1 < mesh.vertices.size()) {
      output << ", ";
    }
  }

  output << "\"></coordinate>\n"
         << "      </indexedFaceSet>\n"
         << "    </shape>\n";
  write_page_end(output);
  return true;
}

bool write_scalar_field_x3dom_file(const char* filename,
                                   const ScalarField& field,
                                   const X3DomWriterOptions& options,
                                   std::string* error_message) {
  PointCloud samples;
  samples.frame = field.frame;
  samples.points.reserve(field.values.size());

  for (unsigned int z = 0; z < field.height; z++) {
    for (unsigned int y = 0; y < field.depth; y++) {
      for (unsigned int x = 0; x < field.width; x++) {
        const std::size_t index =
            static_cast<std::size_t>(z) * field.depth * field.width +
            static_cast<std::size_t>(y) * field.width + x;
        if (index < field.values.size()) {
          samples.points.push_back({
              field.origin.x + x * field.spacing.x,
              field.origin.y + y * field.spacing.y,
              field.origin.z + z * field.spacing.z});
        }
      }
    }
  }

  X3DomWriterOptions point_options = options;
  if (point_options.title.empty()) {
    point_options.title = "MB-System mbmesh scalar field samples";
  }
  return write_pointcloud_x3dom_file(filename, samples, point_options, error_message);
}
