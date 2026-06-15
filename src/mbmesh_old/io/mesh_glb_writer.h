#pragma once

#include "../data_types/geometry_data.h"

#include <string>

struct MeshGlbWriterOptions {
  double base_color[4];
  bool double_sided;

  MeshGlbWriterOptions()
      : base_color{0.35, 0.55, 0.72, 1.0}, double_sided(true) {}
};

bool write_mesh_glb_file(const char* filename,
                         const Mesh& mesh,
                         const MeshGlbWriterOptions& options,
                         std::string* error_message);

inline bool write_mesh_glb_file(const char* filename,
                                const Mesh& mesh,
                                std::string* error_message = nullptr) {
  return write_mesh_glb_file(filename, mesh, MeshGlbWriterOptions(), error_message);
}
