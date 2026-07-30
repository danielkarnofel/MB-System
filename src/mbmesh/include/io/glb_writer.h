#pragma once

#include <filesystem>
#include <string>

#include "../data_types/geometry.h"

[[nodiscard]] bool write_mesh_glb_file(
    const std::filesystem::path &path,
    const Mesh &mesh,
    std::string *error);
