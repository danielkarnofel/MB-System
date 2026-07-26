#pragma once

#include <filesystem>
#include <string>

struct X3DomWriterOptions {
    std::string title = "MB-System mbmesh";
};

[[nodiscard]] bool write_glb_x3dom_file(
    const std::filesystem::path &html_path,
    const std::string &glb_uri,
    const X3DomWriterOptions &options,
    std::string *error);
