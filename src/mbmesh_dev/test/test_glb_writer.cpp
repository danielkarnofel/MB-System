#include "io/glb_writer.h"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

constexpr std::uint32_t glb_magic = 0x46546C67;
constexpr std::uint32_t glb_version = 2;
constexpr std::uint32_t json_chunk_type = 0x4E4F534A;
constexpr std::uint32_t bin_chunk_type = 0x004E4942;

int fail(const char *message) {
    std::fprintf(stderr, "Error: %s\n", message);
    return 1;
}

bool read_u32(std::ifstream &input, std::uint32_t *value) {
    input.read(reinterpret_cast<char *>(value), sizeof(*value));
    return static_cast<bool>(input);
}

bool read_chunk(std::ifstream &input,
                std::uint32_t *chunk_type,
                std::vector<char> *chunk_data) {
    std::uint32_t chunk_length = 0;
    if (!read_u32(input, &chunk_length) || !read_u32(input, chunk_type)) {
        return false;
    }

    chunk_data->resize(chunk_length);
    input.read(chunk_data->data(), chunk_data->size());
    return static_cast<bool>(input);
}

Mesh make_triangle_mesh() {
    Mesh mesh;
    mesh.vertices.push_back(Vec3(0.0, 0.0, 0.0));
    mesh.vertices.push_back(Vec3(1.0, 0.0, 0.0));
    mesh.vertices.push_back(Vec3(0.0, 1.0, 0.0));

    mesh.normals.push_back(Vec3(0.0, 0.0, 1.0));
    mesh.normals.push_back(Vec3(0.0, 0.0, 1.0));
    mesh.normals.push_back(Vec3(0.0, 0.0, 1.0));

    mesh.indices.push_back(0);
    mesh.indices.push_back(1);
    mesh.indices.push_back(2);
    return mesh;
}

} // namespace

int main() {
    const std::string output_path = "/tmp/mbmesh_test_triangle.glb";
    std::string error_message;

    const Mesh mesh = make_triangle_mesh();
    if (!write_mesh_glb_file(output_path.c_str(), mesh, &error_message)) {
        std::fprintf(stderr, "Error: %s\n", error_message.c_str());
        return 1;
    }

    std::ifstream input(output_path, std::ios::binary);
    if (!input) {
        return fail("unable to open written GLB file");
    }

    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    std::uint32_t file_length = 0;
    if (!read_u32(input, &magic) || !read_u32(input, &version) || !read_u32(input, &file_length)) {
        return fail("unable to read GLB header");
    }

    if (magic != glb_magic) {
        return fail("GLB magic value is invalid");
    }

    if (version != glb_version) {
        return fail("GLB version is invalid");
    }

    std::uint32_t chunk_type = 0;
    std::vector<char> json_chunk;
    if (!read_chunk(input, &chunk_type, &json_chunk) || chunk_type != json_chunk_type) {
        return fail("GLB JSON chunk is missing or invalid");
    }

    const std::string json(json_chunk.begin(), json_chunk.end());
    if (json.find("\"meshes\"") == std::string::npos ||
        json.find("\"POSITION\"") == std::string::npos ||
        json.find("\"NORMAL\"") == std::string::npos ||
        json.find("\"indices\"") == std::string::npos) {
        return fail("GLB JSON does not describe the expected mesh attributes");
    }

    std::vector<char> bin_chunk;
    if (!read_chunk(input, &chunk_type, &bin_chunk) || chunk_type != bin_chunk_type) {
        return fail("GLB binary chunk is missing or invalid");
    }

    input.seekg(0, std::ios::end);
    const std::streamoff actual_file_length = input.tellg();
    if (actual_file_length != static_cast<std::streamoff>(file_length)) {
        return fail("GLB header length does not match actual file length");
    }

    Mesh invalid_mesh = mesh;
    invalid_mesh.indices[2] = 42;
    error_message.clear();
    if (write_mesh_glb_file("/tmp/mbmesh_invalid_triangle.glb", invalid_mesh, &error_message)) {
        return fail("invalid mesh unexpectedly wrote successfully");
    }

    if (error_message.find("out-of-range") == std::string::npos) {
        return fail("invalid mesh did not report out-of-range index");
    }

    std::printf("Mesh GLB writer exported %zu vertices and %zu triangles\n",
                mesh.vertices.size(),
                mesh.indices.size() / 3);
    return 0;
}
