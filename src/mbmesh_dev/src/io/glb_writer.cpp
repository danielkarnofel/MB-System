#include "io/glb_writer.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#define TINYGLTF_IMPLEMENTATION
#define TINYGLTF_NO_STB_IMAGE
#define TINYGLTF_NO_STB_IMAGE_WRITE
#include "tinygltf/tiny_gltf.h"

namespace {

void set_error(std::string *error_message, const std::string &message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

bool is_finite_vec3(const Vec3 &value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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

    if (!mesh.normals.empty() && mesh.normals.size() != mesh.vertices.size()) {
        set_error(error_message, "Mesh normal count does not match vertex count");
        return false;
    }

    for (std::size_t i = 0; i < mesh.vertices.size(); i++) {
        if (!is_finite_vec3(mesh.vertices[i])) {
            set_error(error_message, "Mesh contains non-finite vertex at index " + std::to_string(i));
            return false;
        }
    }

    for (std::size_t i = 0; i < mesh.normals.size(); i++) {
        if (!is_finite_vec3(mesh.normals[i])) {
            set_error(error_message, "Mesh contains non-finite normal at index " + std::to_string(i));
            return false;
        }
    }

    for (const unsigned int index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            set_error(error_message, "Mesh contains an out-of-range vertex index");
            return false;
        }
    }

    return true;
}

std::array<double, 3> vec3_min_values(const std::vector<Vec3> &values) {
    std::array<double, 3> result{
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
        std::numeric_limits<double>::max(),
    };

    for (const Vec3 &value : values) {
        result[0] = std::min(result[0], value.x);
        result[1] = std::min(result[1], value.y);
        result[2] = std::min(result[2], value.z);
    }

    return result;
}

std::array<double, 3> vec3_max_values(const std::vector<Vec3> &values) {
    std::array<double, 3> result{
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(),
    };

    for (const Vec3 &value : values) {
        result[0] = std::max(result[0], value.x);
        result[1] = std::max(result[1], value.y);
        result[2] = std::max(result[2], value.z);
    }

    return result;
}

std::array<double, 1> index_min_value(const std::vector<unsigned int> &indices) {
    unsigned int min_value = std::numeric_limits<unsigned int>::max();
    for (const unsigned int index : indices) {
        min_value = std::min(min_value, index);
    }
    return {static_cast<double>(min_value)};
}

std::array<double, 1> index_max_value(const std::vector<unsigned int> &indices) {
    unsigned int max_value = 0;
    for (const unsigned int index : indices) {
        max_value = std::max(max_value, index);
    }
    return {static_cast<double>(max_value)};
}

void append_padding(std::vector<unsigned char> &buffer) {
    while (buffer.size() % 4 != 0) {
        buffer.push_back(0);
    }
}

template <typename T>
std::size_t append_vector_bytes(std::vector<unsigned char> &buffer, const std::vector<T> &values) {
    append_padding(buffer);
    const std::size_t offset = buffer.size();
    const std::size_t byte_count = values.size() * sizeof(T);
    buffer.resize(offset + byte_count);
    if (byte_count > 0) {
        std::memcpy(buffer.data() + offset, values.data(), byte_count);
    }
    return offset;
}

std::vector<float> flatten_vec3_values(const std::vector<Vec3> &values) {
    std::vector<float> flattened;
    flattened.reserve(values.size() * 3);
    for (const Vec3 &value : values) {
        flattened.push_back(static_cast<float>(value.x));
        flattened.push_back(static_cast<float>(value.y));
        flattened.push_back(static_cast<float>(value.z));
    }
    return flattened;
}

int add_buffer_view(tinygltf::Model &model,
                    int target,
                    std::size_t byte_offset,
                    std::size_t byte_length) {
    tinygltf::BufferView buffer_view;
    buffer_view.buffer = 0;
    buffer_view.byteOffset = byte_offset;
    buffer_view.byteLength = byte_length;
    buffer_view.target = target;
    model.bufferViews.push_back(std::move(buffer_view));
    return static_cast<int>(model.bufferViews.size()) - 1;
}

int add_vec3_accessor(tinygltf::Model &model,
                      int buffer_view_index,
                      std::size_t count,
                      const std::array<double, 3> &min_values,
                      const std::array<double, 3> &max_values) {
    tinygltf::Accessor accessor;
    accessor.bufferView = buffer_view_index;
    accessor.byteOffset = 0;
    accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
    accessor.count = count;
    accessor.type = TINYGLTF_TYPE_VEC3;
    accessor.minValues = {min_values[0], min_values[1], min_values[2]};
    accessor.maxValues = {max_values[0], max_values[1], max_values[2]};
    model.accessors.push_back(std::move(accessor));
    return static_cast<int>(model.accessors.size()) - 1;
}

int add_index_accessor(tinygltf::Model &model,
                       int buffer_view_index,
                       std::size_t count,
                       const std::array<double, 1> &min_values,
                       const std::array<double, 1> &max_values) {
    tinygltf::Accessor accessor;
    accessor.bufferView = buffer_view_index;
    accessor.byteOffset = 0;
    accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
    accessor.count = count;
    accessor.type = TINYGLTF_TYPE_SCALAR;
    accessor.minValues = {min_values[0]};
    accessor.maxValues = {max_values[0]};
    model.accessors.push_back(std::move(accessor));
    return static_cast<int>(model.accessors.size()) - 1;
}

} // namespace

bool write_pointcloud_glb_file(
    const char *filename, 
    const PointCloud &pointcloud,
    std::string *error_message
) {
    (void)filename;
    (void)pointcloud;
    set_error(error_message, "PointCloud GLB writer is not implemented");
    return false;
}

bool write_oriented_pointcloud_glb_file(
    const char *filename, 
    const OrientedPointCloud &oriented_pointcloud,
    std::string *error_message
) {
    (void)filename;
    (void)oriented_pointcloud;
    set_error(error_message, "OrientedPointCloud GLB writer is not implemented");
    return false;
}

bool write_mesh_glb_file(
    const char *filename, 
    const Mesh &mesh,
    std::string *error_message
) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    if (filename == nullptr || filename[0] == '\0') {
        set_error(error_message, "GLB output filename is empty");
        return false;
    }

    if (!validate_mesh(mesh, error_message)) {
        return false;
    }

    std::vector<unsigned char> binary_buffer;

    const std::vector<float> positions = flatten_vec3_values(mesh.vertices);
    const std::size_t position_offset = append_vector_bytes(binary_buffer, positions);
    const std::size_t position_byte_length = positions.size() * sizeof(float);

    std::size_t normal_offset = 0;
    std::size_t normal_byte_length = 0;
    if (!mesh.normals.empty()) {
        const std::vector<float> normals = flatten_vec3_values(mesh.normals);
        normal_offset = append_vector_bytes(binary_buffer, normals);
        normal_byte_length = normals.size() * sizeof(float);
    }

    const std::vector<unsigned int> indices = mesh.indices;
    const std::size_t index_offset = append_vector_bytes(binary_buffer, indices);
    const std::size_t index_byte_length = indices.size() * sizeof(unsigned int);
    append_padding(binary_buffer);

    tinygltf::Model model;
    model.asset.version = "2.0";
    model.asset.generator = "MB-System mbmesh_dev";

    tinygltf::Buffer buffer;
    buffer.data = std::move(binary_buffer);
    model.buffers.push_back(std::move(buffer));

    const int position_buffer_view = add_buffer_view(model, TINYGLTF_TARGET_ARRAY_BUFFER, position_offset, position_byte_length);
    const int position_accessor = add_vec3_accessor(
        model,
        position_buffer_view,
        mesh.vertices.size(),
        vec3_min_values(mesh.vertices),
        vec3_max_values(mesh.vertices));

    int normal_accessor = -1;
    if (!mesh.normals.empty()) {
        const int normal_buffer_view = add_buffer_view(model, TINYGLTF_TARGET_ARRAY_BUFFER, normal_offset, normal_byte_length);
        normal_accessor = add_vec3_accessor(
            model,
            normal_buffer_view,
            mesh.normals.size(),
            vec3_min_values(mesh.normals),
            vec3_max_values(mesh.normals));
    }

    const int index_buffer_view = add_buffer_view(model, TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER, index_offset, index_byte_length);
    const int index_accessor = add_index_accessor(
        model,
        index_buffer_view,
        mesh.indices.size(),
        index_min_value(mesh.indices),
        index_max_value(mesh.indices));

    tinygltf::Primitive primitive;
    primitive.mode = TINYGLTF_MODE_TRIANGLES;
    primitive.attributes["POSITION"] = position_accessor;
    if (normal_accessor >= 0) {
        primitive.attributes["NORMAL"] = normal_accessor;
    }
    primitive.indices = index_accessor;
    primitive.material = 0;

    tinygltf::Mesh gltf_mesh;
    gltf_mesh.name = "mesh";
    gltf_mesh.primitives.push_back(std::move(primitive));
    model.meshes.push_back(std::move(gltf_mesh));

    tinygltf::Material material;
    material.doubleSided = true;
    material.pbrMetallicRoughness.baseColorFactor = {0.35, 0.55, 0.72, 1.0};
    material.pbrMetallicRoughness.metallicFactor = 0.0;
    material.pbrMetallicRoughness.roughnessFactor = 0.65;
    model.materials.push_back(std::move(material));

    tinygltf::Node node;
    node.mesh = 0;
    model.nodes.push_back(std::move(node));

    tinygltf::Scene scene;
    scene.nodes.push_back(0);
    model.scenes.push_back(std::move(scene));
    model.defaultScene = 0;

    tinygltf::TinyGLTF gltf;
    if (!gltf.WriteGltfSceneToFile(&model, filename, true, true, true, true)) {
        set_error(error_message, std::string("Failed to write mesh GLB file: ") + filename);
        return false;
    }

    return true;
}
