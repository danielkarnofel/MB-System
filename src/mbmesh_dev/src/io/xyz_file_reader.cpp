#include "io/xyz_file_reader.h"

#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

namespace {

void set_error(std::string *error_message, const std::string &message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

[[nodiscard]] bool is_blank_or_comment(const std::string &line) {
    for (char c : line) {
        if (!std::isspace(static_cast<unsigned char>(c))) {
            return c == '#';
        }
    }
    return true;
}

[[nodiscard]] std::string normalize_separators(std::string line) {
    for (char &c : line) {
        if (c == ',') {
            c = ' ';
        }
    }
    return line;
}

} // namespace

bool read_xyz_file(const std::string &path, PointCloud *pointcloud, std::string *error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    if (pointcloud == nullptr) {
        set_error(error_message, "XYZ output point cloud pointer is null");
        return false;
    }

    std::ifstream input(path);
    if (!input) {
        set_error(error_message, "Unable to open XYZ file: " + path);
        return false;
    }

    PointCloud parsed_pointcloud;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        line_number++;
        if (is_blank_or_comment(line)) {
            continue;
        }

        std::istringstream line_stream(normalize_separators(line));
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!(line_stream >> x >> y >> z)) {
            set_error(error_message, "Invalid XYZ record at line " + std::to_string(line_number));
            return false;
        }

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            set_error(error_message, "Non-finite XYZ value at line " + std::to_string(line_number));
            return false;
        }

        parsed_pointcloud.points.push_back(Point(x, y, z));
    }

    if (!input.eof()) {
        set_error(error_message, "Error while reading XYZ file: " + path);
        return false;
    }

    *pointcloud = parsed_pointcloud;
    return true;
}

PointCloud read_xyz_file(const std::string &path) {
    PointCloud pointcloud;
    std::string error_message;
    if (!read_xyz_file(path, &pointcloud, &error_message)) {
        return PointCloud();
    }
    return pointcloud;
}
