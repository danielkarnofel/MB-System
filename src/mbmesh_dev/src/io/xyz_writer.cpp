#include "io/xyz_writer.h"

#include <cstddef>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

namespace {

void set_error(std::string *error_message, const std::string &message) {
    if (error_message != nullptr) {
        *error_message = message;
    }
}

} // namespace

bool write_xyz_pointcloud(const PointCloud &pointcloud,
                          const std::string &path,
                          std::string *error_message) {
    if (error_message != nullptr) {
        error_message->clear();
    }

    if (path.empty()) {
        set_error(error_message, "XYZ output path is empty");
        return false;
    }

    std::ofstream output(path);
    if (!output) {
        set_error(error_message, "Unable to create XYZ file: " + path);
        return false;
    }

    output << std::setprecision(17);
    for (std::size_t i = 0; i < pointcloud.points.size(); i++) {
        const Point &point = pointcloud.points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            set_error(error_message, "Non-finite point value at index " + std::to_string(i));
            return false;
        }

        output << point.x << ' ' << point.y << ' ' << point.z << '\n';
        if (!output) {
            set_error(error_message, "Error while writing XYZ file: " + path);
            return false;
        }
    }

    return true;
}
