#pragma once

#include <cmath>
#include <vector>
#include <limits>

struct Vec3 {
  double x;
  double y;
  double z;
};

inline Vec3 add(const Vec3 &a, const Vec3 &b) {
  Vec3 result = {a.x + b.x, a.y + b.y, a.z + b.z};
  return result;
}

inline Vec3 subtract(const Vec3 &a, const Vec3 &b) {
  Vec3 result = {a.x - b.x, a.y - b.y, a.z - b.z};
  return result;
}

inline Vec3 multiply(const Vec3 &a, double scale) {
  Vec3 result = {a.x * scale, a.y * scale, a.z * scale};
  return result;
}

inline double dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double length(const Vec3 &a) {
  return std::sqrt(dot(a, a));
}

inline Vec3 normalize(const Vec3 &a) {
  const double n = length(a);
  if (n <= std::numeric_limits<double>::epsilon()) {
    Vec3 fallback = {0.0, 0.0, 1.0};
    return fallback;
  }
  return multiply(a, 1.0 / n);
}

inline Vec3 centroid(const std::vector<Vec3> &points) {
  Vec3 c = {0.0, 0.0, 0.0};
  for (std::size_t i = 0; i < points.size(); i++) {
    c = add(c, points[i]);
  }
  return points.empty() ? c : multiply(c, 1.0 / static_cast<double>(points.size()));
}
