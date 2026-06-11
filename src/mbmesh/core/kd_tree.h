#pragma once

#include "vec3.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

class KDTree {
public:
  struct Neighbor {
    std::size_t index;
    double distance_squared;
  };

  KDTree() : root_(-1) {}

  explicit KDTree(const std::vector<Vec3> &points) : root_(-1) {
    build(points);
  }

  void build(const std::vector<Vec3> &points) {
    nodes_.clear();
    root_ = -1;

    if (points.empty()) {
      return;
    }

    std::vector<std::size_t> indices(points.size());
    for (std::size_t i = 0; i < points.size(); i++) {
      indices[i] = i;
    }

    nodes_.reserve(points.size());
    root_ = build_recursive(points, indices, 0, indices.size(), 0);
  }

  bool empty() const {
    return nodes_.empty();
  }

  std::size_t size() const {
    return nodes_.size();
  }

  std::vector<Neighbor> k_nearest(const Vec3 &query,
                                  std::size_t k,
                                  std::size_t exclude_index = invalid_index()) const {
    std::vector<Neighbor> neighbors;
    if (root_ < 0 || k == 0) {
      return neighbors;
    }

    NeighborQueue queue;
    k_nearest_recursive(root_, query, k, exclude_index, &queue);

    neighbors.reserve(queue.size());
    while (!queue.empty()) {
      neighbors.push_back(queue.top());
      queue.pop();
    }

    std::sort(neighbors.begin(), neighbors.end(), closer_neighbor);
    return neighbors;
  }

  std::vector<Neighbor> radius_search_squared(const Vec3 &query,
                                              double radius_squared,
                                              std::size_t exclude_index = invalid_index()) const {
    std::vector<Neighbor> neighbors;
    if (root_ < 0 || radius_squared < 0.0) {
      return neighbors;
    }

    radius_search_recursive(root_, query, radius_squared, exclude_index, &neighbors);
    std::sort(neighbors.begin(), neighbors.end(), closer_neighbor);
    return neighbors;
  }

  static std::size_t invalid_index() {
    return std::numeric_limits<std::size_t>::max();
  }

private:
  struct Node {
    Vec3 point;
    std::size_t index;
    int left;
    int right;
    unsigned char axis;
  };

  struct FartherNeighbor {
    bool operator()(const Neighbor &a, const Neighbor &b) const {
      return a.distance_squared < b.distance_squared;
    }
  };

  typedef std::priority_queue<Neighbor, std::vector<Neighbor>, FartherNeighbor> NeighborQueue;

  static double coordinate(const Vec3 &point, unsigned char axis) {
    if (axis == 0) {
      return point.x;
    }
    if (axis == 1) {
      return point.y;
    }
    return point.z;
  }

  static double distance_squared(const Vec3 &a, const Vec3 &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
  }

  static bool closer_neighbor(const Neighbor &a, const Neighbor &b) {
    if (a.distance_squared == b.distance_squared) {
      return a.index < b.index;
    }
    return a.distance_squared < b.distance_squared;
  }

  int build_recursive(const std::vector<Vec3> &points,
                      std::vector<std::size_t> &indices,
                      std::size_t begin,
                      std::size_t end,
                      unsigned char depth) {
    if (begin >= end) {
      return -1;
    }

    const unsigned char axis = depth % 3;
    const std::size_t middle = begin + (end - begin) / 2;

    std::nth_element(indices.begin() + begin,
                     indices.begin() + middle,
                     indices.begin() + end,
                     [&points, axis](std::size_t a, std::size_t b) {
                       const double av = coordinate(points[a], axis);
                       const double bv = coordinate(points[b], axis);
                       if (av == bv) {
                         return a < b;
                       }
                       return av < bv;
                     });

    const int node_index = static_cast<int>(nodes_.size());
    Node node = {points[indices[middle]], indices[middle], -1, -1, axis};
    nodes_.push_back(node);

    nodes_[node_index].left = build_recursive(points, indices, begin, middle, depth + 1);
    nodes_[node_index].right = build_recursive(points, indices, middle + 1, end, depth + 1);
    return node_index;
  }

  void k_nearest_recursive(int node_index,
                           const Vec3 &query,
                           std::size_t k,
                           std::size_t exclude_index,
                           NeighborQueue *queue) const {
    if (node_index < 0) {
      return;
    }

    const Node &node = nodes_[node_index];
    const double d2 = distance_squared(query, node.point);
    if (node.index != exclude_index) {
      Neighbor candidate = {node.index, d2};
      if (queue->size() < k) {
        queue->push(candidate);
      } else if (closer_neighbor(candidate, queue->top())) {
        queue->pop();
        queue->push(candidate);
      }
    }

    const double delta = coordinate(query, node.axis) - coordinate(node.point, node.axis);
    const int near_child = delta < 0.0 ? node.left : node.right;
    const int far_child = delta < 0.0 ? node.right : node.left;

    k_nearest_recursive(near_child, query, k, exclude_index, queue);

    const double worst_d2 = queue->size() < k ? std::numeric_limits<double>::max() : queue->top().distance_squared;
    if (delta * delta <= worst_d2) {
      k_nearest_recursive(far_child, query, k, exclude_index, queue);
    }
  }

  void radius_search_recursive(int node_index,
                               const Vec3 &query,
                               double radius_squared,
                               std::size_t exclude_index,
                               std::vector<Neighbor> *neighbors) const {
    if (node_index < 0) {
      return;
    }

    const Node &node = nodes_[node_index];
    const double d2 = distance_squared(query, node.point);
    if (node.index != exclude_index && d2 <= radius_squared) {
      Neighbor neighbor = {node.index, d2};
      neighbors->push_back(neighbor);
    }

    const double delta = coordinate(query, node.axis) - coordinate(node.point, node.axis);
    const int near_child = delta < 0.0 ? node.left : node.right;
    const int far_child = delta < 0.0 ? node.right : node.left;

    radius_search_recursive(near_child, query, radius_squared, exclude_index, neighbors);
    if (delta * delta <= radius_squared) {
      radius_search_recursive(far_child, query, radius_squared, exclude_index, neighbors);
    }
  }

  std::vector<Node> nodes_;
  int root_;
};
