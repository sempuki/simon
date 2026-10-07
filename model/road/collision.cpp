// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace simon::model {

namespace {

// The least and greatest projection of `corners` on the axis (`ax`, `ay`).
auto project(const std::array<Point2, 4>& corners, double ax, double ay)
    -> std::pair<double, double> {
  double least = std::numeric_limits<double>::infinity();
  double most = -least;
  for (const Point2& corner : corners) {
    double along = corner.x * ax + corner.y * ay;
    least = std::min(least, along);
    most = std::max(most, along);
  }
  return {least, most};
}

// The distance from `p` to the segment from `a` to `b`.
auto compute_segment_distance(Point2 p, Point2 a, Point2 b) -> double {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  double t = std::clamp(
      ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

}  // namespace

auto compute_corners(const OrientedBox& box) -> std::array<Point2, 4> {
  double c = std::cos(box.heading);
  double s = std::sin(box.heading);
  double l = 0.5 * box.length;
  double w = 0.5 * box.width;
  auto at = [&](double along, double across) {
    return Point2{.x = box.x + c * along - s * across,
                  .y = box.y + s * along + c * across};
  };
  return {at(l, w), at(-l, w), at(-l, -w), at(l, -w)};
}

// The separating axis theorem: for two rectangles, the candidate axes are
// each box's two edge directions.
auto detect_overlap(const OrientedBox& a, const OrientedBox& b) -> bool {
  std::array<Point2, 4> ca = compute_corners(a);
  std::array<Point2, 4> cb = compute_corners(b);
  for (double heading : {a.heading, b.heading}) {
    for (double angle : {heading, heading + 0.5 * std::numbers::pi}) {
      double ax = std::cos(angle);
      double ay = std::sin(angle);
      auto [a_least, a_most] = project(ca, ax, ay);
      auto [b_least, b_most] = project(cb, ax, ay);
      if (a_most < b_least || b_most < a_least) {
        return false;
      }
    }
  }
  return true;
}

// Apart, two convex polygons are nearest between a vertex of one and an edge
// of the other.
auto compute_gap(const OrientedBox& a, const OrientedBox& b) -> double {
  if (detect_overlap(a, b)) {
    return 0.0;
  }
  std::array<Point2, 4> ca = compute_corners(a);
  std::array<Point2, 4> cb = compute_corners(b);
  double gap = std::numeric_limits<double>::infinity();
  for (int side = 0; side < 2; ++side) {
    const auto& corners = side == 0 ? ca : cb;
    const auto& edges = side == 0 ? cb : ca;
    for (const Point2& corner : corners) {
      for (std::size_t i = 0; i < 4; ++i) {
        gap = std::min(gap, compute_segment_distance(corner, edges[i],
                                                     edges[(i + 1) % 4]));
      }
    }
  }
  return gap;
}

}  // namespace simon::model
