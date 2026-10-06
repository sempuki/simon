// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#include "model/polyline.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace simon::model {

namespace {

constexpr double SMALL = 1e-6;  // esmini's SMALL_NUMBER.
constexpr double CORNER = 2.0;  // m either side of a corner it turns over.

// From `b` to `a`, the short way round.
auto compute_angle_difference(double a, double b) -> double {
  return std::remainder(a - b, 2.0 * std::numbers::pi);
}

}  // namespace

// Each vertex heads along the next segment with any length, esmini's
// PolyLineShape::CalculatePolyLine.
Polyline::Polyline(std::vector<PolylinePoint> points)
    : points_{std::move(points)} {
  distance_.push_back(0.0);
  for (std::size_t i = 1; i < points_.size(); ++i) {
    distance_.push_back(distance_.back() +
                        std::hypot(points_[i].x - points_[i - 1].x,
                                   points_[i].y - points_[i - 1].y));
  }
  double heading = 0.0;
  for (std::size_t i = 0; i + 1 < points_.size(); ++i) {
    if (distance_[i + 1] - distance_[i] > SMALL) {
      heading = std::atan2(points_[i + 1].y - points_[i].y,
                           points_[i + 1].x - points_[i].x);
      break;
    }
  }
  for (std::size_t i = 0; i < points_.size(); ++i) {
    points_[i].heading = heading;
    if (i + 2 < points_.size() && distance_[i + 2] - distance_[i + 1] > SMALL) {
      heading = std::atan2(points_[i + 2].y - points_[i + 1].y,
                           points_[i + 2].x - points_[i + 1].x);
    }
  }
}

// esmini's PolyLineBase::EvaluateSegmentByLocalS with corner interpolation:
// the heading blends the segments either side of a corner over its radius.
auto Polyline::evaluate(double along) const -> PolylinePoint {
  std::size_t last = points_.size() - 1;
  if (along > distance_.back() - SMALL) {
    return points_.back();
  }
  std::size_t i = 0;
  while (i < last - 1 && distance_[i + 1] < along) {
    ++i;
  }
  while (i > 0 && along < distance_[i] + SMALL) {
    --i;
  }
  double length = std::max(distance_[i + 1] - distance_[i], SMALL);
  double local = std::clamp(along - distance_[i], 0.0, length);
  double a = local / length;
  const PolylinePoint& from = points_[i];
  const PolylinePoint& to = points_[i + 1];
  PolylinePoint at{.x = (1.0 - a) * from.x + a * to.x,
                   .y = (1.0 - a) * from.y + a * to.y,
                   .heading = from.heading};
  double radius = std::min(CORNER, length / 2.0);
  if (local < radius && i > 0) {
    double blend = (radius + local) / (2.0 * radius);
    at.heading =
        points_[i - 1].heading +
        blend * compute_angle_difference(from.heading, points_[i - 1].heading);
  } else if (local > length - radius) {
    double blend = i < last - 1 ? (radius + (length - local)) / (2.0 * radius)
                                : (length - local) / radius;
    at.heading = from.heading + (1.0 - blend) * compute_angle_difference(
                                                    to.heading, from.heading);
  }
  at.heading = std::remainder(at.heading, 2.0 * std::numbers::pi);
  return at;
}

}  // namespace simon::model
