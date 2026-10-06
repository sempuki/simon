// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <vector>

// A trajectory's polyline, followed as esmini follows one held to its line
// (see model/REFERENCES.md): straight between its vertices, heading along
// each segment, and turning only within 2 m of a corner, or half the
// segment if shorter. Plain SI numbers in the plane.
namespace simon::model {

struct PolylinePoint final {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;  // Counterclockwise from x.
};

class Polyline final {
 public:
  // The polyline through `points`, their headings ignored: each vertex heads
  // along the next segment of any length, the last along the one before.
  explicit Polyline(std::vector<PolylinePoint> points);

  auto length() const -> double { return distance_.back(); }

  // The point `along` meters from the start, clamped to the polyline, and
  // the heading there.
  auto evaluate(double along) const -> PolylinePoint;

 private:
  std::vector<PolylinePoint> points_;
  std::vector<double> distance_;  // To each vertex.
};

}  // namespace simon::model
