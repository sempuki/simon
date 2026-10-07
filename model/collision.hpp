// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>

// Oriented boxes in the plane: whether two overlap, and how far apart they
// are (see model/REFERENCES.md). Two convex polygons are apart exactly
// when one of their edges' normals separates them, the separating axis
// theorem; boxes that touch overlap, as GEOS counts them. Plain SI numbers.
namespace simon::model {

// A box about its center, its length along its heading, counterclockwise
// from x.
struct OrientedBox final {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double length = 0.0;
  double width = 0.0;
};

struct Point2 final {
  double x = 0.0;
  double y = 0.0;
};

// The box's corners, counterclockwise from its front left.
auto compute_corners(const OrientedBox& box) -> std::array<Point2, 4>;

// Whether two boxes share a point: no axis among their edges' normals
// separates their projections.
auto detect_overlap(const OrientedBox& a, const OrientedBox& b) -> bool;

// The least distance between two boxes' points, zero if they overlap: the
// least distance from a corner of either to an edge of the other.
auto compute_gap(const OrientedBox& a, const OrientedBox& b) -> double;

}  // namespace simon::model
