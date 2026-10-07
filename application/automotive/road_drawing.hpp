// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <string>
#include <vector>

#include "model/road/collision.hpp"
#include "model/road/road.hpp"

// A road network as a map draws it, in the plane: each lane a strip between
// its inner and outer borders, and each road's center line, sampled along
// the road. Plain SI numbers, as the road's data are.
namespace simon::automotive {

// A lane over one lane section: its borders, sampled at the same s, inner
// and outer pairwise.
struct LaneStrip final {
  std::string type;  // As OpenDRIVE names it: driving, shoulder, sidewalk...
  int lane = 0;
  std::vector<model::Point2> inner;
  std::vector<model::Point2> outer;
};

struct RoadDrawing final {
  std::vector<LaneStrip> lanes;
  // The center lane over each lane section, where lanes run either way.
  std::vector<std::vector<model::Point2>> center_lines;
};

// Samples every lane section of every road at most `spacing` meters apart
// along its reference line, and at its start and end.
auto draw_roads(const model::RoadNetwork& network, double spacing)
    -> RoadDrawing;

// The area of a strip: the quadrilaterals between consecutive samples, by
// the shoelace formula.
auto compute_strip_area(const LaneStrip& strip) -> double;

}  // namespace simon::automotive
