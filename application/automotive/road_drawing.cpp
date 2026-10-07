// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/road_drawing.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace simon::automotive {

namespace {

auto convert_to_point(const road::Road& road, const road::PlanPoint& plan,
                      double s, double t) -> model::Point2 {
  Vector3 at = road::compute_position(road, plan, s * meter, t * meter)
                   .numerical_value_in(meter)
                   .eigen();
  return {.x = at.x(), .y = at.y()};
}

}  // namespace

auto draw_roads(const road::Map& network, double spacing) -> RoadDrawing {
  RoadDrawing drawing;
  for (const road::Road& road : network.roads) {
    for (std::size_t k = 0; k < road.lane_sections.size(); ++k) {
      const road::LaneSection& section = road.lane_sections[k];
      double start = section.s0;
      double end = k + 1 < road.lane_sections.size()
                       ? road.lane_sections[k + 1].s0
                       : road.length;
      if (end <= start) {
        continue;
      }
      auto count = static_cast<std::size_t>(
          std::max(1.0, std::ceil((end - start) / spacing)));
      std::size_t first = drawing.lanes.size();
      for (const auto* side : {&section.left, &section.right}) {
        for (const road::Lane& lane : *side) {
          drawing.lanes.push_back({.type = lane.type, .lane = lane.id});
        }
      }
      bool both_ways = !section.left.empty() && !section.right.empty();
      if (both_ways) {
        drawing.center_lines.emplace_back();
      }
      for (std::size_t i = 0; i <= count; ++i) {
        double s = start + (end - start) * static_cast<double>(i) /
                               static_cast<double>(count);
        road::PlanPoint plan = road::compute_plan_point(road, s * meter);
        auto border = [&](int id) {
          return road::compute_lane_border(road, section, s * meter, id)
              .numerical_value_in(meter);
        };
        for (std::size_t j = first; j < drawing.lanes.size(); ++j) {
          LaneStrip& strip = drawing.lanes[j];
          int inner = strip.lane > 0 ? strip.lane - 1 : strip.lane + 1;
          strip.inner.push_back(convert_to_point(road, plan, s, border(inner)));
          strip.outer.push_back(
              convert_to_point(road, plan, s, border(strip.lane)));
        }
        if (both_ways) {
          drawing.center_lines.back().push_back(
              convert_to_point(road, plan, s, border(0)));
        }
      }
    }
  }
  return drawing;
}

auto compute_strip_area(const LaneStrip& strip) -> double {
  double area = 0.0;
  for (std::size_t i = 1; i < strip.inner.size(); ++i) {
    // The quadrilateral inner[i-1], inner[i], outer[i], outer[i-1].
    std::array<model::Point2, 4> corners = {strip.inner[i - 1], strip.inner[i],
                                            strip.outer[i], strip.outer[i - 1]};
    double twice = 0.0;
    for (std::size_t c = 0; c < 4; ++c) {
      const model::Point2& a = corners[c];
      const model::Point2& b = corners[(c + 1) % 4];
      twice += a.x * b.y - b.x * a.y;
    }
    area += std::abs(twice) / 2.0;
  }
  return area;
}

}  // namespace simon::automotive
