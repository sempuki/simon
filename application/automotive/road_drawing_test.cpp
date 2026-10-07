// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <cmath>
#include <numbers>

#include "application/automotive/road_drawing.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "format/opendrive.hpp"

// The map's drawing of the roads: lanes as strips that cover the lanes, and
// meet across lane sections and road links.
namespace simon::automotive {

namespace {

using namespace testing;

auto load(std::string_view file) -> road::Map {
  auto network = format::load_opendrive(find_road_path(file));
  REQUIRE(network);
  return std::move(*network);
}

}  // namespace

TEST_CASE("RoadDrawing") {
  SECTION("ShouldCoverTheRingsLanes") {
    // Two half circles of radius 100 m, two 3.5 m lanes each way: an
    // annulus from 93 m to 107 m, drawn as 315 chords a half, each turning
    // pi / 315. The inscribed polygon's area is the chords' triangles'.
    RoadDrawing drawing = draw_roads(load("ring.xodr"), 1.0);
    double area = 0.0;
    for (const LaneStrip& strip : drawing.lanes) {
      if (strip.type == "driving") {
        area += compute_strip_area(strip);
      }
    }
    double turn = std::numbers::pi / 315.0;
    double exact =
        2.0 * 315.0 * 0.5 * (107.0 * 107.0 - 93.0 * 93.0) * std::sin(turn);
    CAPTURE(area, exact);
    CHECK(std::abs(area - exact) / exact < 1e-12);
    CHECK(drawing.center_lines.size() == 2);
  }

  SECTION("ShouldDrawEveryLane") {
    // Every lane of every lane section of CARLA's Town01, each border
    // sampled at the same s.
    road::Map network = load("Town01.xodr");
    std::size_t lanes = 0;
    for (const road::Road& road : network.roads) {
      for (const road::LaneSection& section : road.lane_sections) {
        lanes += section.left.size() + section.right.size();
      }
    }
    RoadDrawing drawing = draw_roads(network, 2.0);
    CHECK(drawing.lanes.size() == lanes);
    for (const LaneStrip& strip : drawing.lanes) {
      REQUIRE(strip.inner.size() == strip.outer.size());
      REQUIRE(strip.inner.size() >= 2);
    }
  }

  SECTION("ShouldMeetAcrossRoadLinks") {
    // Each ring half's driving lanes end where the next half's start.
    RoadDrawing drawing = draw_roads(load("ring.xodr"), 1.0);
    double worst = 0.0;
    for (const LaneStrip& a : drawing.lanes) {
      double nearest = 1e9;
      for (const LaneStrip& b : drawing.lanes) {
        if (&a == &b || a.type != b.type) {
          continue;
        }
        for (const model::Point2& end : {b.inner.front(), b.inner.back()}) {
          nearest = std::min(nearest, std::hypot(a.inner.back().x - end.x,
                                                 a.inner.back().y - end.y));
        }
      }
      worst = std::max(worst, nearest);
    }
    CAPTURE(worst);
    CHECK(worst < 1e-9);
  }
}

}  // namespace simon::automotive
