// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road/road.hpp"
#include "core/math.hpp"

#include <cmath>
#include <numbers>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::road {

namespace {

using Catch::Matchers::WithinAbs;

auto geometry(double length, auto shape) -> PlanGeometry {
  return PlanGeometry{.s0 = 0.0,
                      .x0 = 10.0,
                      .y0 = -5.0,
                      .heading = 0.0,
                      .length = length,
                      .shape = shape};
}

// A road along `plan`, with one lane section and no lanes.
auto road_along(std::vector<PlanGeometry> plan) -> Road {
  double length = 0.0;
  for (const PlanGeometry& piece : plan) {
    length = std::max(length, piece.s0 + piece.length);
  }
  return Road{.id = "1",
              .junction = "-1",
              .length = length,
              .plan = std::move(plan),
              .lane_sections = {LaneSection{}}};
}

auto convert_to_meters(const Position& position) -> Vector3 {
  return position.numerical_value_in(meter).eigen();
}

}  // namespace

TEST_CASE("Road") {
  SECTION("ShouldFollowFresnelIntegralsGivenClothoid") {
    // Under Test.
    // Heading pi u^2 / 2 over u in [0, 1], so the end is (C(1), S(1)), the
    // Fresnel integrals, to double precision (DLMF section 7.2(iii); see
    // model/REFERENCES.md).
    PlanPoint end = compute_plan_point(
        geometry(1.0, SpiralGeometry{.curvature_start = 0.0,
                                     .curvature_end = std::numbers::pi}),
        1.0);

    // Postconditions.
    CHECK_THAT(end.x - 10.0, WithinAbs(0.7798934003768228, 1e-15));
    CHECK_THAT(end.y + 5.0, WithinAbs(0.4382591473903548, 1e-15));
    CHECK_THAT(end.heading, WithinAbs(std::numbers::pi / 2.0, 1e-15));
    CHECK_THAT(end.curvature, WithinAbs(std::numbers::pi, 1e-15));
  }

  SECTION("ShouldBeArcGivenSpiralOfConstantCurvature") {
    // Preconditions.
    PlanGeometry spiral = geometry(
        80.0, SpiralGeometry{.curvature_start = 0.01, .curvature_end = 0.01});
    PlanGeometry arc = geometry(80.0, ArcGeometry{.curvature = 0.01});

    // Under Test.
    for (double ds : {0.0, 13.0, 47.5, 80.0}) {
      PlanPoint a = compute_plan_point(spiral, ds);
      PlanPoint b = compute_plan_point(arc, ds);
      CHECK_THAT(a.x, WithinAbs(b.x, 1e-12));
      CHECK_THAT(a.y, WithinAbs(b.y, 1e-12));
    }
  }

  SECTION("ShouldTurnQuarterCircleGivenArc") {
    // Preconditions.
    double radius = 100.0;

    // Under Test.
    PlanPoint end =
        compute_plan_point(geometry(radius * std::numbers::pi / 2.0,
                                    ArcGeometry{.curvature = 1.0 / radius}),
                           radius * std::numbers::pi / 2.0);

    // Postconditions.
    CHECK_THAT(end.x, WithinAbs(10.0 + radius, 1e-12));
    CHECK_THAT(end.y, WithinAbs(-5.0 + radius, 1e-12));
    CHECK_THAT(end.heading, WithinAbs(std::numbers::pi / 2.0, 1e-15));
  }

  SECTION("ShouldBeLineGivenArcOfNoCurvature") {
    // Preconditions.
    double curvature = 1e-15;

    // Under Test.
    PlanPoint end = compute_plan_point(
        geometry(50.0, ArcGeometry{.curvature = curvature}), 50.0);

    // Postconditions.
    // It strays from the line by k s^2 / 2 to first order.
    CHECK_THAT(end.x, WithinAbs(60.0, 1e-12));
    CHECK_THAT(end.y + 5.0, WithinAbs(curvature * 50.0 * 50.0 / 2.0, 1e-15));
  }

  SECTION("ShouldGoByArcLengthGivenParamPoly3") {
    // Preconditions.
    // u = 10 p and v = 100 c p^2 over p in [0, 1]: a parabola whose arc length
    // to x is x sqrt(1 + 4 c^2 x^2) / 2 + asinh(2 c x) / (4 c).
    double c = 0.1;
    auto arc_length = [&](double x) {
      return x * std::sqrt(1.0 + 4.0 * c * c * x * x) / 2.0 +
             std::asinh(2.0 * c * x) / (4.0 * c);
    };
    PlanGeometry curve = geometry(arc_length(10.0),
                                  ParamPoly3Geometry{.u = Cubic{.b = 10.0},
                                                     .v = Cubic{.c = 100.0 * c},
                                                     .normalized = true});

    // Under Test.
    PlanPoint point = compute_plan_point(curve, arc_length(4.0));

    // Postconditions.
    CHECK_THAT(point.x, WithinAbs(10.0 + 4.0, 1e-12));
    CHECK_THAT(point.y, WithinAbs(-5.0 + c * 16.0, 1e-12));
    CHECK_THAT(point.heading, WithinAbs(std::atan(2.0 * c * 4.0), 1e-14));
  }

  SECTION("ShouldEndAtCurveEndGivenLengthOffArcLength") {
    // Preconditions.
    // Over pRange arcLength, p runs over [0, length], and here the curve's arc
    // length over that range is longer than the length: s still runs from
    // the curve's start to its end, in proportion to its arc length.
    PlanGeometry curve =
        geometry(20.0, ParamPoly3Geometry{.u = Cubic{.b = 1.0},
                                          .v = Cubic{.c = 0.02},
                                          .normalized = false});

    // Under Test.
    PlanPoint end = compute_plan_point(curve, 20.0);

    // Postconditions.
    CHECK_THAT(end.x, WithinAbs(10.0 + 20.0, 1e-12));
    CHECK_THAT(end.y, WithinAbs(-5.0 + 0.02 * 400.0, 1e-12));

    // Under Test.
    PlanPoint start = compute_plan_point(curve, 0.0);

    // Postconditions.
    CHECK_THAT(start.x, WithinAbs(10.0, 1e-15));
  }

  SECTION("ShouldTiltAcrossGivenSlopeAndSuperelevation") {
    // Preconditions.
    // Along x, climbing at slope g and rolled by r: e_s = (1, 0, g) / |.|, and
    // the horizontal normal (0, 1, 0) turned about it by r is
    // (-sin(r) e_s.z, cos(r), sin(r) e_s.x).
    double slope = 0.05;
    double roll = 0.04;
    Road road = road_along({PlanGeometry{.length = 100.0}});
    road.elevation.pieces = {
        {.start = 0.0, .cubic = Cubic{.a = 2.0, .b = slope}}};
    road.superelevation.pieces = {{.start = 0.0, .cubic = Cubic{.a = roll}}};
    double norm = std::sqrt(1.0 + slope * slope);

    // Under Test.
    Vector3 at =
        convert_to_meters(compute_position(road, 30.0 * meter, 3.0 * meter));

    // Postconditions.
    CHECK_THAT(at.x(),
               WithinAbs(30.0 - 3.0 * std::sin(roll) * slope / norm, 1e-12));
    CHECK_THAT(at.y(), WithinAbs(3.0 * std::cos(roll), 1e-12));
    CHECK_THAT(
        at.z(),
        WithinAbs(2.0 + slope * 30.0 + 3.0 * std::sin(roll) / norm, 1e-12));
  }

  SECTION("ShouldSumWidthsGivenLaneBorders") {
    // Preconditions.
    Road road = road_along({PlanGeometry{.length = 100.0}});
    road.lane_offset.pieces = {{.start = 0.0, .cubic = Cubic{.a = 0.5}}};
    road.lane_sections = {LaneSection{
        .s0 = 0.0,
        .left = {Lane{.id = 1, .widths = {{.cubic = Cubic{.a = 3.5}}}},
                 Lane{.id = 2,
                      .widths = {{.cubic = Cubic{.a = 3.0}},
                                 {.start = 50.0,
                                  .cubic = Cubic{.a = 3.0, .b = 0.01}}}}},
        .right = {Lane{.id = -1, .widths = {{.cubic = Cubic{.a = 3.25}}}}},
    }};

    // Postconditions.
    CHECK(compute_lane_border(road, 60.0 * meter, 0) == 0.5 * meter);
    CHECK_THAT(
        compute_lane_border(road, 60.0 * meter, 2).numerical_value_in(meter),
        WithinAbs(0.5 + 3.5 + 3.1, 1e-12));
    CHECK_THAT(
        compute_lane_border(road, 60.0 * meter, -1).numerical_value_in(meter),
        WithinAbs(0.5 - 3.25, 1e-12));
    CHECK(find_lane(road, 60.0 * meter, 2.0 * meter) == 1);
    // On lane 1's inner border.
    CHECK(find_lane(road, 60.0 * meter, 0.5 * meter) == 1);
    CHECK(find_lane(road, 60.0 * meter, 5.0 * meter) == 2);
    CHECK(find_lane(road, 60.0 * meter, -1.0 * meter) == -1);
    CHECK_FALSE(find_lane(road, 60.0 * meter, 7.2 * meter));
    CHECK_FALSE(find_lane(road, 60.0 * meter, -3.0 * meter));
  }

  SECTION("ShouldFindRoadCoordinatesGivenPosition") {
    // Preconditions.
    // A line, a spiral into an arc, and a spiral out to the other hand.
    std::vector<PlanGeometry> plan;
    PlanGeometry line{.length = 40.0, .shape = LineGeometry{}};
    plan.push_back(line);
    auto append = [&](double length, auto shape) {
      const PlanGeometry& last = plan.back();
      PlanPoint end = compute_plan_point(last, last.length);
      plan.push_back(PlanGeometry{.s0 = last.s0 + last.length,
                                  .x0 = end.x,
                                  .y0 = end.y,
                                  .heading = end.heading,
                                  .length = length,
                                  .shape = shape});
    };
    append(30.0, SpiralGeometry{.curvature_start = 0.0, .curvature_end = 0.02});
    append(50.0, ArcGeometry{.curvature = 0.02});
    append(40.0,
           SpiralGeometry{.curvature_start = 0.02, .curvature_end = -0.01});
    Road road = road_along(std::move(plan));

    // Under Test.
    for (double s : {5.0, 39.0, 41.0, 63.0, 100.0, 141.0, 155.0}) {
      for (double t : {-4.0, -1.0, 0.0, 2.5, 6.0}) {
        CAPTURE(s, t);
        Vector3 at =
            convert_to_meters(compute_position(road, s * meter, t * meter));
        RoadCoordinates found =
            find_road_coordinates(road, at.x() * meter, at.y() * meter);
        CHECK_THAT(found.s.numerical_value_in(meter), WithinAbs(s, 1e-9));
        CHECK_THAT(found.t.numerical_value_in(meter), WithinAbs(t, 1e-9));
      }
    }
  }
}

}  // namespace simon::road
