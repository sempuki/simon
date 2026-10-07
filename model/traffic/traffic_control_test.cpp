// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/traffic/traffic_control.hpp"

#include <chrono>
#include <cmath>

#include "base/testing.hpp"

namespace simon::traffic {

namespace {

using namespace std::chrono_literals;

// A straight road, 100 m, two lanes each way, with a traffic light at
// s = 90 for the right lanes, lane -1 only, and a static sign; a controller
// groups the light, and a junction lists the controller.
auto network_with_light() -> road::Map {
  auto lane = [](int id) {
    return road::Lane{
        .id = id, .type = "driving", .widths = {{.cubic = {.a = 3.5}}}};
  };
  road::Road road{
      .id = "1",
      .junction = "-1",
      .length = 100.0,
      .plan = {road::PlanGeometry{.length = 100.0,
                                  .shape = road::LineGeometry{}}},
      .lane_sections = {road::LaneSection{.left = {lane(1), lane(2)},
                                          .right = {lane(-1), lane(-2)}}}};
  road.signals.push_back(road::Signal{.id = "light",
                                      .validities = {{.from = -1, .to = -1}},
                                      .s = 90.0,
                                      .orientation = road::Direction::POSITIVE,
                                      .dynamic = true});
  road.signals.push_back(road::Signal{.id = "sign", .s = 50.0});
  return road::Map{.roads = {road},
                   .junctions = {road::Junction{
                       .id = "9", .controllers = {{.id = "c", .sequence = 3}}}},
                   .controllers = {road::SignalController{
                       .id = "c", .controls = {{.signal = "light"}}}}};
}

}  // namespace

TEST_CASE("traffic::Control") {
  SECTION("ShouldCycleThroughPhasesGivenPlan") {
    SignalPlan plan{.phases = {{.duration = 20s, .aspect = Aspect::GREEN},
                               {.duration = 3s, .aspect = Aspect::YELLOW},
                               {.duration = 27s, .aspect = Aspect::RED}},
                    .offset = 5s};
    CHECK(plan.cycle() == 50s);
    CHECK(plan.aspect_at(5s) == Aspect::GREEN);
    CHECK(plan.aspect_at(24s) == Aspect::GREEN);
    CHECK(plan.aspect_at(25s) == Aspect::YELLOW);  // Phases are half-open.
    CHECK(plan.aspect_at(28s) == Aspect::RED);
    CHECK(plan.aspect_at(54s) == Aspect::RED);
    CHECK(plan.aspect_at(55s) == Aspect::GREEN);
    CHECK(plan.aspect_at(0s) == Aspect::RED);  // Before the offset too.
    CHECK(plan.keeps_aspect(5s) == 20s);
    CHECK(plan.keeps_aspect(26s) == 2s);
    CHECK(plan.keeps_aspect(30s) == 25s);
    CHECK(plan.keeps_aspect(4s) == 1s);
  }

  SECTION("ShouldGiveOneGroupGreenAtATimeGivenTurns") {
    std::vector<SignalPlan> plans = plan_in_turn(3, 20s, 3s, 2s);
    REQUIRE(plans.size() == 3);
    for (const SignalPlan& plan : plans) {
      CHECK(plan.cycle() == 75s);
    }
    for (auto t = 0ms; t < 150s; t += 100ms) {
      int green_or_yellow = 0;
      for (const SignalPlan& plan : plans) {
        green_or_yellow += plan.aspect_at(t) != Aspect::RED ? 1 : 0;
      }
      CHECK(green_or_yellow <= 1);
    }
    CHECK(plans[1].aspect_at(25s) == Aspect::GREEN);
    CHECK(plans[0].aspect_at(24s) == Aspect::RED);  // All red between turns.
    CHECK(plans[1].aspect_at(24s) == Aspect::RED);
  }

  SECTION("ShouldPutStopLinesOnLanesTheLightHoldsFor") {
    road::Map network = network_with_light();
    Control control = build_control(network);
    REQUIRE(control.groups().size() == 1);
    CHECK(control.groups()[0].junction == "9");
    CHECK(control.groups()[0].sequence == 3);
    REQUIRE(control.stop_lines().size() == 1);  // Not -2, nor the sign.
    const StopLine& line = control.stop_lines()[0];
    CHECK(line.lane == road::LaneKey{.road = 0, .section = 0, .lane = -1});
    CHECK(line.along == 90.0);
    CHECK(control.stop_lines_on(line.lane).size() == 1);
    CHECK(control.stop_lines_on({.lane = 1}).empty());
  }

  SECTION("ShouldPutStopLinesAgainstSGivenNegativeOrientation") {
    road::Map network = network_with_light();
    road::Signal& light = network.roads[0].signals[0];
    light.orientation = road::Direction::NEGATIVE;
    light.validities.clear();
    Control control = build_control(network);
    REQUIRE(control.stop_lines().size() == 2);
    CHECK(control.stop_lines()[0].lane.lane == 1);
    CHECK(control.stop_lines()[0].along == 10.0);  // Measured against s.
    CHECK(control.stop_lines()[1].lane.lane == 2);
  }

  SECTION("ShouldStopForLightsAsMovsimDrivers") {
    IntelligentDriver driver{.desired_speed = 15.0 * meter_per_second};
    LightBraking braking;
    auto stops = [&](Aspect aspect, double speed, double distance) {
      return stops_at_light(driver, braking, aspect, speed * meter_per_second,
                            distance * meter);
    };
    CHECK_FALSE(stops(Aspect::GREEN, 10.0, 100.0));
    CHECK(stops(Aspect::YELLOW, 10.0, 100.0));  // Far off: brakes gently.
    // Close: the IDM's braking to the light would pass 4 m/s^2.
    CHECK_FALSE(stops(Aspect::YELLOW, 15.0, 20.0));
    // Red: whenever it can stop at its maximum, 12.5 m at 15 m/s.
    CHECK(stops(Aspect::RED, 15.0, 40.0));
    CHECK(stops(Aspect::RED, 15.0, 13.0));
    // Creeping up to the line, it can still stop at it.
    CHECK(stops(Aspect::RED, 0.05, 0.01));
    CHECK(stops(Aspect::RED, 0.0, 0.5));  // Waiting at the line.
    // movsim's kinematic test: at 20 m/s, 33.3 m to stop at 6 m/s^2,
    // whatever the IDM would brake.
    LightBraking bold{.yellow = 100.0 * meter_per_second_squared};
    CHECK_FALSE(stops_at_light(driver, bold, Aspect::YELLOW,
                               20.0 * meter_per_second, 30.0 * meter));
    CHECK(stops_at_light(driver, bold, Aspect::YELLOW, 20.0 * meter_per_second,
                         40.0 * meter));
  }

  SECTION("ShouldStopAtTheLineGivenNoMinimumGap") {
    IntelligentDriver driver;
    // Standing at the line it wants nothing more; short of it, it creeps.
    CHECK(
        compute_stop_acceleration(driver, 0.0 * meter_per_second, 1.0 * meter) >
        0.0 * meter_per_second_squared);
    CHECK(compute_stop_acceleration(driver, 1.0 * meter_per_second,
                                    0.02 * meter) <
          -100.0 * meter_per_second_squared);
    // Within a centimeter, it stops at once and stays there.
    CHECK(std::isinf(
        compute_stop_acceleration(driver, 0.1 * meter_per_second, 0.005 * meter)
            .numerical_value_in(meter_per_second_squared)));
    CHECK(compute_stop_acceleration(driver, 0.0 * meter_per_second,
                                    0.005 * meter) ==
          0.0 * meter_per_second_squared);
  }
}

}  // namespace simon::traffic
