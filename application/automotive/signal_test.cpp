// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <span>
#include <string>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "core/argument.hpp"

// Traffic lights against SUMO: a queue at a red light and away on green,
// every vehicle's position and speed compared with sumo_signal.csv.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;

constexpr int VEHICLES = 10;

// The road of light.xodr, its lane, and a world on it holding the ten
// vehicles of sumo_signal.py and the light's controller, red for `red` from
// `offset` and then green.
struct Queue final {
  explicit Queue(std::chrono::nanoseconds offset) {
    auto loaded = load_network(find_road_path("light.xodr"));
    REQUIRE(loaded);
    network = std::move(*loaded);
    REQUIRE(network.control.stop_lines().size() == 1);
    REQUIRE(World::set_up()
                .numbered(1)
                .holding<archetype::TacticalVehicle>(VEHICLES)
                .holding<archetype::SignalController>(1)
                .build(Out(world)));
    LaneKey lane{.road = 0, .section = 0, .lane = -1};
    for (int i = 0; i < VEHICLES; ++i) {
      LaneState state{.lane = lane,
                      .s = (400.0 - 35.0 * i) * meter,
                      .speed = 15.0 * meter_per_second};
      auto built = world.create<archetype::TacticalVehicle>()
                       .with(FollowLane::locate_vehicle(network, state))
                       .with(state)
                       .with(Driver{.following = {.desired_speed =
                                                      20.0 * meter_per_second}})
                       .with(DriveCommand{})
                       .with(Tactical{})
                       .with(Stopped{})
                       .build();
      REQUIRE(built);
      vehicles.push_back(*built);
    }
    REQUIRE(
        world.create<archetype::SignalController>()
            .with(traffic::SignalPlan{
                .phases = {{.duration = 30s, .aspect = traffic::Aspect::RED},
                           {.duration = 1000s,
                            .aspect = traffic::Aspect::GREEN}},
                .offset = offset})
            .with(SignalState{})
            .build());
    world.sync();
  }

  Network network;
  World world;
  std::vector<Entity> vehicles;
};

}  // namespace

TEST_CASE("SignalAgainstSumo") {
  SECTION("ShouldQueueAtRedAndLeaveOnGreenAsSumo") {
    // SUMO's time counts from the step that inserts the vehicles, so its
    // light turns green a step sooner on that clock.
    constexpr auto STEP = 1ms;
    Queue queue{-STEP};
    Scheduler scheduler{make_schedule(queue.network)};
    std::map<std::pair<long, int>, std::pair<double, double>> sumo;
    for (const Row& row : load_rows("sumo_signal.csv")) {
      sumo[{std::lround(number(row, "time") * 1000.0),
            static_cast<int>(number(row, "vehicle"))}] = {
          number(row, "position"), number(row, "speed")};
    }
    double farthest = 0.0;
    double fastest = 0.0;
    double front_stop = 0.0;
    // Until 50 s, before the road's end, a dead end to simon's drivers, comes
    // within their lookahead.
    for (long k = 0; k <= 50000; ++k) {
      if (k % 100 == 0) {
        for (int i = 0; i < VEHICLES; ++i) {
          const LaneState& state =
              queue.world.store_of<LaneState>().component_of(
                  queue.vehicles[static_cast<std::size_t>(i)]);
          auto [position, speed] = sumo.at({k, i});
          farthest = std::max(
              farthest, std::abs(state.s.numerical_value_in(meter) - position));
          fastest = std::max(
              fastest,
              std::abs(state.speed.numerical_value_in(meter_per_second) -
                       speed));
          if (k == 29000 && i == 0) {
            front_stop = state.s.numerical_value_in(meter);
          }
        }
      }
      scheduler.step(Step{.time = TimePoint{} + k * STEP, .dt = STEP},
                     InOut(queue.world));
    }
    CAPTURE(farthest, fastest, front_stop);
    // SUMO's stop: 498.990007 m.
    CHECK(std::abs(front_stop - 498.990007) < 1e-6);
    CHECK(farthest < 0.03);
    CHECK(fastest < 0.005);
  }

  SECTION("ShouldGoOnYellowWhenTooNearAndHoldThroughRed") {
    // Two vehicles at 15 m/s, alone on the road, as the light turns yellow
    // for 3 s and then red: one 15 m from the line, which cannot stop
    // comfortably, and one 80 m away, which can.
    for (double distance : {15.0, 80.0}) {
      Network network = *load_network(find_road_path("light.xodr"));
      World world;
      REQUIRE(World::set_up()
                  .numbered(1)
                  .holding<archetype::TacticalVehicle>(1)
                  .holding<archetype::SignalController>(1)
                  .build(Out(world)));
      LaneState start{.lane = {.road = 0, .section = 0, .lane = -1},
                      .s = (500.0 - distance) * meter,
                      .speed = 15.0 * meter_per_second};
      Entity vehicle =
          *world.create<archetype::TacticalVehicle>()
               .with(FollowLane::locate_vehicle(network, start))
               .with(start)
               .with(Driver{
                   .following = {.desired_speed = 15.0 * meter_per_second}})
               .with(DriveCommand{})
               .with(Tactical{})
               .with(Stopped{})
               .build();
      REQUIRE(world.create<archetype::SignalController>()
                  .with(traffic::SignalPlan{
                      .phases =
                          {{.duration = 3s, .aspect = traffic::Aspect::YELLOW},
                           {.duration = 100s, .aspect = traffic::Aspect::RED}}})
                  .with(SignalState{})
                  .build());
      world.sync();
      Scheduler scheduler{make_schedule(network)};
      for (long k = 0; k < 300; ++k) {
        scheduler.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms},
                       InOut(world));
      }
      double s = world.store_of<LaneState>()
                     .component_of(vehicle)
                     .s.numerical_value_in(meter);
      CAPTURE(distance, s);
      if (distance == 15.0) {
        CHECK(s > 500.0);  // Went on yellow, and on through red.
      } else {
        CHECK(std::abs(s - 499.0) < 0.01);  // Stopped short of the line.
      }
    }
  }

  SECTION("ShouldNeverCrossOnRedUncommittedGivenJunction") {
    // The signalized junction's four approaches, two groups taking turns,
    // 40 vehicles for 10 min: every crossing of a stop line is on green or
    // yellow, or on red by a driver that committed to it before.
    Simulation simulation{
        Scenario{.seed = 5,
                 .roads = std::string{ROADS} + "signalized.xodr",
                 .vehicles = 40,
                 .green = 15s}};
    REQUIRE(simulation.configure());
    const Network& network = simulation.network();
    std::span<const traffic::StopLine> lines = network.control.stop_lines();
    REQUIRE(lines.size() == 4);
    std::map<Entity, LaneState> before;
    std::map<Entity, std::uint32_t> committed;
    int crossings = 0;
    int on_red = 0;
    for (long k = 0; k < 6000; ++k) {
      simulation.world().store_of<LaneState>().for_each(
          [&](Entity owner, const LaneState& state) { before[owner] = state; });
      simulation.world().store_of<Tactical>().for_each(
          [&](Entity owner, const Tactical& tactical) {
            committed[owner] = tactical.committed;
          });
      std::vector<traffic::Aspect> aspects(network.control.groups().size());
      simulation.world().store_of<SignalState>().for_each(
          [&](Entity, const SignalState& signal) {
            aspects[signal.group] = signal.aspect;
          });
      REQUIRE(
          simulation.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms}));
      simulation.world().store_of<LaneState>().for_each(
          [&](Entity owner, const LaneState& state) {
            const LaneState& was = before.at(owner);
            double from = along_lane(network, was.lane, was.s);
            double to = state.lane == was.lane
                            ? along_lane(network, state.lane, state.s)
                            : find_lane_length(network, was.lane);
            for (const traffic::StopLine& line :
                 network.control.stop_lines_on(was.lane)) {
              if (from < line.along && line.along <= to) {
                ++crossings;
                auto index = static_cast<std::uint32_t>(&line - lines.data());
                if (aspects[line.group] == traffic::Aspect::RED) {
                  ++on_red;
                  CHECK(committed.at(owner) == index);
                }
              }
            }
          });
    }
    CAPTURE(crossings, on_red);
    CHECK(crossings > 100);
  }
}

}  // namespace simon::automotive
