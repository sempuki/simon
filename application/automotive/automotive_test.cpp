// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "base/testing.hpp"

namespace simon::automotive {

namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 100ms;

auto run(Simulation& simulation, Duration duration, auto&& each_step) -> void {
  for (TimePoint time{}; time < TimePoint{duration}; time += DT) {
    REQUIRE(simulation.step(Step{.time = time, .dt = DT}));
    each_step(time + DT);
  }
}

// The shortest gap between two vehicles in one lane, rear bumper to front
// bumper.
auto shortest_gap(const Simulation& simulation) -> double {
  const World& world = simulation.world();
  std::map<LaneKey, std::vector<std::pair<double, double>>> by_lane;
  world.store_of<LaneState>().for_each(
      [&](Entity owner, const LaneState& state) {
        const Driver& driver = world.store_of<Driver>().component_of(owner);
        by_lane[state.lane].emplace_back(
            along_lane(simulation.network(), state.lane, state.s),
            driver.length.numerical_value_in(meter));
      });
  double shortest = 1e9;
  for (auto& [lane, vehicles] : by_lane) {
    std::ranges::sort(vehicles);
    for (std::size_t i = 1; i < vehicles.size(); ++i) {
      shortest = std::min(shortest, vehicles[i].first - vehicles[i].second -
                                        vehicles[i - 1].first);
    }
  }
  return shortest;
}

auto mean_speed(const Simulation& simulation) -> double {
  double total = 0.0;
  std::size_t count = 0;
  simulation.world().store_of<LaneState>().for_each(
      [&](Entity, const LaneState& state) {
        total += state.speed.numerical_value_in(meter_per_second);
        ++count;
      });
  return total / static_cast<double>(count);
}

auto states(const Simulation& simulation) -> std::vector<LaneState> {
  std::vector<LaneState> all;
  simulation.world().store_of<LaneState>().for_each(
      [&](Entity, const LaneState& state) { all.push_back(state); });
  return all;
}

}  // namespace

TEST_CASE("Automotive") {
  SECTION("ShouldCirculateWithoutOverlapGivenRing") {
    // 40 vehicles on a ring of 628 m, two lanes each way.
    Simulation simulation{Scenario{.seed = 3, .vehicles = 40}};
    REQUIRE(simulation.configure());
    double shortest = shortest_gap(simulation);
    std::map<Entity, int> lanes;
    int changes = 0;
    simulation.world().store_of<LaneState>().for_each(
        [&](Entity owner, const LaneState& state) {
          lanes[owner] = state.lane.lane;
        });
    run(simulation, 300s, [&](TimePoint) {
      shortest = std::min(shortest, shortest_gap(simulation));
      simulation.world().store_of<LaneState>().for_each(
          [&](Entity owner, const LaneState& state) {
            if (lanes[owner] != state.lane.lane) {
              ++changes;
              lanes[owner] = state.lane.lane;
            }
          });
    });
    CAPTURE(shortest, changes, mean_speed(simulation));
    // Never closer than 1.09 m, after a lane change cuts in.
    CHECK(shortest > 0.5);
    CHECK(mean_speed(simulation) > 15.0);
    CHECK(changes > 0);

    // Keeping right, more drive in the outer lanes, the right of travel.
    int outer = 0;
    for (const LaneState& state : states(simulation)) {
      outer += std::abs(state.lane.lane) == 2 ? 1 : 0;
    }
    CAPTURE(outer);
    CHECK(outer > 20);
  }

  SECTION("ShouldRepeatGivenSameSeed") {
    auto final_states = [] {
      Simulation simulation{Scenario{.seed = 9, .vehicles = 30}};
      REQUIRE(simulation.configure());
      run(simulation, 60s, [](TimePoint) {});
      return states(simulation);
    };
    std::vector<LaneState> first = final_states();
    std::vector<LaneState> again = final_states();
    REQUIRE(first.size() == again.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
      CHECK(first[i].lane == again[i].lane);
      CHECK(first[i].s == again[i].s);
      CHECK(first[i].speed == again[i].speed);
    }
  }

  SECTION("ShouldFlowThroughJunctionsGivenTown") {
    // CARLA's Town01: one lane each way through twelve junctions.
    Simulation simulation{
        Scenario{.seed = 5,
                 .roads = "3rd_party/carla/Town01.xodr",
                 .vehicles = 60,
                 .following = {.desired_speed = 11.0 * meter_per_second}}};
    REQUIRE(simulation.configure());
    std::uint32_t turns = 0;
    run(simulation, 120s, [](TimePoint) {});
    for (const LaneState& state : states(simulation)) {
      turns += state.turns;
    }
    CAPTURE(turns, mean_speed(simulation));
    CHECK(mean_speed(simulation) > 3.0);
    CHECK(turns > 60);
  }
}

}  // namespace simon::automotive
