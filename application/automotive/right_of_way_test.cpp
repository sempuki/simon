// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "core/vocabulary.hpp"
#include "model/collision.hpp"

// Right of way in junctions: who gives way to whom on the test networks,
// gap acceptance against Harders' capacity of a minor stream, and traffic
// that never deadlocks and never runs into itself.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;

// The pairs of vehicles that overlap, the lesser entity first.
auto find_overlaps(const World& world) -> std::set<std::pair<Entity, Entity>> {
  std::vector<std::pair<Entity, model::OrientedBox>> boxes;
  const auto& drivers = world.store_of<Driver>();
  world.store_of<RoadPose>().for_each([&](Entity owner, const RoadPose& pose) {
    if (const Driver* driver = drivers.maybe_component_of(owner)) {
      boxes.emplace_back(owner, create_box(pose, *driver));
    }
  });
  std::set<std::pair<Entity, Entity>> overlaps;
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    for (std::size_t j = i + 1; j < boxes.size(); ++j) {
      if (model::detect_overlap(boxes[i].second, boxes[j].second)) {
        // Each pair once, whichever order the store holds them in.
        overlaps.insert(std::minmax(boxes[i].first, boxes[j].first));
      }
    }
  }
  return overlaps;
}

// How many pairs of vehicles overlap.
auto count_overlaps(const World& world) -> int {
  return static_cast<int>(find_overlaps(world).size());
}

auto road_index(const Network& network, std::string_view id) -> std::uint32_t {
  const model::Road* road = network.roads.find_road(id);
  REQUIRE(road);
  return static_cast<std::uint32_t>(road - network.roads.roads.data());
}

}  // namespace

TEST_CASE("RightOfWay") {
  SECTION("ShouldGiveWayAsPrioritiesSignsTurnsAndTheRightSay") {
    auto network = load_network(find_road_path("priority.xodr"));
    REQUIRE(network);
    std::map<std::string, int> why;
    for (const model::Conflict& conflict : network->rights.conflicts()) {
      std::string pair = network->roads.roads[conflict.lane.road].id + ">" +
                         network->roads.roads[conflict.foe.road].id;
      why[pair] = static_cast<int>(conflict.why);
    }
    using model::Yielding;
    CHECK(why == std::map<std::string, int>{
                     {"es>we", static_cast<int>(Yielding::PRIORITY)},
                     {"es>ws", static_cast<int>(Yielding::TURN)},
                     {"sw>es", static_cast<int>(Yielding::SIGN)},
                     {"sw>we", static_cast<int>(Yielding::PRIORITY)},
                     {"sw>ew", static_cast<int>(Yielding::PRIORITY)},
                     {"se>we", static_cast<int>(Yielding::PRIORITY)}});
  }

  SECTION("ShouldKeepSignalGroupsApartGivenLights") {
    // Left turns give way to the oncoming traffic of their own group; lanes
    // of different groups keep their conflicts both ways, each waiting only
    // for the other still in the junction.
    auto network = load_network(find_road_path("signalized.xodr"));
    REQUIRE(network);
    std::set<std::pair<LaneKey, LaneKey>> lights;
    int turns = 0;
    for (const model::Conflict& conflict : network->rights.conflicts()) {
      if (conflict.why == model::Yielding::LIGHTS) {
        lights.emplace(conflict.lane, conflict.foe);
      } else {
        CHECK(conflict.why == model::Yielding::TURN);
        ++turns;
      }
    }
    CHECK(turns == 8);
    CHECK(lights.size() > 0);
    for (const auto& [lane, foe] : lights) {
      CHECK(lights.contains({foe, lane}));
    }
  }

  SECTION("ShouldTakeEachGapAsItsCriticalGapAllows") {
    // A Poisson major stream of 400 vehicles an hour, and a minor queue
    // that never empties, for an hour: in each gap between major vehicles
    // passing the conflict, as many minor drivers enter as reach where they
    // wait at least their critical gap before the next, the queue's
    // drivers reaching it as it discharges in gaps of 30 s and more.
    auto loaded = load_network(find_road_path("crossing.xodr"));
    REQUIRE(loaded);
    Network network = std::move(*loaded);
    World world;
    REQUIRE(World::set_up()
                .numbered(1)
                .holding<archetype::TacticalVehicle>(400)
                .build(Out(world)));
    LaneKey major_in{.road = road_index(network, "in_w"), .lane = -1};
    LaneKey minor_in{.road = road_index(network, "in_s"), .lane = -1};
    LaneKey major_cross{.road = road_index(network, "we"), .lane = -1};
    std::uint32_t out_e = road_index(network, "out_e");
    std::uint32_t out_n = road_index(network, "out_n");
    Scheduler scheduler{make_schedule(network)};
    std::mt19937_64 random{7};
    double flow = 400.0 / 3600.0;  // Major vehicles a second.
    std::exponential_distribution<double> headway{flow};
    double next_major = headway(random);
    auto add = [&](LaneKey lane, double speed) {
      LaneState state{
          .lane = lane, .s = 0.0 * meter, .speed = speed * meter_per_second};
      REQUIRE(world.create<archetype::TacticalVehicle>()
                  .with(FollowLane::locate_vehicle(network, state))
                  .with(state)
                  .with(Driver{
                      .following = {.desired_speed = 15.0 * meter_per_second}})
                  .with(DriveCommand{})
                  .with(Tactical{})
                  .with(Stopped{})
                  .build());
    };
    std::vector<double> majors;   // Their fronts reaching the conflict.
    std::vector<double> cleared;  // Their tails leaving it.
    std::vector<double> minors;
    std::map<Entity, LaneState> was;
    constexpr auto DT = 100ms;
    long steps = 3900 * 10;
    for (long k = 0; k < steps; ++k) {
      double now = static_cast<double>(k) * 0.1;
      // A major vehicle when it is due, if the lane's start is clear; a
      // minor whenever its lane's start is.
      auto clear = [&](const LaneKey& lane, double ahead) {
        bool free = true;
        world.store_of<LaneState>().for_each([&](Entity, const LaneState& s) {
          free = free &&
                 !(s.lane == lane && s.s.numerical_value_in(meter) < ahead);
        });
        return free;
      };
      if (now >= next_major && clear(major_in, 20.0)) {
        add(major_in, 15.0);
        next_major += headway(random);
      }
      if (clear(minor_in, 12.0)) {
        add(minor_in, 10.0);
      }
      world.sync();
      was.clear();
      world.store_of<LaneState>().for_each(
          [&](Entity owner, const LaneState& state) { was[owner] = state; });
      scheduler.step(Step{.time = TimePoint{} + k * DT, .dt = DT},
                     InOut(world));
      std::vector<Entity> gone;
      world.store_of<LaneState>().for_each([&](Entity owner,
                                               const LaneState& state) {
        auto passed = [&](const LaneKey& lane, double at) {
          const LaneState& before = was.at(owner);
          double from =
              before.lane == lane ? before.s.numerical_value_in(meter) : -1.0;
          double to =
              state.lane == lane ? state.s.numerical_value_in(meter) : 1e9;
          return (before.lane == lane || state.lane == lane) && from < at &&
                 at <= to;
        };
        if (now >= 300.0) {
          if (passed(major_cross, 11.75)) {
            majors.push_back(now);
          }
          if (passed(major_cross, 11.75 + 4.5)) {
            cleared.push_back(now);
          }
          if (passed(minor_in, 299.0)) {
            minors.push_back(now);  // Past where it waits.
          }
        }
        if ((state.lane.road == out_e || state.lane.road == out_n) &&
            state.s.numerical_value_in(meter) > 250.0) {
          gone.push_back(owner);
        }
      });
      for (Entity e : gone) {
        REQUIRE(world.destroy(e).build());
      }
      world.sync();
    }
    // Each minor's way into the junction, by gap: the times it entered,
    // from the gap's start.
    std::vector<std::vector<double>> entries(majors.size());
    std::size_t m = 0;
    for (std::size_t g = 0; g + 1 < majors.size(); ++g) {
      while (m < minors.size() && minors[m] <= cleared[g]) {
        ++m;
      }
      while (m < minors.size() && minors[m] <= majors[g + 1]) {
        entries[g].push_back(minors[m] - cleared[g]);
        ++m;
      }
    }
    // The queue's discharge in gaps of 30 s and more: when its k-th driver
    // enters, on average.
    std::vector<double> profile;
    std::vector<int> samples;
    for (std::size_t g = 0; g + 1 < majors.size(); ++g) {
      if (majors[g + 1] - cleared[g] < 30.0) {
        continue;
      }
      for (std::size_t k = 0; k < entries[g].size(); ++k) {
        if (k >= profile.size()) {
          profile.push_back(0.0);
          samples.push_back(0);
        }
        profile[k] += entries[g][k];
        ++samples[k];
      }
    }
    for (std::size_t k = 0; k < profile.size(); ++k) {
      profile[k] /= samples[k];
    }
    double t_c = 6.0;
    int expected = 0;
    int actual = 0;
    int agree = 0;
    for (std::size_t g = 0; g + 1 < majors.size(); ++g) {
      double gap = majors[g + 1] - cleared[g];
      int n = 0;
      for (double y : profile) n += gap - y >= t_c ? 1 : 0;
      expected += n;
      actual += static_cast<int>(entries[g].size());
      agree += n == static_cast<int>(entries[g].size()) ? 1 : 0;
    }
    auto gaps = static_cast<int>(majors.size() - 1);
    CAPTURE(expected, actual, agree, gaps, profile);
    CHECK(gaps > 350);
    CHECK(agree > 0.9 * gaps);
    CHECK(std::abs(actual - expected) < 0.08 * expected);
  }

  SECTION("ShouldNeverDeadlockGivenTheRightAlone") {
    // Four drivers, one on each arm of the crossroads, 30 m from the
    // junction at 8 m/s, each going whichever way its seed says, for 100
    // seeds: every one gets through, and no two ever overlap.
    auto loaded = load_network(find_road_path("crossroads.xodr"));
    REQUIRE(loaded);
    Network network = std::move(*loaded);
    int stuck = 0;
    int overlapping = 0;
    for (std::uint64_t seed = 0; seed < 100; ++seed) {
      World world;
      REQUIRE(World::set_up()
                  .numbered(1)
                  .holding<archetype::TacticalVehicle>(4)
                  .build(Out(world)));
      std::vector<Entity> vehicles;
      for (const char* arm : {"n", "e", "s", "w"}) {
        LaneState state{.lane = {.road = road_index(network, arm), .lane = -1},
                        .s = 70.0 * meter,
                        .speed = 8.0 * meter_per_second};
        vehicles.push_back(
            *world.create<archetype::TacticalVehicle>()
                 .with(FollowLane::locate_vehicle(network, state))
                 .with(state)
                 .with(Driver{
                     .following = {.desired_speed = 10.0 * meter_per_second},
                     .seed = seed * 4 + vehicles.size()})
                 .with(DriveCommand{})
                 .with(Tactical{})
                 .with(Stopped{})
                 .build());
      }
      world.sync();
      Scheduler scheduler{make_schedule(network)};
      // Each driver gets through once it leaves the junction it entered.
      std::map<Entity, int> through;  // 0 before, 1 in it, 2 through.
      for (long k = 0; k < 600; ++k) {
        scheduler.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms},
                       InOut(world));
        int now = count_overlaps(world);
        overlapping += now;
        for (Entity vehicle : vehicles) {
          const LaneState& state =
              world.store_of<LaneState>().component_of(vehicle);
          bool inside = network.roads.roads[state.lane.road].junction == "1";
          int& stage = through[vehicle];
          stage = stage == 0 && inside ? 1 : stage == 1 && !inside ? 2 : stage;
        }
      }
      for (Entity vehicle : vehicles) {
        stuck += through[vehicle] == 2 ? 0 : 1;
      }
    }
    CAPTURE(stuck, overlapping);
    CHECK(stuck == 0);
    CHECK(overlapping < 10);  // Vehicle-steps in 100 runs of 60 s.
  }

  SECTION("ShouldNotRunIntoEachOtherGivenJunctions") {
    // Traffic for 10 min on each network with junctions.
    // Pairs that start to overlap, against 763, 399, 102 and 228 with no
    // right of way.
    struct Case final {
      const char* file = nullptr;
      int vehicles = 0;
      int limit = 0;
    };
    for (auto [file, vehicles, limit] :
         {Case{"crossroads.xodr", 40, 0}, Case{"priority.xodr", 25, 2},
          Case{"signalized.xodr", 40, 0}, Case{"Town01.xodr", 60, 66}}) {
      Simulation simulation{Scenario{
          .seed = 11, .roads = find_road_path(file), .vehicles = vehicles}};
      REQUIRE(simulation.configure());
      int overlapping = 0;
      int worst = 0;
      int events = 0;  // Pairs that start to overlap.
      std::set<std::pair<Entity, Entity>> before;
      for (long k = 0; k < 6000; ++k) {
        REQUIRE(simulation.step(
            Step{.time = TimePoint{} + k * 100ms, .dt = 100ms}));
        std::set<std::pair<Entity, Entity>> now =
            find_overlaps(simulation.world());
        for (const auto& pair : now) {
          events += before.contains(pair) ? 0 : 1;
        }
        overlapping += static_cast<int>(now.size());
        worst = std::max(worst, static_cast<int>(now.size()));
        before = std::move(now);
      }
      CAPTURE(file, overlapping, worst, events);
      CHECK(events <= limit);
    }
  }
}

}  // namespace simon::automotive
