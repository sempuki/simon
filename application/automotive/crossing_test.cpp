// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <random>
#include <set>
#include <span>
#include <string>
#include <vector>

#include "application/automotive/simulation.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "core/vocabulary.hpp"
#include "model/collision.hpp"

// Pedestrians crossing: how long they wait at a light, and at a crosswalk
// whose traffic does not yield, against the Highway Capacity Manual; and
// vehicles stopping for them where traffic yields.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;

constexpr double WALK = 1.34;  // m/s, every pedestrian's here.

// The crosswalk's crossing, and the sidewalk piece leading to its end from
// the east (lower s) and away from its other end to the east.
struct Crossing final {
  std::uint32_t crossing = 0;
  model::Leg to_kerb;
  model::Leg across;
  model::Leg away;
  double approach = 0.0;  // m, the leading piece's length.
};

auto find_crossing(const Network& network) -> Crossing {
  std::span<const model::WalkEdge> edges = network.walking.edges();
  Crossing found;
  for (std::uint32_t e = 0; e < edges.size(); ++e) {
    if (edges[e].kind == model::WalkEdge::Kind::CROSSING) {
      found.crossing = e;
    }
  }
  const model::WalkEdge& crossing = edges[found.crossing];
  found.across = model::Leg{.edge = found.crossing, .forward = true};
  // A sidewalk piece ending at the crossing's start, coming from lower x.
  for (std::uint32_t e = 0; e < edges.size(); ++e) {
    const model::WalkEdge& edge = edges[e];
    if (edge.kind != model::WalkEdge::Kind::SIDEWALK) {
      continue;
    }
    bool lower = std::min(edge.path.front().x, edge.path.back().x) < 199.0;
    if (lower && (edge.to == crossing.from || edge.from == crossing.from)) {
      found.to_kerb = {.edge = e, .forward = edge.to == crossing.from};
      found.approach = edge.length();
    }
    if (lower && (edge.to == crossing.to || edge.from == crossing.to)) {
      found.away = {.edge = e, .forward = edge.from == crossing.to};
    }
  }
  return found;
}

// A pedestrian `ahead` meters before the kerb, walking at WALK across and
// away.
auto add_walker(const Network& network, const Crossing& crossing, double ahead,
                InOut<World> world) -> Entity {
  WalkRoute route{.legs = {crossing.to_kerb, crossing.across, crossing.away}};
  WalkState state{.along = (crossing.approach - ahead) * meter};
  auto built = world->create<archetype::Pedestrian>()
                   .with(PlaceWalker::locate_walker(network, route, state))
                   .with(state)
                   .with(Walker{.desired_speed = WALK * meter_per_second})
                   .with(std::move(route))
                   .with(WalkCommand{})
                   .build();
  REQUIRE(built);
  return *built;
}

// The Highway Capacity Manual's rule for a pedestrian where drivers do not
// yield: it waits for the first gap of `t_c` with no vehicle on the
// crosswalk. Given when each vehicle is on it, from its front reaching it
// to its rear leaving it, in order, how long one arriving at `at` waits.
auto wait_for_gap(std::span<const std::pair<double, double>> on, double at,
                  double t_c) -> double {
  constexpr double LONGEST = 60.0;  // s any vehicle is on it.
  double t = at;
  for (bool moved = true; moved;) {
    moved = false;
    auto first = std::ranges::lower_bound(on, t - LONGEST, {},
                                          &std::pair<double, double>::first);
    for (auto it = first; it != on.end() && it->first < t + t_c; ++it) {
      if (it->second > t) {
        t = it->second;
        moved = true;
      }
    }
  }
  return t - at;
}

}  // namespace

TEST_CASE("Crossing") {
  SECTION("ShouldWaitAsTheManualSaysGivenLight") {
    // A light on a 60 s cycle, green 30 s and yellow 3 s for the road, red
    // 27 s; a pedestrian walks while the road has red and there is time to
    // walk the 9 m across, so its effective walk time g is 27 s less 6.7 s.
    // Pedestrians arriving evenly through the cycle wait on average
    // (C - g)^2 / 2C, as the Highway Capacity Manual has it: 600 of them,
    // 15.1 s apart, one at each tenth of a second of the cycle.
    Network network = create_network("midblock.xodr");
    REQUIRE(network.crosswalk_groups.size() == 1);
    REQUIRE(network.crosswalk_groups[0]);
    Crossing crossing = find_crossing(network);
    double across = network.walking.edges()[crossing.crossing].length() / WALK;
    World world;
    REQUIRE(World::set_up()
                .numbered(1)
                .holding<archetype::Pedestrian>(16)
                .holding<archetype::SignalController>(1)
                .build(Out(world)));
    REQUIRE(
        world.create<archetype::SignalController>()
            .with(model::SignalPlan{
                .phases = {{.duration = 30s, .aspect = model::Aspect::GREEN},
                           {.duration = 3s, .aspect = model::Aspect::YELLOW},
                           {.duration = 27s, .aspect = model::Aspect::RED}}})
            .with(SignalState{})
            .build());
    world.sync();
    Scheduler scheduler{make_schedule(network)};
    constexpr double AHEAD = 20.0;  // m from the kerb each starts.
    constexpr long EVERY = 151;     // Steps of 0.1 s between them.
    std::map<Entity, double> due;   // When each would reach the kerb alone.
    std::vector<double> waits;
    for (long k = 0; k < 600 * EVERY + 1200; ++k) {
      double now = static_cast<double>(k) * 0.1;
      if (k % EVERY == 0 && k < 600 * EVERY) {
        due[add_walker(network, crossing, AHEAD, InOut(world))] =
            now + AHEAD / WALK;
        world.sync();
      }
      scheduler.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms},
                     InOut(world));
      std::vector<Entity> done;
      world.store_of<WalkState>().for_each(
          [&](Entity owner, const WalkState& state) {
            if (state.leg == 1 && due.contains(owner)) {
              // Onto the crosswalk this step: back to when it stepped on.
              double stepped =
                  now + 0.1 - state.along.numerical_value_in(meter) / WALK;
              waits.push_back(stepped - due.at(owner));
              due.erase(owner);
            }
            if (state.leg == 2) {
              done.push_back(owner);
            }
          });
      for (Entity e : done) {
        REQUIRE(world.destroy(e).build());
      }
      world.sync();
    }
    double mean = 0.0;
    for (double w : waits) {
      mean += w;
    }
    mean /= static_cast<double>(waits.size());
    double cycle = 60.0;
    double green = 27.0 - across;
    double manual = (cycle - green) * (cycle - green) / (2.0 * cycle);
    CAPTURE(waits.size(), mean, manual);
    CHECK(waits.size() == 600);
    CHECK(std::abs(mean - manual) < 0.02 * manual);
  }

  SECTION("ShouldWaitAsTheManualSaysGivenVehiclesArrivingAtRandom") {
    // The manual's delay for a pedestrian where drivers do not yield,
    // (e^(q t_c) - q t_c - 1) / q, assumes vehicles arriving at random, and
    // passing in no time: its rule gives it for 20,000 pedestrians arriving
    // at random against 600 vehicles an hour, needing 8.7 s.
    std::mt19937_64 random{3};
    constexpr double Q = 600.0 / 3600.0;  // Vehicles a second.
    constexpr double T_C = 8.7;           // s.
    constexpr double HOURS = 100.0;
    std::exponential_distribution<double> headway{Q};
    std::vector<std::pair<double, double>> on;
    for (double t = headway(random); t < HOURS * 3600.0; t += headway(random)) {
      on.emplace_back(t, t);
    }
    std::uniform_real_distribution<double> arrival{0.0, HOURS * 3600.0 - 600.0};
    double mean = 0.0;
    for (int k = 0; k < 20000; ++k) {
      mean += wait_for_gap(on, arrival(random), T_C) / 20000.0;
    }
    double manual = (std::exp(Q * T_C) - Q * T_C - 1.0) / Q;
    CAPTURE(mean, manual);
    CHECK(std::abs(mean - manual) < 0.02 * manual);
  }

  SECTION("ShouldWaitForGapsAsTheManualSaysGivenTrafficThatDoesNotYield") {
    // Vehicles that do not yield, set off at random, 300 an hour each way,
    // and pedestrians needing a gap t_c of their 6.7 s across and a 2 s
    // start-up, and a crosswalk clear of vehicles: each waits as the
    // manual's rule has it against the vehicles as they passed. Following
    // each other, they keep their distance, and no longer arrive at random,
    // so the manual's formula does not hold.
    Network network = create_network("zebra.xodr");
    network.vehicles_yield = false;
    Crossing crossing = find_crossing(network);
    double across = network.walking.edges()[crossing.crossing].length() / WALK;
    World world;
    REQUIRE(World::set_up()
                .numbered(1)
                .holding<archetype::Pedestrian>(16)
                .holding<archetype::TacticalVehicle>(64)
                .build(Out(world)));
    Scheduler scheduler{make_schedule(network)};
    std::mt19937_64 random{11};
    std::exponential_distribution<double> headway{300.0 / 3600.0};
    LaneKey east{.road = 0, .section = 0, .lane = -1};
    LaneKey west{.road = 0, .section = 0, .lane = 1};
    std::map<int, double> next{{-1, headway(random)}, {1, headway(random)}};
    constexpr double SPEED = 50.0 / 3.6;  // m/s.
    auto add_vehicle = [&](const LaneKey& lane) {
      LaneState state{.lane = lane,
                      .s = find_s_along(network, lane, 0.0),
                      .speed = SPEED * meter_per_second};
      REQUIRE(world.create<archetype::TacticalVehicle>()
                  .with(FollowLane::locate_vehicle(network, state))
                  .with(state)
                  .with(Driver{
                      .following = {.desired_speed = SPEED * meter_per_second}})
                  .with(DriveCommand{})
                  .with(Tactical{})
                  .with(Stopped{})
                  .build());
    };
    // Where each lane meets the crosswalk.
    std::map<int, model::CrosswalkZone> zones;
    for (const model::CrosswalkZone& zone : network.walking.zones()) {
      zones[zone.lane.lane] = zone;
    }
    REQUIRE(zones.size() == 2);
    constexpr double AHEAD = 20.0;  // m from the kerb each starts.
    constexpr long EVERY = 301;     // Steps between pedestrians.
    constexpr long PEDESTRIANS = 2000;
    // When each pedestrian would reach where it decides, walking on.
    std::map<Entity, double> due;
    std::vector<double> arrivals;
    std::vector<double> waits;
    std::map<Entity, double> entered;
    std::vector<std::pair<double, double>> on;
    for (long k = 0; k < PEDESTRIANS * EVERY + 1200; ++k) {
      double now = static_cast<double>(k) * 0.1;
      for (auto& [lane, at] : next) {
        if (now >= at) {
          add_vehicle(lane < 0 ? east : west);
          at += headway(random);
        }
      }
      if (k % EVERY == 0 && k < PEDESTRIANS * EVERY) {
        due[add_walker(network, crossing, AHEAD, InOut(world))] =
            now + (AHEAD - Pace::KERB) / WALK;
      }
      world.sync();
      std::map<Entity, double> before;
      world.store_of<LaneState>().for_each(
          [&](Entity owner, const LaneState& state) {
            before[owner] = along_lane(network, state.lane, state.s);
          });
      scheduler.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms},
                     InOut(world));
      std::vector<Entity> done;
      world.store_of<LaneState>().for_each(
          [&](Entity owner, const LaneState& state) {
            const model::CrosswalkZone& zone = zones.at(state.lane.lane);
            double along = along_lane(network, state.lane, state.s);
            double back = along - before.at(owner);
            double length = world.store_of<Driver>()
                                .component_of(owner)
                                .length.numerical_value_in(meter);
            // When, in the step, it passed `mark`.
            auto passed = [&](double mark) {
              return now + 0.1 * (mark - before.at(owner)) / back;
            };
            if (before.at(owner) < zone.near && along >= zone.near) {
              entered[owner] = passed(zone.near);
            }
            double clear = zone.far + length;
            if (before.at(owner) < clear && along >= clear) {
              on.emplace_back(entered.at(owner), passed(clear));
              entered.erase(owner);
            }
            if (along > 390.0) {
              done.push_back(owner);
            }
          });
      const auto& commands = world.store_of<WalkCommand>();
      world.store_of<WalkState>().for_each(
          [&](Entity owner, const WalkState& state) {
            if (due.contains(owner) &&
                commands.component_of(owner).crossing != WalkCommand::NONE) {
              arrivals.push_back(due.at(owner));
              waits.push_back(std::max(0.0, now - due.at(owner)));
              due.erase(owner);
            }
            if (state.leg == 2) {
              done.push_back(owner);
            }
          });
      for (Entity e : done) {
        REQUIRE(world.destroy(e).build());
      }
      world.sync();
    }
    std::ranges::sort(on);
    double mean = 0.0;
    double manual = 0.0;
    for (std::size_t i = 0; i < waits.size(); ++i) {
      mean += waits[i] / static_cast<double>(waits.size());
      manual += wait_for_gap(on, arrivals[i], across + 2.0) /
                static_cast<double>(waits.size());
    }
    CAPTURE(waits.size(), mean, manual, on.size());
    CHECK(waits.size() == PEDESTRIANS);
    CHECK(std::abs(mean - manual) < 0.05 * manual);
  }

  SECTION("ShouldStopForPedestriansGivenTrafficThatYields") {
    // Vehicles that yield, 300 an hour each way, and a pedestrian every
    // 10 s: every pedestrian crosses, vehicles stop for them, and none
    // touches one.
    Network network = create_network("zebra.xodr");
    REQUIRE(network.vehicles_yield);
    Crossing crossing = find_crossing(network);
    World world;
    REQUIRE(World::set_up()
                .numbered(1)
                .holding<archetype::Pedestrian>(16)
                .holding<archetype::TacticalVehicle>(64)
                .build(Out(world)));
    Scheduler scheduler{make_schedule(network)};
    std::mt19937_64 random{5};
    std::exponential_distribution<double> headway{300.0 / 3600.0};
    std::map<int, double> next{{-1, headway(random)}, {1, headway(random)}};
    constexpr double SPEED = 50.0 / 3.6;  // m/s.
    auto add_vehicle = [&](int lane) {
      LaneState state{
          .lane = {.road = 0, .section = 0, .lane = lane},
          .s = find_s_along(network, {.road = 0, .section = 0, .lane = lane},
                            0.0),
          .speed = SPEED * meter_per_second};
      REQUIRE(world.create<archetype::TacticalVehicle>()
                  .with(FollowLane::locate_vehicle(network, state))
                  .with(state)
                  .with(Driver{
                      .following = {.desired_speed = SPEED * meter_per_second}})
                  .with(DriveCommand{})
                  .with(Tactical{})
                  .with(Stopped{})
                  .build());
    };
    constexpr long PEDESTRIANS = 300;
    constexpr long EVERY = 100;
    constexpr double BODY = 0.5;  // m, a pedestrian's square.
    int crossed = 0;
    int stops = 0;
    int touches = 0;
    std::set<Entity> stopped;
    for (long k = 0; k < PEDESTRIANS * EVERY + 1200; ++k) {
      double now = static_cast<double>(k) * 0.1;
      for (auto& [lane, at] : next) {
        if (now >= at) {
          add_vehicle(lane);
          at += headway(random);
        }
      }
      if (k % EVERY == 0 && k < PEDESTRIANS * EVERY) {
        add_walker(network, crossing, 20.0, InOut(world));
      }
      world.sync();
      scheduler.step(Step{.time = TimePoint{} + k * 100ms, .dt = 100ms},
                     InOut(world));
      std::vector<Entity> done;
      std::vector<model::OrientedBox> walkers;
      const auto& poses = world.store_of<RoadPose>();
      world.store_of<WalkState>().for_each(
          [&](Entity owner, const WalkState& state) {
            Vector3 at = poses.component_of(owner)
                             .position.numerical_value_in(meter)
                             .eigen();
            walkers.push_back(
                {.x = at.x(), .y = at.y(), .length = BODY, .width = BODY});
            if (state.leg == 2) {
              ++crossed;
              done.push_back(owner);
            }
          });
      const auto& drivers = world.store_of<Driver>();
      world.store_of<LaneState>().for_each([&](Entity owner,
                                               const LaneState& state) {
        model::OrientedBox box =
            create_box(poses.component_of(owner), drivers.component_of(owner));
        for (const model::OrientedBox& walker : walkers) {
          touches += model::detect_overlap(box, walker) ? 1 : 0;
        }
        if (state.speed.numerical_value_in(meter_per_second) < 0.1 &&
            stopped.insert(owner).second) {
          ++stops;
        }
        if (along_lane(network, state.lane, state.s) > 390.0) {
          done.push_back(owner);
        }
      });
      for (Entity e : done) {
        REQUIRE(world.destroy(e).build());
      }
      world.sync();
    }
    CAPTURE(crossed, stops, touches);
    CHECK(crossed == PEDESTRIANS);
    CHECK(stops > 10);
    CHECK(touches == 0);
  }
}

}  // namespace simon::automotive
