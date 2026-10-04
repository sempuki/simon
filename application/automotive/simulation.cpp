// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/simulation.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "format/opendrive.hpp"
#include "model/random.hpp"

namespace simon::automotive {

namespace {

auto count(int value) -> std::size_t {
  return static_cast<std::size_t>(std::max(value, 0));
}

// Every driving lane of `network`, and its length.
// Where on a driving lane vehicles may start: from `from` to `to` along it.
struct Room final {
  LaneKey lane;
  double from = 0.0;
  double to = 0.0;
};

// Every driving lane of `network` outside its junctions, and where on it
// vehicles may start: all of it, but `margin` from an end at a junction, so
// none starts in a junction or beside one about to leave it.
auto driving_lanes(const Network& network, double margin) -> std::vector<Room> {
  std::vector<Room> lanes;
  for (std::uint32_t r = 0; r < network.roads.roads.size(); ++r) {
    const model::Road& road = network.roads.roads[r];
    if (road.junction != "-1") {
      continue;
    }
    bool before = road.predecessor.kind == model::RoadLink::Kind::JUNCTION;
    bool after = road.successor.kind == model::RoadLink::Kind::JUNCTION;
    for (std::uint32_t k = 0; k < road.lane_sections.size(); ++k) {
      const model::LaneSection& section = road.lane_sections[k];
      // Whether this section's start, in s, is at the junction before the
      // road, and its end at the one after it.
      bool start = before && k == 0;
      bool end = after && k + 1 == road.lane_sections.size();
      for (const std::vector<model::Lane>* side :
           {&section.left, &section.right}) {
        for (const model::Lane& lane : *side) {
          LaneKey key{.road = r, .section = k, .lane = lane.id};
          if (lane.type != "driving") {
            continue;
          }
          bool with_s = model::runs_with_s(key);
          double length = lane_length(network, key);
          double from = (with_s ? start : end) ? margin : 0.0;
          double to = length - ((with_s ? end : start) ? margin : 0.0);
          if (to > from) {
            lanes.push_back(Room{.lane = key, .from = from, .to = to});
          }
        }
      }
    }
  }
  return lanes;
}

}  // namespace

auto load_network(const std::string& path)
    -> std::expected<Network, framework::Status> {
  RETURN_OR_ASSIGN(model::RoadNetwork roads, format::load_opendrive(path));
  model::LaneGraph graph = model::build_lane_graph(roads);
  model::TrafficControl control = model::build_traffic_control(roads);
  model::RightOfWay rights = model::build_right_of_way(roads, graph, control);
  return Network{.roads = std::move(roads),
                 .graph = std::move(graph),
                 .control = std::move(control),
                 .rights = std::move(rights)};
}

auto is_tactical(const Network& network) -> bool {
  return !network.control.stop_lines().empty() ||
         !network.rights.conflicts().empty();
}

auto build_world(const Scenario& scenario, const Network& network,
                 Out<World> world) -> std::expected<void, framework::Status> {
  std::size_t vehicles = count(scenario.vehicles);
  bool tactical = is_tactical(network);
  return World::set_up()
      .numbered(1)
      .holding<archetype::Vehicle>(tactical ? 0 : vehicles)
      .holding<archetype::TacticalVehicle>(tactical ? vehicles : 0)
      .holding<archetype::SignalController>(network.control.groups().size())
      .build(world);
}

auto plan_signals(const Scenario& scenario, const Network& network)
    -> std::vector<model::SignalPlan> {
  std::span<const model::SignalGroup> groups = network.control.groups();
  std::vector<model::SignalPlan> plans(groups.size());
  // Each junction's groups in its order; a group in none, alone.
  std::map<std::string, std::vector<std::uint32_t>> turns;
  for (std::uint32_t g = 0; g < groups.size(); ++g) {
    std::string junction = groups[g].junction.empty()
                               ? "group " + std::to_string(g)
                               : groups[g].junction;
    turns[junction].push_back(g);
  }
  for (auto& [junction, members] : turns) {
    std::ranges::stable_sort(
        members, {}, [&](std::uint32_t g) { return groups[g].sequence; });
    std::vector<model::SignalPlan> in_turn = model::plan_in_turn(
        members.size(), scenario.green, scenario.yellow, scenario.all_red);
    for (std::size_t k = 0; k < members.size(); ++k) {
      plans[members[k]] = std::move(in_turn[k]);
    }
  }
  return plans;
}

auto build_scenario(const Scenario& scenario, const Network& network,
                    InOut<World> world)
    -> std::expected<void, framework::Status> {
  double spacing = (scenario.length + scenario.following.minimum_gap)
                       .numerical_value_in(model::meter) +
                   5.0;
  std::vector<Room> lanes = driving_lanes(network, spacing);
  double total = 0.0;
  for (const Room& room : lanes) {
    total += room.to - room.from;
  }
  CHECK_PRECONDITION(total > 0.0);

  model::Random random{scenario.seed};
  std::map<LaneKey, std::vector<double>> taken;
  auto transaction = world->transaction();
  for (std::size_t i = 0; i < count(scenario.vehicles); ++i) {
    // A place on the lanes, by length, not too near another vehicle.
    LaneKey lane;
    double along = 0.0;
    for (int attempt = 0;; ++attempt) {
      CHECK_PRECONDITION(attempt < 10000);
      double at = random.uniform(0.0, total);
      for (const Room& room : lanes) {
        if (at < room.to - room.from) {
          lane = room.lane;
          along = room.from + at;
          break;
        }
        at -= room.to - room.from;
      }
      const std::vector<double>& others = taken[lane];
      if (std::ranges::none_of(others, [&](double other) {
            return std::abs(other - along) < spacing;
          })) {
        break;
      }
    }
    taken[lane].push_back(along);

    model::IntelligentDriver following = scenario.following;
    following.desired_speed *=
        1.0 + random.uniform(-scenario.speed_spread, scenario.speed_spread);
    LaneState state{.lane = lane,
                    .s = s_along(network, lane, along),
                    .speed = scenario.starting_speed};
    auto seed = static_cast<std::uint64_t>(random.uniform(0.0, 0x1.0p53));
    VehiclePose pose = FollowLane::locate_vehicle(network, state);
    Driver driver{.following = following,
                  .changing = scenario.changing,
                  .length = scenario.length,
                  .seed = seed};
    // Only a network with lights or junctions gives its vehicles their
    // tactical state.
    if (!is_tactical(network)) {
      RETURN_IF_UNEXPECTED(world->create<archetype::Vehicle>()
                               .with(pose)
                               .with(state)
                               .with(driver)
                               .with(DriveCommand{})
                               .build());
    } else {
      RETURN_IF_UNEXPECTED(
          world->create<archetype::TacticalVehicle>()
              .with(pose)
              .with(state)
              .with(driver)
              .with(DriveCommand{})
              .with(Tactical{.braking = scenario.braking,
                             .critical_gap = scenario.critical_gap})
              .with(Stopped{})
              .build());
    }
  }
  std::vector<model::SignalPlan> plans = plan_signals(scenario, network);
  for (std::uint32_t g = 0; g < plans.size(); ++g) {
    RETURN_IF_UNEXPECTED(world->create<archetype::SignalController>()
                             .with(std::move(plans[g]))
                             .with(SignalState{.group = g})
                             .build());
  }
  transaction.commit();
  return {};
}

Simulation::Simulation(Scenario scenario) : scenario_{std::move(scenario)} {}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_OR_ASSIGN(Network network, load_network(scenario_.roads));
  network_ = std::make_unique<Network>(std::move(network));
  scheduler_ = std::make_unique<Scheduler>(
      Schedule{RunSignals{}, Decide{*network_}, Drive{*network_},
               FollowLane{*network_}});
  RETURN_IF_UNEXPECTED(build_world(scenario_, *network_, Out(world_)));
  RETURN_IF_UNEXPECTED(build_scenario(scenario_, *network_, InOut(world_)));
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_->step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

}  // namespace simon::automotive
