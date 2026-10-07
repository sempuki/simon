// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/simulation.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "core/random.hpp"
#include "format/opendrive.hpp"

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
    const road::Road& road = network.roads.roads[r];
    if (road.junction != "-1") {
      continue;
    }
    bool before = road.predecessor.kind == road::RoadLink::Kind::JUNCTION;
    bool after = road.successor.kind == road::RoadLink::Kind::JUNCTION;
    for (std::uint32_t k = 0; k < road.lane_sections.size(); ++k) {
      const road::LaneSection& section = road.lane_sections[k];
      // Whether this section's start, in s, is at the junction before the
      // road, and its end at the one after it.
      bool start = before && k == 0;
      bool end = after && k + 1 == road.lane_sections.size();
      for (const std::vector<road::Lane>* side :
           {&section.left, &section.right}) {
        for (const road::Lane& lane : *side) {
          LaneKey key{.road = r, .section = k, .lane = lane.id};
          if (lane.type != "driving") {
            continue;
          }
          bool with_s = road::runs_with_s(key);
          double length = find_lane_length(network, key);
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
  RETURN_OR_ASSIGN(road::RoadNetwork roads, format::load_opendrive(path));
  road::LaneGraph graph = road::build_lane_graph(roads);
  road::LaneGraph driving = road::build_lane_graph(roads, "driving");
  traffic::TrafficControl control = traffic::build_traffic_control(roads);
  traffic::RightOfWay rights =
      traffic::build_right_of_way(roads, graph, control);
  road::WalkingGraph walking = road::build_walking_graph(roads);
  std::vector<std::uint32_t> components = walking.find_components();
  // A crosswalk's light: one on a lane it crosses, within 15 m before it.
  std::vector<std::optional<std::uint32_t>> groups(walking.crosswalks().size());
  for (const road::CrosswalkZone& zone : walking.zones()) {
    for (const traffic::StopLine& line : control.stop_lines_on(zone.lane)) {
      if (line.along <= zone.near && line.along >= zone.near - 15.0) {
        groups[zone.crosswalk] = line.group;
      }
    }
  }
  return Network{.roads = std::move(roads),
                 .graph = std::move(graph),
                 .driving = std::move(driving),
                 .control = std::move(control),
                 .rights = std::move(rights),
                 .walking = std::move(walking),
                 .walking_components = std::move(components),
                 .crosswalk_groups = std::move(groups)};
}

auto is_tactical(const Network& network) -> bool {
  return !network.control.stop_lines().empty() ||
         !network.rights.conflicts().empty() ||
         !network.walking.zones().empty();
}

auto build_world(const Scenario& scenario, const Network& network,
                 Out<World> world) -> std::expected<void, framework::Status> {
  std::size_t vehicles = count(scenario.vehicles);
  bool tactical = is_tactical(network);
  return World::set_up()
      .numbered(1)
      .holding<archetype::Vehicle>(tactical ? 0 : vehicles)
      .holding<archetype::TacticalVehicle>(tactical ? vehicles : 0)
      .holding<archetype::Pedestrian>(count(scenario.pedestrians))
      .holding<archetype::SignalController>(network.control.groups().size())
      .build(world);
}

auto plan_signals(const Scenario& scenario, const Network& network)
    -> std::vector<traffic::SignalPlan> {
  std::span<const traffic::SignalGroup> groups = network.control.groups();
  std::vector<traffic::SignalPlan> plans(groups.size());
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
    std::vector<traffic::SignalPlan> in_turn = traffic::plan_in_turn(
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
                       .numerical_value_in(meter) +
                   5.0;
  std::vector<Room> lanes = driving_lanes(network, spacing);
  double total = 0.0;
  for (const Room& room : lanes) {
    total += room.to - room.from;
  }
  CHECK_PRECONDITION(total > 0.0);

  Random random{scenario.seed};
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

    traffic::IntelligentDriver following = scenario.following;
    following.desired_speed *=
        1.0 + random.uniform(-scenario.speed_spread, scenario.speed_spread);
    LaneState state{.lane = lane,
                    .s = find_s_along(network, lane, along),
                    .speed = scenario.starting_speed};
    auto seed = static_cast<std::uint64_t>(random.uniform(0.0, 0x1.0p53));
    RoadPose pose = FollowLane::locate_vehicle(network, state);
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
  // Pedestrians, at random along the sidewalks, by length, each setting out
  // for a place its seed picks.
  std::vector<std::uint32_t> sidewalks;
  double walkable = 0.0;
  for (std::uint32_t e = 0; e < network.walking.edges().size(); ++e) {
    if (network.walking.edges()[e].kind == road::WalkEdge::Kind::SIDEWALK) {
      sidewalks.push_back(e);
      walkable += network.walking.edges()[e].length();
    }
  }
  std::map<std::uint32_t, std::vector<double>> walking;  // Edge, along.
  for (std::size_t i = 0; i < count(scenario.pedestrians); ++i) {
    CHECK_PRECONDITION(walkable > 0.0);
    double at = 0.0;
    std::uint32_t edge = sidewalks.back();
    for (int attempt = 0;; ++attempt) {
      CHECK_PRECONDITION(attempt < 10000);
      at = random.uniform(0.0, walkable);
      for (std::uint32_t e : sidewalks) {
        double length = network.walking.edges()[e].length();
        if (at < length) {
          edge = e;
          break;
        }
        at -= length;
      }
      if (std::ranges::none_of(walking[edge], [&](double other) {
            return std::abs(other - at) < 1.0;
          })) {
        break;
      }
    }
    walking[edge].push_back(at);
    auto seed = static_cast<std::uint64_t>(random.uniform(0.0, 0x1.0p53));
    WalkRoute route{.legs = {road::Leg{.edge = edge, .forward = true}}};
    std::vector<road::Leg> onward =
        plan_walk(network, network.walking.edges()[edge].to, seed, 0);
    route.legs.insert(route.legs.end(), onward.begin(), onward.end());
    WalkState state{.along = at * meter};
    double speed = std::clamp(
        random.normal(
            scenario.walking_speed.numerical_value_in(meter_per_second),
            scenario.walking_spread.numerical_value_in(meter_per_second)),
        0.5, 2.5);
    RETURN_IF_UNEXPECTED(
        world->create<archetype::Pedestrian>()
            .with(PlaceWalker::locate_walker(network, route, state))
            .with(state)
            .with(Walker{.desired_speed = speed * meter_per_second,
                         .start_up = scenario.start_up,
                         .seed = seed,
                         .complies = random.unit() < scenario.compliance})
            .with(std::move(route))
            .with(WalkCommand{})
            .build());
  }
  std::vector<traffic::SignalPlan> plans = plan_signals(scenario, network);
  for (std::uint32_t g = 0; g < plans.size(); ++g) {
    RETURN_IF_UNEXPECTED(world->create<archetype::SignalController>()
                             .with(std::move(plans[g]))
                             .with(SignalState{.group = g})
                             .build());
  }
  transaction.commit();
  return {};
}

auto keep_road_order(const Network& network, InOut<World> world) -> void {
  const road::LaneNumbering& numbering = network.graph.numbering();
  auto road_order = [&](Entity entity) {
    const LaneState& state = world->store_of<LaneState>().component_of(entity);
    return std::pair{numbering.number_of(state.lane),
                     state.s.numerical_value_in(meter)};
  };
  world->reorder<archetype::Vehicle>(road_order);
  world->reorder<archetype::TacticalVehicle>(road_order);
  world->reorder<archetype::Pedestrian>([&](Entity entity) {
    const WalkState& state = world->store_of<WalkState>().component_of(entity);
    const WalkRoute& route = world->store_of<WalkRoute>().component_of(entity);
    std::uint32_t edge = state.leg < route.legs.size()
                             ? route.legs[state.leg].edge
                             : road::WalkEdge::NONE;
    return std::pair{edge, state.along.numerical_value_in(meter)};
  });
}

Simulation::Simulation(Scenario scenario) : scenario_{std::move(scenario)} {}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_OR_ASSIGN(Network network, load_network(scenario_.roads));
  network.vehicles_yield = scenario_.vehicles_yield;
  network_ = std::make_unique<Network>(std::move(network));
  scheduler_ = std::make_unique<Scheduler>(make_schedule(*network_));
  RETURN_IF_UNEXPECTED(build_world(scenario_, *network_, Out(world_)));
  RETURN_IF_UNEXPECTED(build_scenario(scenario_, *network_, InOut(world_)));
  world_.sync();
  keep_road_order(*network_, InOut(world_));
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const Step& step) -> engine::PhaseResult {
  scheduler_->step(step, InOut(world_));
  if (++steps_ >= ROAD_ORDER_STEPS && world_.pending() == 0) {
    keep_road_order(*network_, InOut(world_));
    steps_ = 0;
  }
  return engine::Flow::CONTINUE;
}

}  // namespace simon::automotive
