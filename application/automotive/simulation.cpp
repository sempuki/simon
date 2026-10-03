// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/simulation.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <utility>
#include <vector>

#include "model/opendrive.hpp"
#include "model/random.hpp"

namespace simon::automotive {

namespace {

auto count(int value) -> std::size_t {
  return static_cast<std::size_t>(std::max(value, 0));
}

// Every driving lane of `network`, and its length.
auto driving_lanes(const Network& network)
    -> std::vector<std::pair<LaneKey, double>> {
  std::vector<std::pair<LaneKey, double>> lanes;
  for (std::uint32_t r = 0; r < network.roads.roads.size(); ++r) {
    const model::Road& road = network.roads.roads[r];
    for (std::uint32_t k = 0; k < road.lane_sections.size(); ++k) {
      const model::LaneSection& section = road.lane_sections[k];
      for (const std::vector<model::Lane>* side :
           {&section.left, &section.right}) {
        for (const model::Lane& lane : *side) {
          LaneKey key{.road = r, .section = k, .lane = lane.id};
          if (lane.type == "driving") {
            lanes.emplace_back(key, lane_length(network, key));
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
  RETURN_OR_ASSIGN(model::RoadNetwork roads, model::load_opendrive(path));
  model::LaneGraph graph = model::build_lane_graph(roads);
  return Network{.roads = std::move(roads), .graph = std::move(graph)};
}

auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status> {
  return World::set_up()
      .numbered(1)
      .holding<archetype::Vehicle>(count(scenario.vehicles))
      .build(world);
}

auto build_scenario(const Scenario& scenario, const Network& network,
                    InOut<World> world)
    -> std::expected<void, framework::Status> {
  std::vector<std::pair<LaneKey, double>> lanes = driving_lanes(network);
  double total = 0.0;
  for (const auto& [lane, length] : lanes) {
    total += length;
  }
  CHECK_PRECONDITION(total > 0.0);
  double spacing = (scenario.length + scenario.following.minimum_gap)
                       .numerical_value_in(model::meter) +
                   5.0;

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
      for (const auto& [key, length] : lanes) {
        if (at < length) {
          lane = key;
          along = at;
          break;
        }
        at -= length;
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
    RETURN_IF_UNEXPECTED(world->create<archetype::Vehicle>()
                             .with(FollowLane::locate_vehicle(network, state))
                             .with(state)
                             .with(Driver{.following = following,
                                          .changing = scenario.changing,
                                          .length = scenario.length,
                                          .seed = seed})
                             .with(DriveCommand{})
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
      Schedule{Decide{*network_}, Drive{*network_}, FollowLane{*network_}});
  RETURN_IF_UNEXPECTED(build_world(scenario_, Out(world_)));
  RETURN_IF_UNEXPECTED(build_scenario(scenario_, *network_, InOut(world_)));
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_->step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

}  // namespace simon::automotive
