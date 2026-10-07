// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <vector>

#include "application/automotive/simulation_components.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "engine/lifecycle.hpp"
#include "model/traffic/traffic.hpp"

namespace simon::automotive {

// Everything a run depends on. The same scenario gives the same run.
//
// Vehicles start along the network's driving lanes outside its junctions,
// at random places at least their length, their minimum gap and 5 m apart
// and as far from a junction, and at `starting_speed`. Each driver's desired
// speed is the scenario's, varied by up to `speed_spread` either way.
//
// Each junction's signal groups take turns in their order, each green for
// `green` and yellow for `yellow`, with every group red for `all_red`
// between turns; a group no junction lists takes turns alone. Where a
// driver gives way, it takes a gap of at least `critical_gap`.
//
// Pedestrians start at random along the sidewalks, by length, at least a
// meter apart, each walking at its own speed, and walk from place to place
// on the walking graph.
struct Scenario final {
  std::uint64_t seed = 1;
  std::string roads = "application/automotive/roads/ring.xodr";
  int vehicles = 40;
  Speed starting_speed = 10.0 * meter_per_second;
  traffic::IntelligentDriver following{.desired_speed =
                                           20.0 * meter_per_second};
  traffic::LaneChanger changing;
  traffic::LightBraking braking;
  Time critical_gap = 6.0 * second;
  Length length = 4.5 * meter;
  int pedestrians = 0;
  // Free walking speeds, normal, clipped to 0.5 to 2.5 m/s: Weidmann's.
  Speed walking_speed = 1.34 * meter_per_second;
  Speed walking_spread = 0.26 * meter_per_second;
  // The share of pedestrians that wait for a light, and the start-up time
  // that with the time to walk across makes the gap a pedestrian needs.
  double compliance = 1.0;
  Time start_up = 2.0 * second;
  bool vehicles_yield = true;  // At crosswalks without a light.
  double speed_spread = 0.1;
  Duration green = std::chrono::seconds{30};
  Duration yellow = std::chrono::seconds{3};
  Duration all_red = std::chrono::seconds{2};
};

// Reads the network a scenario drives on, links its lanes, and finds its
// stop lines and the conflicts in its junctions.
auto load_network(const std::string& path)
    -> std::expected<Network, framework::Status>;

// Whether vehicles on `network` need their tactical state: whether it has
// lights or conflicts in its junctions.
auto is_tactical(const Network& network) -> bool;

// Builds in `world` the world a scenario needs on `network`.
auto build_world(const Scenario& scenario, const Network& network,
                 Out<World> world) -> std::expected<void, framework::Status>;

// The fixed-time plan of each of `network`'s signal groups, by group.
auto plan_signals(const Scenario& scenario, const Network& network)
    -> std::vector<traffic::SignalPlan>;

// Creates every vehicle of a scenario on `network`, and a controller for each
// signal group.
auto build_scenario(const Scenario& scenario, const Network& network,
                    InOut<World> world)
    -> std::expected<void, framework::Status>;

// The automotive simulation: reads the network and builds the scenario when
// configured, and drives until the driver stops. Any driver can run it.
// Puts every vehicle in road order, by lane and then along it, and every
// pedestrian in order of the edge it walks and along it, in each store, so
// that neighbors on the road are neighbors in memory. Between steps.
auto keep_road_order(const Network& network, InOut<World> world) -> void;

// How many steps a simulation takes between putting its entities back in
// road order, as traffic mixes them.
inline constexpr int ROAD_ORDER_STEPS = 100;

class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {});

  // Reads the network and builds the world and the scenario in it.
  auto configure() -> engine::PhaseResult;

  auto step(const Step& step) -> engine::PhaseResult;

  // The world: empty until configured.
  auto world() const -> const World& { return world_; }

  // The network: empty until configured.
  auto network() const -> const Network& { return *network_; }

 private:
  Scenario scenario_;
  std::unique_ptr<Network> network_;  // Shared by the systems; never moves.
  World world_;                       // Empty until configure builds it.
  std::unique_ptr<Scheduler> scheduler_;
  int steps_ = 0;  // Since the last road order.
};

}  // namespace simon::automotive
