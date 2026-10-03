// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "application/automotive/simulation_components.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "base/core.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"
#include "model/traffic.hpp"

namespace simon::automotive {

// Everything a run depends on. The same scenario gives the same run.
//
// Vehicles start along the network's driving lanes, at random places at
// least their length, their minimum gap and 5 m apart, and at
// `starting_speed`. Each driver's desired speed is the scenario's, varied by
// up to `speed_spread` either way.
struct Scenario final {
  std::uint64_t seed = 1;
  std::string roads = "application/automotive/roads/ring.xodr";
  int vehicles = 40;
  Speed starting_speed = 10.0 * model::meter_per_second;
  model::IntelligentDriver following{.desired_speed =
                                         20.0 * model::meter_per_second};
  model::LaneChanger changing;
  Length length = 4.5 * model::meter;
  double speed_spread = 0.1;
};

// Reads the network a scenario drives on and links its lanes.
auto load_network(const std::string& path)
    -> std::expected<Network, framework::Status>;

// Builds in `world` the world a scenario needs.
auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status>;

// Creates every vehicle of a scenario on `network`.
auto build_scenario(const Scenario& scenario, const Network& network,
                    InOut<World> world)
    -> std::expected<void, framework::Status>;

// The automotive simulation: reads the network and builds the scenario when
// configured, and drives until the driver stops. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {});

  // Reads the network and builds the world and the scenario in it.
  auto configure() -> engine::PhaseResult;

  auto step(const framework::Step& step) -> engine::PhaseResult;

  // The world: empty until configured.
  auto world() const -> const World& { return world_; }

  // The network: empty until configured.
  auto network() const -> const Network& { return *network_; }

 private:
  Scenario scenario_;
  std::unique_ptr<Network> network_;  // Shared by the systems; never moves.
  World world_;                       // Empty until configure builds it.
  std::unique_ptr<Scheduler> scheduler_;
};

}  // namespace simon::automotive
