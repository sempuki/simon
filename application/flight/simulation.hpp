// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <expected>

#include "application/flight/components.hpp"
#include "application/flight/systems.hpp"
#include "base/core.hpp"
#include "engine/lifecycle.hpp"

namespace simon::flight {

// Everything a run depends on. The same scenario gives the same run.
//
// Aircraft start spread over a square that grows with their number, so the
// density of traffic stays the same at any size. Each flies a closed route of
// waypoints within `route_reach` of where it starts.
struct Scenario final {
  std::uint64_t seed = 1;

  int aircraft = 100;
  // How many of them opt in to Runge-Kutta 4. The rest fly the single-pass
  // model.
  int precise = 0;

  Length spacing = 5000.0 * model::meter;  // Per aircraft, on average.
  Length route_reach = 20000.0 * model::meter;
  Length lowest = 3000.0 * model::meter;
  Length highest = 9000.0 * model::meter;
  Speed slowest = 180.0 * model::meter_per_second;
  Speed fastest = 240.0 * model::meter_per_second;

  // A generic twin-engine jet.
  Airframe airframe{.mass = 20000.0 * model::kilogram,
                    .wing_area = 50.0 * model::square_meter,
                    .zero_lift_drag = 0.02,
                    .induced_drag = 0.045,
                    .thrust = 100000.0 * model::newton};
  Handling handling{.max_load_factor = 3.0,
                    .min_load_factor = 0.0,
                    .max_bank = 1.0 * model::radian,
                    .roll_rate = 1.0 * model::radian_per_second,
                    .load_factor_lag = 0.5 * model::second,
                    .throttle_lag = 2.0 * model::second};
};

// Builds in `world` the world a scenario needs.
auto build_world(const Scenario& scenario, lib::Out<World> world)
    -> std::expected<void, framework::Status>;

// Creates every aircraft of a scenario, with its route, flying level toward
// its first waypoint at its route's speed.
auto build_scenario(const Scenario& scenario, lib::InOut<World> world)
    -> std::expected<void, framework::Status>;

// The flight simulation: builds the scenario when configured, and flies until
// the driver stops. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {}) : scenario_{scenario} {}

  // Builds the world and the scenario in it. A scenario too big for a world
  // fails this phase with the builder's Status.
  auto configure() -> engine::PhaseResult;

  auto step(const framework::Step& step) -> engine::PhaseResult;

  // The world: empty until configured.
  auto world() const -> const World& { return world_; }

  // Waypoints reached so far, by every aircraft.
  auto waypoints_reached() const -> std::uint64_t;

 private:
  Scenario scenario_;
  World world_;  // Empty until configure builds it.
  Scheduler scheduler_;
};

}  // namespace simon::flight
