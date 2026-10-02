// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "application/flight/components.hpp"
#include "application/flight/systems.hpp"
#include "base/core.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"

namespace simon::flight {

// Everything a run depends on. The same scenario gives the same run.
//
// Aircraft start spread over a square that grows with their number, so the
// density of traffic stays the same at any size. Each flies a closed route of
// waypoints within `route_reach` of where it starts.
struct Scenario final {
  std::uint64_t seed = 1;

  int aircraft = 100;
  // How many of them opt in to Runge-Kutta 4, and how many fly as rigid
  // bodies. The rest fly the single-pass model.
  int precise = 0;
  int rigid = 0;
  // What the rigid aircraft are, as tools/jsbsim/convert.py writes them.
  std::string rigid_aircraft = "application/flight/aircraft/737.aircraft";

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

// A trim for the rigid aircraft: steady, level flight at `altitude` and
// `speed`, held by `alpha`, `pitch_trim` and `throttle`. The default is
// JSBSim's for its 737 (see reference/jsbsim_737_check_initial.csv).
struct RigidTrim final {
  Length altitude = 6000.0 * model::meter;
  Speed speed = 200.0 * model::meter_per_second;
  Angle alpha = 0.031689661 * model::radian;
  double pitch_trim = -0.15092104889583785;
  double throttle = 0.68974850653740216;
};

// Creates a rigid aircraft of type `data` over `earth`, trimmed by `trim`,
// at `x` and `y` in the world's local frame and heading `heading`, flying
// `route` from there.
auto create_rigid_aircraft(const model::AircraftData& data,
                           const model::Earth& earth, const RigidTrim& trim,
                           Length x, Length y, Angle heading,
                           const Route& route, InOut<World> world)
    -> std::expected<Entity, framework::Status>;

// Builds in `world` the world a scenario needs.
auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status>;

// Creates every aircraft of a scenario, with its route, flying level toward
// its first waypoint at its route's speed; rigid aircraft, of type `rigid`,
// start at their trim's altitude and speed.
auto build_scenario(const Scenario& scenario, const model::AircraftData* rigid,
                    InOut<World> world)
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

  // Waypoints reached so far, by every aircraft, and by the rigid ones.
  auto waypoints_reached() const -> std::uint64_t;
  auto rigid_waypoints_reached() const -> std::uint64_t;

 private:
  Scenario scenario_;
  std::unique_ptr<model::AircraftData> rigid_;  // Loaded if there are any.
  World world_;  // Empty until configure builds it.
  Scheduler scheduler_;
};

}  // namespace simon::flight
