// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "application/aeronautic/components.hpp"
#include "application/aeronautic/systems.hpp"
#include "base/core.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"
#include "model/trim.hpp"

namespace simon::aeronautic {

// Everything a run depends on. The same scenario gives the same run.
//
// Aircraft start spread over a square that grows with their number, so the
// density of traffic stays the same at any size. Each flies a closed route of
// waypoints within `route_reach` of where it starts.
struct Scenario final {
  std::uint64_t seed = 1;

  int aircraft = 100;
  // The number of them on Runge-Kutta 4, and the numbers flying as rigid
  // bodies: airliners and fighters. The rest fly the single-pass model.
  int precise = 0;
  int rigid = 0;
  int fighters = 0;
  // The rigid aircraft's data, as tools/jsbsim/convert.py writes it, and the
  // gains their autopilots fly by.
  std::string rigid_aircraft = "application/aeronautic/aircraft/737.aircraft";
  std::string fighter_aircraft = "application/aeronautic/aircraft/f16.aircraft";
  SurfaceGains airliner_gains;
  // A fighter's fly-by-wire turns a stick into rates and load, which the
  // airliner's gains fly as well; it banks to 60 degrees, so it turns on
  // about 2 km at 200 m/s.
  SurfaceGains fighter_gains{.max_bank = 1.05 * model::radian};

  // The air every aircraft flies in, still by default. Point-mass aircraft
  // drift with its steady wind; rigid aircraft fly through it, and through
  // its turbulence, each meeting its own gusts.
  model::WindField wind;

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

// Trims a rigid aircraft of type `data` over `earth` for level flight at
// `altitude` and `speed`, its tanks full, heading north from the world's
// origin.
auto trim_in_cruise(const model::AircraftData& data, const model::Earth& earth,
                    Length altitude = 6000.0 * model::meter,
                    Speed speed = 200.0 * model::meter_per_second)
    -> std::expected<model::Trim, framework::Status>;

// Creates a rigid aircraft of type `data` over `earth`, trimmed by `trim`,
// at `x` and `y` in the world's local frame, at the trim's altitude, and
// heading `heading`, flying `route` from there by `gains`. Over a flat Earth
// a trim holds wherever the aircraft is and whichever way it heads, and in a
// steady wind, whose air it moves with. In a `wind` that is not still the
// aircraft has a Wind, and in turbulence Gusts drawn from `seed`.
auto create_rigid_aircraft(const model::AircraftData& data,
                           const model::Earth& earth, const model::Trim& trim,
                           const SurfaceGains& gains, Length x, Length y,
                           Angle heading, const Route& route,
                           InOut<World> world,
                           const model::WindField& wind = {},
                           std::uint64_t seed = 0)
    -> std::expected<Entity, framework::Status>;

// The rigid aircraft types a scenario flies, read once: null where it flies
// none of that type.
struct RigidTypes final {
  const model::AircraftData* airliner = nullptr;
  const model::AircraftData* fighter = nullptr;
};

// Builds in `world` the world a scenario needs.
auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status>;

// Creates every aircraft of a scenario, with its route, flying level toward
// its first waypoint at its route's speed; rigid aircraft, of `types`, start
// at their trim's altitude and speed.
auto build_scenario(const Scenario& scenario, const RigidTypes& types,
                    InOut<World> world)
    -> std::expected<void, framework::Status>;

// The flight simulation: builds the scenario when configured, and flies until
// the driver stops. Any driver can run it.
class Simulation final {
 public:
  explicit Simulation(Scenario scenario = {});

  // Builds the world and the scenario in it. A scenario too big for a world
  // fails this phase with the builder's Status.
  auto configure() -> engine::PhaseResult;

  auto step(const framework::Step& step) -> engine::PhaseResult;

  // The world: empty until configured.
  auto world() const -> const World& { return world_; }

  // The rigid aircraft types it flies: null until configured, and where it
  // flies none of a type.
  auto rigid_types() const -> RigidTypes {
    return RigidTypes{.airliner = airliner_.get(), .fighter = fighter_.get()};
  }

  // Waypoints reached so far, by every aircraft, and by the rigid ones.
  auto waypoints_reached() const -> std::uint64_t;
  auto rigid_waypoints_reached() const -> std::uint64_t;

 private:
  Scenario scenario_;
  // Each rigid type, read if the scenario flies any.
  std::unique_ptr<model::AircraftData> airliner_;
  std::unique_ptr<model::AircraftData> fighter_;
  World world_;  // Empty until configure builds it.
  Scheduler scheduler_;
};

}  // namespace simon::aeronautic
