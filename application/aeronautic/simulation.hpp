// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <expected>
#include <memory>
#include <string>

#include "application/aeronautic/simulation_components.hpp"
#include "application/aeronautic/simulation_systems.hpp"
#include "base/core.hpp"
#include "core/argument.hpp"
#include "engine/lifecycle.hpp"
#include "framework/timeline.hpp"
#include "model/aircraft/trim.hpp"

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
  std::string rigid_aircraft = "3rd_party/jsbsim/737.aircraft";
  std::string fighter_aircraft = "3rd_party/jsbsim/f16.aircraft";
  SurfaceGains airliner_gains;
  // A fighter's fly-by-wire turns a stick into rates and load, which the
  // airliner's gains fly as well; it banks to 60 degrees, so it turns on
  // about 2 km at 200 m/s.
  SurfaceGains fighter_gains{.max_bank = 1.05 * radian};

  // The air every aircraft flies in, still by default. Point-mass aircraft
  // drift with its steady wind; rigid aircraft fly through it, and through
  // its turbulence, each meeting its own gusts.
  earth::WindField wind;

  Length spacing = 5000.0 * meter;  // Per aircraft, on average.
  Length route_reach = 20000.0 * meter;
  Length lowest = 3000.0 * meter;
  Length highest = 9000.0 * meter;
  Speed slowest = 180.0 * meter_per_second;
  Speed fastest = 240.0 * meter_per_second;

  // A generic twin-engine jet.
  Airframe airframe{.mass = 20000.0 * kilogram,
                    .wing_area = 50.0 * square_meter,
                    .zero_lift_drag = 0.02,
                    .induced_drag = 0.045,
                    .thrust = 100000.0 * newton};
  Handling handling{.max_load_factor = 3.0,
                    .min_load_factor = 0.0,
                    .max_bank = 1.0 * radian,
                    .roll_rate = 1.0 * radian_per_second,
                    .load_factor_lag = 0.5 * second,
                    .throttle_lag = 2.0 * second};
};

// Trims a rigid aircraft of type `data` over `earth` for level flight at
// `altitude` and `speed`, its tanks full, heading north from the world's
// origin.
auto trim_in_cruise(const aircraft::Definition& data,
                    const aircraft::Earth& earth,
                    Length altitude = 6000.0 * meter,
                    Speed speed = 200.0 * meter_per_second)
    -> std::expected<aircraft::Trim, framework::Status>;

// Creates a rigid aircraft of type `data` over `earth`, trimmed by `trim`,
// at `x` and `y` in the world's local frame, at the trim's altitude, and
// heading `heading`, flying `route` from there by `gains`. Over a flat Earth
// a trim holds wherever the aircraft is and whichever way it heads, and in a
// steady wind, whose air it moves with. In a `wind` that is not still the
// aircraft has a Wind, and in turbulence Gusts drawn from `seed`.
auto create_rigid_aircraft(
    const aircraft::Definition& data, const aircraft::Earth& earth,
    const aircraft::Trim& trim, const SurfaceGains& gains, Length x, Length y,
    Angle heading, const Route& route, InOut<World> world,
    const earth::WindField& wind = {}, std::uint64_t seed = 0)
    -> std::expected<Entity, framework::Status>;

// The rigid aircraft types a scenario flies, read once: null where it flies
// none of that type.
struct RigidTypes final {
  const aircraft::Definition* airliner = nullptr;
  const aircraft::Definition* fighter = nullptr;
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

  auto step(const Step& step) -> engine::PhaseResult;

  // When work is next due, for the driver to end its steps there.
  auto timeline() const -> const framework::Timeline& { return timeline_; }

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
  std::unique_ptr<aircraft::Definition> airliner_;
  std::unique_ptr<aircraft::Definition> fighter_;
  World world_;  // Empty until configure builds it.
  framework::Timeline timeline_;
  Scheduler scheduler_;
};

}  // namespace simon::aeronautic
