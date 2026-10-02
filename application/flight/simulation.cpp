// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "application/flight/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "framework/vocabulary.hpp"
#include "model/random.hpp"

namespace simon::flight {

namespace {

auto count(int value) -> std::size_t {
  return static_cast<std::size_t>(std::max(value, 0));
}

// The aircraft that fly the single-pass model, those that opt in to
// Runge-Kutta 4, and those that fly as rigid bodies.
struct Split final {
  std::size_t simple = 0;
  std::size_t precise = 0;
  std::size_t rigid = 0;
};

auto split(const Scenario& scenario) -> Split {
  std::size_t all = count(scenario.aircraft);
  std::size_t rigid = std::min(count(scenario.rigid), all);
  std::size_t precise = std::min(count(scenario.precise), all - rigid);
  return {.simple = all - precise - rigid, .precise = precise, .rigid = rigid};
}

template <typename ArchetypeType>
auto create_aircraft(const Scenario& scenario, const AirState& state,
                     const Route& route, InOut<World> world)
    -> std::expected<void, framework::Status> {
  // Roughly trimmed for level flight, so the autopilot starts near its work.
  constexpr double TRIM_THROTTLE = 0.3;
  auto created =
      world->create<ArchetypeType>()
          .with(state)
          .with(FlightControls{.load_factor = 1.0, .throttle = TRIM_THROTTLE})
          .with(Commands{.load_factor = 1.0, .throttle = TRIM_THROTTLE})
          .with(scenario.airframe)
          .with(scenario.handling)
          .with(Autopilot{.altitude = model::altitude_of(state),
                          .heading = state.heading,
                          .speed = route.speed,
                          .throttle_integral = TRIM_THROTTLE})
          .with(route);
  if constexpr (std::is_same_v<ArchetypeType, archetype::PreciseAircraft>) {
    RETURN_IF_UNEXPECTED(std::move(created).with(AirStateRate{}).build());
  } else {
    RETURN_IF_UNEXPECTED(std::move(created).build());
  }
  return {};
}

}  // namespace

auto create_rigid_aircraft(const model::AircraftData& data,
                           const model::Earth& earth, const RigidTrim& trim,
                           Length x, Length y, Angle heading,
                           const Route& route, InOut<World> world)
    -> std::expected<Entity, framework::Status> {
  double speed = trim.speed.numerical_value_in(model::meter_per_second);
  RigidBody body = earth.body_at(
      model::meters(x.numerical_value_in(model::meter),
                    y.numerical_value_in(model::meter),
                    trim.altitude.numerical_value_in(model::meter)),
      0.0 * model::radian, trim.alpha, heading,
      model::meters_per_second(speed * model::cos(trim.alpha), 0.0,
                               speed * model::sin(trim.alpha)),
      model::QuantityVector{} * model::radian_per_second, 0.0 * model::second);

  FlightSignals signals;
  signals[model::FlightSignal::PITCH_TRIM_COMMAND] = trim.pitch_trim;
  model::settle_flight_controls(data.flight_controls, signals);
  model::run_flight_controls(data.flight_controls, signals,
                             0.0 * model::second);
  EngineControls controls;
  controls.throttle.fill(trim.throttle);
  model::StandardAirTable air;
  Engines engines = model::settled_engines(
      data, controls,
      model::compute_engine_air(body, earth, air, 0.0 * model::second));
  FuelTanks tanks = model::fill_fuel_tanks(data);

  return world->create<archetype::RigidAircraft>()
      .with(earth.air_state(body, 0.0 * model::second))
      .with(body)
      .with(RigidBodyRate{})
      .with(signals)
      .with(ControlSurfaces{})
      .with(controls)
      .with(engines)
      .with(tanks)
      .with(model::compute_mass_balance(data, tanks))
      .with(AircraftType{.data = &data})
      .with(Autopilot{
          .altitude = trim.altitude, .heading = heading, .speed = trim.speed})
      .with(route)
      .with(SurfaceAutopilot{.pitch_trim = trim.pitch_trim,
                             .throttle_trim = trim.throttle})
      .build();
}

auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status> {
  auto [simple, precise, rigid] = split(scenario);
  return World::set_up()
      .numbered(1)
      .holding<archetype::Aircraft>(simple)
      .holding<archetype::PreciseAircraft>(precise)
      .holding<archetype::RigidAircraft>(rigid)
      .build(world);
}

auto build_scenario(const Scenario& scenario,
                    const model::AircraftData* rigid_type, InOut<World> world)
    -> std::expected<void, framework::Status> {
  auto [simple, precise, rigid] = split(scenario);
  CHECK_PRECONDITION(rigid == 0 || rigid_type);
  model::Earth earth = model::Earth::flat();
  model::Random random{scenario.seed};
  double side = scenario.spacing.numerical_value_in(model::meter) *
                std::sqrt(static_cast<double>(simple + precise));
  double reach = scenario.route_reach.numerical_value_in(model::meter);
  double lowest = scenario.lowest.numerical_value_in(model::meter);
  double highest = scenario.highest.numerical_value_in(model::meter);

  auto transaction = world->transaction();
  for (std::size_t i = 0; i < simple + precise + rigid; ++i) {
    double x = random.uniform(0.0, side);
    double y = random.uniform(0.0, side);
    Route route{
        .speed =
            random.uniform(
                scenario.slowest.numerical_value_in(model::meter_per_second),
                scenario.fastest.numerical_value_in(model::meter_per_second)) *
            model::meter_per_second};
    for (Position& waypoint : route.waypoints) {
      waypoint = model::meters(x + random.uniform(-reach, reach),
                               y + random.uniform(-reach, reach),
                               random.uniform(lowest, highest));
    }
    Position start = model::meters(x, y, random.uniform(lowest, highest));
    AirState state{.position = start,
                   .speed = route.speed,
                   .heading = model::bearing(start, route.waypoints[0])};
    if (i < simple) {
      RETURN_IF_UNEXPECTED(
          create_aircraft<archetype::Aircraft>(scenario, state, route, world));
    } else if (i < simple + precise) {
      RETURN_IF_UNEXPECTED(create_aircraft<archetype::PreciseAircraft>(
          scenario, state, route, world));
    } else {
      RETURN_IF_UNEXPECTED(create_rigid_aircraft(
          *rigid_type, earth, RigidTrim{}, x * model::meter, y * model::meter,
          state.heading, route, world));
    }
  }
  transaction.commit();
  return {};
}

auto Simulation::configure() -> engine::PhaseResult {
  if (scenario_.rigid > 0) {
    auto loaded = model::load_aircraft(scenario_.rigid_aircraft);
    if (!loaded) {
      return std::unexpected(loaded.error());
    }
    rigid_ = std::make_unique<model::AircraftData>(std::move(*loaded));
  }
  RETURN_IF_UNEXPECTED(build_world(scenario_, Out(world_)));
  RETURN_IF_UNEXPECTED(build_scenario(scenario_, rigid_.get(), InOut(world_)));
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_.step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

auto Simulation::waypoints_reached() const -> std::uint64_t {
  std::uint64_t reached = 0;
  world_.store_of<Route>().for_each(
      [&](Entity, const Route& route) { reached += route.reached; });
  return reached;
}

auto Simulation::rigid_waypoints_reached() const -> std::uint64_t {
  std::uint64_t reached = 0;
  const auto& bodies = world_.store_of<RigidBody>();
  world_.store_of<Route>().for_each([&](Entity entity, const Route& route) {
    if (bodies.maybe_component_of(entity)) {
      reached += route.reached;
    }
  });
  return reached;
}

}  // namespace simon::flight
