// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/flight/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
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

auto trim_in_cruise(const model::AircraftData& data, const model::Earth& earth,
                    Length altitude, Speed speed)
    -> std::expected<model::Trim, framework::Status> {
  model::FlightCondition condition{
      .position =
          model::meters(0.0, 0.0, altitude.numerical_value_in(model::meter)),
      .speed = speed,
      .tanks = model::fill_fuel_tanks(data),
  };
  return model::trim(data, condition, earth, model::StandardAirTable{});
}

auto create_rigid_aircraft(const model::AircraftData& data,
                           const model::Earth& earth, const model::Trim& trim,
                           Length x, Length y, Angle heading,
                           const Route& route, InOut<World> world)
    -> std::expected<Entity, framework::Status> {
  model::Time start = 0.0 * model::second;
  Length altitude = earth.altitude(trim.body, start);
  Speed speed{magnitude(earth.air_velocity(trim.body).numerical_value_in(
                  model::meter_per_second)) *
              model::meter_per_second};
  auto body_at = [&](const model::AngularVelocity& rate) {
    return earth.body_at(
        model::meters(x.numerical_value_in(model::meter),
                      y.numerical_value_in(model::meter),
                      altitude.numerical_value_in(model::meter)),
        trim.bank, trim.pitch, heading, earth.air_velocity(trim.body), rate,
        start);
  };
  // Level round the Earth on the new heading.
  RigidBody body = body_at(model::QuantityVector{} * model::radian_per_second);
  body = body_at(earth.level_rate(body, start));

  return world->create<archetype::RigidAircraft>()
      .with(earth.air_state(body, start))
      .with(body)
      .with(RigidBodyRate{})
      .with(trim.felt)
      .with(trim.signals)
      .with(trim.engines)
      .with(model::fill_fuel_tanks(data))
      .with(trim.mass)
      .with(AircraftType{.data = &data})
      .with(Autopilot{.altitude = altitude, .heading = heading, .speed = speed})
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

  // One trim serves every rigid aircraft: over a flat Earth it holds
  // anywhere, on any heading.
  std::optional<model::Trim> cruise;
  if (rigid > 0) {
    RETURN_OR_ASSIGN(cruise, trim_in_cruise(*rigid_type, earth));
  }

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
      RETURN_IF_UNEXPECTED(
          create_rigid_aircraft(*rigid_type, earth, *cruise, x * model::meter,
                                y * model::meter, state.heading, route, world));
    }
  }
  transaction.commit();
  return {};
}

auto Simulation::configure() -> engine::PhaseResult {
  if (scenario_.rigid > 0) {
    RETURN_OR_ASSIGN(model::AircraftData loaded,
                     model::load_aircraft(scenario_.rigid_aircraft));
    rigid_ = std::make_unique<model::AircraftData>(std::move(loaded));
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
