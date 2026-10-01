// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "application/flight/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "model/random.hpp"

namespace simon::flight {

namespace {

auto count(int value) -> std::size_t {
  return static_cast<std::size_t>(std::max(value, 0));
}

// The aircraft that fly the single-pass model, and those that opt in to
// Runge-Kutta 4.
auto split(const Scenario& scenario) -> std::pair<std::size_t, std::size_t> {
  std::size_t all = count(scenario.aircraft);
  std::size_t precise = std::min(count(scenario.precise), all);
  return {all - precise, precise};
}

template <typename ArchetypeType>
auto create_aircraft(const Scenario& scenario, const AirState& state,
                     const Route& route, lib::InOut<World> world)
    -> std::expected<void, framework::Status> {
  // Roughly trimmed for level flight, so the autopilot starts near its work.
  constexpr double TRIM_THROTTLE = 0.3;
  auto created = world->create<ArchetypeType>()
                     .with(state)
                     .with(FlightControls{.load_factor = 1.0,
                                          .throttle = TRIM_THROTTLE})
                     .with(Commands{.load_factor = 1.0,
                                    .throttle = TRIM_THROTTLE})
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

auto build_world(const Scenario& scenario, lib::Out<World> world)
    -> std::expected<void, framework::Status> {
  auto [simple, precise] = split(scenario);
  return World::set_up()
      .numbered(1)
      .holding<archetype::Aircraft>(simple)
      .holding<archetype::PreciseAircraft>(precise)
      .build(world);
}

auto build_scenario(const Scenario& scenario, lib::InOut<World> world)
    -> std::expected<void, framework::Status> {
  auto [simple, precise] = split(scenario);
  model::Random random{scenario.seed};
  double side = scenario.spacing.numerical_value_in(model::meter) *
                std::sqrt(static_cast<double>(simple + precise));
  double reach = scenario.route_reach.numerical_value_in(model::meter);
  double lowest = scenario.lowest.numerical_value_in(model::meter);
  double highest = scenario.highest.numerical_value_in(model::meter);

  auto transaction = world->transaction();
  for (std::size_t i = 0; i < simple + precise; ++i) {
    double x = random.uniform(0.0, side);
    double y = random.uniform(0.0, side);
    Route route{.speed = random.uniform(
                             scenario.slowest.numerical_value_in(
                                 model::meter_per_second),
                             scenario.fastest.numerical_value_in(
                                 model::meter_per_second)) *
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
      RETURN_IF_UNEXPECTED(create_aircraft<archetype::Aircraft>(
          scenario, state, route, world));
    } else {
      RETURN_IF_UNEXPECTED(create_aircraft<archetype::PreciseAircraft>(
          scenario, state, route, world));
    }
  }
  transaction.commit();
  return {};
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(build_world(scenario_, lib::Out(world_)));
  RETURN_IF_UNEXPECTED(build_scenario(scenario_, lib::InOut(world_)));
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_.step(step, lib::InOut(world_));
  return engine::Flow::CONTINUE;
}

auto Simulation::waypoints_reached() const -> std::uint64_t {
  std::uint64_t reached = 0;
  world_.store_of<Route>().for_each(
      [&](Entity, const Route& route) { reached += route.reached; });
  return reached;
}

}  // namespace simon::flight
