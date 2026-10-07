// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/aeronautic/simulation.hpp"
#include "format/aircraft_file.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>

#include "core/random.hpp"
#include "core/vocabulary.hpp"

namespace simon::aeronautic {

namespace {

auto count(int value) -> std::size_t {
  return static_cast<std::size_t>(std::max(value, 0));
}

// The aircraft that fly the single-pass model, those that opt in to
// Runge-Kutta 4, and those that fly as rigid bodies, airliners and fighters.
struct Fleet final {
  std::size_t simple = 0;
  std::size_t precise = 0;
  std::size_t airliners = 0;
  std::size_t fighters = 0;
};

auto split_fleet(const Scenario& scenario) -> Fleet {
  std::size_t all = count(scenario.aircraft);
  std::size_t airliners = std::min(count(scenario.rigid), all);
  std::size_t fighters = std::min(count(scenario.fighters), all - airliners);
  std::size_t rigid = airliners + fighters;
  std::size_t precise = std::min(count(scenario.precise), all - rigid);
  return {.simple = all - precise - rigid,
          .precise = precise,
          .airliners = airliners,
          .fighters = fighters};
}

// Builds `builder`'s entity with a Wind if `field` is not still.
template <typename BuilderType>
auto build_in(BuilderType builder, const model::WindField& field)
    -> std::expected<Entity, framework::Status> {
  if (model::is_still(field)) {
    return std::move(builder).build();
  }
  return std::move(builder).with(Wind{}).build();
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
    RETURN_IF_UNEXPECTED(
        build_in(std::move(created).with(AirStateRate{}), scenario.wind));
  } else {
    RETURN_IF_UNEXPECTED(build_in(std::move(created), scenario.wind));
  }
  return {};
}

// The seed of the `i`th aircraft's gusts: a stream of its own.
auto gust_seed(const Scenario& scenario, std::size_t i) -> std::uint64_t {
  return scenario.seed * 0xd1b54a32d192ed03ULL + i;
}

}  // namespace

Simulation::Simulation(Scenario scenario)
    : scenario_{std::move(scenario)},
      scheduler_{Schedule{
          MoveAir{scenario_.wind}, FollowRoute{}, FlyAutopilot{}, Actuate{},
          Fly{}, Precise{}, DriftWithWind{}, FlySurfaces{}, RunFlightControls{},
          RunEngines{}, Rigid{}, BurnFuel{}, FollowRigidBody{}}} {}

auto trim_in_cruise(const model::AircraftData& data, const model::Earth& earth,
                    Length altitude, Speed speed)
    -> std::expected<model::Trim, framework::Status> {
  model::FlightCondition condition{
      .position = meters(0.0, 0.0, altitude.numerical_value_in(meter)),
      .speed = speed,
      .tanks = model::fill_fuel_tanks(data),
  };
  return model::trim(data, condition, earth, model::StandardAirTable{});
}

auto create_rigid_aircraft(const model::AircraftData& data,
                           const model::Earth& earth, const model::Trim& trim,
                           const SurfaceGains& gains, Length x, Length y,
                           Angle heading, const Route& route,
                           InOut<World> world, const model::WindField& wind,
                           std::uint64_t seed)
    -> std::expected<Entity, framework::Status> {
  Time start = 0.0 * second;
  Length altitude = earth.altitude(trim.body, start);
  Speed speed{magnitude(earth.air_velocity(trim.body).numerical_value_in(
                  meter_per_second)) *
              meter_per_second};
  auto body_at = [&](const AngularVelocity& rate) {
    return earth.body_at(
        meters(x.numerical_value_in(meter), y.numerical_value_in(meter),
               altitude.numerical_value_in(meter)),
        trim.bank, trim.pitch, heading, earth.air_velocity(trim.body), rate,
        start);
  };
  // Level round the Earth on the new heading.
  RigidBody body = body_at(QuantityVector{} * radian_per_second);
  body = body_at(earth.level_rate(body, start));
  // Moving with the air.
  model::Wind steady = model::compute_wind(wind);
  body.velocity +=
      QuantityVector{
          earth.place(body, start).north_east_down *
          steady.north_east_down.numerical_value_in(meter_per_second).eigen()} *
      meter_per_second;

  auto create = [&]<typename ArchetypeType>() {
    return world->create<ArchetypeType>()
        .with(earth.air_state(body, start, steady))
        .with(body)
        .with(RigidBodyRate{})
        .with(trim.felt)
        .with(trim.signals)
        .with(trim.engines)
        .with(model::fill_fuel_tanks(data))
        .with(trim.mass)
        .with(AircraftType{.data = &data})
        .with(
            Autopilot{.altitude = altitude, .heading = heading, .speed = speed})
        .with(route)
        .with(SurfaceAutopilot{.gains = gains,
                               .pitch_trim = trim.pitch_trim,
                               .throttle_trim = trim.throttle});
  };
  if (model::is_still(wind)) {
    return create.template operator()<archetype::RigidAircraft>().build();
  }
  auto windy =
      create.template operator()<archetype::RigidAircraftInWind>().with(steady);
  if (wind.turbulence != model::Turbulence::NONE) {
    return std::move(windy).with(Gusts{.seed = seed}).build();
  }
  return std::move(windy).build();
}

auto build_world(const Scenario& scenario, Out<World> world)
    -> std::expected<void, framework::Status> {
  auto [simple, precise, airliners, fighters] = split_fleet(scenario);
  bool still = model::is_still(scenario.wind);
  return World::set_up()
      .numbered(1)
      .holding<archetype::Aircraft>(simple)
      .holding<archetype::PreciseAircraft>(precise)
      .holding<archetype::RigidAircraft>(still ? airliners + fighters : 0)
      .holding<archetype::RigidAircraftInWind>(still ? 0 : airliners + fighters)
      .build(world);
}

auto build_scenario(const Scenario& scenario, const RigidTypes& types,
                    InOut<World> world)
    -> std::expected<void, framework::Status> {
  auto [simple, precise, airliners, fighters] = split_fleet(scenario);
  CHECK_PRECONDITION(airliners == 0 || types.airliner);
  CHECK_PRECONDITION(fighters == 0 || types.fighter);
  model::Earth earth = model::Earth::flat();
  Random random{scenario.seed};
  double side = scenario.spacing.numerical_value_in(meter) *
                std::sqrt(static_cast<double>(simple + precise));
  double reach = scenario.route_reach.numerical_value_in(meter);
  double lowest = scenario.lowest.numerical_value_in(meter);
  double highest = scenario.highest.numerical_value_in(meter);

  // One trim serves every rigid aircraft of a type: over a flat Earth it
  // holds anywhere, on any heading.
  std::optional<model::Trim> airliner_trim;
  if (airliners > 0) {
    RETURN_OR_ASSIGN(airliner_trim, trim_in_cruise(*types.airliner, earth));
  }
  std::optional<model::Trim> fighter_trim;
  if (fighters > 0) {
    RETURN_OR_ASSIGN(fighter_trim, trim_in_cruise(*types.fighter, earth));
  }

  auto transaction = world->transaction();
  std::size_t all = simple + precise + airliners + fighters;
  for (std::size_t i = 0; i < all; ++i) {
    double x = random.uniform(0.0, side);
    double y = random.uniform(0.0, side);
    Route route{.speed =
                    random.uniform(
                        scenario.slowest.numerical_value_in(meter_per_second),
                        scenario.fastest.numerical_value_in(meter_per_second)) *
                    meter_per_second};
    for (Position& waypoint : route.waypoints) {
      waypoint = meters(x + random.uniform(-reach, reach),
                        y + random.uniform(-reach, reach),
                        random.uniform(lowest, highest));
    }
    Position start = meters(x, y, random.uniform(lowest, highest));
    AirState state{
        .position = start,
        .speed = route.speed,
        .heading = model::compute_bearing(start, route.waypoints[0])};
    if (i < simple) {
      RETURN_IF_UNEXPECTED(
          create_aircraft<archetype::Aircraft>(scenario, state, route, world));
    } else if (i < simple + precise) {
      RETURN_IF_UNEXPECTED(create_aircraft<archetype::PreciseAircraft>(
          scenario, state, route, world));
    } else if (i < simple + precise + airliners) {
      RETURN_IF_UNEXPECTED(create_rigid_aircraft(
          *types.airliner, earth, *airliner_trim, scenario.airliner_gains,
          x * meter, y * meter, state.heading, route, world, scenario.wind,
          gust_seed(scenario, i)));
    } else {
      RETURN_IF_UNEXPECTED(create_rigid_aircraft(
          *types.fighter, earth, *fighter_trim, scenario.fighter_gains,
          x * meter, y * meter, state.heading, route, world, scenario.wind,
          gust_seed(scenario, i)));
    }
  }
  transaction.commit();
  return {};
}

auto Simulation::configure() -> engine::PhaseResult {
  if (scenario_.rigid > 0) {
    RETURN_OR_ASSIGN(model::AircraftData loaded,
                     format::load_aircraft(scenario_.rigid_aircraft));
    airliner_ = std::make_unique<model::AircraftData>(std::move(loaded));
  }
  if (scenario_.fighters > 0) {
    RETURN_OR_ASSIGN(model::AircraftData loaded,
                     format::load_aircraft(scenario_.fighter_aircraft));
    fighter_ = std::make_unique<model::AircraftData>(std::move(loaded));
  }
  RETURN_IF_UNEXPECTED(build_world(scenario_, Out(world_)));
  RETURN_IF_UNEXPECTED(build_scenario(
      scenario_,
      RigidTypes{.airliner = airliner_.get(), .fighter = fighter_.get()},
      InOut(world_)));
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const Step& step) -> engine::PhaseResult {
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

}  // namespace simon::aeronautic
