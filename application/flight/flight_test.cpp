// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <chrono>
#include <cmath>
#include <expected>
#include <numbers>
#include <type_traits>
#include <utility>
#include <vector>

#include "application/flight/simulation.hpp"
#include "base/testing.hpp"
#include "engine/driver.hpp"
#include "framework/vocabulary.hpp"

namespace simon::flight {

namespace {

using namespace std::chrono_literals;

constexpr Duration DT = 20ms;
constexpr double PI = std::numbers::pi;

auto build_small_world(Out<World> world) -> void {
  std::expected<void, framework::Status> built =
      World::set_up()
          .numbered(1)
          .holding<archetype::Aircraft>(4)
          .holding<archetype::PreciseAircraft>(4)
          .holding<archetype::RigidAircraft>(4)
          .build(world);
  CHECK_POSTCONDITION(built.has_value());
}

// A route whose every waypoint is `waypoint`.
auto route_to(Position waypoint, Speed speed) -> Route {
  Route route{.speed = speed};
  route.waypoints.fill(waypoint);
  return route;
}

auto level(double altitude, double speed, double heading) -> AirState {
  return AirState{.position = model::meters(0.0, 0.0, altitude),
                  .speed = speed * model::meter_per_second,
                  .heading = heading * model::radian};
}

template <typename ArchetypeType = archetype::Aircraft>
auto create(const AirState& state, const Route& route, InOut<World> world)
    -> Entity {
  Scenario scenario;
  auto builder = world->create<ArchetypeType>()
                     .with(state)
                     .with(FlightControls{.throttle = 0.3})
                     .with(Commands{.throttle = 0.3})
                     .with(scenario.airframe)
                     .with(scenario.handling)
                     .with(Autopilot{.altitude = model::altitude_of(state),
                                     .heading = state.heading,
                                     .speed = route.speed,
                                     .throttle_integral = 0.3})
                     .with(route);
  std::expected<Entity, framework::Status> entity;
  if constexpr (std::is_same_v<ArchetypeType, archetype::PreciseAircraft>) {
    entity = std::move(builder).with(AirStateRate{}).build();
  } else {
    entity = std::move(builder).build();
  }
  REQUIRE(entity);
  return *entity;
}

// Steps `world` for `duration`, DT at a time.
auto fly_for(Duration duration, InOut<Scheduler> scheduler, InOut<World> world)
    -> void {
  world->sync();
  for (TimePoint time{}; time < TimePoint{duration}; time += DT) {
    scheduler->step(framework::Step{.time = time, .dt = DT}, world);
  }
}

auto state_of(const World& world, Entity entity) -> const AirState& {
  return world.store_of<AirState>().component_of(entity);
}

}  // namespace

TEST_CASE("Autopilot") {
  World world;
  build_small_world(Out(world));
  Scheduler scheduler;

  SECTION("ShouldHoldAltitudeAndSpeedGivenTargetsAhead") {
    Entity aircraft = create(level(5000.0, 200.0, 0.0),
                             route_to(model::meters(0.0, 500000.0, 6000.0),
                                      220.0 * model::meter_per_second),
                             InOut(world));

    fly_for(3min, InOut(scheduler), InOut(world));

    const AirState& state = state_of(world, aircraft);
    CHECK(std::abs(model::altitude_of(state).numerical_value_in(model::meter) -
                   6000.0) < 20.0);
    CHECK(std::abs(state.speed.numerical_value_in(model::meter_per_second) -
                   220.0) < 2.0);
    CHECK(std::abs(model::radians(state.heading)) < 0.01);
  }

  SECTION("ShouldTurnToWaypointGivenWaypointBehind") {
    Entity aircraft = create(level(5000.0, 200.0, 0.0),
                             route_to(model::meters(0.0, -500000.0, 5000.0),
                                      200.0 * model::meter_per_second),
                             InOut(world));

    fly_for(90s, InOut(scheduler), InOut(world));

    const AirState& state = state_of(world, aircraft);
    double heading = model::radians(state.heading);
    CHECK(std::abs(std::remainder(heading - PI, 2.0 * PI)) < 0.05);
    CHECK(std::abs(model::altitude_of(state).numerical_value_in(model::meter) -
                   5000.0) < 50.0);
  }
}

TEST_CASE("Route") {
  World world;
  build_small_world(Out(world));
  Scheduler scheduler;

  SECTION("ShouldReachEveryWaypointGivenClosedRoute") {
    Route route{.speed = 200.0 * model::meter_per_second};
    route.waypoints = {model::meters(0.0, 30000.0, 5000.0),
                       model::meters(30000.0, 30000.0, 6000.0),
                       model::meters(30000.0, 0.0, 5000.0),
                       model::meters(0.0, 0.0, 4000.0)};
    Entity aircraft = create(level(5000.0, 200.0, 0.0), route, InOut(world));

    fly_for(15min, InOut(scheduler), InOut(world));

    CHECK(world.store_of<Route>().component_of(aircraft).reached >= 4);
  }
}

TEST_CASE("Fidelity") {
  SECTION("ShouldAgreeGivenSinglePassAndRungeKutta") {
    World world;
    build_small_world(Out(world));
    Scheduler scheduler;
    Route route{.speed = 220.0 * model::meter_per_second};
    route.waypoints = {model::meters(0.0, 30000.0, 6000.0),
                       model::meters(30000.0, 30000.0, 5000.0),
                       model::meters(30000.0, 0.0, 6000.0),
                       model::meters(0.0, 0.0, 5000.0)};
    AirState start = level(5000.0, 200.0, 0.0);
    Entity simple = create(start, route, InOut(world));
    Entity precise =
        create<archetype::PreciseAircraft>(start, route, InOut(world));

    fly_for(5min, InOut(scheduler), InOut(world));

    // The two fly the same route by different integrators: close, but not
    // the same.
    double apart =
        model::distance(state_of(world, simple), state_of(world, precise))
            .numerical_value_in(model::meter);
    CHECK(apart > 0.0);
    CHECK(apart < 100.0);
  }
}

TEST_CASE("Simulation") {
  auto run = [](const Scenario& scenario) {
    Simulation simulation{scenario};
    engine::BatchDriver driver{engine::Timing{.max_step = DT},
                               Depend(simulation)};
    REQUIRE(driver.run(TimePoint{2min}));
    std::vector<AirState> states;
    simulation.world().store_of<AirState>().for_each(
        [&](Entity, const AirState& state) { states.push_back(state); });
    return std::pair{states, simulation.waypoints_reached()};
  };

  SECTION("ShouldRepeatGivenSameSeed") {
    Scenario scenario{.seed = 7, .aircraft = 50, .precise = 10, .rigid = 5};
    auto [first, first_reached] = run(scenario);
    auto [second, second_reached] = run(scenario);

    REQUIRE(first.size() == 50);
    REQUIRE(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
      CHECK(first[i].position == second[i].position);
      CHECK(first[i].speed == second[i].speed);
      CHECK(first[i].heading == second[i].heading);
    }
    CHECK(first_reached == second_reached);
  }

  SECTION("ShouldStayAirborneGivenDefaultScenario") {
    auto [states, reached] = run(Scenario{.aircraft = 100, .precise = 20});

    for (const AirState& state : states) {
      double altitude =
          model::altitude_of(state).numerical_value_in(model::meter);
      CHECK(altitude > 2500.0);
      CHECK(altitude < 9500.0);
      CHECK(state.speed > 150.0 * model::meter_per_second);
    }
    CHECK(reached > 0);
  }

  SECTION("ShouldFlyRoutesGivenRigidAircraftAmongPointMass") {
    // Point-mass and rigid aircraft fly the same kind of routes in one
    // world; the rigid ones stay in the routes' envelope throughout.
    Simulation simulation{
        Scenario{.seed = 3, .aircraft = 40, .precise = 10, .rigid = 6}};
    REQUIRE(simulation.configure());
    double lowest = 1e9;
    double highest = 0.0;
    double slowest = 1e9;
    double fastest = 0.0;
    for (TimePoint time{}; time < TimePoint{5min}; time += DT) {
      REQUIRE(simulation.step(framework::Step{.time = time, .dt = DT}));
      const World& world = simulation.world();
      world.store_of<RigidBody>().for_each([&](Entity entity,
                                               const RigidBody&) {
        const AirState& state = state_of(world, entity);
        double altitude =
            model::altitude_of(state).numerical_value_in(model::meter);
        double speed = state.speed.numerical_value_in(model::meter_per_second);
        lowest = std::min(lowest, altitude);
        highest = std::max(highest, altitude);
        slowest = std::min(slowest, speed);
        fastest = std::max(fastest, speed);
      });
    }
    CAPTURE(lowest, highest, slowest, fastest);
    CHECK(lowest > 2500.0);
    CHECK(highest < 9500.0);
    CHECK(slowest > 150.0);
    CHECK(fastest < 270.0);
    CHECK(simulation.rigid_waypoints_reached() >= 6);
    CHECK(simulation.waypoints_reached() >
          simulation.rigid_waypoints_reached());
  }
}

namespace {

constexpr char BOEING_737[] = "application/flight/aircraft/737.aircraft";

// A 737 trimmed in cruise at 6 km and 200 m/s, heading north from the
// world's origin toward a waypoint 500 km ahead.
auto rigid_737(const model::Earth& earth, const model::AircraftData& data,
               InOut<World> world) -> Entity {
  Route route{.speed = 200.0 * model::meter_per_second};
  route.waypoints.fill(model::meters(0.0, 500000.0, 6000.0));
  auto entity = create_rigid_aircraft(data, earth, RigidTrim{},
                                      0.0 * model::meter, 0.0 * model::meter,
                                      0.0 * model::radian, route, world);
  REQUIRE(entity);
  return *entity;
}

auto scheduler_for(const model::Earth& earth) -> Scheduler {
  return Scheduler{Schedule{FollowRoute{}, FlyAutopilot{}, Actuate{}, Fly{},
                            Precise{}, FlySurfaces{earth},
                            RunFlightControls{earth}, RunEngines{earth},
                            Rigid{SystemList{RigidAircraftRates{earth}}},
                            BurnFuel{}, FollowRigidBody{earth}}};
}

auto fly_rigid(Duration duration, InOut<Scheduler> scheduler,
               InOut<World> world) -> void {
  constexpr Duration STEP = 8ms;
  world->sync();
  for (TimePoint time{}; time < TimePoint{duration}; time += STEP) {
    scheduler->step(framework::Step{.time = time, .dt = STEP}, world);
  }
}

}  // namespace

TEST_CASE("RigidAircraft") {
  auto data = model::load_aircraft(BOEING_737);
  REQUIRE(data);
  World world;
  build_small_world(Out(world));

  SECTION("ShouldFollowItsBodyGivenFlatEarth") {
    model::Earth earth = model::Earth::flat();
    Entity aircraft = rigid_737(earth, *data, InOut(world));
    Scheduler scheduler = scheduler_for(earth);

    fly_rigid(10s, InOut(scheduler), InOut(world));

    const RigidBody& body = world.store_of<RigidBody>().component_of(aircraft);
    const AirState& state = state_of(world, aircraft);
    AirState expected = earth.air_state(body, 10.0 * model::second);
    CHECK(state.position == expected.position);
    CHECK(state.speed == expected.speed);

    // About 2 km north, without engines: gliding, not tumbling.
    model::QuantityVector where =
        state.position.numerical_value_in(model::meter);
    CHECK(std::abs(where.eigen().y() - 1950.0) < 100.0);
    CHECK(std::abs(where.eigen().z() - 6000.0) < 300.0);
    CHECK(std::abs(model::radians(state.heading)) < 0.05);
  }

  SECTION("ShouldAgreeWithFlatEarthGivenRoundEarthOverShortFlight") {
    // Over 10 s the Earth's rotation, curvature and gravity move a 737 a
    // few meters from where a flat Earth puts it.
    model::Earth flat = model::Earth::flat();
    model::Earth round = model::Earth::round(model::wgs84::Geodetic{});
    Entity on_flat = rigid_737(flat, *data, InOut(world));
    World round_world;
    build_small_world(Out(round_world));
    Entity on_round = rigid_737(round, *data, InOut(round_world));
    Scheduler flat_scheduler = scheduler_for(flat);
    Scheduler round_scheduler = scheduler_for(round);

    fly_rigid(10s, InOut(flat_scheduler), InOut(world));
    fly_rigid(10s, InOut(round_scheduler), InOut(round_world));

    double apart = model::distance(state_of(world, on_flat),
                                   state_of(round_world, on_round))
                       .numerical_value_in(model::meter);
    CHECK(apart > 0.01);
    CHECK(apart < 5.0);
  }

  SECTION("ShouldBurnFuelGivenEnginesRunning") {
    model::Earth earth = model::Earth::flat();
    Entity aircraft = rigid_737(earth, *data, InOut(world));
    Scheduler scheduler = scheduler_for(earth);
    model::Mass start = model::mass_balance_of(*data).properties.mass;

    fly_rigid(10s, InOut(scheduler), InOut(world));

    // Two engines at about 0.5 kg/s each, for 10 s.
    model::Mass burned =
        start -
        world.store_of<MassBalance>().component_of(aircraft).properties.mass;
    CHECK(burned > 5.0 * model::kilogram);
    CHECK(burned < 20.0 * model::kilogram);
    // With thrust it holds its speed better than gliding.
    CHECK(state_of(world, aircraft).speed > 195.0 * model::meter_per_second);
  }

  SECTION("ShouldLeavePointMassAircraftAloneGivenSharedWorld") {
    AirState start = level(5000.0, 200.0, 0.0);
    Route route = route_to(model::meters(0.0, 500000.0, 5000.0),
                           200.0 * model::meter_per_second);
    Entity point_mass = create(start, route, InOut(world));
    rigid_737(model::Earth::flat(), *data, InOut(world));
    World alone;
    build_small_world(Out(alone));
    Entity by_itself = create(start, route, InOut(alone));
    Scheduler shared;
    Scheduler single;

    fly_rigid(10s, InOut(shared), InOut(world));
    fly_rigid(10s, InOut(single), InOut(alone));

    CHECK(state_of(world, point_mass).position ==
          state_of(alone, by_itself).position);
  }
}

}  // namespace simon::flight
