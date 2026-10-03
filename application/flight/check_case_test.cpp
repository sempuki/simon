// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "application/flight/components.hpp"
#include "application/flight/systems.hpp"
#include "application/flight/testing.hpp"
#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "framework/vocabulary.hpp"
#include "model/trim.hpp"

// The whole 737, and the whole F-16, against JSBSim's, open loop: from the
// same trim, flown through the check cases reference/jsbsim_737_check_cases.py
// and jsbsim_f16_check_cases.py flew, through simon's systems at its 8 ms step
// and at 0.5 ms. The reference is JSBSim where its integrators and frame lags
// have converged: at 0.5 ms for the 737, and at 0.125 ms for the F-16, whose
// flight controls run every 8 ms in both.
namespace simon::flight {

namespace {

using namespace std::chrono_literals;
using namespace testing;

// An aircraft's check cases: its data, JSBSim's trim and flights, and how
// far each case moves the pilot's commands.
struct Checked final {
  std::string_view aircraft;
  std::string_view initial;
  std::string_view cases;
  std::string_view signals;  // The flight controls the trim leaves, if any.
  // JSBSim's frame where it has converged, the reference.
  std::chrono::microseconds reference{500};
  // The flight control computer's period, if it has its own rate.
  std::optional<Duration> flight_control_period;
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
  double throttle = 0.0;
};

constexpr Checked BOEING_737_CASES{
    .aircraft = BOEING_737,
    .initial = "application/flight/reference/jsbsim_737_check_initial.csv",
    .cases = "application/flight/reference/jsbsim_737_check_cases.csv",
    .elevator = 0.05,
    .aileron = 0.2,
    .rudder = 0.2,
    .throttle = 0.2,
};

constexpr Checked F16_CASES{
    .aircraft = F16,
    .initial = "application/flight/reference/jsbsim_f16_check_initial.csv",
    .cases = "application/flight/reference/jsbsim_f16_check_cases.csv",
    .signals = "application/flight/reference/jsbsim_f16_check_signals.csv",
    .reference = std::chrono::microseconds{125},
    .flight_control_period = 8ms,
    .elevator = 0.05,
    .aileron = 0.2,
    .rudder = 0.2,
    .throttle = 0.4,
};

enum class Case : std::uint8_t {
  HOLD,
  ELEVATOR,
  AILERON,
  RUDDER,
  THROTTLE,
  WIND
};

// The offsets a case adds to the trim at `microseconds`, as the scripts have
// them.
struct Offset final {
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
  double throttle = 0.0;
};

auto offset(const Checked& checked, Case flown, std::int64_t microseconds)
    -> Offset {
  int doublet = microseconds >= 1'000'000 && microseconds < 2'000'000   ? 1
                : microseconds >= 2'000'000 && microseconds < 3'000'000 ? -1
                                                                        : 0;
  return Offset{
      .elevator = flown == Case::ELEVATOR ? checked.elevator * doublet : 0.0,
      .aileron = flown == Case::AILERON ? checked.aileron * doublet : 0.0,
      .rudder = flown == Case::RUDDER ? checked.rudder * doublet : 0.0,
      .throttle = flown == Case::THROTTLE && microseconds >= 1'000'000
                      ? checked.throttle
                      : 0.0,
  };
}

// Sets each rigid aircraft's commands for the case at the step's start.
struct Pilot final  //
    : System<FlightSignals> {
  using SystemWorld = ProjectedWorld<Pilot>;

  Pilot(const Checked& checked, Case flown, const Row& trim,
        std::size_t engines)
      : checked_{&checked}, flown_{flown}, trim_{&trim}, engines_{engines} {}

  auto operator()(SystemWorld&, Entity,    //
                  FlightSignals& signals,  //
                  Step step) const -> void {
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                            step.time.time_since_epoch())
                            .count();
    Offset more = offset(*checked_, flown_, microseconds);
    using enum model::FlightSignal;
    signals[ELEVATOR_COMMAND] = trim_->at("elevator") + more.elevator;
    signals[AILERON_COMMAND] = trim_->at("aileron") + more.aileron;
    signals[RUDDER_COMMAND] = trim_->at("rudder") + more.rudder;
    for (std::size_t i = 0; i < engines_; ++i) {
      signals.values[model::index_of(THROTTLE_COMMAND_0) + i] =
          trim_->at("throttle_" + std::to_string(i)) + more.throttle;
    }
  }

 private:
  const Checked* checked_ = nullptr;
  Case flown_ = Case::HOLD;
  const Row* trim_ = nullptr;
  std::size_t engines_ = 0;
};

// Sets each rigid aircraft's wind for the case at the step's start: still,
// or in the wind case from 1 s on, the scripts' WIND.
struct Blow final  //
    : System<Wind> {
  using SystemWorld = ProjectedWorld<Blow>;

  explicit Blow(Case flown) : flown_{flown} {}

  auto operator()(SystemWorld&, Entity,  //
                  Wind& wind,            //
                  Step step) const -> void {
    bool blowing = flown_ == Case::WIND && step.time >= TimePoint{1004ms};
    wind = Wind{.north_east_down =
                    blowing ? model::meters_per_second(8.0, -12.0, 2.0)
                            : model::meters_per_second(0.0, 0.0, 0.0)};
  }

 private:
  Case flown_ = Case::HOLD;
};

using CheckSchedule = SystemList<Pilot, Blow, RunFlightControls, RunEngines,
                                 Rigid, BurnFuel, FollowRigidBody>;

// The distance between two bodies: in position, velocity, attitude and rate.
struct Apart final {
  double position = 0.0;  // m.
  double velocity = 0.0;  // m/s.
  double attitude = 0.0;  // rad.
  double rate = 0.0;      // rad/s.

  auto widen(const RigidBody& a, const RigidBody& b) -> void {
    position = std::max(
        position,
        magnitude((a.position - b.position).numerical_value_in(model::meter)));
    velocity = std::max(
        velocity, magnitude((a.velocity - b.velocity)
                                .numerical_value_in(model::meter_per_second)));
    attitude = std::max(attitude, a.attitude.angularDistance(b.attitude));
    rate = std::max(
        rate,
        magnitude(
            (a.rate - b.rate).numerical_value_in(model::radian_per_second)));
  }
};

// The flight controls as the trim leaves them: as JSBSim recorded them in
// `recorded`, if it did, with each PID's last input its input now; or else
// settled from the trimmed commands.
auto trimmed_flight_controls(const model::AircraftData& data, const Row& trim,
                             const Row* recorded) -> FlightSignals {
  const model::FlightControlData& controls = data.flight_controls;
  FlightSignals signals;
  using enum model::FlightSignal;
  signals[ELEVATOR_COMMAND] = trim.at("elevator");
  signals[AILERON_COMMAND] = trim.at("aileron");
  signals[RUDDER_COMMAND] = trim.at("rudder");
  signals[PITCH_TRIM_COMMAND] = trim.at("pitch_trim");
  signals[ROLL_TRIM_COMMAND] = trim.at("roll_trim");
  signals[YAW_TRIM_COMMAND] = trim.at("yaw_trim");
  for (std::size_t i = 0; i < data.engines.size(); ++i) {
    signals.values[model::index_of(THROTTLE_COMMAND_0) + i] =
        trim.at("throttle_" + std::to_string(i));
  }

  if (!recorded) {
    model::settle_flight_controls(controls, InOut(signals));
    return signals;
  }
  for (std::size_t s = model::FLIGHT_SIGNAL_COUNT; s < controls.signals.size();
       ++s) {
    if (auto found = recorded->find(controls.signals[s]);
        found != recorded->end()) {
      signals.values[s] = found->second;
    }
  }
  for (const model::FlightBlock& block : controls.blocks) {
    if (block.kind == model::FlightBlock::Kind::PID) {
      const model::FlightBlock::Input& input = block.inputs.front();
      double value = signals.values[input.signal] * input.scale;
      signals.values[block.state + 1] = input.negated ? -value : value;
    }
  }
  return signals;
}

// The fuel the trim leaves in each tank.
auto trimmed_tanks(const model::AircraftData& data, const Row& trim)
    -> FuelTanks {
  FuelTanks tanks;
  for (std::size_t i = 0; i < data.tanks.size(); ++i) {
    tanks.contents[i] = trim.at("fuel_" + std::to_string(i)) * model::kilogram;
  }
  return tanks;
}

// Flies `flown` from the trim at steps of `dt`, and returns the body every
// 0.2 s.
auto fly(const Checked& checked, Case flown, Duration dt,
         const model::AircraftData& data, const Row& trim, const Row* recorded)
    -> std::vector<RigidBody> {
  World world;
  REQUIRE(World::set_up()
              .numbered(1)
              .holding<archetype::RigidAircraftInWind>(1)
              .build(Out(world)));
  model::Earth earth = model::Earth::round(model::wgs84::Geodetic{});
  RigidBody body = read_body(trim);
  model::StandardAirTable air;

  FlightSignals signals = trimmed_flight_controls(data, trim, recorded);
  Engines engines = model::settled_engines(
      data, signals,
      model::compute_engine_air(body, earth, air, Wind{}, 0.0 * model::second));
  FuelTanks tanks = trimmed_tanks(data, trim);
  MassBalance mass = model::compute_mass_balance(data, tanks);
  BodyAcceleration felt;
  model::rigid_aircraft_rate(body, signals, engines, mass, data, earth, air,
                             Wind{}, 0.0 * model::second, Out(felt));
  auto aircraft = world.create<archetype::RigidAircraftInWind>()
                      .with(earth.air_state(body, 0.0 * model::second))
                      .with(body)
                      .with(RigidBodyRate{})
                      .with(felt)
                      .with(signals)
                      .with(engines)
                      .with(tanks)
                      .with(mass)
                      .with(AircraftType{.data = &data})
                      .with(Autopilot{})
                      .with(Route{})
                      .with(SurfaceAutopilot{})
                      .with(Wind{})
                      .build();
  REQUIRE(aircraft);
  world.sync();

  framework::Scheduler<World, CheckSchedule> scheduler{CheckSchedule{
      Pilot{checked, flown, trim, data.engines.size()}, Blow{flown},
      RunFlightControls{earth, checked.flight_control_period},
      RunEngines{earth}, Rigid{SystemList{RigidAircraftRates{earth}}},
      BurnFuel{}, FollowRigidBody{earth}}};
  std::vector<RigidBody> samples{body};
  constexpr Duration SAMPLE = 200ms;
  for (TimePoint time{}; time < TimePoint{30s}; time += dt) {
    scheduler.step(Step{.time = time, .dt = dt}, InOut(world));
    if ((time + dt).time_since_epoch() % SAMPLE == Duration::zero()) {
      samples.push_back(world.store_of<RigidBody>().component_of(*aircraft));
    }
  }
  return samples;
}

// JSBSim's bodies for `flown` at `frame` microseconds, every 0.2 s.
auto jsbsim(const std::vector<Row>& rows, Case flown, int frame)
    -> std::vector<RigidBody> {
  std::vector<RigidBody> bodies;
  for (const Row& row : rows) {
    if (row.at("case") == static_cast<double>(flown) &&
        row.at("frame_us") == frame) {
      bodies.push_back(read_body(row));
    }
  }
  return bodies;
}

auto apart(const std::vector<RigidBody>& a, const std::vector<RigidBody>& b)
    -> Apart {
  REQUIRE(a.size() == b.size());
  Apart result;
  for (std::size_t i = 0; i < a.size(); ++i) {
    result.widen(a[i], b[i]);
  }
  return result;
}

// The distance from JSBSim at 0.5 ms of simon at 0.5 ms, of simon at 8 ms,
// and of JSBSim at 8 ms, for each case.
struct Distances final {
  Apart physics;
  Apart simon;
  Apart theirs;
};

auto measure(const Checked& checked, Case flown) -> Distances {
  auto data = model::load_aircraft(std::string{checked.aircraft});
  REQUIRE(data);
  std::vector<Row> initial = load_rows(checked.initial);
  REQUIRE(initial.size() == 1);
  std::vector<Row> rows = load_rows(checked.cases);
  std::vector<Row> signals;
  if (!checked.signals.empty()) {
    signals = load_rows(checked.signals);
    REQUIRE(signals.size() == 1);
  }
  const Row* recorded = signals.empty() ? nullptr : &signals[0];
  std::vector<RigidBody> reference =
      jsbsim(rows, flown, static_cast<int>(checked.reference.count()));
  REQUIRE(reference.size() == 151);
  return Distances{
      .physics = apart(fly(checked, flown, std::chrono::microseconds{500},
                           *data, initial[0], recorded),
                       reference),
      .simon = apart(fly(checked, flown, 8ms, *data, initial[0], recorded),
                     reference),
      .theirs = apart(jsbsim(rows, flown, 8000), reference),
  };
}

}  // namespace

TEST_CASE("CheckCases737") {
  for (Case flown : {Case::HOLD, Case::ELEVATOR, Case::AILERON, Case::RUDDER,
                     Case::THROTTLE}) {
    auto [physics, simon, theirs] = measure(BOEING_737_CASES, flown);
    CAPTURE(static_cast<int>(flown));
    CAPTURE(physics.position, physics.velocity, physics.attitude, physics.rate);
    CAPTURE(simon.position, simon.velocity, simon.attitude, simon.rate);
    CAPTURE(theirs.position, theirs.velocity, theirs.attitude, theirs.rate);

    // With integration error gone, simon's physics and JSBSim's agree to
    // millimeters, and to 2.4 cm after the rudder doublet.
    CHECK(physics.position < 0.05);
    CHECK(physics.velocity < 0.003);
    CHECK(physics.attitude < 3e-4);

    // At 8 ms simon stays within 6 cm of the reference, and is closer to it
    // than JSBSim is at the same frame, down to a floor of 5 mm where both
    // are at the physics' own agreement.
    CHECK(simon.position < 0.1);
    CHECK(simon.position < std::max(theirs.position, 0.005));
    CHECK(simon.attitude < std::max(theirs.attitude, 1e-6));
  }
}

TEST_CASE("CheckCases737InWind") {
  auto [physics, simon, theirs] = measure(BOEING_737_CASES, Case::WIND);
  CAPTURE(physics.position, physics.velocity, physics.attitude, physics.rate);
  CAPTURE(simon.position, simon.velocity, simon.attitude, simon.rate);
  CAPTURE(theirs.position, theirs.velocity, theirs.attitude, theirs.rate);

  // The 737's pitching moment reads the rate of angle of attack, which
  // JSBSim takes from the velocity over the ground. That leaves out the
  // wind turning in body axes as the 737 pitches (see
  // model::compute_air_acceleration and its test), so in wind simon's 737
  // parts from JSBSim's by 92 cm in 30 s. With JSBSim's rate it agrees to
  // 1.7 cm.
  CHECK(physics.position < 1.0);
  CHECK(physics.attitude < 2e-3);
  CHECK(simon.position < 1.0);
}

TEST_CASE("CheckCasesF16") {
  SECTION("ShouldMatchJsbsimGivenTrimmedMassBalance") {
    auto data = model::load_aircraft(std::string{F16});
    REQUIRE(data);
    std::vector<Row> initial = load_rows(F16_CASES.initial);
    REQUIRE(initial.size() == 1);
    const Row& trim = initial[0];
    MassBalance mass =
        model::compute_mass_balance(*data, trimmed_tanks(*data, trim));
    // JSBSim turns pounds into slugs by a rounded factor, 1.4e-8 off.
    CHECK_THAT(mass.properties.mass.numerical_value_in(model::kilogram),
               Catch::Matchers::WithinRel(trim.at("mass"), 2e-8));
    model::QuantityVector center =
        mass.center_of_mass.numerical_value_in(model::meter);
    CHECK(magnitude(center - read_vector(trim, "cg_x", "cg_y", "cg_z")) <
          1e-12);
  }

  for (Case flown : {Case::HOLD, Case::ELEVATOR, Case::AILERON, Case::RUDDER,
                     Case::THROTTLE}) {
    auto [physics, simon, theirs] = measure(F16_CASES, flown);
    CAPTURE(static_cast<int>(flown));
    CAPTURE(physics.position, physics.velocity, physics.attitude, physics.rate);
    CAPTURE(simon.position, simon.velocity, simon.attitude, simon.rate);
    CAPTURE(theirs.position, theirs.velocity, theirs.attitude, theirs.rate);

    // With the flight controls at the same rate, simon's physics and JSBSim's
    // agree to millimeters, and to 6 cm after the roll doublet, where JSBSim
    // has itself converged only to 5 cm.
    CHECK(physics.position < 0.07);
    CHECK(physics.attitude < 1e-4);

    // At 8 ms simon is as close as at 0.5 ms, but for 15 cm after the step
    // into reheat, and closer than JSBSim is at the same frame, down to a
    // floor of 5 mm.
    CHECK(simon.position < 0.2);
    CHECK(simon.position < std::max(theirs.position, 0.005));
    CHECK(simon.attitude < std::max(theirs.attitude, 1e-6));
  }
}

TEST_CASE("CheckCasesF16InWind") {
  auto [physics, simon, theirs] = measure(F16_CASES, Case::WIND);
  CAPTURE(physics.position, physics.velocity, physics.attitude, physics.rate);
  CAPTURE(simon.position, simon.velocity, simon.attitude, simon.rate);
  CAPTURE(theirs.position, theirs.velocity, theirs.attitude, theirs.rate);

  // The F-16's aerodynamics do not read the rate of angle of attack, and
  // in wind simon's physics and JSBSim's agree to 4.4 mm, as in still air.
  CHECK(physics.position < 0.01);
  CHECK(physics.attitude < 1e-4);

  // The wind starts between 8 ms steps, so at 8 ms simon and JSBSim each
  // start it 4 ms off the reference: simon stays within 4 cm.
  CHECK(simon.position < 0.05);
}

// How far a trimmed aircraft wanders in 30 s: in altitude and in airspeed.
struct Wander final {
  double altitude = 0.0;  // m.
  double speed = 0.0;     // m/s.

  // Widens by `body` at `time`, against `start` at time zero, over `earth`.
  auto widen(const model::Earth& earth, const RigidBody& start,
             const RigidBody& body, model::Time time) -> void {
    auto height = [&](const RigidBody& b, model::Time t) {
      return earth.altitude(b, t).numerical_value_in(model::meter);
    };
    auto airspeed = [&](const RigidBody& b) {
      return magnitude(
          earth.air_velocity(b).numerical_value_in(model::meter_per_second));
    };
    altitude = std::max(altitude, std::abs(height(body, time) -
                                           height(start, 0.0 * model::second)));
    speed = std::max(speed, std::abs(airspeed(body) - airspeed(start)));
  }
};

// With fuel burning, and with the mass held, as a trim alone is judged.
using HoldSchedule =
    SystemList<RunFlightControls, RunEngines, Rigid, BurnFuel, FollowRigidBody>;
using HeldMassSchedule =
    SystemList<RunFlightControls, RunEngines, Rigid, FollowRigidBody>;

// Trims the aircraft of `checked` at the condition of JSBSim's trim, flies it
// 30 s at 8 ms, burning fuel or not, and returns how far it wanders, and how
// far JSBSim's own trim wanders at its reference frame, burning fuel.
template <typename ScheduleType>
auto hold(const Checked& checked) -> std::pair<Wander, Wander> {
  auto data = model::load_aircraft(std::string{checked.aircraft});
  REQUIRE(data);
  std::vector<Row> initial = load_rows(checked.initial);
  REQUIRE(initial.size() == 1);
  model::Earth earth = model::Earth::round(model::wgs84::Geodetic{});
  model::StandardAirTable air;
  AirState start = earth.air_state(read_body(initial[0]), 0.0 * model::second);

  model::FlightCondition condition{
      .position = start.position,
      .speed = start.speed,
      .heading = start.heading,
      .flight_path_angle = start.flight_path_angle,
      .tanks = trimmed_tanks(*data, initial[0]),
  };
  auto trim = model::trim(*data, condition, earth, air);
  REQUIRE(trim);

  World world;
  REQUIRE(
      World::set_up().numbered(1).holding<archetype::RigidAircraft>(1).build(
          Out(world)));
  auto aircraft = world.create<archetype::RigidAircraft>()
                      .with(earth.air_state(trim->body, 0.0 * model::second))
                      .with(trim->body)
                      .with(RigidBodyRate{})
                      .with(trim->felt)
                      .with(trim->signals)
                      .with(trim->engines)
                      .with(condition.tanks)
                      .with(trim->mass)
                      .with(AircraftType{.data = &*data})
                      .with(Autopilot{})
                      .with(Route{})
                      .with(SurfaceAutopilot{})
                      .build();
  REQUIRE(aircraft);
  world.sync();
  model::Earth fixed = earth;
  auto systems = [&] {
    if constexpr (std::same_as<ScheduleType, HoldSchedule>) {
      return HoldSchedule{
          RunFlightControls{fixed, checked.flight_control_period},
          RunEngines{fixed}, Rigid{SystemList{RigidAircraftRates{fixed}}},
          BurnFuel{}, FollowRigidBody{fixed}};
    } else {
      return HeldMassSchedule{
          RunFlightControls{fixed, checked.flight_control_period},
          RunEngines{fixed}, Rigid{SystemList{RigidAircraftRates{fixed}}},
          FollowRigidBody{fixed}};
    }
  };
  framework::Scheduler<World, ScheduleType> scheduler{systems()};
  Wander simon;
  for (TimePoint time{}; time < TimePoint{30s}; time += 8ms) {
    scheduler.step(Step{.time = time, .dt = 8ms}, InOut(world));
    simon.widen(earth, trim->body,
                world.store_of<RigidBody>().component_of(*aircraft),
                model::seconds((time + 8ms).time_since_epoch()));
  }

  // JSBSim's from its own trim, every 0.2 s.
  Wander theirs;
  std::vector<RigidBody> bodies =
      jsbsim(load_rows(checked.cases), Case::HOLD,
             static_cast<int>(checked.reference.count()));
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    theirs.widen(earth, bodies.front(), bodies[i],
                 0.2 * static_cast<double>(i) * model::second);
  }
  return {simon, theirs};
}

TEST_CASE("HoldsTrim") {
  for (const Checked* checked : {&BOEING_737_CASES, &F16_CASES}) {
    CAPTURE(checked->aircraft);

    // Burning fuel lightens the aircraft, which climbs and speeds up: 2 m and
    // 0.1 m/s for the 737, 16 cm for the F-16, against 4 m and 2 m from
    // JSBSim's trims.
    auto [burning, theirs] = hold<HoldSchedule>(*checked);
    CAPTURE(burning.altitude, burning.speed, theirs.altitude, theirs.speed);
    CHECK(burning.altitude < theirs.altitude);
    CHECK(burning.speed < theirs.speed);

    // With the mass held, level over the Earth to millimeters in 30 s.
    auto held = hold<HeldMassSchedule>(*checked).first;
    CAPTURE(held.altitude, held.speed);
    CHECK(held.altitude < 0.01);
    CHECK(held.speed < 0.001);
  }
}

}  // namespace simon::flight
