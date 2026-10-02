// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "application/flight/components.hpp"
#include "application/flight/systems.hpp"
#include "base/testing.hpp"

// The whole 737 against JSBSim's, open loop: from the same trim, flown
// through the check cases reference/jsbsim_737_check_cases.py flew, through
// simon's systems at its 8 ms step and at 0.5 ms. JSBSim at 0.5 ms is the
// reference: its integrators and frame lags have converged there.
namespace simon::flight {

namespace {

using namespace std::chrono_literals;

constexpr char AIRCRAFT[] = "application/flight/aircraft/737.aircraft";
constexpr char INITIAL[] =
    "application/flight/reference/jsbsim_737_check_initial.csv";
constexpr char CASES[] =
    "application/flight/reference/jsbsim_737_check_cases.csv";

enum class Case : std::uint8_t { HOLD, ELEVATOR, AILERON, RUDDER, THROTTLE };

using Row = std::map<std::string, double, std::less<>>;

auto load(const char* path) -> std::vector<Row> {
  std::ifstream file{path};
  REQUIRE(file);
  std::string line;
  std::getline(file, line);
  std::vector<std::string> names;
  for (std::size_t at = 0; at <= line.size();) {
    std::size_t comma = std::min(line.find(',', at), line.size());
    names.emplace_back(line.substr(at, comma - at));
    at = comma + 1;
  }
  std::vector<Row> rows;
  while (std::getline(file, line)) {
    Row row;
    const char* next = line.data();
    const char* end = line.data() + line.size();
    for (const std::string& name : names) {
      double value = 0.0;
      auto [stop, error] = std::from_chars(next, end, value);
      REQUIRE(error == std::errc{});
      row[name] = value;
      next = stop + 1;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

// What a case adds to the trim at `microseconds`, as the script has it.
struct Offset final {
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
  double throttle = 0.0;
};

auto offset(Case flown, std::int64_t microseconds) -> Offset {
  int doublet = microseconds >= 1'000'000 && microseconds < 2'000'000   ? 1
                : microseconds >= 2'000'000 && microseconds < 3'000'000 ? -1
                                                                        : 0;
  return Offset{
      .elevator = flown == Case::ELEVATOR ? 0.05 * doublet : 0.0,
      .aileron = flown == Case::AILERON ? 0.2 * doublet : 0.0,
      .rudder = flown == Case::RUDDER ? 0.2 * doublet : 0.0,
      .throttle =
          flown == Case::THROTTLE && microseconds >= 1'000'000 ? 0.2 : 0.0,
  };
}

// Sets each rigid aircraft's commands for the case at the step's start.
struct Pilot final            //
    : System<EngineControls,  //
             FlightSignals> {
  using SystemWorld = ProjectedWorld<Pilot>;

  Pilot(Case flown, const Row& trim) : flown_{flown}, trim_{&trim} {}

  auto operator()(SystemWorld&, Entity,      //
                  EngineControls& controls,  //
                  FlightSignals* signals,    //
                  Step step) const -> void {
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                            step.time.time_since_epoch())
                            .count();
    Offset more = offset(flown_, microseconds);
    using enum model::FlightSignal;
    (*signals)[ELEVATOR_COMMAND] = trim_->at("elevator") + more.elevator;
    (*signals)[AILERON_COMMAND] = trim_->at("aileron") + more.aileron;
    (*signals)[RUDDER_COMMAND] = trim_->at("rudder") + more.rudder;
    controls.throttle[0] = trim_->at("throttle_0") + more.throttle;
    controls.throttle[1] = trim_->at("throttle_1") + more.throttle;
  }

 private:
  Case flown_;
  const Row* trim_;
};

using CheckSchedule = SystemList<Pilot, RunFlightControls, RunEngines, Rigid,
                                 BurnFuel, FollowRigidBody>;

auto vector_of(const Row& row, const char* x, const char* y, const char* z)
    -> model::Vector3d {
  return model::Vector3d{row.at(x), row.at(y), row.at(z)};
}

auto body_of(const Row& row) -> RigidBody {
  return RigidBody{
      .position = vector_of(row, "x", "y", "z") * model::meter,
      .velocity = vector_of(row, "vx", "vy", "vz") * model::meter_per_second,
      .attitude = model::Quaternion{row.at("qw"), row.at("qx"), row.at("qy"),
                                    row.at("qz")},
      .rate = vector_of(row, "p", "q", "r") * model::radian_per_second,
  };
}

// How far apart two bodies are: in position, velocity, attitude and rate.
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

// Flies `flown` from the trim at steps of `dt`, and returns the body every
// 0.2 s.
auto fly(Case flown, Duration dt, const model::AircraftData& data,
         const Row& trim) -> std::vector<RigidBody> {
  World world;
  REQUIRE(
      World::set_up().numbered(1).holding<archetype::RigidAircraft>(1).build(
          lib::Out(world)));
  model::Earth earth = model::Earth::round(model::wgs84::Geodetic{});
  RigidBody body = body_of(trim);

  FlightSignals signals;
  using enum model::FlightSignal;
  signals[ELEVATOR_COMMAND] = trim.at("elevator");
  signals[AILERON_COMMAND] = trim.at("aileron");
  signals[RUDDER_COMMAND] = trim.at("rudder");
  signals[PITCH_TRIM_COMMAND] = trim.at("pitch_trim");
  signals[ROLL_TRIM_COMMAND] = trim.at("roll_trim");
  signals[YAW_TRIM_COMMAND] = trim.at("yaw_trim");
  model::settle_flight_controls(data.flight_controls, signals);
  model::run_flight_controls(data.flight_controls, signals,
                             0.0 * model::second);
  EngineControls controls{
      .throttle = {trim.at("throttle_0"), trim.at("throttle_1")}};
  model::StandardAirTable air;
  Engines engines = model::settled_engines(
      data, controls,
      model::engine_air_of(body, earth, air, 0.0 * model::second));
  FuelTanks tanks;
  for (std::size_t i = 0; i < 3; ++i) {
    tanks.contents[i] = trim.at("fuel_" + std::to_string(i)) * model::kilogram;
  }
  auto aircraft = world.create<archetype::RigidAircraft>()
                      .with(earth.air_state(body, 0.0 * model::second))
                      .with(body)
                      .with(RigidBodyRate{})
                      .with(signals)
                      .with(ControlSurfaces{})
                      .with(controls)
                      .with(engines)
                      .with(tanks)
                      .with(model::mass_balance_of(data, tanks))
                      .with(AircraftType{.data = &data})
                      .build();
  REQUIRE(aircraft);
  world.sync();

  framework::Scheduler<World, CheckSchedule> scheduler{CheckSchedule{
      Pilot{flown, trim}, RunFlightControls{earth}, RunEngines{earth},
      Rigid{SystemList{RigidAircraftRates{earth}}}, BurnFuel{},
      FollowRigidBody{earth}}};
  std::vector<RigidBody> samples{body};
  constexpr Duration SAMPLE = 200ms;
  for (TimePoint time{}; time < TimePoint{30s}; time += dt) {
    scheduler.step(Step{.time = time, .dt = dt}, lib::InOut(world));
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
      bodies.push_back(body_of(row));
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

}  // namespace

TEST_CASE("CheckCases737") {
  auto data = model::load_aircraft(AIRCRAFT);
  REQUIRE(data);
  std::vector<Row> initial = load(INITIAL);
  REQUIRE(initial.size() == 1);
  std::vector<Row> rows = load(CASES);

  for (Case flown : {Case::HOLD, Case::ELEVATOR, Case::AILERON, Case::RUDDER,
                     Case::THROTTLE}) {
    std::vector<RigidBody> reference = jsbsim(rows, flown, 500);
    REQUIRE(reference.size() == 151);
    Apart physics =
        apart(fly(flown, std::chrono::microseconds{500}, *data, initial[0]),
              reference);
    Apart simon = apart(fly(flown, 8ms, *data, initial[0]), reference);
    Apart theirs = apart(jsbsim(rows, flown, 8000), reference);
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

}  // namespace simon::flight
