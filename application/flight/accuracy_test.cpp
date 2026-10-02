// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <expected>
#include <numbers>
#include <type_traits>
#include <utility>
#include <vector>

#include "application/flight/components.hpp"
#include "application/flight/systems.hpp"
#include "application/flight/testing.hpp"
#include "base/testing.hpp"
#include "framework/vocabulary.hpp"

// Measures the drift of simon's point-mass model from JSBSim's 737 flying the
// same maneuvers. reference/jsbsim_737.py flew them and recorded the controls a
// point mass needs to follow its path; here Fly and Precise replay those
// controls, and the paths are compared. Both see the same controls, so the
// rest is the model's own error, mostly its drag polar, and the
// integrator's.
namespace simon::flight {

namespace {

using namespace std::chrono_literals;

constexpr char REFERENCE[] = "application/flight/reference/jsbsim_737.csv";
constexpr Duration SAMPLE = 200ms;  // Between the reference's rows.

// The 737, as jsbsim_737.py fitted it.
constexpr Airframe BOEING_737{.mass = 48534.3 * model::kilogram,
                              .wing_area = 108.789 * model::square_meter,
                              .zero_lift_drag = 0.02238,
                              .induced_drag = 0.0807,
                              .thrust = 200000.0 * model::newton};

// One row of the reference: the controls at a time, and where the 737 was.
struct Sample final {
  double time = 0.0;  // Seconds.
  double load_factor = 1.0;
  double bank = 0.0;
  double throttle = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double speed = 0.0;
  double flight_path_angle = 0.0;
  double heading = 0.0;

  auto state() const -> AirState {
    return AirState{.position = model::meters(x, y, z),
                    .speed = speed * model::meter_per_second,
                    .flight_path_angle = flight_path_angle * model::radian,
                    .heading = heading * model::radian};
  }
};

auto load_reference() -> std::vector<Sample> {
  std::vector<Sample> samples;
  for (const testing::Row& row : testing::load_rows(REFERENCE)) {
    samples.push_back(Sample{.time = row.at("time"),
                             .load_factor = row.at("load_factor"),
                             .bank = row.at("bank"),
                             .throttle = row.at("throttle"),
                             .x = row.at("x"),
                             .y = row.at("y"),
                             .z = row.at("z"),
                             .speed = row.at("speed"),
                             .flight_path_angle = row.at("flight_path_angle"),
                             .heading = row.at("heading")});
  }
  return samples;
}

// Sets every aircraft's controls to the reference's at the step's start,
// interpolated between rows.
struct Replay final  //
    : System<FlightControls> {
  using SystemWorld = ProjectedWorld<Replay>;

  explicit Replay(const std::vector<Sample>& samples) : samples_{&samples} {}

  auto operator()(SystemWorld&, Entity,      //
                  FlightControls& controls,  //
                  Step step) const -> void {
    double position = std::clamp(
        std::chrono::duration<double>(step.time.time_since_epoch()) / SAMPLE,
        0.0, static_cast<double>(samples_->size() - 1));
    auto i = std::min(static_cast<std::size_t>(position), samples_->size() - 2);
    double weight = position - static_cast<double>(i);
    const Sample& low = (*samples_)[i];
    const Sample& high = (*samples_)[i + 1];
    auto between = [&](double a, double b) { return a + (b - a) * weight; };

    controls = FlightControls{
        .load_factor = between(low.load_factor, high.load_factor),
        .bank = between(low.bank, high.bank) * model::radian,
        .throttle = between(low.throttle, high.throttle)};
  }

 private:
  const std::vector<Sample>* samples_;
};

// The largest differences from the reference over a flight.
struct Drift final {
  double position = 0.0;  // Meters.
  double altitude = 0.0;  // Meters.
  double speed = 0.0;     // Meters per second.
  double heading = 0.0;   // Radians.

  auto add(const AirState& state, const Sample& sample) -> void {
    AirState expected = sample.state();
    position = std::max(
        position,
        model::distance(state, expected).numerical_value_in(model::meter));
    altitude = std::max(
        altitude,
        std::abs((model::altitude_of(state) - model::altitude_of(expected))
                     .numerical_value_in(model::meter)));
    speed = std::max(
        speed, std::abs((state.speed - expected.speed)
                            .numerical_value_in(model::meter_per_second)));
    heading = std::max(
        heading,
        std::abs(std::remainder(model::radians(state.heading) - sample.heading,
                                2.0 * std::numbers::pi)));
  }
};

struct Drifts final {
  Drift simple;
  Drift precise;
};

template <typename ArchetypeType>
auto create(const AirState& state, InOut<World> world) -> Entity {
  auto builder = world->create<ArchetypeType>()
                     .with(state)
                     .with(FlightControls{})
                     .with(Commands{})
                     .with(BOEING_737)
                     .with(Handling{})
                     .with(Autopilot{})
                     .with(Route{});
  std::expected<Entity, framework::Status> entity;
  if constexpr (std::is_same_v<ArchetypeType, archetype::PreciseAircraft>) {
    entity = std::move(builder).with(AirStateRate{}).build();
  } else {
    entity = std::move(builder).build();
  }
  REQUIRE(entity);
  return *entity;
}

// Replays the reference at steps of `dt` through Fly and Precise at once.
auto replay(const std::vector<Sample>& samples, Duration dt) -> Drifts {
  World world;
  REQUIRE(World::set_up()
              .numbered(1)
              .holding<archetype::Aircraft>(1)
              .holding<archetype::PreciseAircraft>(1)
              .build(Out(world)));
  Entity simple =
      create<archetype::Aircraft>(samples.front().state(), InOut(world));
  Entity precise =
      create<archetype::PreciseAircraft>(samples.front().state(), InOut(world));
  world.sync();

  framework::Scheduler<World, SystemList<Replay, Fly, Precise>> scheduler{
      SystemList{Replay{samples}, Fly{}, Precise{}}};
  Drifts drifts;
  TimePoint end{SAMPLE * (samples.size() - 1)};
  for (TimePoint time{}; time < end; time += dt) {
    scheduler.step(Step{.time = time, .dt = dt}, InOut(world));
    Duration done = (time + dt).time_since_epoch();
    if (done % SAMPLE == Duration::zero()) {
      const Sample& sample = samples[static_cast<std::size_t>(done / SAMPLE)];
      drifts.simple.add(world.store_of<AirState>().component_of(simple),
                        sample);
      drifts.precise.add(world.store_of<AirState>().component_of(precise),
                         sample);
    }
  }
  return drifts;
}

}  // namespace

TEST_CASE("Accuracy") {
  std::vector<Sample> samples = load_reference();
  REQUIRE(samples.size() > 3000);  // 640 s.

  // The drag polar is most of the drift: about 3 m/s of speed by the end,
  // which turns into 2 km over 130 km flown. Integration adds little at a
  // small step.
  SECTION("ShouldFollowJsbsimGivenSmallStep") {
    Drifts drifts = replay(samples, 20ms);

    for (const Drift& drift : {drifts.simple, drifts.precise}) {
      CAPTURE(drift.position, drift.altitude, drift.speed, drift.heading);
      CHECK(drift.position < 2500.0);
      CHECK(drift.altitude < 70.0);
      CHECK(drift.speed < 4.5);
      CHECK(drift.heading < 0.03);
    }
  }

  // At a one-second step the single pass drifts further, mostly in
  // altitude; Runge-Kutta 4 barely changes.
  SECTION("ShouldFollowJsbsimGivenLargeStep") {
    Drifts drifts = replay(samples, 1s);

    CAPTURE(drifts.simple.position, drifts.simple.altitude, drifts.simple.speed,
            drifts.simple.heading);
    CAPTURE(drifts.precise.position, drifts.precise.altitude,
            drifts.precise.speed, drifts.precise.heading);
    CHECK(drifts.simple.position < 3000.0);
    CHECK(drifts.simple.altitude < 130.0);
    CHECK(drifts.precise.position < 2500.0);
    CHECK(drifts.precise.altitude < 80.0);
    CHECK(drifts.precise.altitude < drifts.simple.altitude);
  }
}

}  // namespace simon::flight
