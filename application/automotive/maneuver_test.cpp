// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <numbers>
#include <string>
#include <vector>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "format/tire_file.hpp"
#include "model/multibody.hpp"
#include "model/single_track.hpp"

// simon's drift single-track model against Project Chrono's Sedan through
// the handling maneuvers, from the table reference/chrono_reference.cpp
// recorded. simon drives the Sedan's mass, inertia, axle positions and Magic
// Formula tire, steered by Chrono's front wheels' mean angle and held to
// Chrono's speed, and both are measured as the standards measure them.
namespace simon::automotive {

namespace {

using namespace testing;
using model::DriftSingleTrack;
using model::VehicleParameters;

constexpr double GRAVITY = 9.81;
constexpr double SAMPLE = 0.01;  // s, Chrono's samples.
constexpr double SETTLE = 5.0;   // s, when each maneuver steers.

// One sample of a vehicle's motion.
struct Sample final {
  double time = 0.0;      // s.
  double steering = 0.0;  // rad, the front wheels' mean.
  double speed = 0.0;     // m/s, along x.
  double lateral = 0.0;   // m/s, along y.
  double yaw_rate = 0.0;  // rad/s.
  double x = 0.0;         // m.
  double y = 0.0;         // m.
  // Each wheel's steer, left front, right front, left rear, right rear.
  std::array<double, 4> wheels{};  // rad.
};

auto load_maneuvers() -> std::map<std::string, std::vector<Sample>> {
  std::map<std::string, std::vector<Sample>> by_name;
  for (const Row& row : load_rows("chrono_maneuvers.csv")) {
    by_name[row.find("maneuver")->second].push_back(Sample{
        .time = number(row, "time"),
        .steering =
            0.5 * (number(row, "left_steer") + number(row, "right_steer")),
        .speed = number(row, "vx"),
        .lateral = number(row, "vy"),
        .yaw_rate = number(row, "yaw_rate"),
        .x = number(row, "x"),
        .y = number(row, "y"),
        .wheels = {number(row, "steer_lf"), number(row, "steer_rf"),
                   number(row, "steer_lr"), number(row, "steer_rr")},
    });
  }
  return by_name;
}

// The Sedan as the drift model reads it: its mass and yaw inertia, its axles
// about its center of mass, and the height of its center of mass, from
// Chrono's assembly at rest; its wheels' spin inertia, the wheel's, the
// tire's and the axle's, front and rear averaged; its brakes, even front to
// rear, and its front-wheel drive; and its Magic Formula tire. Its steering
// and acceleration are left unbounded, as Chrono steers the wheels.
auto load_sedan() -> VehicleParameters {
  std::map<std::string, double, std::less<>> value;
  for (const Row& row : load_rows("chrono_sedan.csv")) {
    value[row.find("name")->second] = number(row, "value");
  }
  auto tire =
      format::load_tire_file(std::string{TIRES} + "Sedan_Pac02Tire.tir");
  REQUIRE(tire.has_value());
  // Chrono's Pac02 holds camber at zero whatever the wheel's lean, so the
  // tire here has no camber terms.
  for (double model::MagicFormulaTire::* camber :
       {&model::MagicFormulaTire::pdx3, &model::MagicFormulaTire::pdy3,
        &model::MagicFormulaTire::pey4, &model::MagicFormulaTire::pky3,
        &model::MagicFormulaTire::phy3, &model::MagicFormulaTire::pvy3,
        &model::MagicFormulaTire::pvy4, &model::MagicFormulaTire::rvy3,
        &model::MagicFormulaTire::qbz4, &model::MagicFormulaTire::qbz5,
        &model::MagicFormulaTire::qdz3, &model::MagicFormulaTire::qdz4,
        &model::MagicFormulaTire::qdz8, &model::MagicFormulaTire::qdz9,
        &model::MagicFormulaTire::qhz3, &model::MagicFormulaTire::qhz4,
        &model::MagicFormulaTire::qez5, &model::MagicFormulaTire::ssz3,
        &model::MagicFormulaTire::ssz4}) {
    (*tire).*camber = 0.0;
  }
  double com = value.at("com_x");
  VehicleParameters sedan{
      .front = (value.at("front_left_x") - com) * model::meter,
      .rear = (com - value.at("rear_left_x")) * model::meter,
      .sprung_height = 0.411 * model::meter,
      .wheel_radius = value.at("wheel_radius") * model::meter,
      .mass = value.at("mass") * model::kilogram,
      .yaw_inertia = value.at("inertia_zz") * model::kilogram_square_meter,
      .wheel_inertia = 0.5 * ((0.42 + 0.679 + 0.4) + (0.42 + 0.679 + 0.166)) *
                       model::kilogram_square_meter,
      .front_brake_share = 0.5,
      .front_drive_share = 1.0,
      .tire = *tire,
  };
  return sedan;
}

// The Sedan as the multibody model reads it, beyond the drift model's: the
// sprung body, Chrono's chassis, its center of gravity midway between the
// axles and 0.419 m up at rest; the unsprung masses, each corner's upright,
// spindle, wheel, tire and half its arms; the rest of the mass, the
// steering and driveline, sprung. The suspension is measured from Chrono's
// at rest and on the ramp at 80 km/h (see reference/chrono_reference.cpp):
// each corner's rate and the dampers through the same motion ratio; each
// axle's roll stiffness beyond its springs, less the tires' vertical
// compliance; and each axle's roll center, from the load it transfers
// beyond its roll stiffness's share.
auto load_sedan_multibody() -> VehicleParameters {
  VehicleParameters sedan = load_sedan();
  std::map<std::string, double, std::less<>> value;
  for (const Row& row : load_rows("chrono_sedan.csv")) {
    value[row.find("name")->second] = number(row, "value");
  }
  sedan.front = 1.388 * model::meter;
  sedan.rear = 1.388 * model::meter;
  sedan.front_track = 2.0 * value.at("front_left_y") * model::meter;
  sedan.rear_track = 2.0 * value.at("rear_left_y") * model::meter;
  sedan.sprung_height = 0.419 * model::meter;
  sedan.front_unsprung_mass = 2.0 * 28.0 * model::kilogram;
  sedan.rear_unsprung_mass = 2.0 * 39.9 * model::kilogram;
  sedan.sprung_mass =
      sedan.mass - sedan.front_unsprung_mass - sedan.rear_unsprung_mass;
  sedan.roll_inertia = 222.8 * model::kilogram_square_meter;
  sedan.pitch_inertia = 944.1 * model::kilogram_square_meter;
  sedan.roll_yaw_product = 0.0 * model::kilogram_square_meter;
  sedan.front_unsprung_roll_inertia =
      56.0 * 0.8 * 0.8 * model::kilogram_square_meter;
  sedan.rear_unsprung_roll_inertia =
      79.8 * 0.8 * 0.8 * model::kilogram_square_meter;
  sedan.suspension = {
      .front_spring = 16300.0 * model::newton_per_meter,
      .front_damping = 3336.0 * model::newton_second_per_meter,
      .rear_spring = 61000.0 * model::newton_per_meter,
      .rear_damping = 5477.0 * model::newton_second_per_meter,
      .front_roll_stiffness = 5900.0 * model::newton_meter_per_radian,
      .rear_roll_stiffness = 2200.0 * model::newton_meter_per_radian,
      .tire_spring = 280835.2941 * model::newton_per_meter,
      .tire_compliance = 0.0 * model::meter_per_newton,
      .front_camber = 0.0 * model::radian_per_meter,
      .rear_camber = 0.0 * model::radian_per_meter,
  };
  sedan.front_roll_axis_height = 0.124 * model::meter;
  sedan.rear_roll_axis_height = 0.02 * model::meter;
  sedan.steering = {.min = -1.0 * model::radian,
                    .max = 1.0 * model::radian,
                    .min_rate = -100.0 * model::radian_per_second,
                    .max_rate = 100.0 * model::radian_per_second};
  sedan.longitudinal = {
      .max_acceleration = 100.0 * model::meter_per_second_squared,
      .switch_speed = 100.0 * model::meter_per_second,
      .min_speed = -100.0 * model::meter_per_second,
      .max_speed = 100.0 * model::meter_per_second};
  return sedan;
}

// The drift model and the multibody model, each started from Chrono's state
// at the start of a maneuver, and read back as samples.
struct DriftModel final {
  using State = DriftSingleTrack;

  static auto start(const Sample& at, const VehicleParameters& vehicle)
      -> State {
    State state = model::start_drift_single_track(
        std::hypot(at.speed, at.lateral) * model::meter_per_second, vehicle);
    state.steering = at.steering * model::radian;
    state.yaw_rate = at.yaw_rate * model::radian_per_second;
    state.slip_angle = std::atan2(at.lateral, at.speed) * model::radian;
    return state;
  }
  static auto compute_rate(const State& state, const model::VehicleInput& input,
                           const VehicleParameters& vehicle, const Sample&) {
    return model::compute_drift_single_track_rate(state, input, vehicle);
  }
  static auto steering_of(const Sample& sample) -> double {
    return sample.steering;
  }
  static auto read(const State& state, double time) -> Sample {
    double v = state.speed.numerical_value_in(model::meter_per_second);
    double beta = model::radians(state.slip_angle);
    return Sample{
        .time = time,
        .steering = model::radians(state.steering),
        .speed = v * std::cos(beta),
        .lateral = v * std::sin(beta),
        .yaw_rate = state.yaw_rate.numerical_value_in(model::radian_per_second),
        .x = state.x.numerical_value_in(model::meter),
        .y = state.y.numerical_value_in(model::meter)};
  }
};

// The multibody model, whose axes are SAE's, y right and z down: Chrono's
// steering, yaw and lateral motion are turned on the way in and back on the
// way out. With `TOE`, each wheel is also steered as Chrono's suspension
// steers it, beyond the front wheels' mean.
template <bool TOE>
struct MultibodyModel final {
  using State = model::MultibodyVehicle;

  static auto start(const Sample& at, const VehicleParameters& vehicle)
      -> State {
    State state =
        model::start_multibody(at.speed * model::meter_per_second, vehicle);
    state.steering = -at.steering * model::radian;
    state.yaw_rate = -at.yaw_rate * model::radian_per_second;
    state.body.lateral_speed = -at.lateral * model::meter_per_second;
    return state;
  }
  static auto compute_rate(const State& state, const model::VehicleInput& input,
                           const VehicleParameters& vehicle,
                           const Sample& chrono) {
    model::WheelSteer toe;
    if (TOE) {
      toe = {
          .left_front = -(chrono.wheels[0] - chrono.steering) * model::radian,
          .right_front = -(chrono.wheels[1] - chrono.steering) * model::radian,
          .left_rear = -chrono.wheels[2] * model::radian,
          .right_rear = -chrono.wheels[3] * model::radian};
    }
    return model::compute_multibody_rate(state, input, vehicle, toe);
  }
  static auto steering_of(const Sample& sample) -> double {
    return -sample.steering;
  }
  static auto read(const State& state, double time) -> Sample {
    return Sample{
        .time = time,
        .steering = -model::radians(state.steering),
        .speed = state.speed.numerical_value_in(model::meter_per_second),
        .lateral = -state.body.lateral_speed.numerical_value_in(
            model::meter_per_second),
        .yaw_rate =
            -state.yaw_rate.numerical_value_in(model::radian_per_second),
        .x = state.x.numerical_value_in(model::meter),
        .y = -state.y.numerical_value_in(model::meter)};
  }
};

// Drives a model through Chrono's maneuver from `start` seconds: steered at
// the rate Chrono's front wheels turn over each sample, and held to
// Chrono's speed along x by its acceleration and a gain of 2 per second, by
// classic Runge-Kutta 4 at 1 ms.
template <typename Model>
auto drive(const VehicleParameters& vehicle, const std::vector<Sample>& chrono,
           double start) -> std::vector<Sample> {
  constexpr int SUBSTEPS = 10;
  auto first = static_cast<std::size_t>(std::lround(start / SAMPLE));
  typename Model::State state = Model::start(chrono[first], vehicle);
  std::vector<Sample> path{Model::read(state, chrono[first].time)};
  auto dt = framework::Duration{std::chrono::microseconds{1000}};
  for (std::size_t i = first; i + 1 < chrono.size(); ++i) {
    const Sample& now = chrono[i];
    const Sample& next = chrono[i + 1];
    double steering_rate =
        (Model::steering_of(next) - Model::steering_of(now)) / SAMPLE;
    double wanted = (next.speed - now.speed) / SAMPLE;
    for (int k = 0; k < SUBSTEPS; ++k) {
      double ours = Model::read(state, 0.0).speed;
      double target = now.speed + (next.speed - now.speed) * k / SUBSTEPS;
      model::VehicleInput input{
          .steering_rate = steering_rate * model::radian_per_second,
          .acceleration = (wanted + 2.0 * (target - ours)) *
                          model::meter_per_second_squared};
      auto rate = [&](const typename Model::State& at) {
        return Model::compute_rate(at, input, vehicle, now);
      };
      auto k1 = rate(state);
      auto k2 = rate(model::advance(state, 0.5 * k1, dt));
      auto k3 = rate(model::advance(state, 0.5 * k2, dt));
      auto k4 = rate(model::advance(state, k3, dt));
      state = model::advance(state,
                             (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4), dt);
    }
    path.push_back(Model::read(state, next.time));
  }
  return path;
}

// The understeer gradient, in degrees per g: the slope of the steering
// against the lateral acceleration v r, less the kinematic L / v^2, fitted
// by least squares from 1 to 4 m/s^2 (ISO 4138).
auto compute_understeer(const std::vector<Sample>& path, double wheelbase)
    -> double {
  double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, speed = 0;
  for (const Sample& s : path) {
    double ay = s.speed * s.yaw_rate;
    if (s.time < SETTLE || ay < 1.0 || ay > 4.0) {
      continue;
    }
    n += 1;
    sx += ay;
    sy += s.steering;
    sxx += ay * ay;
    sxy += ay * s.steering;
    speed += s.speed;
  }
  REQUIRE(n > 10);
  double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
  speed /= n;
  return (slope - wheelbase / (speed * speed)) * GRAVITY * 180.0 /
         std::numbers::pi;
}

// A step steer's steady yaw rate over its last second, the time from half
// the steering to 90% of that yaw rate, and the overshoot past it (ISO
// 7401).
struct StepResponse final {
  double yaw_rate = 0.0;       // rad/s.
  double response_time = 0.0;  // s.
  double overshoot = 0.0;      // Of the steady yaw rate.
};

auto compute_step_response(const std::vector<Sample>& path) -> StepResponse {
  double end = path.back().time;
  double steady = 0.0;
  double final_steering = 0.0;
  int count = 0;
  for (const Sample& s : path) {
    if (s.time > end - 1.0) {
      steady += s.yaw_rate;
      final_steering += s.steering;
      ++count;
    }
  }
  steady /= count;
  final_steering /= count;
  double half = 0.0;
  double reached = 0.0;
  double peak = 0.0;
  for (const Sample& s : path) {
    if (half == 0.0 && s.steering >= 0.5 * final_steering) {
      half = s.time;
    }
    if (reached == 0.0 && s.yaw_rate >= 0.9 * steady) {
      reached = s.time;
    }
    peak = std::max(peak, s.yaw_rate);
  }
  return {.yaw_rate = steady,
          .response_time = reached - half,
          .overshoot = peak / steady - 1.0};
}

// A sine with dwell's yaw rate 1 s and 1.75 s after the steering completes,
// as ratios of the peak yaw rate after the steering reverses, and the
// lateral displacement 1.07 s after it begins (FMVSS 126).
struct DwellResponse final {
  double ratio_1 = 0.0;
  double ratio_175 = 0.0;
  double displacement = 0.0;  // m.
};

auto compute_dwell_response(const std::vector<Sample>& path) -> DwellResponse {
  double begin = SETTLE;
  double complete = SETTLE + 1.0 / 0.7 + 0.5;
  double peak = 0.0;
  for (const Sample& s : path) {
    if (s.time > begin + 0.75 / 0.7 * 0.5 && s.time < complete + 0.5) {
      peak = std::min(peak, s.yaw_rate);
    }
  }
  auto at = [&](double time) {
    return *std::ranges::min_element(
        path, {}, [&](const Sample& s) { return std::abs(s.time - time); });
  };
  const Sample& start = at(begin);
  const Sample& later = at(begin + 1.07);
  return {.ratio_1 = at(complete + 1.0).yaw_rate / peak,
          .ratio_175 = at(complete + 1.75).yaw_rate / peak,
          .displacement = later.y - start.y};
}

// Chrono's metrics and a model's, through every maneuver.
struct Comparison final {
  std::array<double, 2> understeer{};  // At 80 and 100 km/h, deg/g.
  StepResponse step;
  DwellResponse dwell;  // At 2.5 times the input for 0.3 g.
  bool spun = false;    // At 5 times, past 45 degrees of slip.
};

auto compute_spin(const std::vector<Sample>& path) -> bool {
  return std::ranges::any_of(path, [](const Sample& s) {
    return std::abs(std::atan2(s.lateral, s.speed)) > std::numbers::pi / 4.0;
  });
}

auto measure(const std::map<std::string, std::vector<Sample>>& paths,
             double wheelbase) -> Comparison {
  return {.understeer = {compute_understeer(paths.at("ramp80"), wheelbase),
                         compute_understeer(paths.at("ramp100"), wheelbase)},
          .step = compute_step_response(paths.at("step")),
          .dwell = compute_dwell_response(paths.at("dwell25")),
          .spun = compute_spin(paths.at("dwell50"))};
}

template <typename Model>
auto drive_all(const VehicleParameters& vehicle,
               const std::map<std::string, std::vector<Sample>>& chrono)
    -> std::map<std::string, std::vector<Sample>> {
  std::map<std::string, std::vector<Sample>> paths;
  for (std::string name : {"ramp80", "ramp100", "step", "dwell25", "dwell50"}) {
    paths[name] = drive<Model>(vehicle, chrono.at(name), SETTLE - 0.1);
  }
  return paths;
}

auto capture(const Comparison& theirs, const Comparison& ours) -> void {
  UNSCOPED_INFO("understeer at 80 km/h " << theirs.understeer[0] << " / "
                                         << ours.understeer[0]);
  UNSCOPED_INFO("understeer at 100 km/h " << theirs.understeer[1] << " / "
                                          << ours.understeer[1]);
  UNSCOPED_INFO("step yaw rate "
                << theirs.step.yaw_rate << " / " << ours.step.yaw_rate
                << ", response time " << theirs.step.response_time << " / "
                << ours.step.response_time << ", overshoot "
                << theirs.step.overshoot << " / " << ours.step.overshoot);
  UNSCOPED_INFO("dwell displacement "
                << theirs.dwell.displacement << " / " << ours.dwell.displacement
                << ", yaw ratios " << theirs.dwell.ratio_1 << ", "
                << theirs.dwell.ratio_175 << " / " << ours.dwell.ratio_1 << ", "
                << ours.dwell.ratio_175);
}

}  // namespace

TEST_CASE("ManeuversAgainstChronoSedan") {
  std::map<std::string, std::vector<Sample>> chrono = load_maneuvers();
  VehicleParameters sedan = load_sedan();
  VehicleParameters multibody = load_sedan_multibody();
  Comparison theirs =
      measure(chrono, sedan.wheelbase().numerical_value_in(model::meter));

  SECTION("ShouldMatchGivenMultibodySteeredAsChronoSteersEachWheel") {
    Comparison ours =
        measure(drive_all<MultibodyModel<true>>(multibody, chrono),
                multibody.wheelbase().numerical_value_in(model::meter));
    capture(theirs, ours);
    CHECK(std::abs(ours.understeer[0] - theirs.understeer[0]) < 0.02);
    CHECK(std::abs(ours.understeer[1] - theirs.understeer[1]) < 0.05);
    CHECK(std::abs(ours.step.yaw_rate / theirs.step.yaw_rate - 1.0) < 0.005);
    CHECK(std::abs(ours.step.response_time - theirs.step.response_time) <
          0.015);
    CHECK(std::abs(ours.step.overshoot - theirs.step.overshoot) < 0.01);
    CHECK(std::abs(ours.dwell.displacement / theirs.dwell.displacement - 1.0) <
          0.1);
    CHECK(theirs.spun);
    CHECK(ours.spun);
  }

  SECTION("ShouldMatchGivenMultibodySteeredByTheMean") {
    Comparison ours =
        measure(drive_all<MultibodyModel<false>>(multibody, chrono),
                multibody.wheelbase().numerical_value_in(model::meter));
    capture(theirs, ours);
    CHECK(std::abs(ours.understeer[0] - theirs.understeer[0]) < 0.04);
    CHECK(std::abs(ours.understeer[1] - theirs.understeer[1]) < 0.06);
    CHECK(std::abs(ours.step.yaw_rate / theirs.step.yaw_rate - 1.0) < 0.01);
    CHECK(std::abs(ours.step.overshoot - theirs.step.overshoot) < 0.02);
    CHECK(std::abs(ours.dwell.displacement / theirs.dwell.displacement - 1.0) <
          0.1);
  }

  SECTION("ShouldMatchGivenDriftSingleTrack") {
    Comparison ours =
        measure(drive_all<DriftModel>(sedan, chrono),
                sedan.wheelbase().numerical_value_in(model::meter));
    capture(theirs, ours);
    CHECK(std::abs(ours.understeer[0] - theirs.understeer[0]) < 0.04);
    CHECK(std::abs(ours.understeer[1] - theirs.understeer[1]) < 0.07);
    CHECK(std::abs(ours.step.yaw_rate / theirs.step.yaw_rate - 1.0) < 0.02);
    CHECK(std::abs(ours.dwell.displacement / theirs.dwell.displacement - 1.0) <
          0.15);
  }
}

}  // namespace simon::automotive
