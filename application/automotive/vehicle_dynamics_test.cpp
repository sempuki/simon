// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <map>
#include <numbers>
#include <string>
#include <variant>
#include <vector>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "model/multibody.hpp"
#include "model/single_track.hpp"
#include "model/tire.hpp"

// simon's tire, dynamic and drift single-track models and multibody model
// against CommonRoad's, for its three vehicles: the tire's forces, each
// model's rates, and whole paths through a double lane change, braking and
// speeding up, from the tables reference/commonroad_dynamic.py recorded.
// The script records CommonRoad with four slips corrected; the tire's table
// also records CommonRoad's own forces.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;
using model::DriftSingleTrack;
using model::DynamicSingleTrack;
using model::MultibodyVehicle;
using model::VehicleInput;
using model::VehicleParameters;

using Numbers = std::vector<double>;

// Each model as numbers in CommonRoad's order, so that one test drives all
// three.
struct DynamicModel final {
  static auto numbers_of(const DynamicSingleTrack& s) -> Numbers {
    return {s.x.numerical_value_in(model::meter),
            s.y.numerical_value_in(model::meter),
            model::radians(s.steering),
            s.speed.numerical_value_in(model::meter_per_second),
            model::radians(s.heading),
            s.yaw_rate.numerical_value_in(model::radian_per_second),
            model::radians(s.slip_angle)};
  }
  static auto state_of(const Numbers& x) -> DynamicSingleTrack {
    return {.x = x[0] * model::meter,
            .y = x[1] * model::meter,
            .steering = x[2] * model::radian,
            .speed = x[3] * model::meter_per_second,
            .heading = x[4] * model::radian,
            .yaw_rate = x[5] * model::radian_per_second,
            .slip_angle = x[6] * model::radian};
  }
  static auto rate_numbers(const Numbers& x, const VehicleInput& input,
                           const VehicleParameters& vehicle) -> Numbers {
    model::DynamicSingleTrackRate f =
        model::compute_dynamic_single_track_rate(state_of(x), input, vehicle);
    return {f.x.numerical_value_in(model::meter_per_second),
            f.y.numerical_value_in(model::meter_per_second),
            f.steering.numerical_value_in(model::radian_per_second),
            f.speed.numerical_value_in(model::meter_per_second_squared),
            f.heading.numerical_value_in(model::radian_per_second),
            f.yaw_rate.numerical_value_in(model::radian_per_second_squared),
            f.slip_angle.numerical_value_in(model::radian_per_second)};
  }
  static auto start(const VehicleParameters&) -> Numbers {
    return numbers_of(
        DynamicSingleTrack{.speed = 20.0 * model::meter_per_second});
  }
};

struct DriftModel final {
  static auto numbers_of(const DriftSingleTrack& s) -> Numbers {
    return {s.x.numerical_value_in(model::meter),
            s.y.numerical_value_in(model::meter),
            model::radians(s.steering),
            s.speed.numerical_value_in(model::meter_per_second),
            model::radians(s.heading),
            s.yaw_rate.numerical_value_in(model::radian_per_second),
            model::radians(s.slip_angle),
            s.front_wheel.numerical_value_in(model::radian_per_second),
            s.rear_wheel.numerical_value_in(model::radian_per_second)};
  }
  static auto state_of(const Numbers& x) -> DriftSingleTrack {
    return {.x = x[0] * model::meter,
            .y = x[1] * model::meter,
            .steering = x[2] * model::radian,
            .speed = x[3] * model::meter_per_second,
            .heading = x[4] * model::radian,
            .yaw_rate = x[5] * model::radian_per_second,
            .slip_angle = x[6] * model::radian,
            .front_wheel = x[7] * model::radian_per_second,
            .rear_wheel = x[8] * model::radian_per_second};
  }
  static auto rate_numbers(const Numbers& x, const VehicleInput& input,
                           const VehicleParameters& vehicle) -> Numbers {
    model::DriftSingleTrackRate f =
        model::compute_drift_single_track_rate(state_of(x), input, vehicle);
    return {f.x.numerical_value_in(model::meter_per_second),
            f.y.numerical_value_in(model::meter_per_second),
            f.steering.numerical_value_in(model::radian_per_second),
            f.speed.numerical_value_in(model::meter_per_second_squared),
            f.heading.numerical_value_in(model::radian_per_second),
            f.yaw_rate.numerical_value_in(model::radian_per_second_squared),
            f.slip_angle.numerical_value_in(model::radian_per_second),
            f.front_wheel.numerical_value_in(model::radian_per_second_squared),
            f.rear_wheel.numerical_value_in(model::radian_per_second_squared)};
  }
  static auto start(const VehicleParameters& vehicle) -> Numbers {
    return numbers_of(model::start_drift_single_track(
        20.0 * model::meter_per_second, vehicle));
  }
};

struct MultibodyModel final {
  static auto state_of(const Numbers& x) -> MultibodyVehicle {
    model::MultibodyNumbers numbers{};
    std::ranges::copy(x, numbers.begin());
    return model::convert_numbers_to_multibody(numbers);
  }
  static auto rate_numbers(const Numbers& x, const VehicleInput& input,
                           const VehicleParameters& vehicle) -> Numbers {
    model::MultibodyNumbers f = model::convert_multibody_rate_to_numbers(
        model::compute_multibody_rate(state_of(x), input, vehicle));
    return {f.begin(), f.end()};
  }
  static auto start(const VehicleParameters& vehicle) -> Numbers {
    model::MultibodyNumbers x = model::convert_multibody_to_numbers(
        model::start_multibody(20.0 * model::meter_per_second, vehicle));
    return {x.begin(), x.end()};
  }
};

// The inputs the reference script drives by, at `t` seconds, each held over
// 0.01 s: two periods of a steering sine of 0.04 rad, and braking, a coast and
// speeding up.
auto inputs(double t) -> VehicleInput {
  t = std::floor(t / 0.01 + 1e-9) * 0.01;
  double steering_rate =
      t < 4.0 * std::numbers::pi / 2.5 ? 0.1 * std::cos(2.5 * t) : 0.0;
  double acceleration = 1.0 <= t && t < 2.5 ? -4.0 : t < 5.0 ? 0.0 : 2.0;
  return {.steering_rate = steering_rate * model::radian_per_second,
          .acceleration = acceleration * model::meter_per_second_squared};
}

// Drives a model from the script's start for 8 s at steps of `dt` by classic
// Runge-Kutta 4 on its numbers, the inputs held over each step, and returns
// the state every 0.1 s.
template <typename Model>
auto drive(const VehicleParameters& vehicle, std::chrono::microseconds dt)
    -> std::vector<Numbers> {
  double h = std::chrono::duration<double>(dt).count();
  Numbers x = Model::start(vehicle);
  std::vector<Numbers> path{x};
  auto steps = 8s / dt;
  auto every = 100ms / dt;
  auto step = [](const Numbers& from, const Numbers& rate, double by) {
    Numbers to = from;
    for (std::size_t i = 0; i < to.size(); ++i) {
      to[i] += by * rate[i];
    }
    return to;
  };
  for (std::int64_t k = 0; k < steps; ++k) {
    VehicleInput input = inputs(std::chrono::duration<double>(k * dt).count());
    auto f = [&](const Numbers& at) {
      return Model::rate_numbers(at, input, vehicle);
    };
    Numbers k1 = f(x);
    Numbers k2 = f(step(x, k1, 0.5 * h));
    Numbers k3 = f(step(x, k2, 0.5 * h));
    Numbers k4 = f(step(x, k3, h));
    for (std::size_t i = 0; i < x.size(); ++i) {
      x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    if ((k + 1) % every == 0) {
      path.push_back(x);
    }
  }
  return path;
}

// The largest difference in each model's rates, relative to the larger of 1
// and CommonRoad's rate, over its rows of the rates table.
template <typename Model>
auto compare_rates(std::string_view name,
                   const std::map<int, VehicleParameters>& vehicles,
                   const std::vector<std::vector<std::string>>& lines)
    -> std::pair<double, int> {
  double largest = 0.0;
  int rows = 0;
  for (const std::vector<std::string>& cells : lines) {
    if (cells[0] != name) {
      continue;
    }
    const VehicleParameters& vehicle =
        vehicles.at(static_cast<int>(parse_number(cells[1])));
    VehicleInput input{
        .steering_rate = parse_number(cells[2]) * model::radian_per_second,
        .acceleration =
            parse_number(cells[3]) * model::meter_per_second_squared};
    std::size_t n = (cells.size() - 4) / 2;
    Numbers x(n);
    Numbers theirs(n);
    for (std::size_t i = 0; i < n; ++i) {
      x[i] = parse_number(cells[4 + i]);
      theirs[i] = parse_number(cells[4 + n + i]);
    }
    Numbers ours = Model::rate_numbers(x, input, vehicle);
    REQUIRE(ours.size() == n);
    for (std::size_t i = 0; i < n; ++i) {
      largest = std::max(largest, std::abs(ours[i] - theirs[i]) /
                                      std::max(1.0, std::abs(theirs[i])));
    }
    ++rows;
  }
  return {largest, rows};
}

// How far simon's path is from CommonRoad's at the same step, and from
// CommonRoad's converged path, in position, as the largest over the path.
struct PathAgreement final {
  double same = 0.0;       // m.
  double converged = 0.0;  // m.
};

template <typename Model>
auto compare_paths(std::string_view name, int id,
                   const VehicleParameters& vehicle,
                   const std::vector<std::vector<std::string>>& lines)
    -> PathAgreement {
  std::vector<Numbers> theirs;
  std::vector<Numbers> converged;
  for (const std::vector<std::string>& cells : lines) {
    if (cells[0] != name || static_cast<int>(parse_number(cells[1])) != id) {
      continue;
    }
    Numbers x;
    for (std::size_t i = 4; i < cells.size(); ++i) {
      x.push_back(parse_number(cells[i]));
    }
    (parse_number(cells[2]) == 0.001 ? theirs : converged).push_back(x);
  }
  std::vector<Numbers> ours = drive<Model>(vehicle, 1ms);
  REQUIRE(ours.size() == theirs.size());
  REQUIRE(ours.size() == converged.size());
  PathAgreement agreement;
  for (std::size_t i = 0; i < ours.size(); ++i) {
    auto apart = [&](const Numbers& other) {
      return std::hypot(ours[i][0] - other[0], ours[i][1] - other[1]);
    };
    agreement.same = std::max(agreement.same, apart(theirs[i]));
    agreement.converged = std::max(agreement.converged, apart(converged[i]));
  }
  return agreement;
}

}  // namespace

TEST_CASE("TireAgainstCommonRoad") {
  model::CommonRoadTire tire =
      std::get<model::CommonRoadTire>(load_commonroad_vehicles().at(1).tire);

  SECTION("ShouldMatchCorrectedMagicFormulaGivenSlipsAndLoads") {
    // simon's forces agree with CommonRoad's, corrected, to rounding.
    // CommonRoad's own differ by hundreds of newtons through its two slips:
    // the side force longitudinal slip induces, turned the wrong way, and the
    // vertical shift F_z p_vx1, added inside the sine as an angle, which
    // brakes a tire rolling free.
    double largest = 0.0;
    double original_x = 0.0;
    double original_y = 0.0;
    std::size_t rows = 0;
    for (const Row& row : load_rows("commonroad_tires.csv")) {
      model::TireForce force = model::compute_tire_force(
          tire,
          {.longitudinal = number(row, "kappa"),
           .lateral = number(row, "alpha") * model::radian,
           .camber = number(row, "gamma") * model::radian},
          number(row, "load") * model::newton);
      double fx = force.longitudinal.numerical_value_in(model::newton);
      double fy = force.lateral.numerical_value_in(model::newton);
      largest = std::max({largest,
                          std::abs(fx - number(row, "fx")) /
                              std::max(1.0, std::abs(number(row, "fx"))),
                          std::abs(fy - number(row, "fy")) /
                              std::max(1.0, std::abs(number(row, "fy")))});
      original_x =
          std::max(original_x, std::abs(fx - number(row, "original_fx")));
      original_y =
          std::max(original_y, std::abs(fy - number(row, "original_fy")));
      ++rows;
    }
    CAPTURE(largest, original_x, original_y);
    CHECK(rows == 2000);
    CHECK(largest < 1e-13);
    CHECK(original_x > 500.0);
    CHECK(original_y > 500.0);
  }

  SECTION("ShouldPushNothingGivenNoLoad") {
    for (double load : {0.0, -500.0}) {
      model::TireForce force = model::compute_tire_force(
          tire, {.longitudinal = 0.1, .lateral = 0.05 * model::radian},
          load * model::newton);
      CHECK(force.longitudinal == 0.0 * model::newton);
      CHECK(force.lateral == 0.0 * model::newton);
    }
  }
}

TEST_CASE("VehicleDynamicsAgainstCommonRoad") {
  std::map<int, VehicleParameters> vehicles = load_commonroad_vehicles();
  REQUIRE(vehicles.size() == 3);

  SECTION("ShouldMatchRatesGivenStatesFromCrawlToFast") {
    std::vector<std::vector<std::string>> lines =
        load_cells("commonroad_dynamic_rates.csv");
    auto [dynamic, dynamic_rows] =
        compare_rates<DynamicModel>("st", vehicles, lines);
    auto [drift, drift_rows] =
        compare_rates<DriftModel>("std", vehicles, lines);
    auto [multibody, multibody_rows] =
        compare_rates<MultibodyModel>("mb", vehicles, lines);
    CAPTURE(dynamic, drift, multibody, multibody_rows);
    CHECK(dynamic_rows == 900);
    CHECK(drift_rows == 900);
    CHECK(multibody_rows > 500);
    CHECK(dynamic < 1e-12);
    CHECK(drift < 1e-12);
    CHECK(multibody < 1e-12);
  }

  SECTION("ShouldMatchPathsGivenRungeKutta4") {
    // The same Runge-Kutta 4 at 0.001 s agrees with CommonRoad's to rounding.
    // Against the path converged at 0.0001 s, the dynamic model is within
    // 0.04 nm, the drift model, whose wheels spin fast, 4 um, and the
    // multibody model, stiff in its tires and pins, 0.5 mm.
    std::vector<std::vector<std::string>> lines =
        load_cells("commonroad_dynamic_paths.csv");
    for (const auto& [id, vehicle] : vehicles) {
      CAPTURE(id);
      PathAgreement dynamic =
          compare_paths<DynamicModel>("st", id, vehicle, lines);
      PathAgreement drift =
          compare_paths<DriftModel>("std", id, vehicle, lines);
      PathAgreement multibody =
          compare_paths<MultibodyModel>("mb", id, vehicle, lines);
      CAPTURE(dynamic.same, dynamic.converged, drift.same, drift.converged,
              multibody.same, multibody.converged);
      CHECK(dynamic.same < 1e-12);
      CHECK(drift.same < 1e-12);
      CHECK(multibody.same < 1e-12);
      CHECK(dynamic.converged < 1e-9);
      CHECK(drift.converged < 1e-5);
      CHECK(multibody.converged < 0.001);
    }
  }
}

}  // namespace simon::automotive
