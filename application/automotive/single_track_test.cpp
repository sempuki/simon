// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "model/vehicle/single_track.hpp"

// simon's kinematic single-track model against CommonRoad's, for its three
// vehicles: the rates at states and inputs on and past every limit, and whole
// paths through a steering sine and acceleration steps, from the tables
// reference/commonroad_kinematic.py recorded.
namespace simon::automotive {

namespace {

using namespace std::chrono_literals;
using namespace testing;
using model::KinematicSingleTrack;
using model::KinematicSingleTrackRate;
using model::VehicleInput;
using model::VehicleParameters;

auto read_state(const Row& row) -> KinematicSingleTrack {
  return {.x = number(row, "x") * meter,
          .y = number(row, "y") * meter,
          .steering = number(row, "steering") * radian,
          .speed = number(row, "v") * meter_per_second,
          .heading = number(row, "heading") * radian};
}

// The inputs the reference script drives by, at `t` seconds, each held over
// 0.05 s.
auto inputs(double t) -> VehicleInput {
  t = std::floor(t / 0.05 + 1e-9) * 0.05;
  double acceleration = t < 5.0    ? 3.0
                        : t < 9.0  ? -1.5
                        : t < 11.0 ? 14.0
                        : t < 13.0 ? -12.0
                                   : 0.5;
  return {.steering_rate = 0.05 * std::sin(0.8 * t) * radian_per_second,
          .acceleration = acceleration * meter_per_second_squared};
}

// The largest difference between two states, in each part.
struct Apart final {
  double position = 0.0;  // m.
  double steering = 0.0;  // rad.
  double speed = 0.0;     // m/s.
  double heading = 0.0;   // rad.

  auto widen(const KinematicSingleTrack& a, const KinematicSingleTrack& b)
      -> void {
    position =
        std::max(position, std::hypot((a.x - b.x).numerical_value_in(meter),
                                      (a.y - b.y).numerical_value_in(meter)));
    steering = std::max(
        steering,
        std::abs((a.steering - b.steering).numerical_value_in(radian)));
    speed = std::max(
        speed,
        std::abs((a.speed - b.speed).numerical_value_in(meter_per_second)));
    heading = std::max(
        heading, std::abs((a.heading - b.heading).numerical_value_in(radian)));
  }
};

// Drives `vehicle` from the script's start for 20 s at steps of `dt` seconds
// by classic Runge-Kutta 4, the inputs held over each step, and returns the
// state every 0.1 s.
auto drive(const VehicleParameters& vehicle, std::chrono::microseconds dt)
    -> std::vector<KinematicSingleTrack> {
  KinematicSingleTrack state{.speed = 10.0 * meter_per_second,
                             .heading = 0.3 * radian};
  std::vector<KinematicSingleTrack> path{state};
  auto steps = 20s / dt;
  auto every = std::chrono::microseconds{100'000} / dt;
  for (std::int64_t k = 0; k < steps; ++k) {
    VehicleInput input = inputs(std::chrono::duration<double>(k * dt).count());
    auto rate = [&](const KinematicSingleTrack& at) {
      return model::compute_kinematic_single_track_rate(at, input, vehicle);
    };
    KinematicSingleTrackRate k1 = rate(state);
    KinematicSingleTrackRate k2 = rate(model::advance(state, 0.5 * k1, dt));
    KinematicSingleTrackRate k3 = rate(model::advance(state, 0.5 * k2, dt));
    KinematicSingleTrackRate k4 = rate(model::advance(state, k3, dt));
    state = model::advance(state, (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4),
                           dt);
    if ((k + 1) % every == 0) {
      path.push_back(state);
    }
  }
  return path;
}

}  // namespace

TEST_CASE("KinematicSingleTrackAgainstCommonRoad") {
  std::map<int, VehicleParameters> by_id = load_commonroad_vehicles();
  REQUIRE(by_id.size() == 3);

  SECTION("ShouldMatchRatesGivenStatesOnAndPastLimits") {
    double largest = 0.0;
    int rows = 0;
    for (const Row& row : load_rows("commonroad_rates.csv")) {
      const VehicleParameters& vehicle =
          by_id.at(static_cast<int>(number(row, "vehicle")));
      KinematicSingleTrackRate rate =
          model::compute_kinematic_single_track_rate(
              read_state(row),
              {.steering_rate =
                   number(row, "steering_rate") * radian_per_second,
               .acceleration =
                   number(row, "acceleration") * meter_per_second_squared},
              vehicle);
      for (auto [ours, theirs] :
           {std::pair{rate.x.numerical_value_in(meter_per_second),
                      number(row, "dx")},
            {rate.y.numerical_value_in(meter_per_second), number(row, "dy")},
            {rate.steering.numerical_value_in(radian_per_second),
             number(row, "dsteering")},
            {rate.speed.numerical_value_in(meter_per_second_squared),
             number(row, "dv")},
            {rate.heading.numerical_value_in(radian_per_second),
             number(row, "dheading")}}) {
        largest = std::max(
            largest, std::abs(ours - theirs) / std::max(1.0, std::abs(theirs)));
      }
      ++rows;
    }
    CAPTURE(largest);
    CHECK(rows == 1200);
    CHECK(largest < 1e-15);
  }

  SECTION("ShouldMatchPathsGivenRungeKutta4") {
    // The same Runge-Kutta 4 at 0.05 s agrees with CommonRoad's to rounding,
    // and is within millimeters of the path converged at 0.0005 s.
    std::vector<Row> rows = load_rows("commonroad_paths.csv");
    for (const auto& [id, vehicle] : by_id) {
      CAPTURE(id);
      std::vector<KinematicSingleTrack> theirs;
      std::vector<KinematicSingleTrack> converged;
      for (const Row& row : rows) {
        if (static_cast<int>(number(row, "vehicle")) != id) {
          continue;
        }
        (number(row, "step") == 0.05 ? theirs : converged)
            .push_back(read_state(row));
      }
      std::vector<KinematicSingleTrack> ours = drive(vehicle, 50ms);
      REQUIRE(ours.size() == theirs.size());
      Apart same;
      Apart from_converged;
      for (std::size_t i = 0; i < ours.size(); ++i) {
        same.widen(ours[i], theirs[i]);
        from_converged.widen(ours[i], converged[i]);
      }
      CAPTURE(same.position, same.heading, from_converged.position,
              from_converged.heading);
      CHECK(same.position < 1e-12);
      CHECK(same.heading < 1e-12);
      CHECK(from_converged.position < 0.001);
    }
  }
}

}  // namespace simon::automotive
