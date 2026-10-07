// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "model/traffic/traffic.hpp"

// simon's Intelligent Driver Model and MOBIL against their authors' movsim,
// function by function where the two agree with the published models, and a
// platoon of IDM drivers against SUMO's, from the tables
// reference/movsim_reference.js and reference/sumo_platoon.py recorded.
namespace simon::automotive {

namespace {

using namespace testing;
using traffic::IntelligentDriver;
using traffic::LaneChangeAccelerations;
using traffic::LaneChanger;
using traffic::Leader;

constexpr double LENGTH = 5.0;  // Each vehicle's, in m.
constexpr int FOLLOWERS = 5;

// The SUMO platoon's leader's speed at `t` seconds, by its schedule, and the
// distance it has covered: the integral of that speed.
auto leader_speed(double t) -> double {
  if (t < 10.0) {
    return 25.0;
  }
  if (t < 25.0) {
    return 25.0 - (t - 10.0);
  }
  if (t < 40.0) {
    return 10.0;
  }
  return std::min(25.0, 10.0 + 0.8 * (t - 40.0));
}

auto leader_position(double t) -> double {
  constexpr double START = 300.0;
  if (t < 10.0) {
    return START + 25.0 * t;
  }
  double x = START + 250.0;
  if (t < 25.0) {
    double u = t - 10.0;
    return x + 25.0 * u - 0.5 * u * u;
  }
  x += 25.0 * 15.0 - 0.5 * 15.0 * 15.0;
  if (t < 40.0) {
    return x + 10.0 * (t - 25.0);
  }
  x += 150.0;
  constexpr double SPEEDING = 15.0 / 0.8;  // s to reach 25 m/s again.
  if (t < 40.0 + SPEEDING) {
    double u = t - 40.0;
    return x + 10.0 * u + 0.4 * u * u;
  }
  x += 10.0 * SPEEDING + 0.4 * SPEEDING * SPEEDING;
  return x + 25.0 * (t - 40.0 - SPEEDING);
}

// The platoon's followers: each front bumper's position and speed.
struct Platoon final {
  std::array<double, FOLLOWERS> position{};
  std::array<double, FOLLOWERS> speed{};
};

// The followers' rates at `t`: speeds, and IDM accelerations behind the
// vehicle ahead.
auto platoon_rate(const IntelligentDriver& driver, const Platoon& platoon,
                  double t) -> Platoon {
  Platoon rate;
  for (int i = 0; i < FOLLOWERS; ++i) {
    double ahead = i == 0 ? leader_position(t) : platoon.position[i - 1];
    double ahead_speed = i == 0 ? leader_speed(t) : platoon.speed[i - 1];
    rate.position[i] = platoon.speed[i];
    rate.speed[i] =
        traffic::compute_idm_acceleration(
            driver, platoon.speed[i] * meter_per_second,
            Leader{.gap = (ahead - LENGTH - platoon.position[i]) * meter,
                   .speed = ahead_speed * meter_per_second})
            .numerical_value_in(meter_per_second_squared);
  }
  return rate;
}

auto step(const Platoon& p, const Platoon& rate, double h) -> Platoon {
  Platoon next = p;
  for (int i = 0; i < FOLLOWERS; ++i) {
    next.position[i] += h * rate.position[i];
    next.speed[i] += h * rate.speed[i];
  }
  return next;
}

// The followers every 0.1 s for 80 s, by Runge-Kutta 4 at steps of `dt`.
auto drive(double dt) -> std::vector<Platoon> {
  IntelligentDriver driver{.desired_speed = 33.33 * meter_per_second,
                           .time_headway = 1.0 * second,
                           .minimum_gap = 2.0 * meter,
                           .acceleration = 1.0 * meter_per_second_squared,
                           .deceleration = 1.5 * meter_per_second_squared};
  Platoon platoon;
  for (int i = 0; i < FOLLOWERS; ++i) {
    platoon.position[i] = 300.0 - 45.0 * (i + 1);
    platoon.speed[i] = 25.0;
  }
  std::vector<Platoon> samples{platoon};
  auto steps = static_cast<int>(std::lround(80.0 / dt));
  auto every = static_cast<int>(std::lround(0.1 / dt));
  for (int k = 0; k < steps; ++k) {
    double t = k * dt;
    Platoon k1 = platoon_rate(driver, platoon, t);
    Platoon k2 =
        platoon_rate(driver, step(platoon, k1, 0.5 * dt), t + 0.5 * dt);
    Platoon k3 =
        platoon_rate(driver, step(platoon, k2, 0.5 * dt), t + 0.5 * dt);
    Platoon k4 = platoon_rate(driver, step(platoon, k3, dt), t + dt);
    for (int i = 0; i < FOLLOWERS; ++i) {
      platoon.position[i] += dt / 6.0 *
                             (k1.position[i] + 2.0 * k2.position[i] +
                              2.0 * k3.position[i] + k4.position[i]);
      platoon.speed[i] +=
          dt / 6.0 *
          (k1.speed[i] + 2.0 * k2.speed[i] + 2.0 * k3.speed[i] + k4.speed[i]);
    }
    if ((k + 1) % every == 0) {
      samples.push_back(platoon);
    }
  }
  return samples;
}

// SUMO's followers every 0.1 s at its step `dt`.
auto sumo(const std::vector<Row>& rows, double dt) -> std::vector<Platoon> {
  std::vector<Platoon> samples(801);
  for (const Row& row : rows) {
    if (number(row, "step") != dt || number(row, "vehicle") == 0) {
      continue;
    }
    auto i = static_cast<int>(number(row, "vehicle")) - 1;
    auto k = static_cast<std::size_t>(std::lround(number(row, "time") / 0.1));
    samples.at(k).position[i] = number(row, "position");
    samples.at(k).speed[i] = number(row, "speed");
  }
  return samples;
}

// The largest distance in position between two runs of the platoon.
auto apart(const std::vector<Platoon>& a, const std::vector<Platoon>& b)
    -> double {
  REQUIRE(a.size() == b.size());
  double most = 0.0;
  for (std::size_t k = 0; k < a.size(); ++k) {
    for (int i = 0; i < FOLLOWERS; ++i) {
      most = std::max(most, std::abs(a[k].position[i] - b[k].position[i]));
    }
  }
  return most;
}

}  // namespace

TEST_CASE("TrafficAgainstMovsimAndSumo") {
  SECTION("ShouldMatchMovsimGivenIdm") {
    double largest = 0.0;
    int rows = 0;
    for (const Row& row : load_rows("movsim_idm.csv")) {
      IntelligentDriver driver{
          .desired_speed = number(row, "v0") * meter_per_second,
          .time_headway = number(row, "T") * second,
          .minimum_gap = number(row, "s0") * meter,
          .acceleration = number(row, "a") * meter_per_second_squared,
          .deceleration = number(row, "b") * meter_per_second_squared};
      double ours =
          traffic::compute_idm_acceleration(
              driver, number(row, "speed") * meter_per_second,
              Leader{.gap = number(row, "gap") * meter,
                     .speed = number(row, "leader_speed") * meter_per_second})
              .numerical_value_in(meter_per_second_squared);
      double theirs = number(row, "acceleration");
      largest = std::max(
          largest, std::abs(ours - theirs) / std::max(1.0, std::abs(theirs)));
      ++rows;
    }
    CAPTURE(largest);
    CHECK(rows == 1000);
    CHECK(largest < 1e-14);
  }

  SECTION("ShouldDecideAsMovsimGivenMobil") {
    int differ = 0;
    int changes = 0;
    for (const Row& row : load_rows("movsim_mobil.csv")) {
      LaneChanger changer{
          .politeness = 0.0,
          .threshold = number(row, "threshold") * meter_per_second_squared,
          .safe_deceleration =
              number(row, "safe_deceleration") * meter_per_second_squared,
          .right_bias = number(row, "right_bias") * meter_per_second_squared};
      LaneChangeAccelerations accelerations{
          .self_now = number(row, "self_now") * meter_per_second_squared,
          .self_after = number(row, "self_after") * meter_per_second_squared,
          .new_follower_after =
              number(row, "new_follower_after") * meter_per_second_squared};
      bool ours = traffic::decide_lane_change(changer, accelerations,
                                              number(row, "to_right") == 1.0);
      bool theirs = number(row, "change") == 1.0;
      differ += ours != theirs ? 1 : 0;
      changes += theirs ? 1 : 0;
    }
    CHECK(changes == 207);
    CHECK(differ == 0);
  }

  SECTION("ShouldFollowSumoGivenPlatoon") {
    // SUMO's Euler update is first order: 1.06 m from its converged platoon
    // at its usual 0.1 s step, so about 1 cm at 0.001 s. simon by Runge-Kutta
    // 4 at 0.1 s is within 0.1 mm of its own converged platoon, and 1.07 cm
    // from SUMO's at 0.001 s: SUMO's remaining error.
    std::vector<Row> rows = load_rows("sumo_platoon.csv");
    std::vector<Platoon> converged = sumo(rows, 0.001);
    double ours = apart(drive(0.1), converged);
    double theirs = apart(sumo(rows, 0.1), converged);
    double exact = apart(drive(0.1), drive(0.001));
    CAPTURE(ours, theirs, exact);
    CHECK(exact < 2e-4);
    CHECK(ours < 0.02);
    CHECK(theirs > 50.0 * ours);
  }
}

}  // namespace simon::automotive
