// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <optional>

#include "model/units.hpp"

// Driver models for traffic (see model/REFERENCES.md): the Intelligent Driver
// Model for following (Treiber, Hennecke and Helbing 2000, in the form of
// Treiber and Kesting's Traffic Flow Dynamics, 2013), and MOBIL for changing
// lanes (Kesting, Treiber and Helbing 2007).
namespace simon::model {

// A driver's car-following preferences. The defaults are typical of a
// highway, as Treiber and Kesting give them.
struct IntelligentDriver final {
  Speed desired_speed = 120.0 / 3.6 * meter_per_second;                 // v0.
  Time time_headway = 1.0 * second;                                     // T.
  Length minimum_gap = 2.0 * meter;                                     // s0.
  AccelerationMagnitude acceleration = 1.0 * meter_per_second_squared;  // a.
  AccelerationMagnitude deceleration = 1.5 * meter_per_second_squared;  // b.
  double exponent = 4.0;  // delta.
};

// The vehicle ahead in the same lane: the gap from its rear bumper to this
// vehicle's front bumper, and its speed.
struct Leader final {
  Length gap = 0.0 * meter;
  Speed speed = 0.0 * meter_per_second;
};

// The Intelligent Driver Model's acceleration at `speed`, behind `leader` if
// there is one:
//
//   a [1 - (v / v0)^delta - (s* / s)^2],
//   s* = s0 + max(0, v T + v (v - v_l) / (2 sqrt(a b)))
//
// with s the gap, the interaction term left out with no leader.
inline auto compute_idm_acceleration(const IntelligentDriver& driver,
                                     Speed speed,
                                     const std::optional<Leader>& leader)
    -> AccelerationMagnitude {
  double v = speed.numerical_value_in(meter_per_second);
  double v0 = driver.desired_speed.numerical_value_in(meter_per_second);
  double a = driver.acceleration.numerical_value_in(meter_per_second_squared);
  double b = driver.deceleration.numerical_value_in(meter_per_second_squared);
  // The usual exponent of 4 by squaring twice, a tenth of pow's cost.
  double ratio = v / v0;
  double squared = ratio * ratio;
  double free =
      1.0 - (driver.exponent == 4.0 ? squared * squared
                                    : std::pow(ratio, driver.exponent));
  double interaction = 0.0;
  if (leader) {
    double approach = v - leader->speed.numerical_value_in(meter_per_second);
    double wanted =
        driver.minimum_gap.numerical_value_in(meter) +
        std::max(0.0, v * driver.time_headway.numerical_value_in(second) +
                          v * approach / (2.0 * std::sqrt(a * b)));
    double crowding = wanted / leader->gap.numerical_value_in(meter);
    interaction = crowding * crowding;
  }
  return a * (free - interaction) * meter_per_second_squared;
}

// A driver's lane-changing preferences in MOBIL: how much it weighs other
// drivers' gains and losses against its own, the gain worth a change, the
// deceleration a change may impose on the new follower, and its bias toward
// the right lane, as traffic keeping right has it. The defaults are typical
// values, as Treiber and Kesting give them.
struct LaneChanger final {
  double politeness = 0.2;  // p.
  AccelerationMagnitude threshold = 0.1 * meter_per_second_squared;
  AccelerationMagnitude safe_deceleration = 4.0 * meter_per_second_squared;
  AccelerationMagnitude right_bias = 0.2 * meter_per_second_squared;
};

// The accelerations a lane change would change: this vehicle's, its old
// follower's and its new follower's, now and after the change, each from the
// car-following model.
struct LaneChangeAccelerations final {
  AccelerationMagnitude self_now = 0.0 * meter_per_second_squared;
  AccelerationMagnitude self_after = 0.0 * meter_per_second_squared;
  AccelerationMagnitude old_follower_now = 0.0 * meter_per_second_squared;
  AccelerationMagnitude old_follower_after = 0.0 * meter_per_second_squared;
  AccelerationMagnitude new_follower_now = 0.0 * meter_per_second_squared;
  AccelerationMagnitude new_follower_after = 0.0 * meter_per_second_squared;
};

// Whether MOBIL changes lanes, to the right if `to_right`, by its symmetric
// criterion with a bias toward the right lane. The change must be safe, leaving
// the new follower braking no harder than the safe deceleration, and worth it,
//
//   self_after - self_now + p (new_after - new_now + old_after - old_now)
//     > threshold - bias
//
// with the bias toward the right counting for a change to the right and
// against one to the left.
inline auto decide_lane_change(const LaneChanger& changer,
                               const LaneChangeAccelerations& accelerations,
                               bool to_right) -> bool {
  if (accelerations.new_follower_after < -changer.safe_deceleration) {
    return false;
  }
  AccelerationMagnitude gain =
      accelerations.self_after - accelerations.self_now +
      changer.politeness *
          (accelerations.new_follower_after - accelerations.new_follower_now +
           accelerations.old_follower_after - accelerations.old_follower_now);
  AccelerationMagnitude bias =
      to_right ? changer.right_bias : -changer.right_bias;
  return gain > changer.threshold - bias;
}

}  // namespace simon::model
