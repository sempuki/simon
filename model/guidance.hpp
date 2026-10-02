// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>

#include "model/kinematics.hpp"
#include "model/units.hpp"

// Guidance and steering laws, as free functions of kinematic state. Each
// returns a commanded acceleration for Control.
namespace simon::model {

// `acceleration`, scaled down if needed so its magnitude is at most `limit`,
// which must not be negative. Most commands are within the limit, so it
// compares squares and takes a square root only to scale one down.
inline auto limit(const Acceleration& acceleration, AccelerationMagnitude limit)
    -> Acceleration {
  if (dot(acceleration, acceleration) <= limit * limit) {
    return acceleration;
  }
  return acceleration *
         (limit / norm(acceleration)).numerical_value_in(units::one);
}

// True proportional navigation: acceleration perpendicular to the line of
// sight, proportional to the closing speed and the line-of-sight rate,
//
//   a = N * Vc * (omega x r) / |r|,   omega = (r x v) / |r|^2,
//   Vc = -(r . v) / |r|
//
// where r and v are the target's position and velocity relative to `self`.
// Returns zero when the target is at `self`.
inline auto proportional_navigation(const Kinematics& self,
                                    const Kinematics& target, double gain)
    -> Acceleration {
  Displacement relative_position = target.position - self.position;
  Velocity relative_velocity = target.velocity - self.velocity;
  auto range = norm(relative_position);
  if (range == 0.0 * meter) {
    return meters_per_second_squared(0.0, 0.0, 0.0);
  }
  auto line_of_sight_rate =
      cross(relative_position, relative_velocity) / (range * range);
  Speed closing_speed = -dot(relative_position, relative_velocity) / range;
  return gain * closing_speed * cross(line_of_sight_rate, relative_position) /
         range;
}

// Acceleration that turns `self` toward `goal` at `cruise` speed, correcting
// the velocity error at `response` (per second).
inline auto steer_toward(const Kinematics& self, const Position& goal,
                         Speed cruise, Rate response) -> Acceleration {
  Displacement to_goal = goal - self.position;
  auto distance = norm(to_goal);
  if (distance == 0.0 * meter) {
    return -self.velocity * response;
  }
  Velocity desired = to_goal * (cruise / distance);
  return (desired - self.velocity) * response;
}

// Acceleration along the velocity that brings speed toward `speed` at
// `response` (per second). Zero when `self` is not moving.
inline auto hold_speed(const Kinematics& self, Speed speed, Rate response)
    -> Acceleration {
  Speed current = norm(self.velocity);
  if (current == 0.0 * meter_per_second) {
    return meters_per_second_squared(0.0, 0.0, 0.0);
  }
  return self.velocity * ((speed - current) / current * response);
}

}  // namespace simon::model
