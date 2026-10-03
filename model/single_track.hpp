// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>

#include "framework/step.hpp"
#include "model/units.hpp"

// Single-track vehicle models, as CommonRoad defines them (Althoff and
// Wuersching, "CommonRoad: Vehicle Models"; see model/REFERENCES.md): each
// axle's wheels lumped into one, on the vehicle's center line.
//
// The kinematic model rolls without slip: the vehicle moves along its
// heading and turns at v tan(delta) / l, with l the wheelbase, about its rear
// axle, the point its state follows. It is driven by a steering rate and an
// acceleration, which the vehicle's limits bound as CommonRoad bounds them.
namespace simon::model {

// The steering a vehicle can do: its angle's range, and how fast the angle
// can change.
struct SteeringLimits final {
  Angle min = -0.91 * radian;
  Angle max = 0.91 * radian;
  AngularRate min_rate = -0.4 * radian_per_second;
  AngularRate max_rate = 0.4 * radian_per_second;
};

// The longitudinal acceleration and speed a vehicle can reach. Above
// `switch_speed` the engine's power limits acceleration to
// max_acceleration switch_speed / v.
struct LongitudinalLimits final {
  AccelerationMagnitude max_acceleration = 11.5 * meter_per_second_squared;
  Speed switch_speed = 4.755 * meter_per_second;
  Speed min_speed = -13.9 * meter_per_second;
  Speed max_speed = 45.8 * meter_per_second;
};

// What the single-track models read of a vehicle. The defaults are
// CommonRoad's vehicle 1, a Ford Escort.
struct VehicleParameters final {
  // The wheelbase.
  auto wheelbase() const -> Length { return front + rear; }

  Length length = 4.298 * meter;
  Length width = 1.674 * meter;
  Length front = 0.88392 * meter;  // From the center of gravity to each axle.
  Length rear = 1.50876 * meter;
  SteeringLimits steering;
  LongitudinalLimits longitudinal;
};

// What drives a single-track vehicle.
struct SingleTrackInput final {
  AngularRate steering_rate = 0.0 * radian_per_second;
  AccelerationMagnitude acceleration = 0.0 * meter_per_second_squared;
};

struct KinematicSingleTrackRate;

// The kinematic model's state: the rear axle's position in the plane, the
// front wheel's steering angle, the speed along the heading, and the
// heading, counterclockwise from x.
struct KinematicSingleTrack final {
  using RateComponent = KinematicSingleTrackRate;
  Length x = 0.0 * meter;
  Length y = 0.0 * meter;
  Angle steering = 0.0 * radian;
  Speed speed = 0.0 * meter_per_second;
  Angle heading = 0.0 * radian;
};

struct KinematicSingleTrackRate final {
  Speed x = 0.0 * meter_per_second;
  Speed y = 0.0 * meter_per_second;
  AngularRate steering = 0.0 * radian_per_second;
  AccelerationMagnitude speed = 0.0 * meter_per_second_squared;
  AngularRate heading = 0.0 * radian_per_second;
};

inline auto operator+(const KinematicSingleTrackRate& a,
                      const KinematicSingleTrackRate& b)
    -> KinematicSingleTrackRate {
  return {.x = a.x + b.x,
          .y = a.y + b.y,
          .steering = a.steering + b.steering,
          .speed = a.speed + b.speed,
          .heading = a.heading + b.heading};
}

inline auto operator*(double weight, const KinematicSingleTrackRate& rate)
    -> KinematicSingleTrackRate {
  return {.x = weight * rate.x,
          .y = weight * rate.y,
          .steering = weight * rate.steering,
          .speed = weight * rate.speed,
          .heading = weight * rate.heading};
}

inline auto advance(const KinematicSingleTrack& state,
                    const KinematicSingleTrackRate& rate,
                    framework::Duration dt) -> KinematicSingleTrack {
  Time seconds = model::seconds(dt);
  return {.x = state.x + rate.x * seconds,
          .y = state.y + rate.y * seconds,
          .steering = state.steering + rate.steering * seconds,
          .speed = state.speed + rate.speed * seconds,
          .heading = state.heading + rate.heading * seconds};
}

// The steering rate the vehicle reaches for `wanted` at `steering`: none
// pressing past either end of the steering's range, and within the rate's
// bounds otherwise.
inline auto limit_steering_rate(Angle steering, AngularRate wanted,
                                const SteeringLimits& limits) -> AngularRate {
  if ((steering <= limits.min && wanted <= 0.0 * radian_per_second) ||
      (steering >= limits.max && wanted >= 0.0 * radian_per_second)) {
    return 0.0 * radian_per_second;
  }
  return clamp(wanted, limits.min_rate, limits.max_rate);
}

// The acceleration the vehicle reaches for `wanted` at `speed`: none pressing
// past either end of the speed's range, at most max_acceleration braking, and
// at most the engine's power allows speeding up.
inline auto limit_acceleration(Speed speed, AccelerationMagnitude wanted,
                               const LongitudinalLimits& limits)
    -> AccelerationMagnitude {
  AccelerationMagnitude most =
      speed > limits.switch_speed
          ? limits.max_acceleration * number_of(limits.switch_speed / speed)
          : limits.max_acceleration;
  if ((speed <= limits.min_speed && wanted <= 0.0 * meter_per_second_squared) ||
      (speed >= limits.max_speed && wanted >= 0.0 * meter_per_second_squared)) {
    return 0.0 * meter_per_second_squared;
  }
  return clamp(wanted, -limits.max_acceleration, most);
}

// The kinematic model's rate under `input` (CommonRoad's vehicle_dynamics_ks).
inline auto compute_kinematic_single_track_rate(
    const KinematicSingleTrack& state, const SingleTrackInput& input,
    const VehicleParameters& vehicle) -> KinematicSingleTrackRate {
  double steering = radians(state.steering);
  return KinematicSingleTrackRate{
      .x = state.speed * cos(state.heading),
      .y = state.speed * sin(state.heading),
      .steering = limit_steering_rate(state.steering, input.steering_rate,
                                      vehicle.steering),
      .speed = limit_acceleration(state.speed, input.acceleration,
                                  vehicle.longitudinal),
      .heading =
          state.speed / vehicle.wheelbase() * std::tan(steering) * radian,
  };
}

}  // namespace simon::model
