// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the CommonRoad vehicle models 3.0.2, Copyright 2020 Technical
// University of Munich, BSD-3-Clause; translated to C++ and changed. See
// NOTICE.md.

#pragma once

#include <cmath>

#include "framework/step.hpp"
#include "model/units.hpp"
#include "model/vehicle.hpp"

// Single-track vehicle models, as CommonRoad defines them (Althoff and
// Wuersching, "CommonRoad: Vehicle Models"; see model/REFERENCES.md): each
// axle's wheels lumped into one, on the vehicle's center line.
//
// The kinematic model rolls without slip: the vehicle moves along its
// heading and turns at v tan(delta) / l, with l the wheelbase, about its rear
// axle, the point its state follows.
//
// The dynamic model follows the center of gravity, which slips sideways at
// the slip angle beta and yaws under the tires' lateral forces, linear in
// their slip angles with the load each axle carries as the vehicle speeds up
// or brakes. The drift model adds each axle's wheel spin, and its tires
// follow the Magic Formula (see model/tire.hpp) in combined slip, so that it
// can brake, spin its wheels and drift.
//
// Each is driven by a steering rate and an acceleration, which the vehicle's
// limits bound as CommonRoad bounds them. At a crawl the dynamic models'
// slips are undefined, and they drive as the kinematic model about the
// center of gravity: the dynamic model below 0.1 m/s, and the drift model
// blending into it about 0.2 m/s.
namespace simon::model {

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

// The kinematic model's rate under `input` (CommonRoad's vehicle_dynamics_ks).
inline auto compute_kinematic_single_track_rate(
    const KinematicSingleTrack& state, const VehicleInput& input,
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

//-- Dynamic single-track model ------------------------------------------------

struct DynamicSingleTrackRate;

// The dynamic model's state: the center of gravity's position in the plane,
// the front wheel's steering angle, its speed, the heading, the yaw rate, and
// the slip angle between its velocity and the heading.
struct DynamicSingleTrack final {
  using RateComponent = DynamicSingleTrackRate;
  Length x = 0.0 * meter;
  Length y = 0.0 * meter;
  Angle steering = 0.0 * radian;
  Speed speed = 0.0 * meter_per_second;
  Angle heading = 0.0 * radian;
  AngularRate yaw_rate = 0.0 * radian_per_second;
  Angle slip_angle = 0.0 * radian;
};

struct DynamicSingleTrackRate final {
  Speed x = 0.0 * meter_per_second;
  Speed y = 0.0 * meter_per_second;
  AngularRate steering = 0.0 * radian_per_second;
  AccelerationMagnitude speed = 0.0 * meter_per_second_squared;
  AngularRate heading = 0.0 * radian_per_second;
  AngularAccelerationMagnitude yaw_rate = 0.0 * radian_per_second_squared;
  AngularRate slip_angle = 0.0 * radian_per_second;
};

auto operator+(const DynamicSingleTrackRate& a, const DynamicSingleTrackRate& b)
    -> DynamicSingleTrackRate;
auto operator*(double weight, const DynamicSingleTrackRate& rate)
    -> DynamicSingleTrackRate;
auto advance(const DynamicSingleTrack& state,
             const DynamicSingleTrackRate& rate, framework::Duration dt)
    -> DynamicSingleTrack;

// The dynamic model's rate under `input` (CommonRoad's vehicle_dynamics_st),
// its tires' cornering stiffness and friction those of the Magic Formula
// tire at small slip.
auto compute_dynamic_single_track_rate(const DynamicSingleTrack& state,
                                       const VehicleInput& input,
                                       const VehicleParameters& vehicle)
    -> DynamicSingleTrackRate;

//-- Drift single-track model --------------------------------------------------

struct DriftSingleTrackRate;

// The drift model's state: the dynamic model's, and each axle's wheel speed.
struct DriftSingleTrack final {
  using RateComponent = DriftSingleTrackRate;
  Length x = 0.0 * meter;
  Length y = 0.0 * meter;
  Angle steering = 0.0 * radian;
  Speed speed = 0.0 * meter_per_second;
  Angle heading = 0.0 * radian;
  AngularRate yaw_rate = 0.0 * radian_per_second;
  Angle slip_angle = 0.0 * radian;
  AngularRate front_wheel = 0.0 * radian_per_second;
  AngularRate rear_wheel = 0.0 * radian_per_second;
};

struct DriftSingleTrackRate final {
  Speed x = 0.0 * meter_per_second;
  Speed y = 0.0 * meter_per_second;
  AngularRate steering = 0.0 * radian_per_second;
  AccelerationMagnitude speed = 0.0 * meter_per_second_squared;
  AngularRate heading = 0.0 * radian_per_second;
  AngularAccelerationMagnitude yaw_rate = 0.0 * radian_per_second_squared;
  AngularRate slip_angle = 0.0 * radian_per_second;
  AngularAccelerationMagnitude front_wheel = 0.0 * radian_per_second_squared;
  AngularAccelerationMagnitude rear_wheel = 0.0 * radian_per_second_squared;
};

auto operator+(const DriftSingleTrackRate& a, const DriftSingleTrackRate& b)
    -> DriftSingleTrackRate;
auto operator*(double weight, const DriftSingleTrackRate& rate)
    -> DriftSingleTrackRate;
auto advance(const DriftSingleTrack& state, const DriftSingleTrackRate& rate,
             framework::Duration dt) -> DriftSingleTrack;

// The drift model's state straight ahead at `speed`, its wheels rolling.
auto start_drift_single_track(Speed speed, const VehicleParameters& vehicle)
    -> DriftSingleTrack;

// The drift model's rate under `input` (CommonRoad's vehicle_dynamics_std).
// The acceleration asked becomes engine or brake torque, split between the
// axles, and a wheel spinning backward stops.
auto compute_drift_single_track_rate(const DriftSingleTrack& state,
                                     const VehicleInput& input,
                                     const VehicleParameters& vehicle)
    -> DriftSingleTrackRate;

}  // namespace simon::model
