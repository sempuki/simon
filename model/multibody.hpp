// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>

#include "framework/step.hpp"
#include "model/units.hpp"
#include "model/vehicle.hpp"

// CommonRoad's multibody vehicle model (Althoff and Wuersching, "CommonRoad:
// Vehicle Models"; see model/REFERENCES.md), after the US Department of
// Transportation's vehicle dynamics: a sprung body that yaws, rolls, pitches
// and heaves on its suspension over two unsprung axles, each of which rolls,
// slides and bounces on its tires, and four wheels that spin. The body joins
// each axle at a compliant pin on the axle's roll axis. Each tire's forces
// follow the Magic Formula (see model/tire.hpp) at its slip, its camber
// through the suspension's travel and its load through the tire's spring.
//
// The state is in the vehicle's frame: speeds along its x forward and y to
// its left, heights down from the body's rest. Below 0.1 m/s the body moves
// as the kinematic model about its center of gravity, as the single-track
// models do.
namespace simon::model {

// The sprung body's motion beyond the plane.
struct SprungBody final {
  Angle roll = 0.0 * radian;
  AngularRate roll_rate = 0.0 * radian_per_second;
  Angle pitch = 0.0 * radian;
  AngularRate pitch_rate = 0.0 * radian_per_second;
  Speed lateral_speed = 0.0 * meter_per_second;
  Length height = 0.0 * meter;
  Speed vertical_speed = 0.0 * meter_per_second;
};

// An axle's unsprung mass. Its height is its tires' compression.
struct UnsprungAxle final {
  Angle roll = 0.0 * radian;
  AngularRate roll_rate = 0.0 * radian_per_second;
  Speed lateral_speed = 0.0 * meter_per_second;
  Length height = 0.0 * meter;
  Speed vertical_speed = 0.0 * meter_per_second;
};

struct MultibodyVehicleRate;

// The multibody model's state, in CommonRoad's order: the center of
// gravity's position in the plane, the steering angle, the speed along x,
// the heading and yaw rate; the body; the front and rear axles; the left
// front, right front, left rear and right rear wheels' speeds; and how far
// the front and rear pins have slid sideways.
struct MultibodyVehicle final {
  using RateComponent = MultibodyVehicleRate;
  Length x = 0.0 * meter;
  Length y = 0.0 * meter;
  Angle steering = 0.0 * radian;
  Speed speed = 0.0 * meter_per_second;
  Angle heading = 0.0 * radian;
  AngularRate yaw_rate = 0.0 * radian_per_second;
  SprungBody body;
  UnsprungAxle front;
  UnsprungAxle rear;
  std::array<AngularRate, 4> wheels{};
  Length front_pin = 0.0 * meter;
  Length rear_pin = 0.0 * meter;
};

struct SprungBodyRate final {
  AngularRate roll = 0.0 * radian_per_second;
  AngularAccelerationMagnitude roll_rate = 0.0 * radian_per_second_squared;
  AngularRate pitch = 0.0 * radian_per_second;
  AngularAccelerationMagnitude pitch_rate = 0.0 * radian_per_second_squared;
  AccelerationMagnitude lateral_speed = 0.0 * meter_per_second_squared;
  Speed height = 0.0 * meter_per_second;
  AccelerationMagnitude vertical_speed = 0.0 * meter_per_second_squared;
};

struct UnsprungAxleRate final {
  AngularRate roll = 0.0 * radian_per_second;
  AngularAccelerationMagnitude roll_rate = 0.0 * radian_per_second_squared;
  AccelerationMagnitude lateral_speed = 0.0 * meter_per_second_squared;
  Speed height = 0.0 * meter_per_second;
  AccelerationMagnitude vertical_speed = 0.0 * meter_per_second_squared;
};

struct MultibodyVehicleRate final {
  Speed x = 0.0 * meter_per_second;
  Speed y = 0.0 * meter_per_second;
  AngularRate steering = 0.0 * radian_per_second;
  AccelerationMagnitude speed = 0.0 * meter_per_second_squared;
  AngularRate heading = 0.0 * radian_per_second;
  AngularAccelerationMagnitude yaw_rate = 0.0 * radian_per_second_squared;
  SprungBodyRate body;
  UnsprungAxleRate front;
  UnsprungAxleRate rear;
  std::array<AngularAccelerationMagnitude, 4> wheels{};
  Speed front_pin = 0.0 * meter_per_second;
  Speed rear_pin = 0.0 * meter_per_second;
};

// The state and its rate as CommonRoad's 29 numbers, in SI units, in order.
using MultibodyNumbers = std::array<double, 29>;
auto convert_multibody_to_numbers(const MultibodyVehicle& state)
    -> MultibodyNumbers;
auto convert_numbers_to_multibody(const MultibodyNumbers& numbers)
    -> MultibodyVehicle;
auto convert_multibody_rate_to_numbers(const MultibodyVehicleRate& rate)
    -> MultibodyNumbers;
auto convert_numbers_to_multibody_rate(const MultibodyNumbers& numbers)
    -> MultibodyVehicleRate;

auto operator+(const MultibodyVehicleRate& a, const MultibodyVehicleRate& b)
    -> MultibodyVehicleRate;
auto operator*(double weight, const MultibodyVehicleRate& rate)
    -> MultibodyVehicleRate;
auto advance(const MultibodyVehicle& state, const MultibodyVehicleRate& rate,
             framework::Duration dt) -> MultibodyVehicle;

// The multibody state straight ahead at `speed`, its wheels rolling and its
// suspension at rest (CommonRoad's init_mb).
auto start_multibody(Speed speed, const VehicleParameters& vehicle)
    -> MultibodyVehicle;

// The multibody model's rate under `input` (CommonRoad's
// vehicle_dynamics_mb). The acceleration asked becomes engine or brake
// torque, split between the axles and their wheels, and a wheel spinning
// backward stops.
auto compute_multibody_rate(const MultibodyVehicle& state,
                            const VehicleInput& input,
                            const VehicleParameters& vehicle)
    -> MultibodyVehicleRate;

}  // namespace simon::model
