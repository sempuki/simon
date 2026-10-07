// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the CommonRoad vehicle models 3.0.2, Copyright 2020 Technical
// University of Munich, BSD-3-Clause; translated to C++ and changed. See
// NOTICE.md.

#pragma once

#include <array>

#include "core/time.hpp"
#include "core/units.hpp"
#include "model/vehicle/vehicle.hpp"

// CommonRoad's multibody vehicle model (Althoff and Wuersching, "CommonRoad:
// Vehicle Models"; see model/REFERENCES.md), after the US Department of
// Transportation's vehicle dynamics: a sprung body that yaws, rolls, pitches
// and heaves on its suspension over two unsprung axles, each of which rolls,
// slides and bounces on its tires, and four wheels that spin. The body joins
// each axle at a compliant pin on the axle's roll axis. Each tire's forces
// follow the Magic Formula (see model/tire.hpp) at its slip, its camber
// through the suspension's travel and its load through the tire's spring.
//
// The state is in the vehicle's frame, in SAE's axes: speeds along its x
// forward and y to its right, heights down from the body's rest, the left
// wheels at -y; positive steering and yaw turn it right. Below 0.1 m/s the body
// moves as the kinematic model about its center of gravity, as the single-track
// models do.
namespace simon::vehicle {

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

struct MultibodyRate;

// The multibody model's state, in CommonRoad's order: the center of
// gravity's position in the plane, the steering angle, the speed along x,
// the heading and yaw rate; the body; the front and rear axles; the left
// front, right front, left rear and right rear wheels' speeds; and how far
// the front and rear pins have slid sideways.
struct Multibody final {
  using RateComponent = MultibodyRate;
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

struct MultibodyRate final {
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
auto convert_multibody_to_numbers(const Multibody& state) -> MultibodyNumbers;
auto convert_numbers_to_multibody(const MultibodyNumbers& numbers) -> Multibody;
auto convert_multibody_rate_to_numbers(const MultibodyRate& rate)
    -> MultibodyNumbers;
auto convert_numbers_to_multibody_rate(const MultibodyNumbers& numbers)
    -> MultibodyRate;

auto operator+(const MultibodyRate& a, const MultibodyRate& b) -> MultibodyRate;
auto operator*(double weight, const MultibodyRate& rate) -> MultibodyRate;
auto advance(const Multibody& state, const MultibodyRate& rate, Duration dt)
    -> Multibody;

// The multibody state straight ahead at `speed`, its wheels rolling and its
// suspension at rest (CommonRoad's init_mb).
auto start_multibody(Speed speed, const VehicleParameters& vehicle)
    -> Multibody;

// Each wheel's steer beyond the steering, positive to the right as the
// model's axes have it: its toe, and how a suspension's kinematics steer it
// with the steering and the body's roll. CommonRoad's model has none.
struct WheelSteer final {
  Angle left_front = 0.0 * radian;
  Angle right_front = 0.0 * radian;
  Angle left_rear = 0.0 * radian;
  Angle right_rear = 0.0 * radian;
};

// The multibody model's rate under `input` (CommonRoad's
// vehicle_dynamics_mb), its wheels steered further by `toe`. The
// acceleration asked becomes engine or brake torque, split between the axles
// and their wheels, and a wheel spinning backward stops. A Magic Formula
// tire is mirrored on the right (see model/tire.hpp).
auto compute_multibody_rate(const Multibody& state, const Input& input,
                            const VehicleParameters& vehicle,
                            const WheelSteer& toe = {}) -> MultibodyRate;

}  // namespace simon::vehicle
