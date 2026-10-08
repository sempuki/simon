// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Geometry"
#include "core/math.hpp"
#include "core/time.hpp"
#include "core/units.hpp"
#include "model/kinematics.hpp"

// A rigid body in six degrees of freedom, in an inertial frame: the Earth-
// centered inertial frame for a round, rotating Earth (see earth.hpp), or a
// local frame for a flat, still one. The equations are Stevens and Lewis's
// (section 1.5; see model/REFERENCES.md), with the translational equations in
// the inertial frame as JSBSim has them:
//
//   dr/dt = v
//   dv/dt = R F / m + g
//   dq/dt = q (0, w) / 2
//   dw/dt = J^-1 (M - w x J w)
//
// r and v are the position and velocity in the inertial frame; q turns body
// axes into it, R being its rotation; w is the body's rate relative to the
// inertial frame, in body axes; F and M are the force and the moment about
// the center of mass, in body axes, without gravity; g is the gravitational
// acceleration. Body axes are x forward, y right, z down. The inertia tensor
// is held constant over a step: its rate of change is left out, as it is in
// JSBSim.
namespace simon::model {

struct RigidBodyRate;

struct RigidBody final {
  using RateComponent = RigidBodyRate;
  Position position = meters(0.0, 0.0, 0.0);
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
  Quaternion attitude = Quaternion::Identity();  // Body to inertial.
  AngularVelocity rate = QuantityVector{} * radian_per_second;  // Body axes.
};

struct RigidBodyRate final {
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
  Vector4 attitude = Vector4::Zero();  // Of q's (x, y, z, w).
  AngularAcceleration angular_acceleration =
      QuantityVector{} * radian_per_second_squared;
};

inline auto operator+(const RigidBodyRate& a, const RigidBodyRate& b)
    -> RigidBodyRate {
  return {
      .velocity = a.velocity + b.velocity,
      .acceleration = a.acceleration + b.acceleration,
      .attitude = a.attitude + b.attitude,
      .angular_acceleration = a.angular_acceleration + b.angular_acceleration};
}

inline auto operator*(double weight, const RigidBodyRate& rate)
    -> RigidBodyRate {
  return {.velocity = rate.velocity * weight,
          .acceleration = rate.acceleration * weight,
          .attitude = weight * rate.attitude,
          .angular_acceleration = rate.angular_acceleration * weight};
}

// Moves `body` along `rate` for `dt`, keeping its attitude a unit quaternion.
inline auto advance(const RigidBody& body, const RigidBodyRate& rate,
                    Duration dt) -> RigidBody {
  double seconds = simon::seconds(dt).numerical_value_in(second);
  Quaternion attitude{body.attitude.coeffs() + seconds * rate.attitude};
  attitude.normalize();
  return RigidBody{
      .position = body.position + rate.velocity * (seconds * second),
      .velocity = body.velocity + rate.acceleration * (seconds * second),
      .attitude = attitude,
      .rate = body.rate + rate.angular_acceleration * (seconds * second),
  };
}

// A body's mass, and its inertia tensor about its center of mass in body
// axes, in kg m^2.
struct MassProperties final {
  Mass mass = 1.0 * kilogram;
  Matrix3 inertia = Matrix3::Identity();
  Matrix3 inverse = Matrix3::Identity();
};

// The mass properties of `mass` with `inertia`, its inverse computed once.
inline auto compute_mass_properties(Mass mass, const Matrix3& inertia)
    -> MassProperties {
  return {.mass = mass, .inertia = inertia, .inverse = inertia.inverse()};
}

// The rate of `body` under body-axis `force` and `moment` (about the center
// of mass, gravity left out), and the gravitational acceleration `gravity` in
// the inertial frame. `to_inertial` is the body's attitude as a matrix, for a
// caller that has it already.
inline auto compute_rigid_body_rate(
    const RigidBody& body, const Matrix3& to_inertial, const ForceVector& force,
    const Moment& moment, const MassProperties& mass,
    const Acceleration& gravity) -> RigidBodyRate {
  Vector3 w = body.rate.numerical_value_in(radian_per_second).eigen();
  Vector3 f = force.numerical_value_in(newton).eigen();
  Vector3 m = moment.numerical_value_in(newton_meter).eigen();

  Vector3 specific_force =
      to_inertial * (f / mass.mass.numerical_value_in(kilogram));
  Vector3 angular_acceleration = mass.inverse * (m - w.cross(mass.inertia * w));
  Quaternion spin = body.attitude * Quaternion{0.0, w.x(), w.y(), w.z()};

  return RigidBodyRate{
      .velocity = body.velocity,
      .acceleration =
          QuantityVector{specific_force} * meter_per_second_squared + gravity,
      .attitude = 0.5 * spin.coeffs(),
      .angular_acceleration =
          QuantityVector{angular_acceleration} * radian_per_second_squared,
  };
}

inline auto compute_rigid_body_rate(
    const RigidBody& body, const ForceVector& force, const Moment& moment,
    const MassProperties& mass, const Acceleration& gravity) -> RigidBodyRate {
  return compute_rigid_body_rate(body, body.attitude.toRotationMatrix(), force,
                                 moment, mass, gravity);
}

}  // namespace simon::model
