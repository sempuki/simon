// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "Eigen/Geometry"
#include "core/units.hpp"
#include "model/earth.hpp"
#include "model/flight_path.hpp"
#include "model/rigid_body.hpp"
#include "model/wind.hpp"

// The frames a rigid body flies in: the Earth it flies over, flat or round,
// its place on it, and its motion relative to the air, which turns with the
// Earth and moves with the wind.
namespace simon::model {

// A body's place on the Earth at a time, found once for the frames, gravity
// and the air to share.
struct Place final {
  Matrix3 convert_inertial_to_fixed = Matrix3::Identity();
  Position fixed = meters(0.0, 0.0, 0.0);
  Length altitude = 0.0 * meter;
  // From the local north-east-down frame to the inertial frame.
  Matrix3 north_east_down = Matrix3::Identity();
  double sin_latitude = 0.0;  // Geodetic; none over a flat Earth.
  double cos_latitude = 1.0;
};

// The Earth that rigid aircraft fly over. Over a flat Earth, the inertial
// frame is the world's local frame (x east, y north, z up) and gravity is
// standard. Round a turning WGS84 Earth, the inertial frame is ECI, which
// coincides with ECEF at time zero, and the world's local frame is the plane
// tangent to the ellipsoid at `origin`, so the rest of a world can treat the
// aircraft as it treats any other. The air turns with the Earth, and a Wind
// moves it relative to the Earth.
class Earth final {
 public:
  static auto flat() -> Earth { return Earth{}; }
  static auto round(const wgs84::Geodetic& origin) -> Earth;

  auto is_round() const -> bool { return round_; }

  // The angle the Earth has turned through at `time`.
  auto angle(Time time) const -> Angle;

  // The place of `body` at `time`.
  auto place(const RigidBody& body, Time time) const -> Place;

  // The gravitational acceleration at `body`, in the inertial frame.
  auto gravity(const RigidBody& body, Time time) const -> Acceleration;
  auto gravity(const Place& place) const -> Acceleration;

  // The body's velocity through the air, in body axes.
  auto air_velocity(const RigidBody& body) const -> Velocity;

  // The rate of air_velocity, in body axes, for a body whose inertial
  // acceleration is `acceleration`.
  auto air_acceleration(const RigidBody& body,
                        const Acceleration& acceleration) const -> Acceleration;

  // The body's rate relative to the air, in body axes.
  auto air_rate(const RigidBody& body) const -> AngularVelocity;

  // The rate relative to the air at which `body` keeps its attitude to the
  // local north-east-down frame as it moves over the Earth, in body axes: the
  // frame's turning, which flying level round the Earth needs: its transport
  // rate, from the ellipsoid's radii of curvature (Titterton and Weston; see
  // model/REFERENCES.md). None over a flat Earth.
  auto level_rate(const RigidBody& body, Time time) const -> AngularVelocity;

  // Height above sea level: the geodetic altitude, or z over a flat Earth.
  auto altitude(const RigidBody& body, Time time) const -> Length;

  // The rotation from body axes to the local north-east-down frame.
  auto convert_body_to_north_east_down(const RigidBody& body, Time time) const
      -> Matrix3;

  // The body's position in the world's local frame, and its flight path
  // relative to the air, which moves as `wind` has it: speed, flight-path
  // angle above the local horizon, and heading from local north.
  auto air_state(const RigidBody& body, Time time, const Wind& wind = {}) const
      -> AirState;

  // A body at `position` in the world's local frame, with Euler angles
  // `roll`, `pitch` and `yaw` from the local north-east-down frame, moving
  // through the air at `air_velocity` in body axes and turning relative to it
  // at `air_rate`.
  auto body_at(const Position& position, Angle roll, Angle pitch, Angle yaw,
               const Velocity& air_velocity, const AngularVelocity& air_rate,
               Time time) const -> RigidBody;

 private:
  // From the local north-east-down frame at `fixed` to the inertial frame.
  auto north_east_down(const Position& fixed, Time time) const -> Matrix3;
  auto find_fixed(const RigidBody& body, Time time) const -> Position;

  Position origin_fixed_ = meters(0.0, 0.0, 0.0);
  Matrix3 fixed_to_local_ = Matrix3::Identity();
  bool round_ = false;
};

// The Earth's rotation in the inertial frame, or none over a flat Earth.
inline auto spin(const Earth& earth) -> Vector3 {
  return earth.is_round() ? eigen(wgs84::rotation()) : Vector3::Zero();
}

// The body's velocity through the air at `place`, in the inertial frame: its
// velocity less the air's, which turns with the Earth and moves with `wind`.
inline auto compute_inertial_air_velocity(const RigidBody& body,
                                          const Earth& earth,
                                          const Place& place, const Wind& wind)
    -> Vector3 {
  return eigen(body.velocity) - spin(earth).cross(eigen(body.position)) -
         place.north_east_down * eigen(wind.north_east_down);
}

// A body's motion, found once for everything that reads it: its attitude as
// a matrix, and its velocity and rate relative to the air at `place`, which
// `wind` moves and turns, in body axes.
struct BodyMotion final {
  Matrix3 to_inertial = Matrix3::Identity();
  Vector3 air_velocity = Vector3::Zero();
  Vector3 air_rate = Vector3::Zero();
};

// Compiled for still air apart, where WINDY is false and `wind` is not read,
// so that still air costs what it would without wind.
template <bool WINDY>
inline auto compute_body_motion(const RigidBody& body, const Earth& earth,
                                const Place& place, const Wind& wind)
    -> BodyMotion {
  BodyMotion motion{.to_inertial = body.attitude.toRotationMatrix()};
  Matrix3 to_body = motion.to_inertial.transpose();
  Vector3 turning = spin(earth);
  if constexpr (!WINDY) {
    motion.air_velocity =
        to_body * (eigen(body.velocity) - turning.cross(eigen(body.position)));
    motion.air_rate = eigen(body.rate) - to_body * turning;
    return motion;
  }
  motion.air_velocity =
      to_body * compute_inertial_air_velocity(body, earth, place, wind);
  motion.air_rate =
      eigen(body.rate) -
      to_body * (turning + place.north_east_down * eigen(wind.rotation));
  return motion;
}

// The rate of `motion`'s air velocity, in body axes, under the specific force
// `specific_force` in body axes and gravity `gravity` in the inertial frame.
// With the wind u in the inertial frame, turning with the Earth,
// d/dt R^T (v - W x r - u) = f / m + R^T (g - W x v - W x u) -
// w x R^T (v - W x r - u). Derived here from Stevens and Lewis's equations of
// motion (see rigid_body.hpp) by the rule for a vector's rate in a turning
// frame, with the wind held still in the local frame.
template <bool WINDY>
inline auto compute_air_acceleration(const RigidBody& body,
                                     const BodyMotion& motion,
                                     const Earth& earth, const Place& place,
                                     const Wind& wind,
                                     const Vector3& specific_force,
                                     const Vector3& gravity) -> Vector3 {
  Vector3 turning = spin(earth);
  Vector3 carried = eigen(body.velocity);
  if constexpr (WINDY) {
    carried += place.north_east_down * eigen(wind.north_east_down);
  }
  return specific_force +
         motion.to_inertial.transpose() * (gravity - turning.cross(carried)) -
         eigen(body.rate).cross(motion.air_velocity);
}

// The body's velocity through the air at `place`, which turns with the Earth
// and moves with `wind`, in body axes.
auto compute_air_velocity(const RigidBody& body, const Earth& earth,
                          const Place& place, const Wind& wind) -> Velocity;

// The rate of that velocity, in body axes, for a body under `specific_force`
// (every force but gravity, per unit mass, in body axes) and `gravity` (in
// the inertial frame). The wind holds still in the local north-east-down
// frame, which turns with the Earth; the frame's turning as the body moves
// over the Earth is left out, 5 x 10^-4 m/s^2 at 200 m/s in a 15 m/s wind. In
// wind it differs from the rate of the velocity over the ground by w x R^T u,
// the wind turning in body axes as the body turns. JSBSim takes the rate of
// angle of attack from the rate over the ground.
auto compute_air_acceleration(const RigidBody& body, const Earth& earth,
                              const Place& place, const Wind& wind,
                              const Acceleration& specific_force,
                              const Acceleration& gravity) -> Acceleration;

}  // namespace simon::model
