// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <span>

#include "Eigen/Geometry"
#include "model/aerodynamics.hpp"
#include "model/aircraft_data.hpp"
#include "model/atmosphere.hpp"
#include "model/earth.hpp"
#include "model/flight_path.hpp"
#include "model/rigid_body.hpp"
#include "model/units.hpp"

// An aircraft as a rigid body (see rigid_body.hpp), flown by its control
// surfaces through aerodynamics read from data (see aircraft_data.hpp). It is
// the highest fidelity level (see "Choose fidelity per archetype" in
// documents/design.md), for the few aircraft whose handling matters.
namespace simon::model {

// Where rigid aircraft fly. Over a flat, still Earth, the inertial frame is
// the world's local frame (x east, y north, z up) and gravity is standard.
// Round a turning WGS84 Earth, the inertial frame is ECI, which coincides
// with ECEF at time zero, and the world's local frame is the plane tangent to
// the ellipsoid at `origin`, so the rest of a world can treat the aircraft as
// it treats any other. There is no wind: the air turns with the Earth.
class Earth final {
 public:
  static auto flat() -> Earth { return Earth{}; }
  static auto round(const wgs84::Geodetic& origin) -> Earth;

  auto is_round() const -> bool { return round_; }

  // How far the Earth has turned at `time`.
  auto angle(Time time) const -> Angle;

  // The gravitational acceleration at `body`, in the inertial frame.
  auto gravity(const RigidBody& body, Time time) const -> Acceleration;

  // The body's velocity through the air, in body axes.
  auto air_velocity(const RigidBody& body) const -> Velocity;

  // The rate of air_velocity, in body axes, for a body whose inertial
  // acceleration is `acceleration`.
  auto air_acceleration(const RigidBody& body,
                        const Acceleration& acceleration) const -> Acceleration;

  // The body's rate relative to the air, in body axes.
  auto air_rate(const RigidBody& body) const -> AngularVelocity;

  // Height above sea level: the geodetic altitude, or z over a flat Earth.
  auto altitude(const RigidBody& body, Time time) const -> Length;

  // The rotation from body axes to the local north-east-down frame.
  auto body_to_north_east_down(const RigidBody& body, Time time) const
      -> Eigen::Matrix3d;

  // Where the body is in the world's local frame, and its flight path
  // relative to the air: speed, flight-path angle above the local horizon,
  // and heading from local north.
  auto air_state(const RigidBody& body, Time time) const -> AirState;

  // A body at `position` in the world's local frame, with Euler angles
  // `roll`, `pitch` and `yaw` from the local north-east-down frame, moving
  // through the air at `air_velocity` in body axes and turning relative to it
  // at `air_rate`.
  auto body_at(const Position& position, Angle roll, Angle pitch, Angle yaw,
               const Velocity& air_velocity, const AngularVelocity& air_rate,
               Time time) const -> RigidBody;

 private:
  // From the local north-east-down frame at `fixed` to the inertial frame.
  auto north_east_down(const Position& fixed, Time time) const
      -> Eigen::Matrix3d;
  auto fixed_of(const RigidBody& body, Time time) const -> Position;

  bool round_ = false;
  Position origin_fixed_ = meters(0.0, 0.0, 0.0);
  Eigen::Matrix3d fixed_to_local_ = Eigen::Matrix3d::Identity();
};

// Where the control surfaces stand: deflections in radians, extensions from
// 0 to 1.
struct ControlSurfaces final {
  double elevator = 0.0;
  double left_aileron = 0.0;
  double right_aileron = 0.0;
  double rudder = 0.0;
  double flaps = 0.0;
  double gear = 0.0;
  double speedbrake = 0.0;
  double spoilers = 0.0;
};

// An aircraft's mass properties as loaded: its mass, its inertia about its
// center of mass in body axes, and where that center is, in the structural
// frame (x aft, y right, z up).
struct MassBalance final {
  MassProperties properties;
  Displacement center_of_mass = meters(0.0, 0.0, 0.0);
};

// The mass balance of `aircraft` with `contents` in its tanks, one for each,
// by the parallel axis theorem: the empty aircraft and each tank's fuel, as a
// point mass, about the combined center of mass.
auto mass_balance_of(const AircraftData& aircraft,
                     std::span<const Mass> contents) -> MassBalance;

// The same, with each tank as the aircraft data fills it.
auto mass_balance_of(const AircraftData& aircraft) -> MassBalance;

// A point in the structural frame, from the center of mass, in body axes.
auto body_offset(const Displacement& structural,
                 const Displacement& center_of_mass) -> Displacement;

// The rate of a rigid aircraft's body, under its aerodynamics and gravity
// (engines come later). Lift is summed first, so induced drag reads this
// step's lift coefficient, and the forces before the moments, so the rate of
// angle of attack that the moments read is this step's exact one, as long as
// no force reads it; if one does, the forces are found again with it.
auto rigid_aircraft_rate(const RigidBody& body, const ControlSurfaces& surfaces,
                         const MassBalance& mass, const AircraftData& aircraft,
                         const Earth& earth, const StandardAirTable& air,
                         Time time) -> RigidBodyRate;

// What the aerodynamics read at `body`, before the rate of angle of attack,
// which needs the body's acceleration. `reference` is the aerodynamic
// reference point from the center of mass, in body axes: ground effect reads
// its height.
auto aero_inputs_of(const RigidBody& body, const ControlSurfaces& surfaces,
                    const AircraftData& aircraft, const Displacement& reference,
                    const Earth& earth, const StandardAirTable& air, Time time)
    -> AeroInputs;

}  // namespace simon::model
