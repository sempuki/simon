// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

#include "Eigen/Geometry"
#include "core/coordinates.hpp"
#include "core/math.hpp"
#include "core/time.hpp"
#include "core/units.hpp"
#include "model/earth/atmosphere.hpp"
#include "model/kinematics.hpp"

// A point-mass flight-path model: an aircraft as a point with a speed, a
// flight-path angle and a heading, flown by commanding its load factor, bank
// and throttle. It is the middle fidelity level between kinematics and 6-DOF
// (see "Choose fidelity per archetype" in framework/Design.md): turns, climbs
// and energy behave as they should, and rotational dynamics are left out.
//
// The frame is local Cartesian: x east, y north, z up, so z is altitude.
// Headings are measured from north toward east.
namespace simon::aircraft {

struct AirStateRate;

// An aircraft's position and its motion through the air. Its velocity follows
// from speed, flight-path angle and heading. A world of aircraft can use it as
// its spatial component, and Continuous can integrate it.
struct AirState final {
  using RateComponent = AirStateRate;
  Position position = meters(0.0, 0.0, 0.0);
  Speed speed = 0.0 * meter_per_second;
  Angle flight_path_angle = 0.0 * radian;  // Above the horizon.
  Angle heading = 0.0 * radian;            // From north toward east.
};

struct AirStateRate final {
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
  AccelerationMagnitude acceleration = 0.0 * meter_per_second_squared;
  AngularRate climb = 0.0 * radian_per_second;  // Of the flight-path angle.
  AngularRate turn = 0.0 * radian_per_second;   // Of the heading.
};

inline auto operator+(const AirStateRate& a, const AirStateRate& b)
    -> AirStateRate {
  return {.velocity = a.velocity + b.velocity,
          .acceleration = a.acceleration + b.acceleration,
          .climb = a.climb + b.climb,
          .turn = a.turn + b.turn};
}

inline auto operator*(double weight, const AirStateRate& rate) -> AirStateRate {
  return {.velocity = rate.velocity * weight,
          .acceleration = weight * rate.acceleration,
          .climb = weight * rate.climb,
          .turn = weight * rate.turn};
}

// Moves `state` along `rate` for `dt`, keeping the heading in [-pi, pi].
inline auto advance(const AirState& state, const AirStateRate& rate,
                    Duration dt) -> AirState {
  Time seconds = simon::seconds(dt);
  return AirState{
      .position = state.position + rate.velocity * seconds,
      .speed = state.speed + rate.acceleration * seconds,
      .flight_path_angle = state.flight_path_angle + rate.climb * seconds,
      .heading = wrap(state.heading + rate.turn * seconds),
  };
}

inline auto compute_velocity(const AirState& state) -> Velocity {
  double horizontal = cos(state.flight_path_angle);
  return QuantityVector{horizontal * sin(state.heading),
                        horizontal * cos(state.heading),
                        sin(state.flight_path_angle)} *
         state.speed;
}

// The height of a position above sea level: its z.
inline auto altitude_of(const Position& position) -> Length {
  return position.numerical_value_ref_in(meter).z() * meter;
}
inline auto altitude_of(const AirState& state) -> Length {
  return altitude_of(state.position);
}

//-- As a spatial component ---------------------------------------------------

inline auto distance(const AirState& a, const AirState& b) -> Length {
  return norm(a.position - b.position);
}
inline auto coordinates(const AirState& state) -> Coordinates {
  return model::coordinates(state.position);
}
inline auto coordinate_length(const AirState&, Length length) -> double {
  return length.numerical_value_in(meter);
}
// Pointing along the velocity, with no bank.
inline auto pose(const AirState& state) -> model::Pose {
  double yaw = std::numbers::pi / 2.0 - radians(state.heading);
  double pitch = radians(state.flight_path_angle);
  Quaternion orientation = Quaternion{AngleAxis{yaw, Vector3::UnitZ()}} *
                           Quaternion{AngleAxis{-pitch, Vector3::UnitY()}};
  return model::Pose{.position = state.position, .orientation = orientation};
}

//-- Dynamics -----------------------------------------------------------------

// The controls the pilot or autopilot has actually set, after the airframe's
// lags and limits: lift as a multiple of weight, bank, and the fraction of
// thrust.
struct FlightControls final {
  double load_factor = 1.0;
  Angle bank = 0.0 * radian;
  double throttle = 0.0;
};

// The properties of an aircraft the dynamics read every step: mass, the drag
// polar CD = CD0 + K1 * CL + K * CL^2 over its wing area, and its sea-level
// thrust, which falls with air density. K1 lets the polar's least drag fall at
// a lift other than zero, as a cambered wing's does (Raymer; see
// model/REFERENCES.md).
struct Airframe final {
  Mass mass = 1.0 * kilogram;
  Area wing_area = 1.0 * square_meter;
  double zero_lift_drag = 0.02;  // CD0.
  double lift_drag = 0.0;        // K1.
  double induced_drag = 0.05;    // K.
  Force thrust = 0.0 * newton;   // At full throttle, at sea level.
};

// The rate of `state` under `controls`, for the point-mass equations (Hull;
// see model/REFERENCES.md):
//
//   dV/dt     = (T - D) / m - g sin(gamma)
//   dgamma/dt = g / V * (n cos(mu) - cos(gamma))
//   dchi/dt   = g n sin(mu) / (V cos(gamma))
//
// with lift n m g, drag from the drag polar, and thrust scaled by the density
// ratio. Speed and cos(gamma) are kept away from zero in the divisions.
inline auto compute_point_mass_rate(const AirState& state,
                                    const FlightControls& controls,
                                    const Airframe& airframe,
                                    const earth::Air& air) -> AirStateRate {
  constexpr Density SEA_LEVEL_DENSITY = 1.225 * kilogram_per_cubic_meter;
  constexpr AccelerationMagnitude g = earth::STANDARD_GRAVITY;
  Speed v = max(state.speed, 1.0 * meter_per_second);  // To divide by.

  // Each sine and cosine once: they are most of this function's cost.
  double sin_gamma = sin(state.flight_path_angle);
  double cos_gamma = cos(state.flight_path_angle);
  double sin_chi = sin(state.heading);
  double cos_chi = cos(state.heading);
  double sin_mu = sin(controls.bank);
  double cos_mu = cos(controls.bank);

  // Lift, drag and thrust.
  Force dynamic_pressure_area = 0.5 * air.density * v * v * airframe.wing_area;
  Force lift = controls.load_factor * airframe.mass * g;
  double lift_coefficient = dynamic_pressure_area > 0.0 * newton
                                ? number_of(lift / dynamic_pressure_area)
                                : 0.0;

  Force drag = dynamic_pressure_area *
               (airframe.zero_lift_drag +
                lift_coefficient * (airframe.lift_drag +
                                    airframe.induced_drag * lift_coefficient));
  Force thrust = controls.throttle * airframe.thrust *
                 number_of(air.density / SEA_LEVEL_DENSITY);

  return AirStateRate{
      .velocity =
          QuantityVector{cos_gamma * sin_chi, cos_gamma * cos_chi, sin_gamma} *
          state.speed,
      .acceleration = (thrust - drag) / airframe.mass - g * sin_gamma,
      .climb = g / v * (controls.load_factor * cos_mu - cos_gamma) * radian,
      .turn = g * controls.load_factor * sin_mu /
              (v * std::max(cos_gamma, 1e-3)) * radian,
  };
}

// Advances `state` by `dt` in one pass, semi-implicitly, as symplectic Euler
// does (Hairer, Lubich and Wanner; see model/REFERENCES.md): speed and angles
// first, from `rate`, then position along the new velocity. One evaluation
// per step, and stable for the slow modes this model has at the steps a large
// simulation takes. `rate` must be compute_point_mass_rate's for `state`, whose
// velocity is the state's.
//
// The new velocity is the old one plus its derivative over the step, which
// follows from the rate and the old direction of flight without another sine
// or cosine.
inline auto fly(const AirState& state, const AirStateRate& rate, Duration dt)
    -> AirState {
  Time seconds = simon::seconds(dt);
  AirState next{
      .position = state.position,
      .speed = state.speed + rate.acceleration * seconds,
      .flight_path_angle = state.flight_path_angle + rate.climb * seconds,
      .heading = wrap(state.heading + rate.turn * seconds),
  };

  // At rest there is no direction of flight to work from.
  if (state.speed <= 0.0 * meter_per_second) {
    next.position += compute_velocity(next) * seconds;
    return next;
  }

  // The direction of flight, a unit vector, and its sines and cosines.
  QuantityVector along = number_of(rate.velocity / state.speed);

  double sin_gamma = along.z();
  double cos_gamma = std::sqrt(along.x() * along.x() + along.y() * along.y());
  double sin_chi = cos_gamma > 0.0 ? along.x() / cos_gamma : 0.0;
  double cos_chi = cos_gamma > 0.0 ? along.y() / cos_gamma : 1.0;

  // The velocity changes along the direction of flight with speed, and across
  // it with flight-path angle and heading.
  QuantityVector by_gamma{-sin_gamma * sin_chi, -sin_gamma * cos_chi,
                          cos_gamma};
  QuantityVector by_heading{cos_gamma * cos_chi, -cos_gamma * sin_chi, 0.0};
  Acceleration acceleration =
      along * rate.acceleration +
      (by_gamma * (rate.climb / radian) + by_heading * (rate.turn / radian)) *
          state.speed;

  next.position += (rate.velocity + acceleration * seconds) * seconds;
  return next;
}

//-- Autopilot laws -----------------------------------------------------------

// The flight-path angle that climbs or descends toward `altitude` at
// `response` (per second) of the error, at most `steepest` either way.
inline auto compute_climb_command(const AirState& state, Length altitude,
                                  Rate response, Angle steepest) -> Angle {
  Speed v = max(state.speed, 1.0 * meter_per_second);
  Speed climb_rate = response * (altitude - altitude_of(state));
  Angle climb = arcsin(std::clamp(number_of(climb_rate / v), -1.0, 1.0));
  return clamp(climb, -steepest, steepest);
}

// The load factor that turns the flight-path angle toward `commanded` at
// `response` (per second) of the error, in a bank of `bank`.
inline auto compute_load_factor_command(const AirState& state, Angle commanded,
                                        Angle bank, Rate response) -> double {
  Speed v = max(state.speed, 1.0 * meter_per_second);
  AngularRate climb = response * (commanded - state.flight_path_angle);
  double pull_up = number_of(v / earth::STANDARD_GRAVITY * (climb / radian));
  double cos_mu = std::max(cos(bank), 0.1);
  return (cos(state.flight_path_angle) + pull_up) / cos_mu;
}

// The bank that turns toward `heading` at `response` (per second) of the
// error, the short way round, in a coordinated turn of at most `steepest`.
inline auto compute_bank_command(const AirState& state, Angle heading,
                                 Rate response, Angle steepest) -> Angle {
  AngularRate turn = response * wrap(heading - state.heading);
  Angle bank = arctan(
      number_of(state.speed * (turn / radian) / earth::STANDARD_GRAVITY));
  return clamp(bank, -steepest, steepest);
}

// The heading from `from` to `to`, over the ground. The components are in
// meters only to reach atan2.
inline auto compute_bearing(const Position& from, const Position& to) -> Angle {
  QuantityVector apart = (to - from).numerical_value_in(meter);
  return std::atan2(apart.x(), apart.y()) * radian;
}

// The distance from `from` to `to`, over the ground.
inline auto ground_distance(const Position& from, const Position& to)
    -> Length {
  QuantityVector apart = (to - from).numerical_value_in(meter);
  return std::hypot(apart.x(), apart.y()) * meter;
}

}  // namespace simon::aircraft
