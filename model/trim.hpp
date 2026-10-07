// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <array>
#include <cstddef>
#include <expected>

#include "base/status.hpp"
#include "core/units.hpp"
#include "model/aircraft_data.hpp"
#include "model/atmosphere.hpp"
#include "model/flight_control.hpp"
#include "model/rigid_aircraft.hpp"
#include "model/rigid_body.hpp"

// A rigid aircraft's trim: the attitude, controls and throttle at which it
// flies a steady, straight path, its forces and moments in balance. Round the
// Earth the path is level over the Earth, at a steady altitude, the body
// turning as the local frame turns under it; or, if asked, straight in space,
// as JSBSim's trim is, which rises as the Earth curves away beneath it.
//
// Six unknowns balance six accelerations, as JSBSim's full trim pairs them:
// angle of attack the acceleration along body z, throttle along body x, pitch
// trim the pitch, bank the acceleration along body y, aileron the roll and
// rudder the yaw. There is no sideslip. Newton's method solves them together,
// with a finite-difference Jacobian (Dennis and Schnabel; see
// model/REFERENCES.md), to accelerations near rounding.
//
// Each evaluation settles the flight controls with the airframe: the blocks
// stand as they do when their inputs hold still, and read the state and the
// accelerations the pilot feels, which their surfaces set, until the two
// agree.
namespace simon::model {

// A steady, straight flight: where, how fast through the air, heading where,
// and climbing at what angle, with what fuel and the pilot's other commands,
// such as flaps and gear.
struct FlightCondition final {
  Position position = meters(0.0, 0.0, 0.0);  // In the world's local frame.
  Speed speed = 0.0 * meter_per_second;
  Angle heading = 0.0 * radian;  // From local north.
  Angle flight_path_angle = 0.0 * radian;
  FuelTanks tanks;
  FlightSignals commands;
  bool level = true;  // Over the Earth; else straight in space.
};

// A trimmed aircraft: what it starts flying with, and the trim found.
struct Trim final {
  RigidBody body;
  FlightSignals signals;
  Engines engines;
  MassBalance mass;
  BodyAcceleration felt;

  Angle alpha = 0.0 * radian;
  Angle pitch = 0.0 * radian;
  Angle bank = 0.0 * radian;
  double throttle = 0.0;  // Each engine's command.
  double pitch_trim = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;

  // The largest acceleration left: m/s^2 along the body's axes, or rad/s^2
  // about them times the wingspan.
  double residual = 0.0;
};

// The reasons an aircraft cannot be trimmed.
enum class TrimError {
  DIVERGED,   // Newton's method stopped short of balance.
  SATURATED,  // Balance needs a control past its limit.
  COUNT,
};

inline constexpr std::size_t TRIM_ERROR_COUNT =
    static_cast<std::size_t>(TrimError::COUNT);

// Trims `aircraft` for `condition` over `earth`, at time zero.
auto trim(const AircraftData& aircraft, const FlightCondition& condition,
          const Earth& earth, const StandardAirTable& air)
    -> std::expected<Trim, lib::Status>;

}  // namespace simon::model

// Messages for each TrimError, defined in trim.cpp.
template <>
const std::array<lib::StatusConditionEntry, simon::model::TRIM_ERROR_COUNT>
    lib::EnumStatusKindConditionMixin<
        simon::model::TrimError, simon::model::TRIM_ERROR_COUNT>::conditions_;
