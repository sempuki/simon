// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstdint>

#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/step.hpp"
#include "framework/world.hpp"
#include "model/flight_path.hpp"
#include "model/units.hpp"

// Aircraft fly routes of waypoints under an autopilot. Most fly a cheap,
// single-pass point-mass model; aircraft whose archetype opts in are
// integrated with Runge-Kutta 4 instead.
namespace simon::flight {

using framework::Duration;
using framework::Entity;
using framework::Step;
using framework::TimePoint;
using model::AirState;
using model::AirStateRate;
using model::Airframe;
using model::Angle;
using model::FlightControls;
using model::Length;
using model::Position;
using model::Rate;
using model::Speed;
using model::Time;

// What the autopilot commands; the airframe follows it with lags and limits
// (see Handling).
struct Commands final {
  double load_factor = 1.0;
  Angle bank = 0.0 * model::radian;
  double throttle = 0.0;
};

// How the airframe follows its commands, and how hard it may be flown. Read
// by the autopilot and the actuators, never by the dynamics.
struct Handling final {
  double max_load_factor = 3.0;
  double min_load_factor = 0.0;
  Angle max_bank = 1.0 * model::radian;
  model::AngularRate roll_rate = 1.0 * model::radian_per_second;
  Time load_factor_lag = 0.5 * model::second;
  Time throttle_lag = 2.0 * model::second;
};

// What the autopilot holds: an altitude, a heading and a speed, which the
// route sets. Its throttle integral is the speed loop's state.
struct Autopilot final {
  Length altitude = 0.0 * model::meter;
  Angle heading = 0.0 * model::radian;
  Speed speed = 0.0 * model::meter_per_second;
  double throttle_integral = 0.0;
};

// A closed route of waypoints, flown in order. Each waypoint's altitude is
// its z, and the aircraft flies to it at `speed`.
struct Route final {
  static constexpr std::size_t SIZE = 4;
  std::array<Position, SIZE> waypoints{};
  Speed speed = 0.0 * model::meter_per_second;
  std::uint32_t next = 0;     // The waypoint flown to.
  std::uint32_t reached = 0;  // Waypoints reached so far.
};

namespace archetype {

using framework::Archetype;
using framework::Requires;

// Flies the single-pass point-mass model.
struct Aircraft final                                                   //
    : Archetype<"aircraft",                                             //
                Requires<AirState, FlightControls, Commands, Airframe,  //
                         Handling, Autopilot, Route>> {};               //

// Opts in to Runge-Kutta 4 by having the rate of its AirState.
struct PreciseAircraft final                                            //
    : Archetype<"precise aircraft",                                     //
                Requires<AirState, AirStateRate, FlightControls,        //
                         Commands, Airframe, Handling, Autopilot,       //
                         Route>> {};                                    //

}  // namespace archetype

using World = framework::World<
    AirState,
    framework::TypeList<AirStateRate, FlightControls, Commands, Airframe,
                        Handling, Autopilot, Route>,
    framework::TypeList<archetype::Aircraft, archetype::PreciseAircraft>>;

}  // namespace simon::flight
