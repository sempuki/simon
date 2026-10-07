// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstdint>

#include "core/time.hpp"
#include "core/units.hpp"
#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/world.hpp"
#include "model/aircraft/aircraft_data.hpp"
#include "model/aircraft/flight_path.hpp"
#include "model/aircraft/rigid_aircraft.hpp"
#include "model/earth/wind.hpp"
#include "model/rigid_body.hpp"

// Aircraft fly routes of waypoints under an autopilot. Most fly a cheap,
// single-pass point-mass model; aircraft whose archetype opts in are
// integrated with Runge-Kutta 4 instead. Rigid aircraft, the highest fidelity,
// fly six degrees of freedom by their control surfaces. Aircraft that have a
// Wind fly in moving air, and rigid aircraft that have Gusts fly through
// turbulence.
namespace simon::aeronautic {

using aircraft::Airframe;
using aircraft::AirState;
using aircraft::AirStateRate;
using aircraft::BodyAcceleration;
using aircraft::Engines;
using aircraft::FlightControls;
using aircraft::FlightSignals;
using aircraft::FuelTanks;
using aircraft::MassBalance;
using earth::Gusts;
using earth::Wind;
using framework::Entity;
using model::RigidBody;
using model::RigidBodyRate;

// The autopilot's commands, which the airframe follows with lags and limits
// (see Handling).
struct Commands final {
  double load_factor = 1.0;
  Angle bank = 0.0 * radian;
  double throttle = 0.0;
};

// The airframe's response to its commands, and its limits. Read by the
// autopilot and the actuators, never by the dynamics.
struct Handling final {
  double max_load_factor = 3.0;
  double min_load_factor = 0.0;
  Angle max_bank = 1.0 * radian;
  AngularRate roll_rate = 1.0 * radian_per_second;
  Time load_factor_lag = 0.5 * second;
  Time throttle_lag = 2.0 * second;
};

// The altitude, heading and speed the autopilot holds, which the route sets.
// Its throttle integral is the speed loop's state.
struct Autopilot final {
  Length altitude = 0.0 * meter;
  Angle heading = 0.0 * radian;
  Speed speed = 0.0 * meter_per_second;
  double throttle_integral = 0.0;
};

// A closed route of waypoints, flown in order. Each waypoint's altitude is
// its z, and the aircraft flies to it at `speed`.
struct Route final {
  static constexpr std::size_t SIZE = 4;
  std::array<Position, SIZE> waypoints{};
  Speed speed = 0.0 * meter_per_second;
  std::uint32_t next = 0;     // The waypoint flown to.
  std::uint32_t reached = 0;  // Waypoints reached so far.
};

// A rigid aircraft's autopilot gains. The targets come from the point-mass
// autopilot's laws; these fly the surfaces to them. The defaults fly a 737,
// whose stick moves its elevator; an aircraft whose stick commands rates and
// load through fly-by-wire needs others.
struct SurfaceGains final {
  double bank = 1.0;      // Aileron per radian of bank error.
  double roll = 2.0;      // Aileron per rad/s of roll rate.
  double climb = 1.2;     // Elevator per radian of flight-path angle error.
  double pitch = 4.0;     // Elevator per rad/s of pitch rate.
  double integral = 1.0;  // Elevator per radian second of the same error.
  double speed = 0.05;    // Throttle per m/s of speed error.
  // Up to 45 degrees of bank by default: a 737 turns on a radius of about 4 km
  // at 200 m/s, near the 3 km FollowRoute captures a waypoint at, so it
  // seldom circles one.
  Angle max_bank = 0.79 * radian;
};

// A rigid aircraft's autopilot: its gains, the trim it flies about, and the
// integral of its flight-path angle error.
struct SurfaceAutopilot final {
  SurfaceGains gains;
  double pitch_trim = 0.0;  // The pitch trim command it holds.
  double throttle_trim = 0.0;
  double climb_integral = 0.0;  // rad s.
};

// The data a rigid aircraft flies by, which every aircraft of its type
// shares. It outlives the world.
struct AircraftType final {
  const aircraft::Definition* data = nullptr;
};

namespace archetype {

using framework::Allows;
using framework::Archetype;
using framework::Requires;

// Flies the single-pass point-mass model. A Wind carries it with the air.
struct Aircraft final                                                   //
    : Archetype<"aircraft",                                             //
                Requires<AirState, FlightControls, Commands, Airframe,  //
                         Handling, Autopilot, Route>,                   //
                Allows<Wind>> {};                                       //

// Opts in to Runge-Kutta 4 by having the rate of its AirState.
struct PreciseAircraft final                                       //
    : Archetype<"precise aircraft",                                //
                Requires<AirState, AirStateRate, FlightControls,   //
                         Commands, Airframe, Handling, Autopilot,  //
                         Route>,                                   //
                Allows<Wind>> {};                                  //

// Flies six degrees of freedom, integrated with Runge-Kutta 4: its flight
// controls turn its commands, in its FlightSignals, into the control surface
// positions its aerodynamics read, and its engines make thrust from its
// throttles and burn its fuel. It follows a route as any aircraft does, its
// autopilot flying the surfaces. Its AirState follows its body, for the rest of
// the world.
struct RigidAircraft final                                      //
    : Archetype<"rigid aircraft",                               //
                Requires<AirState, RigidBody, RigidBodyRate,    //
                         BodyAcceleration, FlightSignals,       //
                         Engines, FuelTanks,                    //
                         MassBalance, AircraftType, Autopilot,  //
                         Route, SurfaceAutopilot>> {};          //

// A rigid aircraft in moving air: its Wind moves the air it flies through, and
// Gusts, if it has them, add turbulence. An archetype of its own, so that
// rigid aircraft in still air never look for a wind.
struct RigidAircraftInWind final                                //
    : Archetype<"rigid aircraft in wind",                       //
                Requires<AirState, RigidBody, RigidBodyRate,    //
                         BodyAcceleration, FlightSignals,       //
                         Engines, FuelTanks,                    //
                         MassBalance, AircraftType, Autopilot,  //
                         Route, SurfaceAutopilot, Wind>,        //
                Allows<Gusts>> {};                              //

}  // namespace archetype

using World = framework::World<
    AirState,
    framework::TypeList<AirStateRate, FlightControls, Commands, Airframe,
                        Handling, Autopilot, Route, RigidBody, RigidBodyRate,
                        BodyAcceleration, FlightSignals, Engines, FuelTanks,
                        MassBalance, AircraftType, SurfaceAutopilot, Wind,
                        Gusts>,
    framework::TypeList<archetype::Aircraft, archetype::PreciseAircraft,
                        archetype::RigidAircraft,
                        archetype::RigidAircraftInWind>>;

}  // namespace simon::aeronautic
