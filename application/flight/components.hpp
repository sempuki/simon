// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstdint>

#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/step.hpp"
#include "framework/world.hpp"
#include "model/aircraft_data.hpp"
#include "model/flight_path.hpp"
#include "model/rigid_aircraft.hpp"
#include "model/rigid_body.hpp"
#include "model/units.hpp"

// Aircraft fly routes of waypoints under an autopilot. Most fly a cheap,
// single-pass point-mass model; aircraft whose archetype opts in are
// integrated with Runge-Kutta 4 instead. Rigid aircraft, the highest fidelity,
// fly six degrees of freedom by their control surfaces.
namespace simon::flight {

using framework::Duration;
using framework::Entity;
using framework::Step;
using framework::TimePoint;
using model::Airframe;
using model::AirState;
using model::AirStateRate;
using model::Angle;
using model::BodyAcceleration;
using model::Engines;
using model::FlightControls;
using model::FlightSignals;
using model::FuelTanks;
using model::Length;
using model::MassBalance;
using model::Position;
using model::Rate;
using model::RigidBody;
using model::RigidBodyRate;
using model::Speed;
using model::Time;

// The autopilot's commands, which the airframe follows with lags and limits
// (see Handling).
struct Commands final {
  double load_factor = 1.0;
  Angle bank = 0.0 * model::radian;
  double throttle = 0.0;
};

// The airframe's response to its commands, and its limits. Read by the
// autopilot and the actuators, never by the dynamics.
struct Handling final {
  double max_load_factor = 3.0;
  double min_load_factor = 0.0;
  Angle max_bank = 1.0 * model::radian;
  model::AngularRate roll_rate = 1.0 * model::radian_per_second;
  Time load_factor_lag = 0.5 * model::second;
  Time throttle_lag = 2.0 * model::second;
};

// The altitude, heading and speed the autopilot holds, which the route sets.
// Its throttle integral is the speed loop's state.
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

// A rigid aircraft's autopilot state: the trim it flies about, and the
// integral of its flight-path angle error.
struct SurfaceAutopilot final {
  double pitch_trim = 0.0;  // The pitch trim command it holds.
  double throttle_trim = 0.0;
  double climb_integral = 0.0;  // rad s.
};

// The data a rigid aircraft flies by, which every aircraft of its type
// shares. It outlives the world.
struct AircraftType final {
  const model::AircraftData* data = nullptr;
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
struct PreciseAircraft final                                       //
    : Archetype<"precise aircraft",                                //
                Requires<AirState, AirStateRate, FlightControls,   //
                         Commands, Airframe, Handling, Autopilot,  //
                         Route>> {};                               //

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

}  // namespace archetype

using World = framework::World<
    AirState,
    framework::TypeList<AirStateRate, FlightControls, Commands, Airframe,
                        Handling, Autopilot, Route, RigidBody, RigidBodyRate,
                        BodyAcceleration, FlightSignals, Engines, FuelTanks,
                        MassBalance, AircraftType, SurfaceAutopilot>,
    framework::TypeList<archetype::Aircraft, archetype::PreciseAircraft,
                        archetype::RigidAircraft>>;

}  // namespace simon::flight
