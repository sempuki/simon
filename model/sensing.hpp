// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#pragma once

#include "model/aircraft_data.hpp"
#include "model/atmosphere.hpp"
#include "model/frames.hpp"
#include "model/mass_balance.hpp"
#include "model/rigid_body.hpp"
#include "model/units.hpp"
#include "model/wind.hpp"

// What a rigid aircraft's flight controls sense: its air data, attitude and
// motion, and the accelerations its pilot feels.
namespace simon::model {

// What an aircraft's body feels, in body axes: the force on it but gravity's,
// per unit mass, and its angular acceleration.
struct BodyAcceleration final {
  Acceleration specific_force = meters_per_second_squared(0.0, 0.0, 0.0);
  AngularAcceleration angular = QuantityVector{} * radian_per_second_squared;
};

// Sets the state the flight controls read in `signals`, at `body` in the air
// that `wind` moves: air data, ground speed, body velocity, attitude, and the
// accelerations the pilot feels at the aircraft's eye point from `felt`. It
// finds only what the aircraft's flight controls read, besides its body
// velocity and angles of attack. JSBSim's flight controls read these from the
// frame before; here only `felt` is, because a step's own needs the surfaces
// the flight controls are about to set. No wheel carries weight.
auto sense_flight_state(const RigidBody& body, const BodyAcceleration& felt,
                        const MassBalance& mass, const AircraftData& aircraft,
                        const Earth& earth, const StandardAirTable& air,
                        const Wind& wind, Time time,
                        InOut<FlightSignals> signals) -> void;

}  // namespace simon::model
