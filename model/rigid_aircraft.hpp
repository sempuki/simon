// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#pragma once

#include "core/units.hpp"
#include "model/aerodynamics.hpp"
#include "model/aircraft_data.hpp"
#include "model/atmosphere.hpp"
#include "model/frames.hpp"
#include "model/mass_balance.hpp"
#include "model/propulsion.hpp"
#include "model/rigid_body.hpp"
#include "model/sensing.hpp"
#include "model/wind.hpp"

// An aircraft as a rigid body (see rigid_body.hpp), flown by its control
// surfaces through aerodynamics read from data (see aircraft_data.hpp). It is
// the highest fidelity level (see "Choose fidelity per archetype" in
// framework/Design.md), for the few aircraft whose handling matters. Its
// frames, mass balance, engines and sensing have headers of their own, which
// this one includes.
namespace simon::model {

// Computes the aerodynamics' inputs at `body`, all but the rate of angle of
// attack, which needs the body's acceleration. `reference` is the aerodynamic
// reference point from the center of mass, in body axes: ground effect reads
// its height.
auto compute_aero_inputs(const RigidBody& body, const FlightSignals& signals,
                         const AircraftData& aircraft,
                         const Displacement& reference, const Earth& earth,
                         const StandardAirTable& air, Time time) -> AeroInputs;
auto compute_aero_inputs(const RigidBody& body, const FlightSignals& signals,
                         const AircraftData& aircraft,
                         const Displacement& reference, const Earth& earth,
                         const Place& place, const StandardAirTable& air)
    -> AeroInputs;

// The rate of a rigid aircraft's body, under its aerodynamics, which read its
// flight control `signals` and its motion through the air that `wind` moves,
// its engines' thrust, along body x from where each is mounted, and gravity.
// Lift is summed first, so induced drag reads this step's lift coefficient,
// and the forces before the moments, so the rate of angle of attack that the
// moments read is this step's exact one, as long as no force reads it; if one
// does, the forces are found again with it, from the rate of the air velocity
// (see compute_air_acceleration). It writes what the body feels to `felt`.
auto compute_rigid_aircraft_rate(
    const RigidBody& body, const FlightSignals& signals, const Engines& engines,
    const MassBalance& mass, const AircraftData& aircraft, const Earth& earth,
    const StandardAirTable& air, const Wind& wind, Time time,
    Out<BodyAcceleration> felt) -> RigidBodyRate;

}  // namespace simon::model
