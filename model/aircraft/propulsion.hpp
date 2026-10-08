// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <array>

#include "core/argument.hpp"
#include "core/units.hpp"
#include "model/aircraft/definition.hpp"
#include "model/aircraft/frames.hpp"
#include "model/aircraft/mass_balance.hpp"
#include "model/aircraft/turbine.hpp"
#include "model/earth/atmosphere.hpp"
#include "model/earth/wind.hpp"
#include "model/rigid_body.hpp"

// A rigid aircraft's engines: the air they breathe, their spools and thrust
// at their throttles, and the fuel they burn from its tanks.
namespace simon::aircraft {

// Each engine's state, in the aircraft data's order.
struct Engines final {
  std::array<TurbineState, MAX_ENGINES> turbines{};
};

// The engines settled at the throttles in `signals`, in `air`.
auto compute_settled_engines(const Definition& aircraft,
                             const FlightSignals& signals, const EngineAir& air)
    -> Engines;

// The air the engines breathe at `body`, moving as `wind` has it. Over the
// standard atmosphere the density altitude is the altitude.
auto compute_engine_air(const model::RigidBody& body, const Earth& earth,
                        const earth::StandardAirTable& air,
                        const earth::Wind& wind, Time time) -> EngineAir;

// Advances each engine by `dt` at its throttle in `signals`. An engine whose
// tanks are empty makes no thrust and burns nothing.
auto run_engines(const Definition& aircraft, InOut<Engines> engines,
                 const FlightSignals& signals, const FuelTanks& tanks,
                 const EngineAir& air, Time dt) -> void;

// Burns each engine's fuel flow for `dt`, from its feed tanks that have fuel,
// in equal shares, as JSBSim's FGPropulsion does.
auto burn_fuel(const Definition& aircraft, const Engines& engines,
               InOut<FuelTanks> tanks, Time dt) -> void;

}  // namespace simon::aircraft
