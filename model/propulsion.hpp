// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>

#include "model/aircraft_data.hpp"
#include "model/atmosphere.hpp"
#include "model/frames.hpp"
#include "model/mass_balance.hpp"
#include "model/rigid_body.hpp"
#include "model/turbine.hpp"
#include "model/units.hpp"
#include "model/wind.hpp"

// A rigid aircraft's engines: the air they breathe, their spools and thrust
// at their throttles, and the fuel they burn from its tanks.
namespace simon::model {

// Each engine's state, in the aircraft data's order.
struct Engines final {
  std::array<TurbineState, MAX_ENGINES> turbines{};
};

// The engines settled at the throttles in `signals`, in `air`.
auto settled_engines(const AircraftData& aircraft, const FlightSignals& signals,
                     const EngineAir& air) -> Engines;

// The air the engines breathe at `body`, moving as `wind` has it. Over the
// standard atmosphere the density altitude is the altitude.
auto compute_engine_air(const RigidBody& body, const Earth& earth,
                        const StandardAirTable& air, const Wind& wind,
                        Time time) -> EngineAir;

// Advances each engine by `dt` at its throttle in `signals`. An engine whose
// tanks are empty makes no thrust and burns nothing.
auto run_engines(const AircraftData& aircraft, InOut<Engines> engines,
                 const FlightSignals& signals, const FuelTanks& tanks,
                 const EngineAir& air, Time dt) -> void;

// Burns each engine's fuel flow for `dt`, from its feed tanks that have fuel,
// in equal shares, as JSBSim does.
auto burn_fuel(const AircraftData& aircraft, const Engines& engines,
               InOut<FuelTanks> tanks, Time dt) -> void;

}  // namespace simon::model
