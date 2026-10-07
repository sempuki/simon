// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#pragma once

#include "core/units.hpp"
#include "model/aircraft/aircraft_data.hpp"

// A running turbine, as JSBSim models one (FGTurbine; see
// model/REFERENCES.md): its spools turn toward speeds the throttle sets, at
// rates that fall as they slow and as the air thins, and its thrust runs from
// idle to military as the core spool's speed above idle, squared. Fuel flows
// at the thrust times a specific consumption that rises at part power and
// with temperature, at least the idle flow, and following its target at
// limited rates.
//
// A turbine with reheat lights it with a throttle past 1, as JSBSim's does
// with its augmentation method 2: thrust runs from military to maximum as the
// throttle goes from 1 to 2, and fuel flows at the thrust times the reheat's
// specific consumption. A step after the reheat goes out, fuel flow and bleed
// are as they were, as JSBSim's are for a frame.
//
// Thrust is held for a step, as JSBSim holds it for a frame. Starting,
// stalling and water injection are left out.
namespace simon::model {

// The air a turbine breathes.
struct EngineAir final {
  double mach = 0.0;
  Length density_altitude = 0.0 * meter;
  double density_ratio = 1.0;  // To the standard sea level's.
  Temperature temperature = units::delta<kelvin>(288.15);
};

struct TurbineState final {
  double n1 = 0.0;         // Fan speed, percent.
  double n2 = 0.0;         // Core speed, percent.
  double fuel_flow = 0.0;  // kg/s.
  Force thrust = 0.0 * newton;
  bool reheat = false;  // Lit through the step.
};

// A turbine settled at `throttle` (from 0 to 1, or to 2 with reheat) in
// `air`.
auto compute_steady_turbine(const TurbineData& turbine, double throttle,
                            const EngineAir& air) -> TurbineState;

// `state` advanced by `dt` at `throttle` in `air`.
auto run_turbine(const TurbineData& turbine, const TurbineState& state,
                 double throttle, const EngineAir& air, Time dt)
    -> TurbineState;

}  // namespace simon::model
