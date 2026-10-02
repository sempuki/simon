// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/turbine.hpp"

#include "model/aerodynamics.hpp"

#include <algorithm>
#include <cmath>

namespace simon::model {

namespace {

// The rates fuel flow follows its target at, up and down: 1000 and 10000
// pounds per hour per second in JSBSim.
constexpr double FUEL_FLOW_UP = 1000.0 * 0.45359237 / 3600.0;     // kg/s^2.
constexpr double FUEL_FLOW_DOWN = 10000.0 * 0.45359237 / 3600.0;  // kg/s^2.

// The temperature JSBSim's specific consumption is normalized to: 389.7
// degrees Rankine, the tropopause's.
constexpr double CONSUMPTION_TEMPERATURE = 389.7 / 1.8;  // K.

// `value` moved toward `target` by at most `up` or `down` times `dt`.
auto seek(double value, double target, double up, double down, double dt)
    -> double {
  if (value > target) {
    return std::max(value - dt * down, target);
  }
  if (value < target) {
    return std::min(value + dt * up, target);
  }
  return value;
}

auto inputs_of(const EngineAir& air) -> AeroInputs {
  AeroInputs inputs;
  inputs[AeroVariable::MACH] = air.mach;
  inputs[AeroVariable::DENSITY_ALTITUDE] =
      air.density_altitude.numerical_value_in(meter);
  return inputs;
}

// Thrust before bleed, at a core speed `n2_fraction` of the way from idle to
// full.
auto gross_thrust(const TurbineData& turbine, const EngineAir& air,
                  double n2_fraction) -> double {
  AeroInputs inputs = inputs_of(air);
  double military = turbine.military_thrust.numerical_value_in(newton);
  double idle = military * (*turbine.idle_thrust)(inputs);
  // JSBSim scales the span from idle by the military table.
  double span = (military - idle) * (*turbine.military_thrust_factor)(inputs);
  return idle + span * n2_fraction * n2_fraction;
}

// Specific fuel consumption, kg/s per N, at a core speed `n2_fraction` above
// idle.
auto consumption(const TurbineData& turbine, const EngineAir& air,
                 double n2_fraction) -> double {
  double kelvins = air.temperature.numerical_value_in(kelvin);
  double part = 1.0 - n2_fraction;
  return turbine.thrust_specific_fuel_consumption *
         std::sqrt(kelvins / CONSUMPTION_TEMPERATURE) * (0.84 + part * part);
}

// A spool's rate, scaled down when the core is slow and the air thin.
auto spool_rate(double rate, double n2_fraction, double density_ratio)
    -> double {
  double n = std::min(1.0, n2_fraction + 0.1);
  double slow = 1.0 - n;
  return rate / (1.0 + 3.0 * slow * slow * slow + (1.0 - density_ratio));
}

}  // namespace

auto steady_turbine(const TurbineData& turbine, double throttle,
                    const EngineAir& air) -> TurbineState {
  double fraction = std::clamp(throttle, 0.0, 1.0);
  double gross = gross_thrust(turbine, air, fraction);
  double flow = std::max(gross * consumption(turbine, air, fraction),
                         turbine.idle_fuel_flow);
  return TurbineState{
      .n1 = turbine.idle_n1 + fraction * (turbine.max_n1 - turbine.idle_n1),
      .n2 = turbine.idle_n2 + fraction * (turbine.max_n2 - turbine.idle_n2),
      .fuel_flow = flow,
      .thrust = gross * (1.0 - turbine.bleed) * newton,
  };
}

auto run_turbine(const TurbineData& turbine, const TurbineState& state,
                 double throttle, const EngineAir& air, Time dt)
    -> TurbineState {
  double seconds = dt.numerical_value_in(second);
  throttle = std::clamp(throttle, 0.0, 1.0);
  double n1_span = turbine.max_n1 - turbine.idle_n1;
  double n2_span = turbine.max_n2 - turbine.idle_n2;

  // The spools' rates follow the core's speed before this step.
  double before = (state.n2 - turbine.idle_n2) / n2_span;
  double ratio = air.density_ratio;
  double n2 = seek(state.n2, turbine.idle_n2 + throttle * n2_span,
                   spool_rate(turbine.n2_spool_up, before, ratio),
                   spool_rate(turbine.n2_spool_down, before, ratio), seconds);
  double n1 = seek(state.n1, turbine.idle_n1 + throttle * n1_span,
                   spool_rate(turbine.n1_spool_up, before, ratio),
                   spool_rate(turbine.n1_spool_down, before, ratio), seconds);

  double fraction = (n2 - turbine.idle_n2) / n2_span;
  double gross = gross_thrust(turbine, air, fraction);
  double flow =
      seek(state.fuel_flow, gross * consumption(turbine, air, fraction),
           FUEL_FLOW_UP, FUEL_FLOW_DOWN, seconds);
  return TurbineState{
      .n1 = n1,
      .n2 = n2,
      .fuel_flow = std::max(flow, turbine.idle_fuel_flow),
      .thrust = gross * (1.0 - turbine.bleed) * newton,
  };
}

}  // namespace simon::model
