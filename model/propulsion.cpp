// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#include "model/propulsion.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace simon::model {

auto compute_settled_engines(const AircraftData& aircraft,
                             const FlightSignals& signals, const EngineAir& air)
    -> Engines {
  Engines engines;
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    engines.turbines[i] = compute_steady_turbine(
        aircraft.engines[i],
        signals.values[aircraft.flight_controls.throttles[i]], air);
  }
  return engines;
}

auto compute_engine_air(const RigidBody& body, const Earth& earth,
                        const StandardAirTable& air, const Wind& wind,
                        Time time) -> EngineAir {
  Length altitude = 0.0 * meter;
  double speed = 0.0;
  if (is_still(wind)) {
    altitude = earth.altitude(body, time);
    speed = eigen(earth.air_velocity(body)).norm();
  } else {
    Place place = earth.place(body, time);
    altitude = place.altitude;
    speed = compute_inertial_air_velocity(body, earth, place, wind).norm();
  }
  Air here = air(altitude);
  double sound = here.speed_of_sound.numerical_value_in(meter_per_second);
  // The speed of sound is sqrt(gamma R T).
  double temperature =
      sound * sound /
      (internal::HEAT_RATIO *
       internal::GAS_CONSTANT.numerical_value_in(joule_per_kilogram_kelvin));
  return EngineAir{
      .mach = speed / sound,
      .density_altitude = altitude,
      .density_ratio =
          number_of(here.density / standard_air(0.0 * meter).density),
      .temperature = units::delta<kelvin>(temperature),
  };
}

namespace {

auto has_fuel(const TurbineData& turbine, const FuelTanks& tanks) -> bool {
  return std::ranges::any_of(turbine.feeds, [&](std::size_t tank) {
    return tanks.contents[tank] > 0.0 * kilogram;
  });
}

}  // namespace

auto run_engines(const AircraftData& aircraft, InOut<Engines> engines,
                 const FlightSignals& signals, const FuelTanks& tanks,
                 const EngineAir& air, Time dt) -> void {
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    const TurbineData& turbine = aircraft.engines[i];
    TurbineState& state = engines->turbines[i];
    if (!has_fuel(turbine, tanks)) {
      state.thrust = 0.0 * newton;
      state.fuel_flow = 0.0;
      continue;
    }
    state = run_turbine(turbine, state,
                        signals.values[aircraft.flight_controls.throttles[i]],
                        air, dt);
  }
}

auto burn_fuel(const AircraftData& aircraft, const Engines& engines,
               InOut<FuelTanks> tanks, Time dt) -> void {
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    const TurbineData& turbine = aircraft.engines[i];
    auto feeding = std::ranges::count_if(turbine.feeds, [&](std::size_t tank) {
      return tanks->contents[tank] > 0.0 * kilogram;
    });
    if (feeding == 0) {
      continue;
    }
    Mass share = engines.turbines[i].fuel_flow * dt.numerical_value_in(second) /
                 static_cast<double>(feeding) * kilogram;
    for (std::size_t tank : turbine.feeds) {
      if (tanks->contents[tank] > 0.0 * kilogram) {
        tanks->contents[tank] =
            max(tanks->contents[tank] - share, 0.0 * kilogram);
      }
    }
  }
}

}  // namespace simon::model
