// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cmath>

#include "model/units.hpp"

// The International Standard Atmosphere, from sea level to 20 km: a
// troposphere whose temperature falls linearly to 11 km, then an isothermal
// lower stratosphere.
namespace simon::model {

struct Air final {
  Density density = 0.0 * kilogram_per_cubic_meter;
  Speed speed_of_sound = 0.0 * meter_per_second;
};

namespace internal {
inline constexpr double GAS_CONSTANT = 287.052874;  // J / (kg K), dry air.
inline constexpr double GRAVITY = 9.80665;          // m / s^2.
inline constexpr double HEAT_RATIO = 1.4;
inline constexpr double SEA_LEVEL_TEMPERATURE = 288.15;  // K.
inline constexpr double SEA_LEVEL_PRESSURE = 101325.0;   // Pa.
inline constexpr double LAPSE_RATE = 0.0065;             // K / m.
inline constexpr double TROPOPAUSE = 11000.0;            // m.
inline constexpr double TROPOPAUSE_TEMPERATURE =
    SEA_LEVEL_TEMPERATURE - LAPSE_RATE * TROPOPAUSE;
inline constexpr double CEILING = 20000.0;  // m.
}  // namespace internal

// Standard gravity.
inline constexpr AccelerationMagnitude STANDARD_GRAVITY =
    internal::GRAVITY * meter_per_second_squared;

// The air at `altitude` above sea level. Altitudes above 20 km get the air at
// 20 km.
inline auto standard_air(Length altitude) -> Air {
  using namespace internal;
  double h = std::min(altitude.numerical_value_in(meter), CEILING);
  double temperature = 0.0;
  double pressure = 0.0;
  if (h <= TROPOPAUSE) {
    temperature = SEA_LEVEL_TEMPERATURE - LAPSE_RATE * h;
    pressure = SEA_LEVEL_PRESSURE *
               std::pow(temperature / SEA_LEVEL_TEMPERATURE,
                        GRAVITY / (LAPSE_RATE * GAS_CONSTANT));
  } else {
    temperature = TROPOPAUSE_TEMPERATURE;
    double tropopause_pressure =
        SEA_LEVEL_PRESSURE *
        std::pow(TROPOPAUSE_TEMPERATURE / SEA_LEVEL_TEMPERATURE,
                 GRAVITY / (LAPSE_RATE * GAS_CONSTANT));
    pressure = tropopause_pressure *
               std::exp(-GRAVITY / (GAS_CONSTANT * temperature) *
                        (h - TROPOPAUSE));
  }
  return Air{
      .density = pressure / (GAS_CONSTANT * temperature) *
                 kilogram_per_cubic_meter,
      .speed_of_sound = std::sqrt(HEAT_RATIO * GAS_CONSTANT * temperature) *
                        meter_per_second,
  };
}

}  // namespace simon::model
