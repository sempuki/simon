// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "model/units.hpp"
#include "mp-units/math.h"

// The International Standard Atmosphere, from sea level to 20 km: a
// troposphere whose temperature falls linearly to 11 km, then an isothermal
// lower stratosphere. Its layers are in geopotential altitude, which counts
// height by the work done against gravity as it weakens with height, so
// geometric altitudes are converted first, as the 1976 standard and JSBSim
// do.
namespace simon::model {

struct Air final {
  Density density = 0.0 * kilogram_per_cubic_meter;
  Speed speed_of_sound = 0.0 * meter_per_second;
};

// Standard gravity.
inline constexpr AccelerationMagnitude STANDARD_GRAVITY =
    9.80665 * meter_per_second_squared;

namespace internal {

// Dry air.
inline constexpr auto GAS_CONSTANT = 287.052874 * joule_per_kilogram_kelvin;
inline constexpr double HEAT_RATIO = 1.4;

// Temperatures are from absolute zero, so they divide and scale as plain
// quantities. mp-units asks for kelvins to be made with `delta`.
inline constexpr Temperature SEA_LEVEL_TEMPERATURE =
    units::delta<kelvin>(288.15);
inline constexpr Pressure SEA_LEVEL_PRESSURE = 101325.0 * pascal;
inline constexpr auto LAPSE_RATE = units::delta<kelvin>(0.0065) / meter;

// The Earth's radius the standard converts altitudes with.
inline constexpr Length EARTH_RADIUS = 6356766.0 * meter;

inline constexpr Length TROPOPAUSE = 11000.0 * meter;  // Geopotential.
inline constexpr Temperature TROPOPAUSE_TEMPERATURE =
    SEA_LEVEL_TEMPERATURE - LAPSE_RATE * TROPOPAUSE;
inline constexpr Length CEILING = 20000.0 * meter;  // Geometric.

// The exponent of pressure in temperature through the troposphere.
inline auto pressure_exponent() -> double {
  return number_of(STANDARD_GRAVITY / (LAPSE_RATE * GAS_CONSTANT));
}

}  // namespace internal

// The geopotential altitude of a geometric `altitude`.
inline auto geopotential(Length altitude) -> Length {
  using internal::EARTH_RADIUS;
  return EARTH_RADIUS * altitude / (EARTH_RADIUS + altitude);
}

namespace internal {

// The air at geopotential altitude `h`, at most the ceiling's.
inline auto air_at_geopotential(Length h) -> Air {
  Temperature temperature = TROPOPAUSE_TEMPERATURE;
  Pressure pressure = 0.0 * pascal;
  if (h <= TROPOPAUSE) {
    temperature = SEA_LEVEL_TEMPERATURE - LAPSE_RATE * h;
    pressure = SEA_LEVEL_PRESSURE *
               std::pow(number_of(temperature / SEA_LEVEL_TEMPERATURE),
                        pressure_exponent());
  } else {
    Pressure tropopause_pressure =
        SEA_LEVEL_PRESSURE *
        std::pow(number_of(TROPOPAUSE_TEMPERATURE / SEA_LEVEL_TEMPERATURE),
                 pressure_exponent());
    pressure = tropopause_pressure *
               std::exp(-number_of(STANDARD_GRAVITY * (h - TROPOPAUSE) /
                                   (GAS_CONSTANT * temperature)));
  }

  Speed speed_of_sound{units::sqrt(HEAT_RATIO * GAS_CONSTANT * temperature)};
  return Air{
      .density = pressure / (GAS_CONSTANT * temperature),
      .speed_of_sound = speed_of_sound,
  };
}

}  // namespace internal

// The air at `altitude` above sea level, geometric. Altitudes above 20 km get
// the air at 20 km.
inline auto standard_air(Length altitude) -> Air {
  return internal::air_at_geopotential(
      geopotential(std::min(altitude, internal::CEILING)));
}

// The standard atmosphere, tabulated every `spacing` from sea level to 20 km
// and interpolated linearly. Much cheaper than standard_air, which takes a
// power or an exponential, and within a few parts in 10^5 of it at the default
// 100 m. Altitudes outside the table get the air at its ends. The table is
// spaced in geopotential altitude, so the tropopause falls on a breakpoint.
class StandardAirTable final {
 public:
  explicit StandardAirTable(Length spacing = 100.0 * meter)
      : spacing_{spacing},
        top_{number_of(geopotential(internal::CEILING) / spacing)} {
    // The last breakpoint may lie past the ceiling, still in the isothermal
    // layer.
    auto points = static_cast<std::size_t>(std::ceil(top_)) + 1;
    air_.reserve(points);
    for (std::size_t i = 0; i < points; ++i) {
      air_.push_back(
          internal::air_at_geopotential(static_cast<double>(i) * spacing_));
    }
  }

  auto operator()(Length altitude) const -> Air {
    double position =
        std::clamp(number_of(geopotential(altitude) / spacing_), 0.0, top_);
    auto i = std::min(static_cast<std::size_t>(position), air_.size() - 2);
    double weight = position - static_cast<double>(i);

    const Air& low = air_[i];
    const Air& high = air_[i + 1];
    return Air{
        .density = low.density + (high.density - low.density) * weight,
        .speed_of_sound = low.speed_of_sound +
                          (high.speed_of_sound - low.speed_of_sound) * weight,
    };
  }

 private:
  Length spacing_;
  double top_;  // The ceiling's position in the table.
  std::vector<Air> air_;
};

}  // namespace simon::model
