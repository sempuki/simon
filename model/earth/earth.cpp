// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/earth/earth.hpp"

#include <cmath>

namespace simon::model::wgs84 {

auto convert_geodetic_to_fixed(const Geodetic& where) -> Position {
  double sin_latitude = sin(where.latitude);
  double cos_latitude = cos(where.latitude);
  double normal = SEMIMAJOR_AXIS /
                  std::sqrt(1.0 - ECCENTRICITY_SQUARED * sin_latitude *
                                      sin_latitude);  // Prime vertical radius.
  double h = where.altitude.numerical_value_in(meter);
  return meters((normal + h) * cos_latitude * cos(where.longitude),
                (normal + h) * cos_latitude * sin(where.longitude),
                (normal * (1.0 - ECCENTRICITY_SQUARED) + h) * sin_latitude);
}

auto convert_fixed_to_geodetic(const Position& fixed) -> Geodetic {
  Location where = locate(fixed);
  return Geodetic{
      .latitude = std::atan2(where.sin_latitude, where.cos_latitude) * radian,
      .longitude =
          std::atan2(where.sin_longitude, where.cos_longitude) * radian,
      .altitude = where.altitude,
  };
}

}  // namespace simon::model::wgs84
