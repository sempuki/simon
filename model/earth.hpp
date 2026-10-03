// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>

#include "Eigen/Geometry"
#include "model/units.hpp"

// The Earth, for vehicles whose fidelity needs it: the WGS84 ellipsoid, its
// gravity to the J2 term, and its rotation. Defined by the WGS84 standard;
// the gravity is the usual zonal-harmonic expansion truncated after J2, as in
// Vallado and JSBSim. Sources are in model/REFERENCES.md.
//
// Positions are in meters from the Earth's center. The Earth-centered,
// Earth-fixed frame (ECEF) turns with the Earth; the Earth-centered inertial
// frame (ECI) does not, and the two share their z axis, the Earth's axis.
// At `earth_angle` zero they coincide.
namespace simon::model::wgs84 {

inline constexpr double SEMIMAJOR_AXIS = 6378137.0;                // m.
inline constexpr double FLATTENING = 1.0 / 298.257223563;          //
inline constexpr double SEMIMINOR_AXIS =                           // m.
    SEMIMAJOR_AXIS * (1.0 - FLATTENING);                           //
inline constexpr double GRAVITATIONAL_PARAMETER = 3.986004418e14;  // m^3/s^2.
inline constexpr double J2 = 1.08262982e-3;
inline constexpr double ROTATION_RATE = 7.292115e-5;  // rad/s.

// The first eccentricity, squared.
inline constexpr double ECCENTRICITY_SQUARED = FLATTENING * (2.0 - FLATTENING);

// The Earth's rotation, in ECI and in ECEF alike.
inline auto rotation() -> AngularVelocity {
  return QuantityVector{0.0, 0.0, ROTATION_RATE} * radian_per_second;
}

// The rotation from ECI to ECEF at `earth_angle`, how far the Earth has
// turned.
inline auto inertial_to_fixed(Angle earth_angle) -> Matrix3 {
  return AngleAxis{-radians(earth_angle), Vector3::UnitZ()}.toRotationMatrix();
}

struct Geodetic final {
  Angle latitude = 0.0 * radian;  // Geodetic: of the ellipsoid's normal.
  Angle longitude = 0.0 * radian;
  Length altitude = 0.0 * meter;  // Above the ellipsoid, along its normal.
};

// The ECEF position of a geodetic one.
inline auto geodetic_to_fixed(const Geodetic& where) -> Position {
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

// Where an ECEF position is on the ellipsoid, by Heikkinen's closed form
// (1982), exact to a few nanometers for any point not near the Earth's
// center: its altitude, and the sines and cosines of its latitude and
// longitude, which the local frame needs without the angles themselves.
struct Location final {
  Length altitude = 0.0 * meter;
  double sin_latitude = 0.0;
  double cos_latitude = 1.0;
  double sin_longitude = 0.0;
  double cos_longitude = 1.0;
};

inline auto locate(const Position& fixed) -> Location {
  constexpr double a = SEMIMAJOR_AXIS;
  constexpr double b = SEMIMINOR_AXIS;
  constexpr double e2 = ECCENTRICITY_SQUARED;
  constexpr double second_e2 = (a * a - b * b) / (b * b);

  Vector3 xyz = eigen(fixed);
  double x = xyz.x();
  double y = xyz.y();
  double z = xyz.z();
  double p = std::sqrt(x * x + y * y);

  double f = 54.0 * b * b * z * z;
  double g = p * p + (1.0 - e2) * z * z - e2 * (a * a - b * b);
  double c = e2 * e2 * f * p * p / (g * g * g);
  double s = std::cbrt(1.0 + c + std::sqrt(c * c + 2.0 * c));
  double k = s + 1.0 + 1.0 / s;
  double big_p = f / (3.0 * k * k * g * g);
  double q = std::sqrt(1.0 + 2.0 * e2 * e2 * big_p);
  double r0 = -big_p * e2 * p / (1.0 + q) +
              std::sqrt(0.5 * a * a * (1.0 + 1.0 / q) -
                        big_p * (1.0 - e2) * z * z / (q * (1.0 + q)) -
                        0.5 * big_p * p * p);
  double across = p - e2 * r0;
  double u = std::sqrt(across * across + z * z);
  double v = std::sqrt(across * across + (1.0 - e2) * z * z);
  double z0 = b * b * z / (a * v);

  // tan(latitude) = (z + e'^2 z0) / p.
  double rise = z + second_e2 * z0;
  double slant = std::sqrt(rise * rise + p * p);
  return Location{
      .altitude = u * (1.0 - b * b / (a * v)) * meter,
      .sin_latitude = rise / slant,
      .cos_latitude = p / slant,
      .sin_longitude = p > 0.0 ? y / p : 0.0,
      .cos_longitude = p > 0.0 ? x / p : 1.0,
  };
}

// The geodetic position of an ECEF one (see locate).
inline auto fixed_to_geodetic(const Position& fixed) -> Geodetic {
  Location where = locate(fixed);
  return Geodetic{
      .latitude = std::atan2(where.sin_latitude, where.cos_latitude) * radian,
      .longitude =
          std::atan2(where.sin_longitude, where.cos_longitude) * radian,
      .altitude = where.altitude,
  };
}

// The gravitational acceleration at an ECEF position, to the J2 term. It
// leaves out the centrifugal acceleration of the Earth's rotation, which
// comes from the equations of motion.
inline auto gravitation(const Position& fixed) -> Acceleration {
  QuantityVector xyz = fixed.numerical_value_in(meter);
  double r = magnitude(xyz);
  double sin_latitude = xyz.eigen().z() / r;  // Geocentric.
  double ratio = SEMIMAJOR_AXIS / r;
  double j2 = 1.5 * J2 * ratio * ratio;
  double equatorial = 1.0 + j2 * (1.0 - 5.0 * sin_latitude * sin_latitude);
  double polar = 1.0 + j2 * (3.0 - 5.0 * sin_latitude * sin_latitude);
  double scale = -GRAVITATIONAL_PARAMETER / (r * r * r);
  return QuantityVector{scale * equatorial * xyz.eigen().x(),
                        scale * equatorial * xyz.eigen().y(),
                        scale * polar * xyz.eigen().z()} *
         meter_per_second_squared;
}

}  // namespace simon::model::wgs84
