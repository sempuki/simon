// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

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
  return Vector3d{0.0, 0.0, ROTATION_RATE} * radian_per_second;
}

// The rotation from ECI to ECEF at `earth_angle`, how far the Earth has
// turned.
inline auto inertial_to_fixed(Angle earth_angle) -> Eigen::Matrix3d {
  return Eigen::AngleAxisd{-radians(earth_angle), Eigen::Vector3d::UnitZ()}
      .toRotationMatrix();
}

struct Geodetic final {
  Angle latitude = 0.0 * radian;  // Geodetic: of the ellipsoid's normal.
  Angle longitude = 0.0 * radian;
  Length altitude = 0.0 * meter;  // Above the ellipsoid, along its normal.
};

// The ECEF position of a geodetic one.
inline auto fixed_of(const Geodetic& where) -> Position {
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

// The geodetic position of an ECEF one, by Heikkinen's closed form (1982),
// exact to a few nanometers for any point not near the Earth's center.
inline auto geodetic_of(const Position& fixed) -> Geodetic {
  constexpr double a = SEMIMAJOR_AXIS;
  constexpr double b = SEMIMINOR_AXIS;
  constexpr double e2 = ECCENTRICITY_SQUARED;
  constexpr double second_e2 = (a * a - b * b) / (b * b);

  Vector3d xyz = fixed.numerical_value_in(meter);
  double x = xyz.eigen().x();
  double y = xyz.eigen().y();
  double z = xyz.eigen().z();
  double p = std::hypot(x, y);

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
  double u = std::hypot(p - e2 * r0, z);
  double v = std::sqrt((p - e2 * r0) * (p - e2 * r0) + (1.0 - e2) * z * z);
  double z0 = b * b * z / (a * v);

  return Geodetic{
      .latitude = std::atan2(z + second_e2 * z0, p) * radian,
      .longitude = std::atan2(y, x) * radian,
      .altitude = u * (1.0 - b * b / (a * v)) * meter,
  };
}

// The gravitational acceleration at an ECEF position, to the J2 term. It
// leaves out the centrifugal acceleration of the Earth's rotation, which
// comes from the equations of motion.
inline auto gravitation(const Position& fixed) -> Acceleration {
  Vector3d xyz = fixed.numerical_value_in(meter);
  double r = magnitude(xyz);
  double sin_latitude = xyz.eigen().z() / r;  // Geocentric.
  double ratio = SEMIMAJOR_AXIS / r;
  double j2 = 1.5 * J2 * ratio * ratio;
  double equatorial = 1.0 + j2 * (1.0 - 5.0 * sin_latitude * sin_latitude);
  double polar = 1.0 + j2 * (3.0 - 5.0 * sin_latitude * sin_latitude);
  double scale = -GRAVITATIONAL_PARAMETER / (r * r * r);
  return Vector3d{scale * equatorial * xyz.eigen().x(),
                  scale * equatorial * xyz.eigen().y(),
                  scale * polar * xyz.eigen().z()} *
         meter_per_second_squared;
}

}  // namespace simon::model::wgs84
