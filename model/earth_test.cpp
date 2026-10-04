// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/earth.hpp"

#include <cmath>
#include <numbers>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::model::wgs84 {

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

constexpr double DEGREE = std::numbers::pi / 180.0;

}  // namespace

TEST_CASE("Geodetic") {
  SECTION("ShouldRoundTripGivenAnyPlaceAndAltitude") {
    for (double latitude = -89.0; latitude <= 89.0; latitude += 7.3) {
      for (double altitude : {-500.0, 0.0, 6000.0, 100000.0}) {
        Geodetic where{.latitude = latitude * DEGREE * radian,
                       .longitude = (latitude * 2.0 + 10.0) * DEGREE * radian,
                       .altitude = altitude * meter};
        Geodetic back =
            convert_fixed_to_geodetic(convert_geodetic_to_fixed(where));
        CHECK_THAT(radians(back.latitude),
                   WithinAbs(radians(where.latitude), 1e-12));
        CHECK_THAT(
            std::remainder(radians(back.longitude) - radians(where.longitude),
                           2.0 * std::numbers::pi),
            WithinAbs(0.0, 1e-12));
        CHECK_THAT(back.altitude.numerical_value_in(meter),
                   WithinAbs(altitude, 1e-6));
      }
    }
  }

  SECTION("ShouldMatchIterationGivenHighLatitude") {
    // Bowring's fixed-point iteration, run to convergence, as an independent
    // reference (see model/REFERENCES.md).
    QuantityVector xyz =
        convert_geodetic_to_fixed(Geodetic{.latitude = 60.0 * DEGREE * radian,
                                           .longitude = 10.0 * DEGREE * radian,
                                           .altitude = 6000.0 * meter})
            .numerical_value_in(meter);
    double p = std::hypot(xyz.eigen().x(), xyz.eigen().y());
    double z = xyz.eigen().z();
    double latitude = std::atan2(z, p * (1.0 - ECCENTRICITY_SQUARED));
    double height = 0.0;
    for (int i = 0; i < 20; ++i) {
      double sin_latitude = std::sin(latitude);
      double normal =
          SEMIMAJOR_AXIS /
          std::sqrt(1.0 - ECCENTRICITY_SQUARED * sin_latitude * sin_latitude);
      height = p / std::cos(latitude) - normal;
      latitude = std::atan2(
          z, p * (1.0 - ECCENTRICITY_SQUARED * normal / (normal + height)));
    }
    Geodetic found = convert_fixed_to_geodetic(xyz * meter);
    CHECK_THAT(found.altitude.numerical_value_in(meter),
               WithinAbs(height, 1e-6));
    CHECK_THAT(radians(found.latitude), WithinAbs(latitude, 1e-12));
  }

  SECTION("ShouldLieOnAxesGivenEquatorAndPole") {
    QuantityVector equator =
        convert_geodetic_to_fixed(Geodetic{}).numerical_value_in(meter);
    CHECK(equator.is_approximately(QuantityVector{SEMIMAJOR_AXIS, 0.0, 0.0}));
    QuantityVector pole =
        convert_geodetic_to_fixed(Geodetic{.latitude = 90.0 * DEGREE * radian})
            .numerical_value_in(meter);
    CHECK_THAT(pole.eigen().z(), WithinAbs(SEMIMINOR_AXIS, 1e-6));
  }
}

TEST_CASE("Gravitation") {
  SECTION("ShouldMatchWgs84GivenEquatorAndPole") {
    // Effective gravity is 9.7803253 m/s^2 at the equator and 9.8321849 at
    // the poles (WGS84); gravitation is that plus the centrifugal part.
    double centrifugal = ROTATION_RATE * ROTATION_RATE * SEMIMAJOR_AXIS;
    double equator =
        magnitude(compute_gravitation(convert_geodetic_to_fixed(Geodetic{}))
                      .numerical_value_in(meter_per_second_squared));
    CHECK_THAT(equator - centrifugal, WithinRel(9.7803253, 1e-4));
    double pole =
        magnitude(compute_gravitation(convert_geodetic_to_fixed(Geodetic{
                                          .latitude = 90.0 * DEGREE * radian}))
                      .numerical_value_in(meter_per_second_squared));
    CHECK_THAT(pole, WithinRel(9.8321849, 1e-4));
  }
}

}  // namespace simon::model::wgs84
