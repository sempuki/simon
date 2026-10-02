// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

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
        Geodetic back = geodetic_of(fixed_of(where));
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

  SECTION("ShouldLieOnAxesGivenEquatorAndPole") {
    Vector3d equator = fixed_of(Geodetic{}).numerical_value_in(meter);
    CHECK(equator.is_approximately(Vector3d{SEMIMAJOR_AXIS, 0.0, 0.0}));
    Vector3d pole = fixed_of(Geodetic{.latitude = 90.0 * DEGREE * radian})
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
        magnitude(gravitation(fixed_of(Geodetic{}))
                      .numerical_value_in(meter_per_second_squared));
    CHECK_THAT(equator - centrifugal, WithinRel(9.7803253, 1e-4));
    double pole = magnitude(
        gravitation(fixed_of(Geodetic{.latitude = 90.0 * DEGREE * radian}))
            .numerical_value_in(meter_per_second_squared));
    CHECK_THAT(pole, WithinRel(9.8321849, 1e-4));
  }
}

}  // namespace simon::model::wgs84
