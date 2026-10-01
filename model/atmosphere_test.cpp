// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/atmosphere.hpp"

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::model {

using Catch::Matchers::WithinRel;

TEST_CASE("StandardAir") {
  auto density = [](double altitude) {
    return standard_air(altitude * meter)
        .density.numerical_value_in(kilogram_per_cubic_meter);
  };
  auto sound = [](double altitude) {
    return standard_air(altitude * meter)
        .speed_of_sound.numerical_value_in(meter_per_second);
  };

  // Published ISA values.
  SECTION("ShouldMatchStandardGivenSeaLevel") {
    CHECK_THAT(density(0.0), WithinRel(1.2250, 1e-4));
    CHECK_THAT(sound(0.0), WithinRel(340.29, 1e-4));
  }

  SECTION("ShouldMatchStandardGivenTroposphere") {
    CHECK_THAT(density(5000.0), WithinRel(0.73643, 1e-3));
    CHECK_THAT(sound(5000.0), WithinRel(320.53, 1e-3));
  }

  SECTION("ShouldMatchStandardGivenStratosphere") {
    CHECK_THAT(density(11000.0), WithinRel(0.36392, 1e-3));
    CHECK_THAT(density(20000.0), WithinRel(0.08803, 1e-3));
    CHECK_THAT(sound(15000.0), WithinRel(295.07, 1e-3));
  }

  SECTION("ShouldHoldCeilingAirGivenAltitudeAboveCeiling") {
    CHECK(density(30000.0) == density(20000.0));
  }
}

TEST_CASE("StandardAirTable") {
  StandardAirTable table;

  SECTION("ShouldMatchStandardAirGivenAnyAltitude") {
    for (double altitude = 0.0; altitude <= 20000.0; altitude += 37.0) {
      Air exact = standard_air(altitude * meter);
      Air tabulated = table(altitude * meter);
      CHECK_THAT(
          tabulated.density.numerical_value_in(kilogram_per_cubic_meter),
          WithinRel(exact.density.numerical_value_in(kilogram_per_cubic_meter),
                    1e-4));
      CHECK_THAT(
          tabulated.speed_of_sound.numerical_value_in(meter_per_second),
          WithinRel(exact.speed_of_sound.numerical_value_in(meter_per_second),
                    1e-4));
    }
  }

  SECTION("ShouldClampGivenAltitudeOutsideTable") {
    CHECK(table(-100.0 * meter).density == table(0.0 * meter).density);
    CHECK(table(25000.0 * meter).density == table(20000.0 * meter).density);
  }
}

}  // namespace simon::model
