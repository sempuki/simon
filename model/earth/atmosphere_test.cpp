// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/earth/atmosphere.hpp"

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

  // The 1976 standard's values, by geometric altitude.
  SECTION("ShouldMatchStandardGivenSeaLevel") {
    CHECK_THAT(density(0.0), WithinRel(1.2250, 1e-4));
    CHECK_THAT(sound(0.0), WithinRel(340.29, 1e-4));
  }

  SECTION("ShouldMatchStandardGivenTroposphere") {
    CHECK_THAT(density(5000.0), WithinRel(0.73643, 1e-4));
    CHECK_THAT(sound(5000.0), WithinRel(320.55, 1e-4));
  }

  SECTION("ShouldMatchStandardGivenStratosphere") {
    CHECK_THAT(density(11000.0), WithinRel(0.36480, 1e-4));
    CHECK_THAT(density(20000.0), WithinRel(0.088910, 1e-4));
    CHECK_THAT(sound(15000.0), WithinRel(295.07, 1e-4));
  }

  SECTION("ShouldLayerByGeopotentialAltitude") {
    // The tropopause is 11 km geopotential, 11019 m geometric.
    CHECK_THAT(sound(11019.1), WithinRel(sound(15000.0), 1e-9));
    CHECK(sound(10990.0) > sound(15000.0));
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

TEST_CASE("CalibratedAirspeed") {
  SECTION("ShouldMatchJsbsimGivenSubsonicAndSupersonicFlight") {
    // JSBSim's velocities/vc-kts in m/s, at its velocities/mach, over the
    // standard atmosphere at each altitude. They differ by a few parts in
    // 10^6, because JSBSim rounds its gas constant in English units.
    struct Point final {
      double altitude;  // m.
      double mach;
      double calibrated;  // m/s.
    };
    for (Point point :
         {Point{0.0, 0.3, 102.08805610600513},
          Point{3000.0, 0.57935238046670334, 165.99256485009212},
          Point{6000.0, 0.83694263780500477, 202.75850442467558},
          Point{10000.0, 1.2323001259090951, 239.8446487478603},
          Point{15000.0, 1.7342033881976764, 242.57718268966568}}) {
      Speed speed = compute_calibrated_airspeed(
          point.mach, standard_air(point.altitude * meter));
      CAPTURE(point.altitude);
      CHECK_THAT(speed.numerical_value_in(meter_per_second),
                 WithinRel(point.calibrated, 1e-5));
    }
  }
}

}  // namespace simon::model
