// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/tire_file.hpp"

#include <string>

#include "base/testing.hpp"

// Reading Project Chrono's Pac02 tire files (see 3rd_party/chrono/LICENSE).
namespace simon::format {

TEST_CASE("TireFile") {
  SECTION("ShouldReadTheSedanTire") {
    // Preconditions.
    const std::string sedan = "3rd_party/chrono/Sedan_Pac02Tire.tir";

    // Under Test.
    auto tire = load_tire_file(sedan);

    // Postconditions.
    REQUIRE(tire.has_value());
    CHECK(tire->fnomin == 4850.0);
    CHECK(tire->unloaded_radius == 0.344);
    CHECK(tire->lfzo == 0.81);
    CHECK(tire->pky1 == -21.92);
    CHECK(tire->qbz1 == 10.904);
  }

  SECTION("ShouldFailGivenMissingFile") {
    // Preconditions.
    const std::string missing = "3rd_party/chrono/nonesuch.tir";

    // Under Test.
    auto tire = load_tire_file(missing);

    // Postconditions.
    CHECK_FALSE(tire.has_value());
  }
}

}  // namespace simon::format
