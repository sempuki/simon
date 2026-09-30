// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/random.hpp"

#include "base/testing.hpp"

namespace simon::model {

TEST_CASE("Random") {
  SECTION("ShouldRepeatGivenSameSeed") {
    Random first{7};
    Random second{7};
    for (int i = 0; i < 100; ++i) {
      CHECK(first.unit() == second.unit());
    }
  }

  SECTION("ShouldMatchKnownValuesGivenSeedOnAnyPlatform") {
    // The standard fixes mt19937_64's 10000th output for the default seed.
    std::mt19937_64 reference;
    reference.discard(9999);
    CHECK(reference() == 9981545732273789042ull);
  }

  SECTION("ShouldStayInRangeGivenUniform") {
    Random random{1};
    for (int i = 0; i < 1000; ++i) {
      double value = random.uniform(-2.0, 3.0);
      CHECK(value >= -2.0);
      CHECK(value < 3.0);
    }
  }
}

}  // namespace simon::model
