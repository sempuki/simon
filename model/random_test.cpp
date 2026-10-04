// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/random.hpp"

#include <cmath>

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

  SECTION("ShouldHaveMeanAndDeviationGivenNormal") {
    Random random{3};
    constexpr int SAMPLES = 100000;
    double sum = 0.0;
    double squares = 0.0;
    for (int i = 0; i < SAMPLES; ++i) {
      double value = random.normal(1.34, 0.26);
      sum += value;
      squares += value * value;
    }
    double mean = sum / SAMPLES;
    double deviation = std::sqrt(squares / SAMPLES - mean * mean);
    // Within four standard errors.
    CHECK(std::abs(mean - 1.34) < 4.0 * 0.26 / std::sqrt(SAMPLES));
    CHECK(std::abs(deviation - 0.26) < 0.003);
  }
}

}  // namespace simon::model
