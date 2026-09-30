// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "engine/rate_gate.hpp"

#include <chrono>
#include <vector>

#include "base/testing.hpp"

namespace simon::engine {

using namespace std::chrono_literals;

TEST_CASE("RateGate") {
  SECTION("ShouldFireEveryTenthStepGivenTenHertzOnHundredHertz") {
    RateGate gate{100ms};
    std::vector<TimePoint> fired;
    std::vector<Duration> elapsed;

    for (TimePoint time{}; time < TimePoint{300ms}; time += 10ms) {
      if (auto firing = gate.fire(Step{.time = time, .dt = 10ms})) {
        fired.push_back(time);
        elapsed.push_back(firing->elapsed);
        CHECK(firing->periods == 1u);
      }
    }

    CHECK(fired ==
          std::vector{TimePoint{0ms}, TimePoint{100ms}, TimePoint{200ms}});
    CHECK(elapsed == std::vector<Duration>{0ms, 100ms, 100ms});
  }

  SECTION("ShouldReportEveryPeriodGivenEveryAndStepLongerThanPeriod") {
    RateGate gate{100ms, CatchUp::EVERY};

    auto first = gate.fire(Step{.time = TimePoint{0ms}, .dt = 250ms});
    auto second = gate.fire(Step{.time = TimePoint{250ms}, .dt = 250ms});

    REQUIRE(first);
    REQUIRE(second);
    CHECK(first->periods == 3u);   // 0, 100, 200 ms.
    CHECK(second->periods == 2u);  // 300, 400 ms.
    CHECK(second->elapsed == 250ms);
  }

  SECTION("ShouldFireOnceGivenSkipAndStepLongerThanPeriod") {
    RateGate gate{100ms, CatchUp::SKIP};

    auto first = gate.fire(Step{.time = TimePoint{0ms}, .dt = 250ms});

    REQUIRE(first);
    CHECK(first->periods == 1u);
    CHECK_FALSE(
        gate.fire(Step{.time = TimePoint{250ms}, .dt = 40ms}));  // Next is 300.
    CHECK(gate.fire(Step{.time = TimePoint{290ms}, .dt = 20ms}));
  }

  SECTION("ShouldNotFireGivenNoBoundaryInStep") {
    RateGate gate{100ms};
    REQUIRE(gate.fire(Step{.time = TimePoint{0ms}, .dt = 10ms}));

    // Steps are half-open: [10, 100) does not contain the 100 ms boundary.
    CHECK_FALSE(gate.fire(Step{.time = TimePoint{10ms}, .dt = 90ms}));
    CHECK(gate.fire(Step{.time = TimePoint{100ms}, .dt = 1ms}));
  }
}

}  // namespace simon::engine
