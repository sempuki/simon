// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/timeline.hpp"

#include <chrono>
#include <optional>

#include "base/testing.hpp"

namespace simon::framework {
namespace {

using namespace std::chrono_literals;

TEST_CASE("Timeline") {
  Timeline timeline;

  SECTION("ShouldHaveNothingDueGivenNoSourceSet") {
    timeline.add();
    CHECK(timeline.earliest() == std::nullopt);
    CHECK_FALSE(timeline.continuous());
  }

  SECTION("ShouldGiveEarliestGivenSeveralSources") {
    Timeline::Source a = timeline.add();
    Timeline::Source b = timeline.add();
    timeline.set(a, TimePoint{300ms});
    timeline.set(b, TimePoint{100ms});
    CHECK(timeline.earliest() == TimePoint{100ms});
    timeline.set(b, std::nullopt);
    CHECK(timeline.earliest() == TimePoint{300ms});
  }

  SECTION("ShouldBeContinuousWhileAnyEveryStepWorkGivenCounts") {
    timeline.add_continuous(2);
    timeline.add_continuous(-1);
    CHECK(timeline.continuous());
    timeline.add_continuous(-1);
    CHECK_FALSE(timeline.continuous());
  }
}

}  // namespace
}  // namespace simon::framework
