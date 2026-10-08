// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/timeline.hpp"

#include <chrono>
#include <optional>
#include <vector>

#include "base/testing.hpp"
#include "core/argument.hpp"

namespace simon::framework {
namespace {

using namespace std::chrono_literals;

// Due at fixed times.
struct Fixed final : Timeline::Source {
  auto earliest() const -> std::optional<TimePoint> override {
    std::optional<TimePoint> first;
    for (TimePoint time : due) {
      if (!first || time < *first) {
        first = time;
      }
    }
    return first;
  }
  auto earliest_after(TimePoint after) const
      -> std::optional<TimePoint> override {
    std::optional<TimePoint> first;
    for (TimePoint time : due) {
      if (time > after && (!first || time < *first)) {
        first = time;
      }
    }
    return first;
  }
  std::vector<TimePoint> due;
};

TEST_CASE("Timeline") {
  Timeline timeline;

  SECTION("ShouldHaveNothingDueGivenNoSourceDue") {
    // Preconditions.
    Fixed idle;

    // Under Test.
    timeline.add(Depend<const Timeline::Source>(idle));

    // Postconditions.
    CHECK(timeline.earliest() == std::nullopt);
    CHECK(timeline.earliest_after(TimePoint{}) == std::nullopt);
    CHECK_FALSE(timeline.continuous());
  }

  SECTION("ShouldGiveEarliestOverSourcesGivenSeveralSources") {
    // Preconditions.
    Fixed a;
    a.due = {TimePoint{300ms}};
    Fixed b;
    b.due = {TimePoint{100ms}, TimePoint{200ms}};

    // Under Test.
    timeline.add(Depend<const Timeline::Source>(a));
    timeline.add(Depend<const Timeline::Source>(b));

    // Postconditions.
    CHECK(timeline.earliest() == TimePoint{100ms});
    // After 100 ms, as before the work due then has run.
    CHECK(timeline.earliest_after(TimePoint{100ms}) == TimePoint{200ms});
    CHECK(timeline.earliest_after(TimePoint{300ms}) == std::nullopt);
  }

  SECTION("ShouldBeContinuousWhileAnyEveryStepWorkGivenCounts") {
    // Under Test.
    timeline.add_continuous(2);
    timeline.add_continuous(-1);

    // Postconditions.
    CHECK(timeline.continuous());

    // Under Test.
    timeline.add_continuous(-1);

    // Postconditions.
    CHECK_FALSE(timeline.continuous());
  }
}

}  // namespace
}  // namespace simon::framework
