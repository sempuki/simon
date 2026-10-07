// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "core/time.hpp"

#include <chrono>
#include <type_traits>

#include "base/testing.hpp"

namespace simon {

using namespace std::chrono_literals;

template <typename A, typename B>
concept Subtractable = requires(A a, B b) { a - b; };

TEST_CASE("Duration") {
  SECTION("ShouldCountIntegerNanosecondsGivenDefinition") {
    static_assert(std::is_same_v<Duration, std::chrono::nanoseconds>);
  }

  SECTION("ShouldBeExactGivenManySmallSteps") {
    // Ten steps of 0.1 s land exactly on 1 s, which double seconds do not.
    Duration total{};
    for (int i = 0; i < 10; ++i) {
      total += 100ms;
    }
    CHECK(total == 1s);
  }

  SECTION("ShouldBeZeroGivenValueInitialization") {
    CHECK(Duration{}.count() == 0);
  }
}

TEST_CASE("TimePoint") {
  SECTION("ShouldBeZeroGivenDefault") {
    CHECK(TimePoint{}.time_since_epoch() == Duration::zero());
  }

  SECTION("ShouldAdvanceByDurationGivenAddition") {
    CHECK((TimePoint{} + 250ms).time_since_epoch() == 250ms);
    CHECK(TimePoint{2s} - TimePoint{500ms} == 1500ms);
  }

  SECTION("ShouldNotMixGivenWallClockTimePoint") {
    static_assert(
        !Subtractable<TimePoint, std::chrono::steady_clock::time_point>);
    static_assert(Subtractable<TimePoint, TimePoint>);
  }
}

TEST_CASE("Step") {
  SECTION("ShouldCountInItsTickGivenCoarserTick") {
    using Years = std::chrono::duration<std::int64_t, std::ratio<31556952>>;
    BasicStep<Years> step{.time = BasicTimePoint<Years>{Years{3}},
                          .dt = Years{1}};
    static_assert(std::is_same_v<BasicStep<Years>::Tick, Years>);
    CHECK((step.time + step.dt).time_since_epoch() == Years{4});
  }

  SECTION("ShouldDefaultToNanosecondsGivenStep") {
    static_assert(std::is_same_v<Step::Tick, Duration>);
  }
}

}  // namespace simon
