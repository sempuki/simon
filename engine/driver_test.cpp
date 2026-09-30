// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "engine/driver.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "base/testing.hpp"

namespace simon::engine {

enum class TestCondition { FAILED, COUNT };

}  // namespace simon::engine

template <>
const std::array<lib::StatusConditionEntry, 1>
    lib::EnumStatusKindConditionMixin<simon::engine::TestCondition,
                                      1>::conditions_ = {
        lib::StatusConditionEntry{"failed"},
};

namespace simon::engine {
namespace {

using namespace std::chrono_literals;

// Records every phase and step, and stops or fails when told to.
struct Recorder final {
  PhaseResult configure() {
    phases.push_back("configure");
    return configure_result;
  }
  PhaseResult initialize() {
    phases.push_back("initialize");
    return Flow::CONTINUE;
  }
  PhaseResult step(const Step& step) {
    steps.push_back(step);
    if (fail_at && step.time >= *fail_at) {
      return std::unexpected(lib::raise(TestCondition::FAILED, "step failed"));
    }
    return stop_at && step.time >= *stop_at ? Flow::STOP : Flow::CONTINUE;
  }
  FinishResult finalize() {
    phases.push_back("finalize");
    return {};
  }

  std::vector<std::string> phases;
  std::vector<Step> steps;
  PhaseResult configure_result = Flow::CONTINUE;
  std::optional<TimePoint> stop_at;
  std::optional<TimePoint> fail_at;
};

// A wall clock the test moves by hand.
struct FakeClock final {
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<FakeClock>;

  static time_point now() { return current; }
  static inline time_point current{};
};

std::vector<Duration> step_lengths(const Recorder& recorder) {
  std::vector<Duration> lengths;
  for (const Step& step : recorder.steps) {
    lengths.push_back(step.dt);
  }
  return lengths;
}

}  // namespace

TEST_CASE("Driver") {
  Recorder recorder;
  Driver driver{lib::Depend<Recorder>{recorder}, Timing{.max_step = 30ms}};

  SECTION("ShouldLandExactlyOnTargetGivenStepNotDividingIt") {
    REQUIRE(driver.start());
    REQUIRE(driver.advance_to(TimePoint{100ms}) == Flow::CONTINUE);

    CHECK(step_lengths(recorder) ==
          std::vector<Duration>{30ms, 30ms, 30ms, 10ms});
    CHECK(driver.now() == TimePoint{100ms});
    CHECK(recorder.steps.back().time == TimePoint{90ms});
  }

  SECTION("ShouldThrowGivenAdvanceBeforeStart") {
    CHECK_THROWS_AS(driver.advance_to(TimePoint{1s}), std::logic_error);
  }

  SECTION("ShouldRefuseToStepGivenStopped") {
    recorder.stop_at = TimePoint{0ms};
    REQUIRE(driver.start());
    REQUIRE(driver.advance_to(TimePoint{1s}) == Flow::STOP);

    CHECK(driver.phase() == Phase::STOPPED);
    CHECK_THROWS_AS(driver.advance_to(TimePoint{2s}), std::logic_error);
  }
}

TEST_CASE("BatchDriver") {
  Recorder recorder;
  BatchDriver driver{lib::Depend<Recorder>{recorder}, Timing{.max_step = 10ms}};

  SECTION("ShouldRunLifecycleInOrderGivenRunToEnd") {
    auto reached = driver.run(TimePoint{50ms});

    REQUIRE(reached);
    CHECK(*reached == TimePoint{50ms});
    CHECK(recorder.steps.size() == 5u);
    CHECK(recorder.phases ==
          std::vector<std::string>{"configure", "initialize", "finalize"});
  }

  SECTION("ShouldStopAfterStoppingStepGivenSimulationStops") {
    recorder.stop_at = TimePoint{20ms};

    auto reached = driver.run(TimePoint{1s});

    REQUIRE(reached);
    CHECK(*reached == TimePoint{30ms});  // The end of the step that stopped.
    CHECK(recorder.steps.size() == 3u);
    CHECK(recorder.phases.back() == "finalize");
  }

  SECTION("ShouldReturnErrorAndStillFinalizeGivenFailingStep") {
    recorder.fail_at = TimePoint{10ms};

    auto reached = driver.run(TimePoint{1s});

    REQUIRE_FALSE(reached);
    CHECK(reached.error() == lib::watch(TestCondition::FAILED));
    CHECK(recorder.phases.back() == "finalize");
  }

  SECTION("ShouldNotStepGivenConfigureStops") {
    recorder.configure_result = Flow::STOP;

    auto reached = driver.run(TimePoint{1s});

    REQUIRE(reached);
    CHECK(recorder.steps.empty());
    CHECK(recorder.phases == std::vector<std::string>{"configure", "finalize"});
  }
}

TEST_CASE("RealTimeDriver") {
  Recorder recorder;
  FakeClock::current = FakeClock::time_point{};
  RealTimeDriver<Recorder, FakeClock> driver{lib::Depend<Recorder>{recorder},
                                             Timing{.max_step = 20ms}, 2.0};

  SECTION("ShouldTakeOnlyWholeStepsGivenWallTimeBetweenSteps") {
    REQUIRE(driver.tick() == Flow::CONTINUE);  // Starts; nothing due yet.
    CHECK(recorder.steps.empty());

    FakeClock::current += 25ms;  // 50 ms simulated: two whole steps are due.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(step_lengths(recorder) == std::vector<Duration>{20ms, 20ms});

    FakeClock::current += 5ms;  // 60 ms simulated: one more.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(step_lengths(recorder) == std::vector<Duration>{20ms, 20ms, 20ms});
    CHECK(driver.driver().now() == TimePoint{60ms});
  }

  SECTION("ShouldTakeSameStepsAsBatchGivenIrregularWallTicks") {
    for (auto wall : {3ms, 17ms, 18ms, 41ms, 100ms, 101ms, 150ms}) {
      FakeClock::current = FakeClock::time_point{wall};
      REQUIRE(driver.tick() == Flow::CONTINUE);
    }
    Recorder batch_recorder;
    BatchDriver batch{lib::Depend<Recorder>{batch_recorder},
                      Timing{.max_step = 20ms}};
    REQUIRE(batch.run(driver.driver().now()));

    REQUIRE(recorder.steps.size() == batch_recorder.steps.size());
    for (std::size_t i = 0; i < recorder.steps.size(); ++i) {
      CHECK(recorder.steps[i].time == batch_recorder.steps[i].time);
      CHECK(recorder.steps[i].dt == batch_recorder.steps[i].dt);
    }
  }

  SECTION("ShouldNotStepOrCatchUpGivenPauseThenResume") {
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current += 20ms;  // 40 ms simulated: two steps.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    REQUIRE(recorder.steps.size() == 2u);

    driver.pause();
    FakeClock::current += 1s;
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(recorder.steps.size() == 2u);  // Paused.

    driver.resume();
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(recorder.steps.size() == 2u);  // The paused second is not caught up.
    FakeClock::current += 10ms;          // 20 ms simulated: one step.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(recorder.steps.size() == 3u);
    CHECK(driver.driver().now() == TimePoint{60ms});
  }

  SECTION("ShouldChangeRateWithoutJumpGivenNewSpeed") {
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current += 20ms;  // 40 ms simulated at 2x.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    REQUIRE(driver.driver().now() == TimePoint{40ms});

    driver.set_speed(4.0);
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(driver.driver().now() == TimePoint{40ms});  // No jump.
    FakeClock::current += 10ms;                       // 40 ms simulated at 4x.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    CHECK(driver.driver().now() == TimePoint{80ms});
    CHECK(driver.speed() == 4.0);
  }

  SECTION("ShouldReportStopGivenSimulationStopped") {
    recorder.stop_at = TimePoint{0ms};
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current += 50ms;

    CHECK(driver.tick() == Flow::STOP);
    CHECK(driver.tick() == Flow::STOP);
    CHECK(recorder.steps.size() == 1u);
  }
}

}  // namespace simon::engine
