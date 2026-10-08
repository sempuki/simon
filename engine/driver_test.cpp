// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "engine/driver.hpp"

#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "base/testing.hpp"
#include "core/argument.hpp"

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
  auto configure() -> PhaseResult {
    phases.push_back("configure");
    return configure_result;
  }
  auto initialize() -> PhaseResult {
    phases.push_back("initialize");
    return Flow::CONTINUE;
  }
  auto step(const Step& step) -> PhaseResult {
    steps.push_back(step);
    if (fail_at && step.time >= *fail_at) {
      return std::unexpected(lib::raise(TestCondition::FAILED, "step failed"));
    }
    return stop_at && step.time >= *stop_at ? Flow::STOP : Flow::CONTINUE;
  }
  auto finalize() -> FinishResult {
    phases.push_back("finalize");
    return {};
  }

  std::vector<std::string> phases;
  std::vector<Step> steps;
  PhaseResult configure_result = Flow::CONTINUE;
  std::optional<TimePoint> stop_at;
  std::optional<TimePoint> fail_at;
};

// A timeline due at the times it is given, each until a step covers it.
struct FakeTimeline final {
  auto earliest() const -> std::optional<TimePoint> {
    std::optional<TimePoint> first;
    for (TimePoint time : due) {
      if (!first || time < *first) {
        first = time;
      }
    }
    return first;
  }
  auto earliest_after(TimePoint time) const -> std::optional<TimePoint> {
    std::optional<TimePoint> first;
    for (TimePoint due_at : due) {
      if (due_at > time && (!first || due_at < *first)) {
        first = due_at;
      }
    }
    return first;
  }
  auto continuous() const -> bool { return every_step; }

  std::vector<TimePoint> due;
  bool every_step = false;
};

// A Recorder whose work falls due at set times; a step does the work due in
// it, or at it if it has no length.
struct TimedRecorder final {
  auto step(const Step& step) -> PhaseResult {
    std::erase_if(timeline_.due, [&](TimePoint time) {
      return time < step.time + step.dt || time <= step.time;
    });
    return recorder.step(step);
  }
  auto timeline() const -> const FakeTimeline& { return timeline_; }

  Recorder recorder;
  FakeTimeline timeline_;
};

// A wall clock the test moves by hand.
struct FakeClock final {
  using duration = std::chrono::nanoseconds;
  using rep = duration::rep;
  using period = duration::period;
  using time_point = std::chrono::time_point<FakeClock>;

  static auto now() -> time_point { return current; }
  static inline time_point current{};
};

auto step_lengths(const Recorder& recorder) -> std::vector<Duration> {
  std::vector<Duration> lengths;
  for (const Step& step : recorder.steps) {
    lengths.push_back(step.dt);
  }
  return lengths;
}

}  // namespace

TEST_CASE("Driver") {
  Recorder recorder;
  Driver driver{Timing{.max_step = 30ms}, Depend(recorder)};

  SECTION("ShouldLandExactlyOnTargetGivenStepNotDividingIt") {
    // Preconditions.
    REQUIRE(driver.start());

    // Under Test.
    REQUIRE(driver.advance_to(TimePoint{100ms}) == Flow::CONTINUE);

    // Postconditions.
    CHECK(step_lengths(recorder) ==
          std::vector<Duration>{30ms, 30ms, 30ms, 10ms});
    CHECK(driver.now() == TimePoint{100ms});
    CHECK(recorder.steps.back().time == TimePoint{90ms});
  }

  SECTION("ShouldEndStepsAtDueTimesGivenTimeline") {
    // Preconditions.
    TimedRecorder timed;
    timed.timeline_.due = {TimePoint{45ms}, TimePoint{60ms}};
    Driver landing{Timing{.max_step = 30ms}, Depend(timed)};
    REQUIRE(landing.start());

    // Under Test.
    REQUIRE(landing.advance_to(TimePoint{100ms}) == Flow::CONTINUE);

    // Postconditions.
    CHECK(step_lengths(timed.recorder) ==
          std::vector<Duration>{30ms, 15ms, 15ms, 30ms, 10ms});
  }

  SECTION("ShouldStepStraightToDueTimesGivenIdleAndNothingEveryStep") {
    // Preconditions.
    TimedRecorder timed;
    timed.timeline_.due = {TimePoint{45ms}, TimePoint{60ms}};
    Driver idling{Timing{.max_step = 30ms, .idle = true}, Depend(timed)};
    REQUIRE(idling.start());

    // Under Test.
    REQUIRE(idling.advance_to(TimePoint{100ms}) == Flow::CONTINUE);

    // Postconditions.
    // Instants at the due times, the clock moved between them.
    CHECK(step_lengths(timed.recorder) == std::vector<Duration>{0ms, 0ms});
    CHECK(timed.recorder.steps.at(0).time == TimePoint{45ms});
    CHECK(timed.recorder.steps.at(1).time == TimePoint{60ms});
    CHECK(idling.now() == TimePoint{100ms});
  }

  SECTION("ShouldStepAsUsualGivenIdleButSomethingEveryStep") {
    // Preconditions.
    TimedRecorder timed;
    timed.timeline_.due = {TimePoint{45ms}};
    timed.timeline_.every_step = true;
    Driver idling{Timing{.max_step = 30ms, .idle = true}, Depend(timed)};
    REQUIRE(idling.start());

    // Under Test.
    REQUIRE(idling.advance_to(TimePoint{100ms}) == Flow::CONTINUE);

    // Postconditions.
    CHECK(step_lengths(timed.recorder) ==
          std::vector<Duration>{30ms, 15ms, 30ms, 25ms});
  }

  SECTION("ShouldThrowGivenAdvanceBeforeStart") {
    // Postconditions.
    CHECK_THROWS_AS(driver.advance_to(TimePoint{1s}), std::logic_error);
  }

  SECTION("ShouldRefuseToStepGivenStopped") {
    // Preconditions.
    recorder.stop_at = TimePoint{0ms};
    REQUIRE(driver.start());

    // Under Test.
    REQUIRE(driver.advance_to(TimePoint{1s}) == Flow::STOP);

    // Postconditions.
    CHECK(driver.phase() == Phase::STOPPED);
    CHECK_THROWS_AS(driver.advance_to(TimePoint{2s}), std::logic_error);
  }
}

TEST_CASE("BatchDriver") {
  Recorder recorder;
  BatchDriver driver{Timing{.max_step = 10ms}, Depend(recorder)};

  SECTION("ShouldRunLifecycleInOrderGivenRunToEnd") {
    // Under Test.
    auto reached = driver.run(TimePoint{50ms});

    // Postconditions.
    REQUIRE(reached);
    CHECK(*reached == TimePoint{50ms});
    CHECK(recorder.steps.size() == 5u);
    CHECK(recorder.phases ==
          std::vector<std::string>{"configure", "initialize", "finalize"});
  }

  SECTION("ShouldStopAfterStoppingStepGivenSimulationStops") {
    // Preconditions.
    recorder.stop_at = TimePoint{20ms};

    // Under Test.
    auto reached = driver.run(TimePoint{1s});

    // Postconditions.
    REQUIRE(reached);
    CHECK(*reached == TimePoint{30ms});  // The end of the step that stopped.
    CHECK(recorder.steps.size() == 3u);
    CHECK(recorder.phases.back() == "finalize");
  }

  SECTION("ShouldReturnErrorAndStillFinalizeGivenFailingStep") {
    // Preconditions.
    recorder.fail_at = TimePoint{10ms};

    // Under Test.
    auto reached = driver.run(TimePoint{1s});

    // Postconditions.
    REQUIRE_FALSE(reached);
    CHECK(reached.error() == lib::watch(TestCondition::FAILED));
    CHECK(recorder.phases.back() == "finalize");
  }

  SECTION("ShouldNotStepGivenConfigureStops") {
    // Preconditions.
    recorder.configure_result = Flow::STOP;

    // Under Test.
    auto reached = driver.run(TimePoint{1s});

    // Postconditions.
    REQUIRE(reached);
    CHECK(recorder.steps.empty());
    CHECK(recorder.phases == std::vector<std::string>{"configure", "finalize"});
  }
}

TEST_CASE("RealTimeDriver") {
  Recorder recorder;
  FakeClock::current = FakeClock::time_point{};
  RealTimeDriver<Recorder, FakeClock> driver{Timing{.max_step = 20ms}, 2.0,
                                             Depend(recorder)};

  SECTION("ShouldTakeOnlyWholeStepsGivenWallTimeBetweenSteps") {
    // Under Test.
    REQUIRE(driver.tick() == Flow::CONTINUE);  // Starts; nothing due yet.

    // Postconditions.
    CHECK(recorder.steps.empty());

    // Under Test.
    FakeClock::current += 25ms;  // 50 ms simulated: two whole steps are due.
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(step_lengths(recorder) == std::vector<Duration>{20ms, 20ms});

    // Under Test.
    FakeClock::current += 5ms;  // 60 ms simulated: one more.
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(step_lengths(recorder) == std::vector<Duration>{20ms, 20ms, 20ms});
    CHECK(driver.driver().now() == TimePoint{60ms});
  }

  SECTION("ShouldTakeStepGivenWallClockExactlyAtStepBoundary") {
    // Preconditions.
    Recorder fast_recorder;
    RealTimeDriver<Recorder, FakeClock> fast{Timing{.max_step = 10ms}, 5.0,
                                             Depend(fast_recorder)};
    REQUIRE(fast.tick() == Flow::CONTINUE);

    // Under Test.
    // 22 ms at speed 5 is exactly 110 ms, eleven 10 ms steps. Truncating the
    // floating-point product lands at 109.999999 ms, one step short.
    FakeClock::current += 22ms;
    REQUIRE(fast.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(fast_recorder.steps.size() == 11u);
    CHECK(fast.driver().now() == TimePoint{110ms});
  }

  SECTION("ShouldTakeSameStepsAsBatchGivenIrregularWallTicks") {
    // Under Test.
    for (auto wall : {3ms, 17ms, 18ms, 41ms, 100ms, 101ms, 150ms}) {
      FakeClock::current = FakeClock::time_point{wall};
      REQUIRE(driver.tick() == Flow::CONTINUE);
    }
    Recorder batch_recorder;
    BatchDriver batch{Timing{.max_step = 20ms}, Depend(batch_recorder)};
    REQUIRE(batch.run(driver.driver().now()));

    // Postconditions.
    REQUIRE(recorder.steps.size() == batch_recorder.steps.size());
    for (std::size_t i = 0; i < recorder.steps.size(); ++i) {
      CHECK(recorder.steps[i].time == batch_recorder.steps[i].time);
      CHECK(recorder.steps[i].dt == batch_recorder.steps[i].dt);
    }
  }

  SECTION("ShouldNotStepOrCatchUpGivenPauseThenResume") {
    // Preconditions.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current += 20ms;  // 40 ms simulated: two steps.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    REQUIRE(recorder.steps.size() == 2u);

    // Under Test.
    driver.pause();
    FakeClock::current += 1s;
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(recorder.steps.size() == 2u);  // Paused.

    // Under Test.
    driver.resume();
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(recorder.steps.size() == 2u);  // The paused second is not caught up.

    // Under Test.
    FakeClock::current += 10ms;  // 20 ms simulated: one step.
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(recorder.steps.size() == 3u);
    CHECK(driver.driver().now() == TimePoint{60ms});
  }

  SECTION("ShouldCatchUpAtMostMaxLagGivenSlowFrames") {
    // Preconditions.
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Under Test.
    // A second behind at 2x is 2 s simulated; a tick takes only 100 ms of
    // wall time's worth, 200 ms in ten steps, and drops the rest.
    FakeClock::current += 1s;
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(recorder.steps.size() == 10u);
    CHECK(driver.driver().now() == TimePoint{200ms});

    // Under Test.
    FakeClock::current += 10ms;  // 20 ms simulated from there: one step.
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(recorder.steps.size() == 11u);
    CHECK(driver.driver().now() == TimePoint{220ms});
  }

  SECTION("ShouldChangeRateWithoutJumpGivenNewSpeed") {
    // Preconditions.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current += 20ms;  // 40 ms simulated at 2x.
    REQUIRE(driver.tick() == Flow::CONTINUE);
    REQUIRE(driver.driver().now() == TimePoint{40ms});

    // Under Test.
    driver.set_speed(4.0);
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(driver.driver().now() == TimePoint{40ms});  // No jump.

    // Under Test.
    FakeClock::current += 10ms;  // 40 ms simulated at 4x.
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Postconditions.
    CHECK(driver.driver().now() == TimePoint{80ms});
    CHECK(driver.speed() == 4.0);
  }

  SECTION("ShouldReportStopGivenSimulationStopped") {
    // Preconditions.
    recorder.stop_at = TimePoint{0ms};
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current += 50ms;

    // Postconditions.
    CHECK(driver.tick() == Flow::STOP);
    CHECK(driver.tick() == Flow::STOP);
    CHECK(recorder.steps.size() == 1u);
  }

  SECTION("ShouldSleepUntilDueGivenIdleRun") {
    // Preconditions.
    TimedRecorder timed;
    timed.timeline_.due = {TimePoint{10s}};
    timed.recorder.stop_at = TimePoint{10s};
    RealTimeDriver<TimedRecorder> sleeper{Timing{.max_step = 1ms, .idle = true},
                                          1000.0, Depend(timed)};

    // Under Test.
    REQUIRE(sleeper.run());

    // Postconditions.
    // 10 s of simulated time at 1000x is 10 ms of wall time, which a 1 ms
    // step would cross in 10,000 steps.
    CHECK(timed.recorder.steps.size() < 10u);
    CHECK(timed.recorder.steps.back().time == TimePoint{10s});
  }

  SECTION("ShouldStepGivenWakeWithNothingDue") {
    // Preconditions.
    TimedRecorder timed;
    timed.recorder.stop_at = TimePoint{};
    RealTimeDriver<TimedRecorder> sleeper{Timing{.max_step = 1ms, .idle = true},
                                          1000.0, Depend(timed)};

    // Under Test.
    // As an input thread would: publish work due now, then wake the driver.
    std::thread waker{[&] {
      std::this_thread::sleep_for(20ms);
      timed.timeline_.due.push_back(TimePoint{});
      sleeper.wake();
    }};
    REQUIRE(sleeper.run());
    waker.join();

    // Postconditions.
    CHECK(timed.recorder.steps.size() == 1u);
  }
}

TEST_CASE("RealTimeDriverNextWake") {
  FakeClock::current = FakeClock::time_point{};
  TimedRecorder timed;
  timed.timeline_.due = {TimePoint{1s}};

  SECTION("ShouldWakeAtNextDueWallTimeGivenIdling") {
    // Preconditions.
    RealTimeDriver<TimedRecorder, FakeClock> driver{
        Timing{.max_step = 10ms, .idle = true}, 2.0, Depend(timed)};
    REQUIRE(driver.tick() == Flow::CONTINUE);  // Starts at wall time zero.

    // Under Test.
    auto wake = driver.next_wake();

    // Postconditions.
    // 1 s simulated at twice real time is half a second of wall time.
    CHECK(wake == FakeClock::time_point{500ms});
  }

  SECTION("ShouldNotWakeGivenPaused") {
    // Preconditions.
    RealTimeDriver<TimedRecorder, FakeClock> driver{
        Timing{.max_step = 10ms, .idle = true}, 2.0, Depend(timed)};
    REQUIRE(driver.tick() == Flow::CONTINUE);

    // Under Test.
    driver.pause();

    // Postconditions.
    CHECK(driver.next_wake() == std::nullopt);
  }

  SECTION("ShouldWakeNowGivenSomethingEveryStep") {
    // Preconditions.
    timed.timeline_.every_step = true;
    RealTimeDriver<TimedRecorder, FakeClock> driver{
        Timing{.max_step = 10ms, .idle = true}, 2.0, Depend(timed)};
    REQUIRE(driver.tick() == Flow::CONTINUE);
    FakeClock::current = FakeClock::time_point{3ms};

    // Under Test.
    auto wake = driver.next_wake();

    // Postconditions.
    CHECK(wake == FakeClock::time_point{3ms});
  }
}

}  // namespace simon::engine
