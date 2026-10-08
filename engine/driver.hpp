// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <concepts>
#include <condition_variable>
#include <expected>
#include <mutex>
#include <optional>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/time.hpp"
#include "engine/lifecycle.hpp"

// A driver owns time and makes a simulation go. Every driver uses one
// contract, `advance_to(target)`: step toward `target` at no more than the
// maximum step, clamping the last step so it lands exactly on `target`.
// Drivers differ only in where the target comes from:
//
//   BatchDriver     a fixed end time, as fast as possible
//   RealTimeDriver  the wall clock, paced
namespace simon::engine {

// When a run starts and its longest step, in ticks of `TickType`, and
// whether it may idle: step straight to the next time work is due, sleeping
// in real time, while its timeline has nothing that runs every step.
template <typename TickType = Duration>
struct BasicTiming final {
  BasicTimePoint<TickType> start{};
  TickType max_step{};
  bool idle = false;
};

using Timing = BasicTiming<>;

// A simulation that says when it next has work, through a timeline whose
// `earliest()` is the next due time, perhaps already reached,
// `earliest_after(time)` the next one after a time, and `continuous()`
// whether something runs every step (see framework/timeline.hpp). Its driver
// ends each step at the next due time. An event a handler schedules inside
// the step it handles waits for the next step, at most `max_step`, unless the
// driver is idling.
template <typename SimulationType>
concept HasTimeline = requires(const SimulationType& simulation) {
  {
    simulation.timeline().earliest()
  } -> std::same_as<std::optional<BasicTimePoint<tick_of_t<SimulationType>>>>;
  {
    simulation.timeline().earliest_after(
        BasicTimePoint<tick_of_t<SimulationType>>{})
  } -> std::same_as<std::optional<BasicTimePoint<tick_of_t<SimulationType>>>>;
  { simulation.timeline().continuous() } -> std::same_as<bool>;
};

// The lifecycle and `advance_to`, shared by every driver.
template <Simulation SimulationType>
class Driver final {
 public:
  using Duration = tick_of_t<SimulationType>;
  using TimePoint = BasicTimePoint<Duration>;
  using Step = BasicStep<Duration>;

  // Keeps a reference to `simulation` for as long as the driver lives.
  Driver(BasicTiming<Duration> timing, Depend<SimulationType> simulation)
      : simulation_{simulation.get()},
        now_{timing.start},
        max_step_{timing.max_step},
        idle_{timing.idle} {
    CHECK_PRECONDITION(max_step_ > Duration::zero());
  }

  auto now() const -> TimePoint { return now_; }
  auto max_step() const -> Duration { return max_step_; }

  // Whether it steps straight to the next due time: it may idle, and nothing
  // runs every step.
  auto idling() const -> bool {
    if constexpr (HasTimeline<SimulationType>) {
      return idle_ && !simulation_->timeline().continuous();
    }
    return false;
  }

  // When the simulation next has work: the next due time while idling (none
  // if nothing is due), else the next step.
  auto next_due() const -> std::optional<TimePoint> {
    if constexpr (HasTimeline<SimulationType>) {
      if (idling()) {
        return simulation_->timeline().earliest();
      }
    }
    return now_ + max_step_;
  }
  auto phase() const -> Phase { return phase_; }

  // Configures and initializes the simulation.
  auto start() -> PhaseResult {
    CHECK_PRECONDITION(phase_ == Phase::NEW);
    for (auto phase : {&Driver::configure, &Driver::initialize}) {
      PhaseResult result = (this->*phase)();
      if (!result || *result == Flow::STOP) {
        phase_ = Phase::STOPPED;
        return result;
      }
    }
    phase_ = Phase::RUNNING;
    return Flow::CONTINUE;
  }

  // Steps toward `target`, at most `max_step` at a time, landing exactly on it,
  // and on each time the simulation's timeline says work is due. While
  // idling, it runs only the instants at which work is due, and moves the
  // clock between them without stepping, since nothing runs there.
  // Stops early, and for good, when a step returns Flow::STOP or an error.
  auto advance_to(TimePoint target) -> PhaseResult {
    CHECK_PRECONDITION(phase_ == Phase::RUNNING);
    while (now_ < target) {
      if (idling()) {
        PhaseResult result = idle_toward(target);
        if (!result || *result == Flow::STOP) {
          phase_ = Phase::STOPPED;
          return result;
        }
        continue;
      }
      Duration dt = std::min(max_step_, target - now_);
      if constexpr (HasTimeline<SimulationType>) {
        if (std::optional<TimePoint> due =
                simulation_->timeline().earliest_after(now_)) {
          dt = std::min(dt, *due - now_);
        }
      }
      PhaseResult result = simulation_->step(Step{.time = now_, .dt = dt});
      now_ += dt;
      if (!result || *result == Flow::STOP) {
        phase_ = Phase::STOPPED;
        return result;
      }
    }
    return Flow::CONTINUE;
  }

  // While idling, one move toward `target`: an instant, a step of no length,
  // when work is due now, so that what it schedules is seen before the clock
  // moves on; else the clock straight to the next due time or `target`.
  auto idle_toward(TimePoint target) -> PhaseResult {
    if constexpr (HasTimeline<SimulationType>) {
      std::optional<TimePoint> due = simulation_->timeline().earliest();
      if (due && *due <= now_) {
        return simulation_->step(Step{.time = now_, .dt = Duration::zero()});
      }
      now_ = due ? std::min(*due, target) : target;
    }
    return Flow::CONTINUE;
  }

  // Finalizes the simulation. Allowed once, after start.
  auto finish() -> FinishResult {
    CHECK_PRECONDITION(phase_ == Phase::RUNNING || phase_ == Phase::STOPPED);
    phase_ = Phase::FINISHED;
    if constexpr (requires { simulation_->finalize(); }) {
      return simulation_->finalize();
    }
    return {};
  }

 private:
  auto configure() -> PhaseResult {
    if constexpr (requires { simulation_->configure(); }) {
      return simulation_->configure();
    }
    return Flow::CONTINUE;
  }

  auto initialize() -> PhaseResult {
    if constexpr (requires { simulation_->initialize(); }) {
      return simulation_->initialize();
    }
    return Flow::CONTINUE;
  }

  // Never null once constructed; Depend checks it.
  SimulationType* simulation_ = nullptr;
  TimePoint now_;
  Duration max_step_;
  bool idle_;
  Phase phase_ = Phase::NEW;
};

// Runs a simulation as fast as possible to a fixed end time, or until it
// stops. For tests and batch runs.
template <Simulation SimulationType>
class BatchDriver final {
 public:
  using Duration = tick_of_t<SimulationType>;
  using TimePoint = BasicTimePoint<Duration>;

  BatchDriver(BasicTiming<Duration> timing, Depend<SimulationType> simulation)
      : driver_{timing, simulation} {}

  // Runs the whole lifecycle. Returns the time the simulation reached.
  auto run(TimePoint end) -> std::expected<TimePoint, Status> {
    PhaseResult started = driver_.start();
    if (started && *started == Flow::CONTINUE) {
      started = driver_.advance_to(end);
    }
    FinishResult finished = driver_.finish();
    RETURN_IF_UNEXPECTED(started);
    RETURN_IF_UNEXPECTED(finished);
    return driver_.now();
  }

  auto driver() const -> const Driver<SimulationType>& { return driver_; }

 private:
  Driver<SimulationType> driver_;
};

// Paces a simulation to a wall clock, `speed` simulated seconds per wall
// second. Targets are always whole multiples of the maximum step, so a
// real-time run takes exactly the steps a batch run would: the wall clock
// decides when steps happen, never how long they are. While idling, a target
// also reaches any due time the wall clock has passed; the steps then differ
// from a batch run's, but only by steps in which nothing runs.
template <Simulation SimulationType,
          typename WallClockType = std::chrono::steady_clock>
class RealTimeDriver final {
 public:
  using Duration = tick_of_t<SimulationType>;
  using TimePoint = BasicTimePoint<Duration>;

  // Runs at `speed` times real time.
  RealTimeDriver(BasicTiming<Duration> timing, double speed,
                 Depend<SimulationType> simulation)
      : driver_{timing, simulation}, start_{timing.start}, speed_{speed} {
    CHECK_PRECONDITION(speed_ > 0.0);
  }

  // Advances to the step the wall clock has reached. Call it from an
  // application's frame loop. The first call starts the simulation.
  //
  // A tick catches up at most MAX_LAG of wall time, at the current speed. A
  // simulation that cannot keep up drops the rest, and runs slower than its
  // speed, so the frame loop keeps drawing; each tick still takes whole steps.
  auto tick() -> PhaseResult {
    if (driver_.phase() == Phase::NEW) {
      wall_start_ = WallClockType::now();
      PhaseResult started = driver_.start();
      if (!started || *started == Flow::STOP) {
        return started;
      }
    }
    if (driver_.phase() != Phase::RUNNING) {
      return Flow::STOP;
    }
    if (paused_) {
      return Flow::CONTINUE;
    }
    typename WallClockType::time_point wall = WallClockType::now();
    TimePoint target = target_at(wall);
    if (driver_.idling()) {
      // Due times need not fall on whole steps; reach one the wall clock has
      // passed.
      if (std::optional<TimePoint> due = driver_.next_due();
          due && *due <= simulated_at(wall)) {
        target = std::max(target, *due);
      }
    }
    TimePoint most = driver_.now() + most_steps();
    if (target <= most) {
      return driver_.advance_to(target);
    }
    PhaseResult result = driver_.advance_to(most);
    rebase();  // The dropped time is not owed.
    return result;
  }

  // The most wall time a tick catches up.
  static constexpr std::chrono::milliseconds MAX_LAG{100};

  // Stops simulated time until `resume`. Wall time that passes while paused is
  // never caught up.
  auto pause() -> void { paused_ = true; }
  auto resume() -> void {
    if (paused_) {
      paused_ = false;
      rebase();
    }
  }
  auto paused() const -> bool { return paused_; }

  // When the run next needs a tick, for a frame loop that can wait: now while
  // something runs every step, the wall time of the next due work while
  // idling, and nothing while paused, stopped, or idling with nothing due,
  // until something else wakes it.
  auto next_wake() const -> std::optional<typename WallClockType::time_point> {
    Phase phase = driver_.phase();
    if (phase == Phase::NEW) {
      return WallClockType::now();
    }
    if (paused_ || phase != Phase::RUNNING) {
      return std::nullopt;
    }
    if (!driver_.idling()) {
      return WallClockType::now();
    }
    std::optional<TimePoint> due = driver_.next_due();
    if (!due) {
      return std::nullopt;
    }
    return to_wall_time(*due);
  }

  // Changes how many simulated seconds pass per wall second, from now on.
  // Simulated time does not jump.
  auto set_speed(double speed) -> void {
    CHECK_PRECONDITION(speed > 0.0);
    rebase();
    speed_ = speed;
  }
  auto speed() const -> double { return speed_; }

  // Ticks until the simulation stops, sleeping until each step is due, or,
  // while idling, until work is due or `wake` is called. For headless runs;
  // it needs a WallClockType a condition variable can wait on.
  auto run() -> FinishResult {
    for (;;) {
      PhaseResult result = tick();
      if (!result) {
        // The step's error is the one to report; finishing is best effort.
        FinishResult _ = driver_.finish();
        return std::unexpected(result.error());
      }
      if (*result == Flow::STOP) {
        return driver_.finish();
      }
      std::optional<TimePoint> due = driver_.next_due();
      std::unique_lock lock{mutex_};
      auto woken = [this] { return woken_; };
      if (due) {
        wake_.wait_until(lock, to_wall_time(*due), woken);
      } else {
        wake_.wait(lock, woken);  // Nothing is due until something wakes it.
      }
      woken_ = false;
    }
  }

  // Ends `run`'s sleep now, from any thread: after publishing an event, say,
  // so that it is delivered without waiting for the next due time.
  auto wake() -> void {
    {
      std::lock_guard lock{mutex_};
      woken_ = true;
    }
    wake_.notify_one();
  }

  auto finish() -> FinishResult { return driver_.finish(); }

  auto driver() const -> const Driver<SimulationType>& { return driver_; }

 private:
  // The simulated time `wall` corresponds to.
  auto simulated_at(typename WallClockType::time_point wall) const
      -> TimePoint {
    // Rounded, not truncated: at an exact step boundary the floating-point
    // product can fall a fraction of a nanosecond short.
    return start_ +
           std::chrono::round<Duration>(
               std::chrono::duration<double>(wall - wall_start_) * speed_);
  }

  // The last whole step at or before the simulated time `wall` corresponds to.
  auto target_at(typename WallClockType::time_point wall) const -> TimePoint {
    Duration step = driver_.max_step();
    return start_ + ((simulated_at(wall) - start_) / step) * step;
  }

  // MAX_LAG at the current speed, in whole steps, and at least one.
  auto most_steps() const -> Duration {
    auto simulated = std::chrono::round<Duration>(
        std::chrono::duration<double>(MAX_LAG) * speed_);
    Duration step = driver_.max_step();
    return std::max<typename Duration::rep>(simulated / step, 1) * step;
  }

  // Anchors the wall clock to the simulation's current time, so targets are
  // measured from here. The simulation's time is always a whole number of
  // steps from its start, so targets stay whole steps and runs stay
  // deterministic.
  auto rebase() -> void {
    start_ = driver_.now();
    wall_start_ = WallClockType::now();
  }

  auto to_wall_time(TimePoint time) const ->
      typename WallClockType::time_point {
    auto simulated = std::chrono::duration<double>(time - start_);
    // Rounded up, so waiting until then never wakes just before the time.
    return wall_start_ + std::chrono::ceil<typename WallClockType::duration>(
                             simulated / speed_);
  }

  Driver<SimulationType> driver_;
  TimePoint start_;
  double speed_;
  typename WallClockType::time_point wall_start_{};
  bool paused_ = false;
  std::mutex mutex_;  // Guards woken_.
  std::condition_variable wake_;
  bool woken_ = false;
};

}  // namespace simon::engine
