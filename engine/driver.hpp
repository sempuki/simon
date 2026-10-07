// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <expected>
#include <thread>

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

// When a run starts and its longest step, in ticks of `TickType`.
template <typename TickType = Duration>
struct BasicTiming final {
  BasicTimePoint<TickType> start{};
  TickType max_step{};
};

using Timing = BasicTiming<>;

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
        max_step_{timing.max_step} {
    CHECK_PRECONDITION(max_step_ > Duration::zero());
  }

  auto now() const -> TimePoint { return now_; }
  auto max_step() const -> Duration { return max_step_; }
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

  // Steps toward `target`, at most `max_step` at a time, landing exactly on it.
  // Stops early, and for good, when a step returns Flow::STOP or an error.
  auto advance_to(TimePoint target) -> PhaseResult {
    CHECK_PRECONDITION(phase_ == Phase::RUNNING);
    while (now_ < target) {
      Duration dt = std::min(max_step_, target - now_);
      PhaseResult result = simulation_->step(Step{.time = now_, .dt = dt});
      now_ += dt;
      if (!result || *result == Flow::STOP) {
        phase_ = Phase::STOPPED;
        return result;
      }
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
// decides when steps happen, never how long they are.
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
    TimePoint target = target_at(WallClockType::now());
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

  // Changes how many simulated seconds pass per wall second, from now on.
  // Simulated time does not jump.
  auto set_speed(double speed) -> void {
    CHECK_PRECONDITION(speed > 0.0);
    rebase();
    speed_ = speed;
  }
  auto speed() const -> double { return speed_; }

  // Ticks until the simulation stops, sleeping until each step is due. For
  // headless runs; it needs a WallClockType that sleep_until understands.
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
      std::this_thread::sleep_until(
          to_wall_time(driver_.now() + driver_.max_step()));
    }
  }

  auto finish() -> FinishResult { return driver_.finish(); }

  auto driver() const -> const Driver<SimulationType>& { return driver_; }

 private:
  // The last whole step at or before the simulated time `wall` corresponds to.
  auto target_at(typename WallClockType::time_point wall) const -> TimePoint {
    // Rounded, not truncated: at an exact step boundary the floating-point
    // product can fall a fraction of a nanosecond short.
    auto simulated = std::chrono::round<Duration>(
        std::chrono::duration<double>(wall - wall_start_) * speed_);
    Duration step = driver_.max_step();
    return start_ + (simulated / step) * step;
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
};

}  // namespace simon::engine
