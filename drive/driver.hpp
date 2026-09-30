// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <expected>
#include <thread>

#include "base/core.hpp"
#include "core/step.hpp"
#include "drive/lifecycle.hpp"

// A driver owns time and makes a simulation go. Every driver uses one
// contract, `advance_to(target)`: step toward `target` at no more than the
// maximum step, clamping the last step so it lands exactly on `target`.
// Drivers differ only in where the target comes from:
//
//   BatchDriver     a fixed end time, as fast as possible
//   RealTimeDriver  the wall clock, paced
namespace simon::drive {

using core::Duration;
using core::Step;
using core::TimePoint;

struct Timing {
  TimePoint start{};
  Duration max_step{};
};

// The lifecycle and `advance_to`, shared by every driver.
template <Simulation SimulationType>
class Driver final {
 public:
  // Keeps a reference to `simulation` for as long as the driver lives.
  Driver(lib::Depend<SimulationType> simulation, Timing timing)
      : simulation_{simulation.get()},
        now_{timing.start},
        max_step_{timing.max_step} {
    CHECK_PRECONDITION(max_step_ > Duration::zero());
  }

  TimePoint now() const { return now_; }
  Duration max_step() const { return max_step_; }
  Phase phase() const { return phase_; }

  // Configures and initializes the simulation.
  PhaseResult start() {
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
  PhaseResult advance_to(TimePoint target) {
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
  FinishResult finish() {
    CHECK_PRECONDITION(phase_ == Phase::RUNNING || phase_ == Phase::STOPPED);
    phase_ = Phase::FINISHED;
    if constexpr (requires { simulation_->finalize(); }) {
      return simulation_->finalize();
    }
    return {};
  }

 private:
  PhaseResult configure() {
    if constexpr (requires { simulation_->configure(); }) {
      return simulation_->configure();
    }
    return Flow::CONTINUE;
  }

  PhaseResult initialize() {
    if constexpr (requires { simulation_->initialize(); }) {
      return simulation_->initialize();
    }
    return Flow::CONTINUE;
  }

  SimulationType* simulation_;  // Never null; checked once by Depend.
  TimePoint now_;
  Duration max_step_;
  Phase phase_ = Phase::NEW;
};

// Runs a simulation as fast as possible to a fixed end time, or until it
// stops. For tests and batch runs.
template <Simulation SimulationType>
class BatchDriver final {
 public:
  BatchDriver(lib::Depend<SimulationType> simulation, Timing timing)
      : driver_{simulation, timing} {}

  // Runs the whole lifecycle. Returns the time the simulation reached.
  std::expected<TimePoint, Status> run(TimePoint end) {
    PhaseResult started = driver_.start();
    if (started && *started == Flow::CONTINUE) {
      started = driver_.advance_to(end);
    }
    FinishResult finished = driver_.finish();
    if (!started) {
      return std::unexpected(started.error());
    }
    if (!finished) {
      return std::unexpected(finished.error());
    }
    return driver_.now();
  }

  const Driver<SimulationType>& driver() const { return driver_; }

 private:
  Driver<SimulationType> driver_;
};

// Paces a simulation to a wall clock, `speed` simulated seconds per wall
// second. Targets are always whole multiples of the maximum step, so a
// real-time run takes exactly the steps a batch run would: the wall clock
// decides when steps happen, never how long they are.
template <Simulation SimulationType,
          typename WallClock = std::chrono::steady_clock>
class RealTimeDriver final {
 public:
  RealTimeDriver(lib::Depend<SimulationType> simulation, Timing timing,
                 double speed = 1.0)
      : driver_{simulation, timing}, start_{timing.start}, speed_{speed} {
    CHECK_PRECONDITION(speed_ > 0.0);
  }

  // Advances to the step the wall clock has reached. Call it from an
  // application's frame loop. The first call starts the simulation.
  PhaseResult tick() {
    if (driver_.phase() == Phase::NEW) {
      wall_start_ = WallClock::now();
      PhaseResult started = driver_.start();
      if (!started || *started == Flow::STOP) {
        return started;
      }
    }
    if (driver_.phase() != Phase::RUNNING) {
      return Flow::STOP;
    }
    return driver_.advance_to(target_at(WallClock::now()));
  }

  // Ticks until the simulation stops, sleeping until each step is due. For
  // headless runs; it needs a WallClock that sleep_until understands.
  FinishResult run() {
    for (;;) {
      PhaseResult result = tick();
      if (!result) {
        (void)driver_.finish();
        return std::unexpected(result.error());
      }
      if (*result == Flow::STOP) {
        return driver_.finish();
      }
      std::this_thread::sleep_until(
          wall_time_of(driver_.now() + driver_.max_step()));
    }
  }

  FinishResult finish() { return driver_.finish(); }

  const Driver<SimulationType>& driver() const { return driver_; }

 private:
  // The last whole step at or before the simulated time `wall` corresponds to.
  TimePoint target_at(typename WallClock::time_point wall) const {
    auto simulated = std::chrono::duration_cast<Duration>(
        std::chrono::duration<double>(wall - wall_start_) * speed_);
    Duration step = driver_.max_step();
    return start_ + (simulated / step) * step;
  }

  typename WallClock::time_point wall_time_of(TimePoint time) const {
    auto simulated = std::chrono::duration<double>(time - start_);
    return wall_start_ +
           std::chrono::duration_cast<typename WallClock::duration>(simulated /
                                                                    speed_);
  }

  Driver<SimulationType> driver_;
  TimePoint start_;
  double speed_;
  typename WallClock::time_point wall_start_{};
};

}  // namespace simon::drive
