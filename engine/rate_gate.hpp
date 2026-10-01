// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>

#include "base/core.hpp"
#include "framework/step.hpp"

namespace simon::engine {

using framework::Duration;
using framework::Step;
using framework::TimePoint;

// What a gate does when a step spans more than one of its periods.
enum class CatchUp {
  SKIP,   // Fire once; the missed periods are dropped.
  EVERY,  // Fire once, reporting every period that fell in the step.
};

// When a gate fires: the time since it last fired, and how many periods to
// run (always 1 for CatchUp::SKIP).
struct Firing final {
  Duration elapsed{};
  std::uint32_t periods = 1;
};

// Runs work at a lower rate than the step, such as a radar scanning at 10 Hz on
// a 100 Hz step. A component holds a gate; its system asks the gate each step.
// The gate fires on the first step it is asked about, then on each step that
// contains one of its period boundaries. The work receives the elapsed time
// since the gate last fired (zero the first time), never the driver's step.
//
// A gate given a first firing time fires first on the step that contains it
// (or on the first step asked after it), and on each period after it. Gates of the same period with different first
// times spread their work over the period instead of firing on one step.
class RateGate final {
 public:
  RateGate() = default;
  explicit RateGate(Duration period, CatchUp catch_up = CatchUp::SKIP)
      : period_{period}, catch_up_{catch_up} {
    CHECK_PRECONDITION(period_ > Duration::zero());
  }
  RateGate(Duration period, CatchUp catch_up, TimePoint first)
      : period_{period}, catch_up_{catch_up}, next_{first} {
    CHECK_PRECONDITION(period_ > Duration::zero());
  }

  auto period() const -> Duration { return period_; }
  auto catch_up() const -> CatchUp { return catch_up_; }

  auto fire(const Step& step) -> std::optional<Firing> {
    CHECK_PRECONDITION(period_ > Duration::zero());
    if (!next_) {
      next_ = step.time;
    }
    TimePoint step_end = step.time + step.dt;
    if (*next_ >= step_end) {
      return std::nullopt;
    }
    // The number of period boundaries in [next_, step_end).
    auto boundaries = static_cast<std::uint32_t>(
        (step_end - *next_ - Duration{1}) / period_ + 1);
    *next_ += boundaries * period_;
    Firing firing{
        .elapsed = last_ ? step.time - *last_ : Duration::zero(),
        .periods = catch_up_ == CatchUp::EVERY ? boundaries : 1u,
    };
    last_ = step.time;
    return firing;
  }

 private:
  Duration period_{};
  CatchUp catch_up_ = CatchUp::SKIP;
  std::optional<TimePoint> next_;
  std::optional<TimePoint> last_;
};

}  // namespace simon::engine
