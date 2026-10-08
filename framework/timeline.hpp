// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "base/core.hpp"
#include "core/argument.hpp"
#include "core/time.hpp"

// When a simulation next has work, asked of each thing whose work falls due:
// a scheduler's systems that run at their own periods, and an event queue. A
// driver ends each step at the next of those times, so each fires in a step
// that starts at its own time.
namespace simon::framework {

class Timeline final {
 public:
  // Something whose work falls due at times it knows.
  class Source {
   public:
    virtual ~Source() = default;

    // The earliest time its work is due, perhaps already past, or nothing.
    virtual auto earliest() const -> std::optional<TimePoint> = 0;

    // The earliest time after `time` its work is due, as far as it knows
    // before the work due at `time` runs.
    virtual auto earliest_after(TimePoint time) const
        -> std::optional<TimePoint> = 0;
  };

  // Asks `source` when it is due, for as long as the timeline lives.
  auto add(Depend<const Source> source) -> void {
    sources_.push_back(source.get());
  }

  // The earliest time any source is due, perhaps already past, or nothing.
  auto earliest() const -> std::optional<TimePoint> {
    std::optional<TimePoint> first;
    for (const Source* source : sources_) {
      first = sooner(first, source->earliest());
    }
    return first;
  }

  // The earliest time after `time` any source is due, or nothing.
  auto earliest_after(TimePoint time) const -> std::optional<TimePoint> {
    std::optional<TimePoint> first;
    for (const Source* source : sources_) {
      first = sooner(first, source->earliest_after(time));
    }
    return first;
  }

  // Whether something must run every step, so that the simulation always has
  // work: a scheduled system without a period.
  auto continuous() const -> bool { return continuous_ > 0; }

  // Counts what must run every step, up as each is added and down as each
  // gets a period.
  auto add_continuous(std::int32_t count) -> void {
    continuous_ += count;
    CHECK_POSTCONDITION(continuous_ >= 0);
  }

 private:
  static auto sooner(std::optional<TimePoint> a, std::optional<TimePoint> b)
      -> std::optional<TimePoint> {
    if (!a) {
      return b;
    }
    if (!b) {
      return a;
    }
    return *a < *b ? a : b;
  }

  std::vector<const Source*> sources_;  // Never null; Depend checks each.
  std::int32_t continuous_ = 0;
};

}  // namespace simon::framework
