// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "base/core.hpp"
#include "core/time.hpp"

// When a simulation next has work: the next boundary of each system that runs
// at its own period, and the next event. A driver ends each step at the
// earliest of them, so each fires in a step that starts at its own time.
namespace simon::framework {

class Timeline final {
 public:
  // A source's place in the timeline.
  using Source = std::uint32_t;

  // Adds a source with nothing due yet.
  auto add() -> Source {
    due_.emplace_back();
    return static_cast<Source>(due_.size() - 1);
  }

  // Makes `source` due at `time`, or at nothing.
  auto set(Source source, std::optional<TimePoint> time) -> void {
    CHECK_PRECONDITION(source < due_.size());
    due_[source] = time;
  }

  auto due(Source source) const -> std::optional<TimePoint> {
    CHECK_PRECONDITION(source < due_.size());
    return due_[source];
  }

  // The earliest time any source is due, or nothing. A linear scan: a
  // timeline holds one source per rated system and one for events.
  auto earliest() const -> std::optional<TimePoint> {
    std::optional<TimePoint> first;
    for (const std::optional<TimePoint>& time : due_) {
      if (time && (!first || *time < *first)) {
        first = time;
      }
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
  std::vector<std::optional<TimePoint>> due_;
  std::int32_t continuous_ = 0;
};

}  // namespace simon::framework
