// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

namespace simon {

class SimClock final {
 public:
  using rep = double;
  using period = std::ratio<1>;  // durations are fractions of one second
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<SimClock>;
  static constexpr bool is_steady = true;

  static time_point now() {
    static SimClock static_clock;
    return static_clock.time_;
  }

 private:
  time_point time_{duration{0.0}};
};

using Duration = SimClock::duration;
using TimePoint = SimClock::time_point;

}  // namespace simon
