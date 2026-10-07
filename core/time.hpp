// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

#include "base/time.hpp"

namespace simon {

using lib::Duration;
using lib::TimePoint;

// A simulation time point counted in `TickType`, an integer std::chrono
// duration. Nanoseconds, the default, cover about 292 years; a simulation
// that runs longer chooses a coarser tick, such as years.
template <typename TickType>
using BasicTimePoint = std::chrono::time_point<lib::SimTime, TickType>;

// The time a system runs at. Drivers produce steps; there is no global clock.
template <typename TickType>
struct BasicStep final {
  using Tick = TickType;

  BasicTimePoint<TickType> time;  // Start of this step.
  TickType dt;                    // Length of this step.
};

using Step = BasicStep<Duration>;

}  // namespace simon
