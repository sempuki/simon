// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>

// Simulation time, as opposed to wall time. Time counts integer nanoseconds,
// which is exact and covers about 292 years, so runs repeat exactly.
namespace simon {

// A tag that makes a simulation time point a different type from a
// wall-clock time point, so the two cannot be mixed by accident. It has no
// now(): a global clock would give code a hidden source of time, so the
// current time comes from whatever drives the simulation.
struct SimTime;

using Duration = std::chrono::nanoseconds;
using TimePoint = std::chrono::time_point<SimTime, Duration>;

// A simulation time point counted in `TickType`, an integer std::chrono
// duration. Nanoseconds, the default, cover about 292 years; a simulation
// that runs longer chooses a coarser tick, such as years.
template <typename TickType>
using BasicTimePoint = std::chrono::time_point<SimTime, TickType>;

// The time a system runs at. Drivers produce steps; there is no global clock.
template <typename TickType>
struct BasicStep final {
  using Tick = TickType;

  BasicTimePoint<TickType> time;  // Start of this step.
  TickType dt;                    // Length of this step.
};

using Step = BasicStep<Duration>;

}  // namespace simon
