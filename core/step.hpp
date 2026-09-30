// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "base/time.hpp"

namespace simon::core {

using lib::Duration;
using lib::TimePoint;

// The time a system runs at. Drivers produce steps; there is no global clock.
struct Step {
  TimePoint time;  // Start of this step.
  Duration dt;     // Length of this step.
};

}  // namespace simon::core
