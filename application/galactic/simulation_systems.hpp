// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "application/galactic/simulation_components.hpp"
#include "framework/system.hpp"
#include "model/gravity.hpp"
#include "model/leapfrog.hpp"

namespace simon::galactic {

// Computes every body's gravity before the first step.
using StartSchedule = framework::SystemList<model::SumGravity>;
using StartScheduler = framework::Scheduler<World, StartSchedule>;

using Schedule = model::Leapfrog<model::SumGravity>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::galactic
