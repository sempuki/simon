// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "application/galactic/simulation_components.hpp"
#include "framework/system.hpp"
#include "model/gravity/gravity.hpp"
#include "model/gravity/leapfrog.hpp"

namespace simon::galactic {

// Gravity by direct summation or by tree, whichever the scenario enables.
using Gravities = framework::SystemList<model::SumGravity, model::TreeGravity>;

// Computes every body's gravity before the first step.
using StartScheduler = framework::Scheduler<World, Gravities>;

using Schedule = model::Leapfrog<Gravities>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::galactic
