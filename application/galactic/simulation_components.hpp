// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "framework/archetype.hpp"
#include "framework/vocabulary.hpp"
#include "framework/world.hpp"
#include "model/gravity.hpp"
#include "model/kinematics.hpp"

// The galactic world: bodies under each other's gravity. A body stands for
// many stars or much dark matter, so that a galaxy of 10^11 stars is
// sampled by 10^4 to 10^6 bodies.
namespace simon::galactic {

using framework::Entity;
using model::Gravity;
using model::Kinematics;
using model::PointMass;

// A body that pulls on every other and is pulled by them.
struct Body final
    : framework::Archetype<
          "body", framework::Requires<Kinematics, PointMass, Gravity>> {};

using World = framework::World<Kinematics,                               //
                               framework::TypeList<PointMass, Gravity>,  //
                               framework::TypeList<Body>>;

}  // namespace simon::galactic
