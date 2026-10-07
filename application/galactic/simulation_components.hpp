// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "core/vocabulary.hpp"
#include "framework/archetype.hpp"
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

// A test particle: pulled by every body, it pulls on nothing. A star of a
// disk in Toomre and Toomre's restricted encounters, it costs one pass over
// the bodies that pull and nothing to them.
struct TestParticle final
    : framework::Archetype<"test particle",
                           framework::Requires<Kinematics, Gravity>> {};

using World = framework::World<Kinematics,                               //
                               framework::TypeList<PointMass, Gravity>,  //
                               framework::TypeList<Body, TestParticle>>;

}  // namespace simon::galactic
