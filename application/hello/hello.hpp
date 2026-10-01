// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <expected>

#include "base/core.hpp"
#include "engine/lifecycle.hpp"
#include "framework/archetype.hpp"
#include "framework/system.hpp"
#include "framework/world.hpp"
#include "model/kinematics.hpp"
#include "model/motion.hpp"

// Two balls under thrust and wind that stop when they collide: the smallest
// complete use of the architecture.
namespace simon::hello {

using framework::Entity;
using model::Acceleration;
using model::Control;
using model::Kinematics;
using model::Length;
using model::meter;
using model::meters;
using model::meters_per_second;
using model::meters_per_second_squared;
using model::per_second;
using model::Rate;
using model::Velocity;

//-- Components ----------------------------------------------------------------

struct Thrust final {
  Acceleration acceleration = meters_per_second_squared(0.0, 0.0, 0.0);
};

struct Wind final {
  Velocity velocity = meters_per_second(0.0, 0.0, 0.0);
};

struct Drag final {
  Rate factor = 0.0 * per_second;  // acceleration = factor * (wind - velocity)
};

struct Collider final {
  Length radius = 0.0 * meter;
};

// Written by DetectCollisions on the entity that collided.
struct Collision final {
  bool hit = false;
  Entity other;
};

// A ball must have a position, a size and a collision record; it may be driven.
struct Ball final
    : framework::Archetype<"ball",
                           framework::Requires<Kinematics, Collider, Collision>,
                           framework::Allows<Control, Thrust, Wind, Drag>> {};

using World = framework::World<
    Kinematics,
    framework::TypeList<Control, Thrust, Wind, Drag, Collider, Collision>,
    framework::TypeList<Ball>>;

//-- Systems ------------------------------------------------------------------

// Sums thrust and wind drag into the commanded acceleration.
struct ApplyForces final                   //
    : framework::System<Control,           //
                        const Kinematics,  //
                        const Thrust,      //
                        const Wind,        //
                        const Drag> {
  auto operator()(auto&, Entity,                 //
                  Control& control,              //
                  const Kinematics* kinematics,  //
                  const Thrust* thrust,          //
                  const Wind* wind,              //
                  const Drag* drag) const -> void {
    control.acceleration = thrust ? thrust->acceleration
                                  : meters_per_second_squared(0.0, 0.0, 0.0);
    if (kinematics && drag) {
      Velocity air = wind ? wind->velocity : meters_per_second(0.0, 0.0, 0.0);
      control.acceleration += drag->factor * (air - kinematics->velocity);
    }
  }
};

// Each collider records its own collision: it writes only its own entity and
// finds the others through a spatial query.
struct DetectCollisions final            //
    : framework::System<Collision,       //
                        const Collider,  //
                        const Kinematics> {
  using AllowComponentList = framework::TypeList<Kinematics, Collider>;
  using SequenceAfterSystemList = framework::SystemList<model::Integrate>;

  auto prepare(auto& world) -> void {
    largest_radius = 0.0 * meter;
    store_of<Collider>(world).for_each([&](Entity, const Collider& collider) {
      largest_radius = std::max(largest_radius, collider.radius);
    });
  }

  auto operator()(auto& world, Entity self,  //
                  Collision& collision,      //
                  const Collider* collider,  //
                  const Kinematics* kinematics) const -> void {
    if (!collider || !kinematics) return;
    world.within(*kinematics, collider->radius + largest_radius,
                 [&](Entity other, const Kinematics& other_kinematics) {
                   const Collider* other_collider =
                       maybe_component_of<Collider>(world, other);
                   if (other == self || !other_collider) return;
                   if (distance(*kinematics, other_kinematics) <=
                       collider->radius + other_collider->radius) {
                     collision = Collision{.hit = true, .other = other};
                   }
                 });
  }

  Length largest_radius = 0.0 * meter;
};

using Schedule =
    framework::SystemList<ApplyForces, model::Motion, DetectCollisions>;
using Scheduler = framework::Scheduler<World, Schedule>;

//-- Scenario ------------------------------------------------------------------

struct Balls final {
  Entity red;
  Entity blue;
};

// Builds the two balls. One meter is drawn as one screen pixel.
auto build_balls(lib::InOut<World> world) -> Balls;

// Builds in `world` a world holding `balls` balls.
auto build_world(std::size_t balls, lib::Out<World> world)
    -> std::expected<void, framework::Status>;

// Whether any ball has collided.
auto any_collision(const World& world) -> bool;

// The hello simulation: builds the balls when configured, and stops at the
// first collision. Any driver can run it.
class Simulation final {
 public:
  // Builds the world and the two balls in it.
  auto configure() -> engine::PhaseResult;
  auto step(const framework::Step& step) -> engine::PhaseResult;

  // The world: empty until configured.
  auto world() const -> const World& { return world_; }
  auto balls() const -> const Balls& { return balls_; }

 private:
  World world_;  // Empty until configure builds it.
  Scheduler scheduler_;
  Balls balls_{};
};

}  // namespace simon::hello
