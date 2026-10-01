// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <cstddef>
#include <expected>
#include <optional>

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
struct ApplyForces final
    : framework::System<Control, const Kinematics, const Thrust, const Wind,
                        const Drag> {
  void operator()(auto&, Entity, Control& control, const Kinematics* kinematics,
                  const Thrust* thrust, const Wind* wind,
                  const Drag* drag) const {
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
struct DetectCollisions final
    : framework::System<Collision, const Collider, const Kinematics> {
  using AllowComponentList = framework::TypeList<Kinematics, Collider>;
  using SequenceAfterSystemList = framework::SystemList<model::Integrate>;

  void prepare(auto& world) {
    largest_radius = 0.0 * meter;
    store_of<Collider>(world).for_each([&](Entity, const Collider& collider) {
      largest_radius = std::max(largest_radius, collider.radius);
    });
  }

  void operator()(auto& world, Entity self, Collision& collision,
                  const Collider* collider,
                  const Kinematics* kinematics) const {
    if (!collider || !kinematics) return;
    world.within(*kinematics, collider->radius + largest_radius,
                 [&](Entity other, const Kinematics& other_kinematics) {
                   const Collider* other_collider =
                       try_component_of<Collider>(world, other);
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
inline Balls build_balls(lib::InOut<World> world) {
  auto red =
      world->create<Ball>("red")
          .with(Kinematics{.position = meters(360.0, 100.0, 0.0),
                           .velocity = meters_per_second(10.0, -10.0, 0.0)})
          .with(Control{})
          .with(
              Thrust{.acceleration = meters_per_second_squared(0.0, 9.8, 0.0)})
          .with(Wind{.velocity = meters_per_second(-1.0, 0.0, 0.0)})
          .with(Drag{.factor = 0.4 * per_second})
          .with(Collider{.radius = 10.0 * meter})
          .with(Collision{})
          .build();
  auto blue = world->create<Ball>("blue")
                  .with(Kinematics{.position = meters(360.0, 600.0, 0.0)})
                  .with(Collider{.radius = 10.0 * meter})
                  .with(Collision{})
                  .build();
  CHECK_POSTCONDITION(red.has_value() && blue.has_value());
  world->sync();
  return Balls{.red = *red, .blue = *blue};
}

// A world holding `balls` balls.
inline std::expected<World, framework::Status> world_for(std::size_t balls) {
  return World::set_up().numbered(1).holding<Ball>(balls).build();
}

inline bool any_collision(const World& world) {
  bool hit = false;
  world.store_of<Collision>().for_each(
      [&](Entity, const Collision& collision) { hit = hit || collision.hit; });
  return hit;
}

// The hello simulation: builds the balls when configured, and stops at the
// first collision. Any driver can run it.
class Simulation final {
 public:
  // Builds the world and the two balls in it.
  engine::PhaseResult configure() {
    std::expected<World, framework::Status> world = world_for(2);
    if (!world) {
      return std::unexpected(world.error());
    }
    world_.emplace(*std::move(world));
    balls_ = build_balls(lib::InOut(*world_));
    return engine::Flow::CONTINUE;
  }

  engine::PhaseResult step(const framework::Step& step) {
    scheduler_.step(lib::InOut(*world_), step);
    return any_collision(*world_) ? engine::Flow::STOP : engine::Flow::CONTINUE;
  }

  // The world, once configured.
  const World& world() const {
    CHECK_PRECONDITION(world_.has_value());
    return *world_;
  }
  const Balls& balls() const { return balls_; }

 private:
  std::optional<World> world_;  // Built by configure.
  Scheduler scheduler_;
  Balls balls_{};
};

}  // namespace simon::hello
