// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>

#include "base/core.hpp"
#include "core/archetype.hpp"
#include "core/system.hpp"
#include "core/world.hpp"
#include "model/kinematics.hpp"
#include "model/motion.hpp"

// Two balls under thrust and wind that stop when they collide: the smallest
// complete use of the architecture.
namespace simon::hello {

using core::Entity;
using model::Acceleration;
using model::Control;
using model::Kinematics;
using model::Length;
using model::metre;
using model::metres;
using model::metres_per_second;
using model::metres_per_second_squared;
using model::per_second;
using model::Rate;
using model::Velocity;

//-- Components ----------------------------------------------------------------

struct Thrust {
  Acceleration acceleration = metres_per_second_squared(0.0, 0.0, 0.0);
};

struct Wind {
  Velocity velocity = metres_per_second(0.0, 0.0, 0.0);
};

struct Drag {
  Rate factor = 0.0 * per_second;  // acceleration = factor * (wind - velocity)
};

struct Collider {
  Length radius = 0.0 * metre;
};

// Written by DetectCollisions on the entity that collided.
struct Collision {
  bool hit = false;
  Entity other;
};

using World =
    core::World<Kinematics, Control, Thrust, Wind, Drag, Collider, Collision>;

// A ball must have a position, a size and a collision record; it may be driven.
struct Ball
    : core::Archetype<"ball", core::Requires<Kinematics, Collider, Collision>,
                      core::Allows<Control, Thrust, Wind, Drag>> {};

//-- Systems -------------------------------------------------------------------

// Sums thrust and wind drag into the commanded acceleration.
struct ApplyForces : core::System<Control, const Kinematics, const Thrust,
                                  const Wind, const Drag> {
  void operator()(Entity, Control& control, const Kinematics* kinematics,
                  const Thrust* thrust, const Wind* wind, const Drag* drag,
                  auto&) const {
    control.acceleration = thrust ? thrust->acceleration
                                  : metres_per_second_squared(0.0, 0.0, 0.0);
    if (kinematics && drag) {
      Velocity air = wind ? wind->velocity : metres_per_second(0.0, 0.0, 0.0);
      control.acceleration += drag->factor * (air - kinematics->velocity);
    }
  }
};

// Each collider records its own collision: it writes only its own entity and
// finds the others through a spatial query.
struct DetectCollisions
    : core::System<Collision, const Collider, const Kinematics> {
  using Lookups = core::Stores<Kinematics, Collider>;
  using After = core::Systems<model::Integrate>;

  void prepare(auto& context) {
    largest_radius = 0.0 * metre;
    for (const Collider& collider : store<Collider>(context).values()) {
      largest_radius = std::max(largest_radius, collider.radius);
    }
  }

  void operator()(Entity self, Collision& collision, const Collider* collider,
                  const Kinematics* kinematics, auto& context) const {
    if (!collider || !kinematics) return;
    context.within(*kinematics, collider->radius + largest_radius,
                   [&](Entity other, const Kinematics& other_kinematics) {
                     const Collider* other_collider =
                         lookup<Collider>(context, other);
                     if (other == self || !other_collider) return;
                     if (distance(*kinematics, other_kinematics) <=
                         collider->radius + other_collider->radius) {
                       collision = Collision{.hit = true, .other = other};
                     }
                   });
  }

  Length largest_radius = 0.0 * metre;
};

using Schedule = core::Systems<ApplyForces, model::Motion, DetectCollisions>;
using Scheduler = core::Scheduler<World, Schedule>;

//-- Scenario ------------------------------------------------------------------

struct Balls {
  Entity red;
  Entity blue;
};

// Builds the two balls. One metre is drawn as one screen pixel.
inline Balls build_balls(lib::InOut<World> world) {
  Balls balls{
      .red = *world->create<Ball>("red")
                  .with(Kinematics{
                      .position = metres(360.0, 100.0, 0.0),
                      .velocity = metres_per_second(10.0, -10.0, 0.0)})
                  .with(Control{})
                  .with(Thrust{.acceleration =
                                   metres_per_second_squared(0.0, 9.8, 0.0)})
                  .with(Wind{.velocity = metres_per_second(-1.0, 0.0, 0.0)})
                  .with(Drag{.factor = 0.4 * per_second})
                  .with(Collider{.radius = 10.0 * metre})
                  .with(Collision{})
                  .build(),
      .blue = *world->create<Ball>("blue")
                   .with(Kinematics{.position = metres(360.0, 600.0, 0.0)})
                   .with(Collider{.radius = 10.0 * metre})
                   .with(Collision{})
                   .build(),
  };
  world->sync();
  return balls;
}

inline bool any_collision(const World& world) {
  for (const Collision& collision : world.store<Collision>().values()) {
    if (collision.hit) return true;
  }
  return false;
}

}  // namespace simon::hello
