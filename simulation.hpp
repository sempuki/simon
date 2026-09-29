// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "base/time.hpp"
#include "component/controls.hpp"
#include "component/environment.hpp"
#include "component/movement.hpp"
#include "component/physical.hpp"
#include "framework/component_system.hpp"
#include "framework/entity.hpp"
#include "framework/event_queue.hpp"

namespace simon {

using framework::Entity;

struct Collision final {
  component::Physical* a = nullptr;
  component::Physical* b = nullptr;
};

inline bool has_collision(component::Physical* a, component::Physical* b) {
  auto distance = static_cast<Vec3>(a->movement->position - b->movement->position).norm();
  return distance <= (a->radius + b->radius);
}

struct DetectSphericalCollision : public framework::ComputeBase<component::Physical> {
  void prepare(component::Physical* current) { others.push_back(current); }
  void operator()(component::Physical* current,
                  TimePoint time,
                  Duration /*step*/,
                  framework::EventQueue* events) {
    // Publish each colliding pair once, from the lower-addressed side.
    for (auto* other : others) {
      if (std::less<>{}(current, other) && has_collision(current, other)) {
        events->publish<Collision>(time, current, other);
      }
    }
  }
  void resolve(component::Physical* /*current*/) { others.clear(); }
  std::vector<component::Physical*> others;
};

inline auto compute_acceleration(component::Movement* m) {
  return m->controls->acceleration +
         (m->physical->wind_resistance_factor * (m->environment->wind - m->velocity));
}

struct ForwardEulerMovement : public framework::ComputeBase<component::Movement> {
  void operator()(component::Movement* movement,
                  TimePoint /*time*/,
                  Duration step,
                  framework::EventQueue* /*events*/) {
    auto prev = *movement;
    auto& next = *movement;
    auto acceleration = compute_acceleration(movement);

    next.velocity = prev.velocity + acceleration * step.count();
    next.position = prev.position + prev.velocity * step.count();
  }
};

struct TrapezoidMovement : public framework::ComputeBase<component::Movement> {
  void operator()(component::Movement* movement,
                  TimePoint /*time*/,
                  Duration step,
                  framework::EventQueue* /*events*/) {
    auto prev = *movement;
    auto& next = *movement;
    auto acceleration = compute_acceleration(movement);

    next.velocity = prev.velocity + acceleration * step.count();
    next.position = prev.position + (prev.velocity + next.velocity) * step.count() * 0.5;
  }
};

struct RungeKutta2Movement : public framework::ComputeBase<component::Movement> {
  void operator()(component::Movement* movement,
                  TimePoint /*time*/,
                  Duration step,
                  framework::EventQueue* /*events*/) {
    auto prev = *movement;
    auto& next = *movement;
    auto acceleration = compute_acceleration(movement);

    // Midpoint method: advance position by the velocity at half a step.
    auto mid_velocity = prev.velocity + acceleration * step.count() * 0.5;

    next.velocity = prev.velocity + acceleration * step.count();
    next.position = prev.position + mid_velocity * step.count();
  }
};

struct ComputeMovement : public RungeKutta2Movement {};

class Simulation final {
 public:
  static constexpr Duration STEP_SIZE{0.1};
  static constexpr double SUB_STEP_FACTOR{0.1};

  Entity* create() {
    entities_.emplace_back(std::make_unique<Entity>());

    auto* entity = entities_.back().get();
    auto* controls = controls_.attach(entity);
    auto* environment = environment_.attach(entity);
    auto* physical = physical_.attach(entity);
    auto* movement = movement_.attach(entity);

    movement->environment = environment;
    movement->physical = physical;
    movement->controls = controls;
    physical->movement = movement;

    return entity;
  }

  void operator()(TimePoint time, Duration step) {
    static auto do_substep =
      [](auto& system, TimePoint time, Duration step, framework::EventQueue* events) {
        // Count substeps instead of accumulating time, which drifts in floating point.
        const auto count = static_cast<int>(std::lround(1.0 / SUB_STEP_FACTOR));
        const Duration substep = step / count;
        for (int i = 0; i < count; ++i) {
          system(time + substep * i, substep, events);
        }
      };

    events.process_until(time);
    do_substep(environment_, time, step, &events);
    do_substep(physical_, time, step, &events);
    do_substep(controls_, time, step, &events);
    do_substep(movement_, time, step, &events);
  }

  framework::EventQueue events;

 private:
  framework::ComponentSystem<component::Controls, framework::ComputeNone> controls_;
  framework::ComponentSystem<component::Environment, framework::ComputeNone> environment_;
  framework::ComponentSystem<component::Physical, DetectSphericalCollision> physical_;
  framework::ComponentSystem<component::Movement, ComputeMovement> movement_;
  std::vector<std::unique_ptr<Entity>> entities_;
};

}  // namespace simon
