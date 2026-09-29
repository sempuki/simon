// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "simulation.hpp"

#include "base/testing.hpp"

namespace simon {

namespace {

struct Body {
  component::Controls controls;
  component::Environment environment;
  component::Physical physical;
  component::Movement movement;

  Body() {
    movement.controls = &controls;
    movement.environment = &environment;
    movement.physical = &physical;
    physical.movement = &movement;
    physical.wind_resistance_factor = 0.0;
  }
};

}  // namespace

TEST_CASE("RungeKutta2Movement") {
  Body body;
  framework::EventQueue events;
  RungeKutta2Movement integrate;
  const Duration step{0.5};

  SECTION("ShouldMoveByVelocityTimesStepGivenZeroAcceleration") {
    body.movement.position = {1.0, 2.0, 3.0};
    body.movement.velocity = {4.0, -2.0, 0.0};

    integrate(&body.movement, TimePoint{}, step, &events);

    CHECK(body.movement.position.isApprox(Vec3{3.0, 1.0, 3.0}));
    CHECK(body.movement.velocity.isApprox(Vec3{4.0, -2.0, 0.0}));
  }

  SECTION("ShouldMatchKinematicsGivenConstantAcceleration") {
    body.movement.velocity = {1.0, 0.0, 0.0};
    body.controls.acceleration = {0.0, 2.0, 0.0};

    integrate(&body.movement, TimePoint{}, step, &events);

    // p = v*t + a*t^2/2, v' = v + a*t
    CHECK(body.movement.position.isApprox(Vec3{0.5, 0.25, 0.0}));
    CHECK(body.movement.velocity.isApprox(Vec3{1.0, 1.0, 0.0}));
  }
}

TEST_CASE("DetectSphericalCollision") {
  Body a;
  Body b;
  Body c;
  a.physical.radius = b.physical.radius = c.physical.radius = 1.0;
  a.movement.position = {0.0, 0.0, 0.0};
  b.movement.position = {1.5, 0.0, 0.0};
  c.movement.position = {100.0, 0.0, 0.0};

  framework::EventQueue events;
  int collisions = 0;
  events.subscribe<Collision>([&](TimePoint /*time*/, const Collision& collision) {
    CHECK(((collision.a == &a.physical && collision.b == &b.physical) ||
           (collision.a == &b.physical && collision.b == &a.physical)));
    collisions++;
  });

  SECTION("ShouldPublishOncePerPairGivenOverlappingSpheres") {
    DetectSphericalCollision detect;
    for (auto* body : {&a, &b, &c}) {
      detect.prepare(&body->physical);
    }
    for (auto* body : {&a, &b, &c}) {
      detect(&body->physical, TimePoint{}, Duration{0.1}, &events);
    }
    for (auto* body : {&a, &b, &c}) {
      detect.resolve(&body->physical);
    }
    events.process_until(TimePoint{});

    CHECK(collisions == 1);
  }
}

TEST_CASE("Simulation") {
  Simulation simulation;
  auto* entity = simulation.create();
  auto* movement = entity->component<component::Movement>();
  entity->component<component::Physical>()->wind_resistance_factor = 0.0;

  SECTION("ShouldAdvanceOneFullStepGivenOneCall") {
    movement->velocity = {1.0, 0.0, 0.0};

    simulation(TimePoint{}, Simulation::STEP_SIZE);

    CHECK(movement->position.isApprox(Vec3{Simulation::STEP_SIZE.count(), 0.0, 0.0}));
  }

  SECTION("ShouldAdvanceOneFullStepGivenLateStartTime") {
    // Accumulating substeps from t=5 used to run one substep too many.
    movement->velocity = {1.0, 0.0, 0.0};

    simulation(TimePoint{Duration{5.0}}, Simulation::STEP_SIZE);

    CHECK(movement->position.isApprox(Vec3{Simulation::STEP_SIZE.count(), 0.0, 0.0}));
  }

  SECTION("ShouldStayAtRestGivenNoForces") {
    simulation(TimePoint{}, Simulation::STEP_SIZE);

    CHECK(movement->position.isZero());
    CHECK(movement->velocity.isZero());
  }
}

}  // namespace simon
