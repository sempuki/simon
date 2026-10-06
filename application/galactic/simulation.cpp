// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/galactic/simulation.hpp"

#include "framework/vocabulary.hpp"

namespace simon::galactic {

auto build_bodies(const Scenario& scenario, Out<World> world,
                  Out<std::vector<Entity>> bodies)
    -> std::expected<void, framework::Status> {
  RETURN_IF_UNEXPECTED(World::set_up()
                           .numbered(1)
                           .holding<Body>(scenario.bodies.size())
                           .build(world));
  bodies->clear();
  for (const BodyStart& start : scenario.bodies) {
    RETURN_OR_ASSIGN(Entity body,
                     world->create<Body>()
                         .with(Kinematics{.position = start.position,
                                          .velocity = start.velocity})
                         .with(PointMass{.mass = start.mass})
                         .with(Gravity{})
                         .build());
    bodies->push_back(body);
  }
  world->sync();
  return {};
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(build_bodies(scenario_, Out(world_), Out(bodies_)));
  start_scheduler_.system<model::SumGravity>().softening = scenario_.softening;
  scheduler_.system<model::SumGravity>().softening = scenario_.softening;
  return engine::Flow::CONTINUE;
}

auto Simulation::initialize() -> engine::PhaseResult {
  start_scheduler_.step(Step{}, InOut(world_));
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const Step& step) -> engine::PhaseResult {
  scheduler_.step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

}  // namespace simon::galactic
