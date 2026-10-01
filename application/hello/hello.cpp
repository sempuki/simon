// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "application/hello/hello.hpp"

namespace simon::hello {

auto build_balls(lib::InOut<World> world) -> Balls {
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

auto build_world(std::size_t balls, lib::Out<World> world)
    -> std::expected<void, framework::Status> {
  return World::set_up().numbered(1).holding<Ball>(balls).build(world);
}

auto any_collision(const World& world) -> bool {
  bool hit = false;
  world.store_of<Collision>().for_each(
      [&](Entity, const Collision& collision) { hit = hit || collision.hit; });
  return hit;
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(build_world(2, lib::Out(world_)));
  balls_ = build_balls(lib::InOut(world_));
  return engine::Flow::CONTINUE;
}

auto Simulation::step(const framework::Step& step) -> engine::PhaseResult {
  scheduler_.step(step, lib::InOut(world_));
  return any_collision(world_) ? engine::Flow::STOP : engine::Flow::CONTINUE;
}

}  // namespace simon::hello
