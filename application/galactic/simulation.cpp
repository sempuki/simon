// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/galactic/simulation.hpp"

#include <algorithm>
#include <utility>

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

namespace {

// Every body's position, velocity and mass, in store order. SI.
struct Sample final {
  std::vector<model::GravitySource> sources;
  std::vector<Vector3> velocities;
};

auto collect_sample(const World& world) -> Sample {
  Sample sample;
  world.store_of<PointMass>().for_each(
      [&](Entity entity, const PointMass& point) {
        const Kinematics& kinematics =
            world.store_of<Kinematics>().component_of(entity);
        sample.sources.push_back(model::GravitySource{
            .position =
                kinematics.position.numerical_value_in(model::meter).eigen(),
            .mass = point.mass.numerical_value_in(model::kilogram),
            .entity = entity});
        sample.velocities.push_back(
            kinematics.velocity.numerical_value_in(model::meter_per_second)
                .eigen());
      });
  return sample;
}

}  // namespace

auto measure_mechanics(const World& world, model::Length softening)
    -> Mechanics {
  Sample sample = collect_sample(world);
  Mechanics mechanics;
  double mass = 0.0;
  for (std::size_t i = 0; i < sample.sources.size(); ++i) {
    const model::GravitySource& body = sample.sources[i];
    const Vector3& velocity = sample.velocities[i];
    mechanics.kinetic += 0.5 * body.mass * velocity.squaredNorm();
    mechanics.momentum += body.mass * velocity;
    mechanics.angular_momentum += body.mass * body.position.cross(velocity);
    mechanics.center += body.mass * body.position;
    mass += body.mass;
  }
  mechanics.center /= mass;
  mechanics.potential = model::compute_potential_energy(
      sample.sources, softening.numerical_value_in(model::meter));
  return mechanics;
}

auto compute_mass_radii(const World& world, std::span<const double> fractions)
    -> std::vector<model::Length> {
  Sample sample = collect_sample(world);
  Vector3 center = Vector3::Zero();
  double mass = 0.0;
  for (const model::GravitySource& body : sample.sources) {
    center += body.mass * body.position;
    mass += body.mass;
  }
  center /= mass;

  std::vector<std::pair<double, double>> radii;  // Radius, mass.
  for (const model::GravitySource& body : sample.sources) {
    radii.emplace_back((body.position - center).norm(), body.mass);
  }
  std::ranges::sort(radii);

  std::vector<model::Length> found;
  for (double fraction : fractions) {
    double inside = 0.0;
    double radius = 0.0;
    for (const auto& [r, m] : radii) {
      inside += m;
      radius = r;
      if (inside >= fraction * mass) break;
    }
    found.push_back(radius * model::meter);
  }
  return found;
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(build_bodies(scenario_, Out(world_), Out(bodies_)));
  auto set_gravity = [&](auto& scheduler) {
    auto& direct = scheduler.template system<model::SumGravity>();
    auto& tree = scheduler.template system<model::TreeGravity>();
    direct.enabled = scenario_.gravity == GravityMethod::DIRECT;
    direct.softening = scenario_.softening;
    tree.enabled = scenario_.gravity == GravityMethod::TREE;
    tree.softening = scenario_.softening;
    tree.opening_angle = scenario_.opening_angle;
  };
  set_gravity(start_scheduler_);
  set_gravity(scheduler_);
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
