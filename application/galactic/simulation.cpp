// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/galactic/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <utility>

#include "core/argument.hpp"
#include "core/math.hpp"
#include "core/random.hpp"

namespace simon::galactic {

auto build_bodies(const Scenario& scenario, Out<World> world,
                  Out<std::vector<Entity>> bodies,
                  Out<std::vector<Entity>> test_particles)
    -> std::expected<void, framework::Status> {
  RETURN_IF_UNEXPECTED(
      World::set_up()
          .numbered(1)
          .holding<Body>(scenario.bodies.size())
          .holding<TestParticle>(scenario.test_particles.size())
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
  test_particles->clear();
  for (const BodyStart& start : scenario.test_particles) {
    RETURN_OR_ASSIGN(Entity particle,
                     world->create<TestParticle>()
                         .with(Kinematics{.position = start.position,
                                          .velocity = start.velocity})
                         .with(Gravity{})
                         .build());
    test_particles->push_back(particle);
  }
  world->sync();
  return {};
}

auto make_encounter_scenario(const Encounter& encounter) -> Scenario {
  gravity::Separation separation = gravity::compute_parabolic_separation(
      gravity::ParabolicOrbit{.first = encounter.victim,
                              .second = encounter.companion,
                              .pericenter = encounter.pericenter},
      -encounter.before);
  double total =
      (encounter.victim + encounter.companion).numerical_value_in(kilogram);
  double victim_share =
      -encounter.companion.numerical_value_in(kilogram) / total;
  double companion_share =
      encounter.victim.numerical_value_in(kilogram) / total;

  BodyStart victim{.position = victim_share * separation.position,
                   .velocity = victim_share * separation.velocity,
                   .mass = encounter.victim};
  BodyStart companion{.position = companion_share * separation.position,
                      .velocity = companion_share * separation.velocity,
                      .mass = encounter.companion};
  Scenario scenario{.bodies = {victim, companion},
                    .softening = encounter.softening};
  gravity::append_ring_disk(
      gravity::make_toomre_disk(encounter.pericenter, encounter.softening),
      victim, InOut(scenario.test_particles));
  return scenario;
}

auto make_collision_scenario(const Collision& collision) -> Scenario {
  const gravity::DiskGalaxy& galaxy = collision.galaxy;
  double cut =
      number_of(galaxy.halo_cutoff / (galaxy.halo_cutoff + galaxy.halo_scale));
  Mass mass = galaxy.disk_mass + galaxy.halo_mass * cut * cut;
  gravity::Separation separation = gravity::compute_parabolic_separation(
      gravity::ParabolicOrbit{
          .first = mass, .second = mass, .pericenter = collision.pericenter},
      -collision.before);

  Scenario scenario{.softening = collision.softening,
                    .gravity = GravityMethod::TREE,
                    .opening_angle = collision.opening_angle};
  Random random{collision.seed};
  auto append_galaxy = [&](std::string_view name, double share,
                           Angle inclination) {
    std::size_t first = scenario.bodies.size();
    gravity::append_disk_galaxy(galaxy, InOut(random), InOut(scenario.bodies));
    Matrix3 tilt = Eigen::AngleAxisd(radians(inclination), Vector3::UnitX())
                       .toRotationMatrix();
    gravity::place_bodies(first, tilt, share * separation.position,
                          share * separation.velocity, InOut(scenario.bodies));
    scenario.groups.push_back(BodyGroup{.name = std::format("{} disk", name),
                                        .first = first,
                                        .count = galaxy.disk_bodies});
    scenario.groups.push_back(BodyGroup{.name = std::format("{} halo", name),
                                        .first = first + galaxy.disk_bodies,
                                        .count = galaxy.halo_bodies});
  };
  append_galaxy("first", -0.5, collision.first_inclination);
  append_galaxy("second", 0.5, collision.second_inclination);
  return scenario;
}

auto make_standard_galaxy(std::size_t disk_bodies) -> gravity::DiskGalaxy {
  using gravity::KILOPARSEC;
  using gravity::SOLAR_MASS;
  return gravity::DiskGalaxy{.disk_mass = 5e10 * SOLAR_MASS,
                             .disk_scale = 3.0 * KILOPARSEC,
                             .disk_thickness = 0.6 * KILOPARSEC,
                             .halo_mass = 5e11 * SOLAR_MASS,
                             .halo_scale = 10.0 * KILOPARSEC,
                             .halo_cutoff = 100.0 * KILOPARSEC,
                             .stability = 1.5,
                             .stability_radius = 7.5 * KILOPARSEC,
                             .disk_bodies = disk_bodies,
                             .halo_bodies = 4 * disk_bodies};
}

auto make_standard_collision(std::size_t disk_bodies) -> Collision {
  return Collision{.galaxy = make_standard_galaxy(disk_bodies),
                   .pericenter = 15.0 * gravity::KILOPARSEC,
                   .before = 6e8 * gravity::JULIAN_YEAR,
                   .first_inclination = 0.0 * radian,
                   .second_inclination = std::numbers::pi / 4.0 * radian,
                   .softening = 0.24 * gravity::KILOPARSEC,
                   .opening_angle = 0.6};
}

auto make_standard_disk_scenario(std::size_t disk_bodies) -> Scenario {
  gravity::DiskGalaxy galaxy = make_standard_galaxy(disk_bodies);
  Scenario scenario{.softening = 0.24 * gravity::KILOPARSEC,
                    .gravity = GravityMethod::TREE,
                    .opening_angle = 0.6};
  Random random{3};
  gravity::append_disk_galaxy(galaxy, InOut(random), InOut(scenario.bodies));
  scenario.groups = {
      BodyGroup{.name = "disk", .first = 0, .count = galaxy.disk_bodies},
      BodyGroup{.name = "halo",
                .first = galaxy.disk_bodies,
                .count = galaxy.halo_bodies}};
  return scenario;
}

auto make_toomre_encounter() -> Encounter {
  return Encounter{.victim = 1e11 * gravity::SOLAR_MASS,
                   .companion = 1e11 * gravity::SOLAR_MASS,
                   .pericenter = 25.0 * gravity::KILOPARSEC,
                   .before = 1e9 * gravity::JULIAN_YEAR,
                   .softening = 0.1 * gravity::KILOPARSEC};
}

auto compute_group_center(const World& world, std::span<const Entity> bodies,
                          const BodyGroup& group, Length reach) -> Position {
  const auto& kinematics = world.store_of<Kinematics>();
  const auto& masses = world.store_of<PointMass>();
  double limit2 = std::pow(reach.numerical_value_in(meter), 2.0);
  Vector3 center = Vector3::Zero();
  for (int pass = 0; pass < 4; ++pass) {
    Vector3 moment = Vector3::Zero();
    double mass = 0.0;
    for (std::size_t i = group.first; i < group.first + group.count; ++i) {
      Vector3 p = kinematics.component_of(bodies[i])
                      .position.numerical_value_in(meter)
                      .eigen();
      if (pass > 0 && (p - center).squaredNorm() > limit2) continue;
      double m =
          masses.component_of(bodies[i]).mass.numerical_value_in(kilogram);
      moment += m * p;
      mass += m;
    }
    if (mass > 0.0) center = moment / mass;
  }
  return QuantityVector{center} * meter;
}

namespace {

// Every body's position, velocity and mass, in store order. SI.
struct Sample final {
  std::vector<gravity::Source> sources;
  std::vector<Vector3> velocities;
};

auto collect_sample(const World& world) -> Sample {
  Sample sample;
  world.store_of<PointMass>().for_each(
      [&](Entity entity, const PointMass& point) {
        const Kinematics& kinematics =
            world.store_of<Kinematics>().component_of(entity);
        sample.sources.push_back(gravity::Source{
            .position = kinematics.position.numerical_value_in(meter).eigen(),
            .mass = point.mass.numerical_value_in(kilogram),
            .id = entity.index});
        sample.velocities.push_back(
            kinematics.velocity.numerical_value_in(meter_per_second).eigen());
      });
  return sample;
}

}  // namespace

auto measure_mechanics(const World& world, Length softening) -> Mechanics {
  Sample sample = collect_sample(world);
  Mechanics mechanics;
  double mass = 0.0;
  for (std::size_t i = 0; i < sample.sources.size(); ++i) {
    const gravity::Source& body = sample.sources[i];
    const Vector3& velocity = sample.velocities[i];
    mechanics.kinetic += 0.5 * body.mass * velocity.squaredNorm();
    mechanics.momentum += body.mass * velocity;
    mechanics.angular_momentum += body.mass * body.position.cross(velocity);
    mechanics.center += body.mass * body.position;
    mass += body.mass;
  }
  mechanics.center /= mass;
  mechanics.potential = gravity::compute_potential_energy(
      sample.sources, softening.numerical_value_in(meter));
  return mechanics;
}

auto compute_mass_radii(const World& world, std::span<const double> fractions)
    -> std::vector<Length> {
  Sample sample = collect_sample(world);
  Vector3 center = Vector3::Zero();
  double mass = 0.0;
  for (const gravity::Source& body : sample.sources) {
    center += body.mass * body.position;
    mass += body.mass;
  }
  center /= mass;

  std::vector<std::pair<double, double>> radii;  // Radius, mass.
  for (const gravity::Source& body : sample.sources) {
    radii.emplace_back((body.position - center).norm(), body.mass);
  }
  std::ranges::sort(radii);

  std::vector<Length> found;
  for (double fraction : fractions) {
    double inside = 0.0;
    double radius = 0.0;
    for (const auto& [r, m] : radii) {
      inside += m;
      radius = r;
      if (inside >= fraction * mass) break;
    }
    found.push_back(radius * meter);
  }
  return found;
}

auto Simulation::configure() -> engine::PhaseResult {
  RETURN_IF_UNEXPECTED(
      build_bodies(scenario_, Out(world_), Out(bodies_), Out(test_particles_)));
  auto set_gravity = [&](auto& scheduler) {
    auto& direct = scheduler.template system<SumGravity>();
    auto& tree = scheduler.template system<TreeGravity>();
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
