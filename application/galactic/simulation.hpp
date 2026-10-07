// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <ratio>
#include <span>
#include <string>
#include <vector>

#include "application/galactic/simulation_components.hpp"
#include "application/galactic/simulation_systems.hpp"
#include "core/units.hpp"
#include "core/vocabulary.hpp"
#include "engine/driver.hpp"
#include "engine/lifecycle.hpp"
#include "model/galaxy.hpp"

namespace simon::galactic {

// Galactic time counts Julian years, 365.25 days of 86,400 s, which in an
// int64 cover 9 x 10^18 years: nanoseconds would cover only 292.
using Year = std::chrono::duration<std::int64_t, std::ratio<31557600>>;
using Step = BasicStep<Year>;
using Timing = engine::BasicTiming<Year>;

using model::BodyStart;

// How a run computes gravity: by summing every pair, exactly, or by Barnes
// and Hut's tree, opening cells by `opening_angle`.
enum class GravityMethod { DIRECT, TREE };

// Bodies a scenario keeps together, such as one galaxy's disk: `count` of
// them from `first`, in its bodies.
struct BodyGroup final {
  std::string name;
  std::size_t first = 0;
  std::size_t count = 0;
};

// Everything a run depends on. The same scenario gives the same run.
struct Scenario final {
  std::vector<BodyStart> bodies;
  std::vector<BodyStart> test_particles;  // Their masses are not used.
  Length softening = 0.0 * meter;
  GravityMethod gravity = GravityMethod::DIRECT;
  double opening_angle = 0.5;
  std::vector<BodyGroup> groups;
};

// Builds in `world` the scenario's bodies and test particles, in its order,
// and appends each to `bodies` or `test_particles`.
auto build_bodies(const Scenario& scenario, Out<World> world,
                  Out<std::vector<Entity>> bodies,
                  Out<std::vector<Entity>> test_particles)
    -> std::expected<void, framework::Status>;

// Toomre and Toomre's restricted encounter: a `victim` mass with their disk
// of test particles, and a bare `companion`, on a parabolic orbit that passes
// within `pericenter`, starting `before` pericenter. Their disk and the orbit
// lie in the x-y plane, both turning counterclockwise: a flat direct passage.
// The center of mass is at the origin and at rest.
struct Encounter final {
  Mass victim = 0.0 * kilogram;
  Mass companion = 0.0 * kilogram;
  Length pericenter = 0.0 * meter;
  Time before = 0.0 * second;
  Length softening = 0.0 * meter;
};

auto make_encounter_scenario(const Encounter& encounter) -> Scenario;

// Two disk galaxies alike, sampled from `seed`, on a parabolic orbit about
// each other, taken as two points of their whole mass, that passes within
// `pericenter`, starting `before` pericenter. The orbit lies in the x-y plane
// and turns counterclockwise; each disk turns counterclockwise about its own
// axis, tilted from z about x by its inclination, so that 0 is a direct
// passage. The groups are each galaxy's disk and halo.
struct Collision final {
  model::DiskGalaxy galaxy;
  Length pericenter = 0.0 * meter;
  Time before = 0.0 * second;
  Angle first_inclination = 0.0 * radian;
  Angle second_inclination = 0.0 * radian;
  Length softening = 0.0 * meter;
  double opening_angle = 0.5;
  std::uint64_t seed = 1;
};

auto make_collision_scenario(const Collision& collision) -> Scenario;

// The disk galaxy galactic's tests, runner and viewer use, of `disk_bodies`
// in its disk and four times as many in its halo: a disk of 5 x 10^10 suns,
// 3 kpc in scale and 0.6 kpc thick, in a halo of 5 x 10^11 suns with a scale
// of 10 kpc, cut off at 100 kpc, with Q = 1.5 at 2.5 scale lengths.
// Hernquist's (1993) proportions of thickness to scale, and a halo that
// outweighs the disk at every radius but its center, keep it in shape.
auto make_standard_galaxy(std::size_t disk_bodies) -> model::DiskGalaxy;

// The collision galactic's runner and viewer show: two standard galaxies of
// `disk_bodies` each, passing within 15 kpc 600 million years after the
// start, the first face on to the orbit and the second tilted by 45 degrees.
// Softened by 0.24 kpc, on the tree at an opening angle of 0.6.
auto make_standard_collision(std::size_t disk_bodies) -> Collision;

// One standard galaxy of `disk_bodies`, alone, softened and on the tree as
// the standard collision is. The groups are its disk and its halo.
auto make_standard_disk_scenario(std::size_t disk_bodies) -> Scenario;

// Toomre and Toomre's flat direct passage in their units: two masses of
// 10^11 suns passing within 25 kpc, a billion years after the start,
// softened by 0.1 kpc.
auto make_toomre_encounter() -> Encounter;

// Computes the center of `group`'s bodies: their center of mass, then three
// times the center of mass of those within `reach` of it, so that bodies
// thrown into tails do not pull it off the galaxy.
auto compute_group_center(const World& world, std::span<const Entity> bodies,
                          const BodyGroup& group, Length reach) -> Position;

// A run's conserved quantities, in SI units, and its energy's two parts.
struct Mechanics final {
  auto energy() const -> double { return kinetic + potential; }
  // 2 T / |W|, which is 1 for a system in equilibrium.
  auto virial_ratio() const -> double { return 2.0 * kinetic / -potential; }

  double kinetic = 0.0;    // Joules.
  double potential = 0.0;  // Joules, softened as the run's gravity is.
  Vector3 momentum = Vector3::Zero();          // kg m/s.
  Vector3 angular_momentum = Vector3::Zero();  // kg m^2/s, about the origin.
  Vector3 center = Vector3::Zero();            // Of mass, meters.
};

// Measures every body in `world`, its potential energy softened by
// `softening`. It sums each pair once, so it costs N^2.
auto measure_mechanics(const World& world, Length softening) -> Mechanics;

// Computes the radii about the center of mass inside which each of
// `fractions` of the mass lies.
auto compute_mass_radii(const World& world, std::span<const double> fractions)
    -> std::vector<Length>;

// The galactic simulation. Any driver can run it.
class Simulation final {
 public:
  using Tick = Year;

  explicit Simulation(Scenario scenario) : scenario_{std::move(scenario)} {}

  // Builds the world and its bodies.
  auto configure() -> engine::PhaseResult;
  // Computes the bodies' gravity, which the first step opens with.
  auto initialize() -> engine::PhaseResult;
  auto step(const Step& step) -> engine::PhaseResult;

  auto scenario() const -> const Scenario& { return scenario_; }
  auto world() const -> const World& { return world_; }
  // The bodies and test particles, in the scenario's order.
  auto bodies() const -> const std::vector<Entity>& { return bodies_; }
  auto test_particles() const -> const std::vector<Entity>& {
    return test_particles_;
  }

 private:
  Scenario scenario_;
  World world_;
  std::vector<Entity> bodies_;
  std::vector<Entity> test_particles_;
  StartScheduler start_scheduler_;
  Scheduler scheduler_;
};

}  // namespace simon::galactic
