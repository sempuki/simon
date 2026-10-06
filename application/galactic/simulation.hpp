// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <ratio>
#include <span>
#include <vector>

#include "application/galactic/simulation_components.hpp"
#include "application/galactic/simulation_systems.hpp"
#include "engine/driver.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"
#include "model/galaxy.hpp"
#include "model/units.hpp"

namespace simon::galactic {

// Galactic time counts Julian years, 365.25 days of 86,400 s, which in an
// int64 cover 9 x 10^18 years: nanoseconds would cover only 292.
using Year = std::chrono::duration<std::int64_t, std::ratio<31557600>>;
using Step = framework::BasicStep<Year>;
using Timing = engine::BasicTiming<Year>;

using model::BodyStart;

// How a run computes gravity: by summing every pair, exactly, or by Barnes
// and Hut's tree, opening cells by `opening_angle`.
enum class GravityMethod { DIRECT, TREE };

// Everything a run depends on. The same scenario gives the same run.
struct Scenario final {
  std::vector<BodyStart> bodies;
  std::vector<BodyStart> test_particles;  // Their masses are not used.
  model::Length softening = 0.0 * model::meter;
  GravityMethod gravity = GravityMethod::DIRECT;
  double opening_angle = 0.5;
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
  model::Mass victim = 0.0 * model::kilogram;
  model::Mass companion = 0.0 * model::kilogram;
  model::Length pericenter = 0.0 * model::meter;
  model::Time before = 0.0 * model::second;
  model::Length softening = 0.0 * model::meter;
};

auto make_encounter_scenario(const Encounter& encounter) -> Scenario;

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
auto measure_mechanics(const World& world, model::Length softening)
    -> Mechanics;

// Computes the radii about the center of mass inside which each of
// `fractions` of the mass lies.
auto compute_mass_radii(const World& world, std::span<const double> fractions)
    -> std::vector<model::Length>;

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
