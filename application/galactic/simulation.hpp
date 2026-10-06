// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstdint>
#include <expected>
#include <ratio>
#include <vector>

#include "application/galactic/simulation_components.hpp"
#include "application/galactic/simulation_systems.hpp"
#include "engine/driver.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"
#include "model/units.hpp"

namespace simon::galactic {

// Galactic time counts Julian years, 365.25 days of 86,400 s, which in an
// int64 cover 9 x 10^18 years: nanoseconds would cover only 292.
using Year = std::chrono::duration<std::int64_t, std::ratio<31557600>>;
using Step = framework::BasicStep<Year>;
using Timing = engine::BasicTiming<Year>;

// A body as a run starts it.
struct BodyStart final {
  model::Position position = model::meters(0.0, 0.0, 0.0);
  model::Velocity velocity = model::meters_per_second(0.0, 0.0, 0.0);
  model::Mass mass = 0.0 * model::kilogram;
};

// Everything a run depends on. The same scenario gives the same run.
struct Scenario final {
  std::vector<BodyStart> bodies;
  model::Length softening = 0.0 * model::meter;
};

// Builds in `world` the scenario's bodies, in its order, and appends each to
// `bodies`.
auto build_bodies(const Scenario& scenario, Out<World> world,
                  Out<std::vector<Entity>> bodies)
    -> std::expected<void, framework::Status>;

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
  // The bodies, in the scenario's order.
  auto bodies() const -> const std::vector<Entity>& { return bodies_; }

 private:
  Scenario scenario_;
  World world_;
  std::vector<Entity> bodies_;
  StartScheduler start_scheduler_;
  Scheduler scheduler_;
};

}  // namespace simon::galactic
