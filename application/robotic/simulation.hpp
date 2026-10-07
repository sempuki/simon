// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "application/robotic/simulation_components.hpp"
#include "application/robotic/simulation_systems.hpp"
#include "engine/lifecycle.hpp"

namespace simon::robotic {

// What to simulate: a model, read from MJCF, each of its trees an entity,
// starting at rest at the model's positions unless the start says
// otherwise, by the model's position and velocity addresses.
struct Scenario final {
  std::string model;
  std::vector<std::pair<std::uint32_t, double>> qpos;
  std::vector<std::pair<std::uint32_t, double>> qvel;
  std::vector<double> control;  // By the model's actuators.
  Feedback feedback;            // Overrides the controls, if given.
  bool constrained = true;      // Contacts, limits and dry friction.
  std::optional<articulated::Physics::Solver> solver;  // Overrides the model's.
  std::optional<articulated::Physics::Cone> cone;      // Likewise.
  std::optional<articulated::Physics::Integrator> integrator;
};

class Simulation final {
 public:
  explicit Simulation(Scenario scenario);

  // Reads the model and builds the world, a tree an entity.
  auto configure() -> engine::PhaseResult;

  auto step(const Step& step) -> engine::PhaseResult;

  // Whether configure has read the model and built the world.
  auto ready() const -> bool { return scheduler_ != nullptr; }

  auto world() const -> const World& { return world_; }
  auto mechanics() const -> const Mechanics& { return *mechanics_; }
  auto contacts() const -> const std::vector<articulated::Contact>& {
    return contacts_->contacts;
  }
  auto constraints() const -> const ConstraintSolution& { return *solution_; }

  // Every geom's pose in the world, by the model's geoms, from each tree's
  // last poses.
  auto read_geom_frames() const -> std::vector<articulated::GeomFrame>;

  // The model's positions and velocities, gathered from every tree.
  auto read_qpos() const -> std::vector<double>;
  auto read_qvel() const -> std::vector<double>;

 private:
  Scenario scenario_;
  std::unique_ptr<Mechanics> mechanics_;
  std::unique_ptr<ContactSet> contacts_ = std::make_unique<ContactSet>();
  std::unique_ptr<ConstraintSolution> solution_ =
      std::make_unique<ConstraintSolution>();
  World world_;
  std::unique_ptr<Scheduler> scheduler_;
  std::chrono::nanoseconds pending_{0};  // Driven time not yet stepped.
};

}  // namespace simon::robotic
