// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "application/robotic/simulation_components.hpp"
#include "application/robotic/simulation_systems.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"

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
};

class Simulation final {
 public:
  explicit Simulation(Scenario scenario);

  // Reads the model and builds the world, a tree an entity.
  auto configure() -> engine::PhaseResult;

  auto step(const framework::Step& step) -> engine::PhaseResult;

  auto world() const -> const World& { return world_; }
  auto mechanics() const -> const Mechanics& { return *mechanics_; }

  // The model's positions and velocities, gathered from every tree.
  auto read_qpos() const -> std::vector<double>;
  auto read_qvel() const -> std::vector<double>;

 private:
  Scenario scenario_;
  std::unique_ptr<Mechanics> mechanics_;
  World world_;
  std::unique_ptr<Scheduler> scheduler_;
};

}  // namespace simon::robotic
