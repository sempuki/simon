// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <memory>
#include <string>

#include "application/automotive/simulation_components.hpp"
#include "application/automotive/simulation_systems.hpp"
#include "engine/lifecycle.hpp"
#include "framework/vocabulary.hpp"
#include "scenario/openscenario.hpp"
#include "scenario/storyboard.hpp"

namespace simon::automotive {

// Plays an OpenSCENARIO scenario: reads it and its road network when
// configured, creates a vehicle for each of its entities, and steps its
// storyboard and vehicles until its stop trigger fires.
class ScenarioSimulation final {
 public:
  explicit ScenarioSimulation(std::string path);

  // Reads the scenario and its roads, and builds the world.
  auto configure() -> engine::PhaseResult;

  // One step: the storyboard, then every vehicle. Stops once the
  // storyboard's stop trigger has fired.
  auto step(const framework::Step& step) -> engine::PhaseResult;

  // The world: empty until configured.
  auto world() const -> const ScenarioWorld& { return world_; }

  // The scenario, its roads and its storyboard's player: until configured,
  // none.
  auto scenario() const -> const scenario::Scenario& { return *scenario_; }
  auto roads() const -> const model::RoadNetwork& { return *roads_; }
  auto player() const -> const scenario::StoryboardPlayer& { return *player_; }

 private:
  std::string path_;
  // Shared by the systems; none moves once configured.
  std::unique_ptr<scenario::Scenario> scenario_;
  std::unique_ptr<model::RoadNetwork> roads_;
  std::unique_ptr<scenario::StoryboardPlayer> player_;
  std::unique_ptr<ScenarioContext> context_;
  ScenarioWorld world_;
  std::unique_ptr<ScenarioScheduler> scheduler_;
};

}  // namespace simon::automotive
