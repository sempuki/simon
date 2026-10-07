// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/scenario_simulation.hpp"

#include <utility>

#include "format/opendrive.hpp"
#include "format/openscenario.hpp"

namespace simon::automotive {

ScenarioSimulation::ScenarioSimulation(
    std::string path, std::vector<scenario::ParameterAssignment> assignments)
    : path_{std::move(path)}, assignments_{std::move(assignments)} {}

auto ScenarioSimulation::configure() -> engine::PhaseResult {
  RETURN_OR_ASSIGN(scenario::Scenario scenario,
                   format::load_openscenario(path_, assignments_));
  scenario_ = std::make_unique<scenario::Scenario>(std::move(scenario));
  RETURN_OR_ASSIGN(road::Map roads,
                   format::load_opendrive(scenario_->road_network));
  roads_ = std::make_unique<road::Map>(std::move(roads));
  lanes_ = std::make_unique<road::LaneGraph>(road::build_lane_graph(*roads_));
  player_ = std::make_unique<scenario::StoryboardPlayer>(*scenario_, *roads_);
  context_ = std::make_unique<ScenarioContext>(
      ScenarioContext{.roads = roads_.get(),
                      .lanes = lanes_.get(),
                      .scenario = scenario_.get(),
                      .player = player_.get()});
  scheduler_ = std::make_unique<ScenarioScheduler>(
      ScenarioSchedule{RunStoryboard{*context_}, ControlSpeed{*context_},
                       MoveOnRoad{*context_}, PlaceOnRoad{*context_}});

  std::size_t count = scenario_->entities.size();
  RETURN_IF_UNEXPECTED(ScenarioWorld::set_up()
                           .numbered(1)
                           .holding<archetype::ScenarioEntity>(count)
                           .build(Out(world_)));
  auto transaction = world_.transaction();
  for (std::size_t i = 0; i < count; ++i) {
    RETURN_IF_UNEXPECTED(world_.create<archetype::ScenarioEntity>()
                             .with(RoadPose{})
                             .with(ScenarioActor{.entity = i})
                             .with(ScenarioOrders{})
                             .with(ScenarioSpeed{})
                             .with(ScenarioMotion{})
                             .build());
  }
  transaction.commit();
  world_.sync();
  return engine::Flow::CONTINUE;
}

auto ScenarioSimulation::step(const Step& step) -> engine::PhaseResult {
  if (context_->started && !player_->running()) {
    return engine::Flow::STOP;
  }
  scheduler_->step(step, InOut(world_));
  return engine::Flow::CONTINUE;
}

}  // namespace simon::automotive
