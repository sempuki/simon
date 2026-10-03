// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <cmath>
#include <string>
#include <variant>

#include "base/testing.hpp"
#include "format/openscenario.hpp"

// Reading esmini's scenarios (see 3rd_party/esmini/LICENSE): their entities
// from the vehicle catalog, parameters and expressions, and storyboards.
namespace simon::automotive {

namespace {

namespace osc = scenario;

constexpr std::string_view SCENARIOS = "3rd_party/esmini/xosc/";

auto load(std::string_view name) -> osc::Scenario {
  auto scenario =
      format::load_openscenario(std::string{SCENARIOS} + std::string{name});
  if (!scenario) {
    FAIL(scenario.error().message());
  }
  return *scenario;
}

}  // namespace

TEST_CASE("OpenScenario") {
  SECTION("ShouldEvaluateExpressionsGivenParameters") {
    std::vector<osc::Parameter> parameters{{.name = "Speed", .value = "20"}};
    CHECK(format::evaluate_expression("250/3.6", parameters).value() ==
          250.0 / 3.6);
    CHECK(format::evaluate_expression("-($Speed + 4) * 2 % 7", parameters)
              .value() == std::fmod(-48.0, 7.0));
    CHECK(format::evaluate_expression("1.5e2 - $Speed", parameters).value() ==
          130.0);
    CHECK(!format::evaluate_expression("$Missing", parameters).has_value());
    CHECK(!format::evaluate_expression("(1 + 2", parameters).has_value());
  }

  SECTION("ShouldReadEntitiesFromTheCatalog") {
    osc::Scenario scenario = load("cut-in_simple.xosc");
    CHECK(scenario.road_network.ends_with("esmini/xodr/straight_500m.xodr"));
    REQUIRE(scenario.entities.size() == 2);
    const osc::Entity* ego = scenario.find_entity("Ego");
    REQUIRE(ego != nullptr);
    CHECK(ego->vehicle.name == "car_white");
    CHECK(ego->vehicle.dimensions[0] == 5.04);
    CHECK(ego->vehicle.center[0] == 1.4);
    CHECK(ego->vehicle.max_speed == 250.0 / 3.6);
    CHECK(ego->vehicle.max_acceleration == 5.0);
    CHECK(ego->vehicle.wheelbase == 2.98);
    CHECK(scenario.find_entity("OverTaker")->vehicle.name == "car_red");
  }

  SECTION("ShouldReadTheStoryboard") {
    osc::Scenario scenario = load("cut-in_simple.xosc");
    const osc::Storyboard& storyboard = scenario.storyboard;
    REQUIRE(storyboard.init.size() == 2);
    const auto& teleport =
        std::get<osc::TeleportAction>(storyboard.init[0].actions[1]);
    const auto& lane = std::get<osc::LanePosition>(teleport.position);
    CHECK(lane.road == "1");
    CHECK(lane.lane == -1);
    CHECK(lane.s == 50.0);
    const auto& relative = std::get<osc::RelativeRoadPosition>(
        std::get<osc::TeleportAction>(storyboard.init[1].actions[1]).position);
    CHECK(relative.entity == "Ego");
    CHECK(relative.dt == 3.1);
    REQUIRE(relative.orientation.has_value());
    CHECK(!relative.orientation->relative);

    REQUIRE(storyboard.stories.size() == 1);
    const osc::Act& act = storyboard.stories[0].acts[0];
    REQUIRE(act.groups.size() == 1);
    // $owner is the story's parameter.
    CHECK(act.groups[0].actors == std::vector<std::string>{"OverTaker"});
    const osc::Event& cut_in = act.groups[0].maneuvers[0].events[0];
    CHECK(cut_in.name == "CutInEvent");
    const auto& change = std::get<osc::LaneChangeAction>(
        std::get<osc::PrivateAction>(cut_in.actions[0].action));
    CHECK(change.dynamics.shape == osc::DynamicsShape::SINUSOIDAL);
    CHECK(change.dynamics.value == 3.0);
    CHECK(std::get<osc::RelativeTargetLane>(change.target).entity == "Ego");
    const osc::Condition& condition = cut_in.start->groups[0][0];
    CHECK(condition.edge == osc::Condition::Edge::RISING);
    const auto& headway = std::get<osc::TimeHeadwayCondition>(
        std::get<osc::EntityCondition>(condition.condition).condition);
    CHECK(headway.value == 0.4);
    CHECK(headway.freespace);
    CHECK(headway.along_road);
    CHECK(storyboard.stop.has_value());
  }

  SECTION("ShouldReadEveryScenario") {
    for (std::string_view name :
         {"cut-in_simple.xosc", "cut-in.xosc", "lane_change_simple.xosc"}) {
      CAPTURE(name);
      CHECK(!load(name).storyboard.stories.empty());
    }
  }

  SECTION("ShouldRefuseWhatItDoesNotRun") {
    constexpr std::string_view ROUTED = R"(<OpenSCENARIO>
      <RoadNetwork><LogicFile filepath="road.xodr"/></RoadNetwork>
      <Entities/>
      <Storyboard><Init><Actions><Private entityRef="Ego"><PrivateAction>
        <RoutingAction/>
      </PrivateAction></Private></Actions></Init></Storyboard>
    </OpenSCENARIO>)";
    auto scenario = format::parse_openscenario(ROUTED, ".");
    REQUIRE(!scenario.has_value());
    CHECK(scenario.error().message().find("RoutingAction") !=
          std::string::npos);
  }
}

}  // namespace simon::automotive
