// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <string>
#include <vector>

#include "base/testing.hpp"
#include "format/openscenario.hpp"
#include "model/road/road.hpp"
#include "scenario/storyboard.hpp"

// OpenSCENARIO's traffic signal controllers, which esmini does not run,
// against the standard: phases in turn from a delay after their reference's,
// a controller action jumping to a phase, a signal action holding until the
// next phase, and a condition on the phase.
namespace simon::automotive {

namespace {

namespace osc = scenario;

// Two controllers, the second 5 s behind the first; at 40 s the first jumps
// to its stop phase, and at 42 s signal 1 is set by hand. An event starts
// the first time the first controller enters its stop phase.
constexpr std::string_view CONTROLLED = R"(<OpenSCENARIO>
  <RoadNetwork>
    <LogicFile filepath="road.xodr"/>
    <TrafficSignals>
      <TrafficSignalController name="main">
        <Phase name="go" duration="20">
          <TrafficSignalState trafficSignalId="1" state="off;off;on"/>
        </Phase>
        <Phase name="stop" duration="10">
          <TrafficSignalState trafficSignalId="1" state="on;off;off"/>
        </Phase>
      </TrafficSignalController>
      <TrafficSignalController name="side" delay="5" reference="main">
        <Phase name="a" duration="15"/>
        <Phase name="b" duration="15"/>
      </TrafficSignalController>
    </TrafficSignals>
  </RoadNetwork>
  <Entities/>
  <Storyboard>
    <Init><Actions/></Init>
    <Story name="story"><Act name="act">
      <ManeuverGroup name="group" maximumExecutionCount="1">
        <Actors selectTriggeringEntities="false"/>
        <Maneuver name="maneuver">
          <Event name="jump" priority="parallel">
            <Action name="to_stop"><GlobalAction><InfrastructureAction>
              <TrafficSignalAction>
                <TrafficSignalControllerAction
                    trafficSignalControllerRef="main" phase="stop"/>
              </TrafficSignalAction>
            </InfrastructureAction></GlobalAction></Action>
            <StartTrigger><ConditionGroup>
              <Condition name="at40" delay="0" conditionEdge="none">
                <ByValueCondition>
                  <SimulationTimeCondition value="40" rule="greaterOrEqual"/>
                </ByValueCondition>
              </Condition>
            </ConditionGroup></StartTrigger>
          </Event>
          <Event name="by_hand" priority="parallel">
            <Action name="amber"><GlobalAction><InfrastructureAction>
              <TrafficSignalAction>
                <TrafficSignalStateAction name="1" state="off;on;off"/>
              </TrafficSignalAction>
            </InfrastructureAction></GlobalAction></Action>
            <StartTrigger><ConditionGroup>
              <Condition name="at42" delay="0" conditionEdge="none">
                <ByValueCondition>
                  <SimulationTimeCondition value="42" rule="greaterOrEqual"/>
                </ByValueCondition>
              </Condition>
            </ConditionGroup></StartTrigger>
          </Event>
          <Event name="stopped" priority="parallel">
            <StartTrigger><ConditionGroup>
              <Condition name="in_stop" delay="0" conditionEdge="rising">
                <ByValueCondition>
                  <TrafficSignalControllerCondition
                      trafficSignalControllerRef="main" phase="stop"/>
                </ByValueCondition>
              </Condition>
            </ConditionGroup></StartTrigger>
          </Event>
        </Maneuver>
      </ManeuverGroup>
      <StartTrigger><ConditionGroup>
        <Condition name="go" delay="0" conditionEdge="none">
          <ByValueCondition>
            <SimulationTimeCondition value="0" rule="greaterOrEqual"/>
          </ByValueCondition>
        </Condition>
      </ConditionGroup></StartTrigger>
    </Act></Story>
  </Storyboard>
</OpenSCENARIO>)";

}  // namespace

TEST_CASE("Storyboard") {
  SECTION("ShouldRunTrafficSignalControllersAsTheStandardSays") {
    auto scenario = format::parse_openscenario(CONTROLLED, ".");
    REQUIRE(scenario.has_value());
    road::Map roads;
    osc::StoryboardPlayer player{*scenario, roads};
    struct Expected final {
      double time = 0.0;
      std::string main;
      std::string side;
      std::string signal;
    };
    // The side controller's cycle began 5 s after the main one's, so at the
    // start it is 25 s into its 30 s cycle.
    std::vector<Expected> expected{
        {0.0, "go", "b", "off;off;on"},    {4.5, "go", "b", "off;off;on"},
        {5.0, "go", "a", "off;off;on"},    {19.5, "go", "a", "off;off;on"},
        {20.0, "stop", "b", "on;off;off"}, {30.0, "go", "b", "off;off;on"},
        {39.5, "go", "a", "off;off;on"},   {40.0, "stop", "a", "on;off;off"},
        {42.0, "stop", "a", "off;on;off"}, {49.5, "stop", "a", "off;on;off"},
        {50.0, "go", "b", "off;off;on"},   {70.0, "stop", "a", "on;off;off"}};
    std::size_t next = 0;
    for (int k = 0; k <= 140; ++k) {
      double time = 0.5 * k;
      if (k == 0) {
        player.start(time);
      }
      player.step(time, {}, {});
      if (next < expected.size() && expected[next].time == time) {
        CAPTURE(time);
        CHECK(*player.find_phase("main") == expected[next].main);
        CHECK(*player.find_phase("side") == expected[next].side);
        CHECK(*player.find_signal_state("1") == expected[next].signal);
        ++next;
      }
    }
    CHECK(next == expected.size());
    // The event on the phase started as the main controller first stopped.
    std::vector<double> started;
    for (const auto& transition : player.transitions()) {
      if (transition.element == "stopped" &&
          transition.transition ==
              osc::StoryboardElementState::START_TRANSITION) {
        started.push_back(transition.time);
      }
    }
    CHECK(started == std::vector<double>{20.0});
  }
}

}  // namespace simon::automotive
