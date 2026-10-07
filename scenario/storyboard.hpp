// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "model/road/road.hpp"
#include "model/road/road_placement.hpp"
#include "scenario/openscenario.hpp"

// Runs an OpenSCENARIO storyboard (see model/REFERENCES.md): its elements'
// states, standby, running and complete, and the start, end, stop and skip
// transitions between them; the triggers that start and stop them, each
// condition counted on its edge and after its delay; which private actions
// start and stop when; and the traffic signals' states, set by actions and
// by the controllers' phases. The world carries the private actions out, and
// tells the storyboard each step what it has finished.
//
// Where the standard leaves the order open, the storyboard follows esmini's:
// each step evaluates every trigger on the world as the last step left it,
// and an action started in a step also runs in it; an event that overwrites
// stops its maneuver's running events, and an action ends the entity's
// running action of the same domain, longitudinal or lateral. A condition's
// edge needs a value before it, so none fires on its first evaluation, and a
// trigger that fires starts its conditions over.
namespace simon::scenario {

// What the storyboard reads of each entity, as the last step left it.
struct EntityState final {
  road::RoadPlacement placement;
  road::PlacementPose pose;
  double speed = 0.0;         // m/s, along the heading.
  double acceleration = 0.0;  // m/s^2.
  double end_of_road = -1.0;  // s at the end of its road, -1 if not there.
  double offroad = -1.0;      // s off every lane, -1 if not off.
};

// A private action to start on an entity, known by its handle until it
// finishes or is stopped.
struct ActionOrder final {
  std::size_t entity = 0;
  std::uint32_t handle = 0;
  const PrivateAction* action = nullptr;
};

// What a step of the storyboard asks of the world: actions to stop, then
// actions to start, in order.
struct StoryboardOrders final {
  std::vector<std::uint32_t> stops;
  std::vector<ActionOrder> starts;
};

class StoryboardPlayer final {
 public:
  // An element's transition, and when.
  struct Transition final {
    std::string element;
    StoryboardElementState transition =
        StoryboardElementState::START_TRANSITION;
    double time = 0.0;
  };

  // The storyboard of `scenario` on `network`, both of which it keeps.
  StoryboardPlayer(const Scenario& scenario, const road::RoadNetwork& network);

  // The entity named `name`'s index, or the entity count if none is.
  auto find_entity(std::string_view name) const -> std::size_t;

  // Starts the storyboard at `time`: its init actions, and the stories.
  auto start(double time) -> StoryboardOrders;

  // One step at `time`: ends the actions the world has `finished`, then
  // evaluates every trigger on `entities`.
  auto step(double time, std::span<const EntityState> entities,
            std::span<const std::uint32_t> finished) -> StoryboardOrders;

  // Whether the storyboard still runs: its stop trigger has not fired.
  auto running() const -> bool;

  // The value of parameter `name`, as text.
  auto find_parameter(std::string_view name) const -> const std::string*;

  // Traffic signal `signal`'s state, as text, if anything has set it.
  auto find_signal_state(std::string_view signal) const -> const std::string*;

  // The name of the phase traffic signal controller `controller` is in, if
  // there is such a controller.
  auto find_phase(std::string_view controller) const -> const std::string*;

  // Every element's transitions so far, and when, for checking against
  // another player.
  auto transitions() const -> const std::vector<Transition>&;

  // Where `position` is, entities' positions as `entities` has them.
  auto locate(const Position& position,
              std::span<const EntityState> entities) const
      -> road::RoadPlacement;

 private:
  enum class State : std::uint8_t { INIT, STANDBY, RUNNING, COMPLETE };

  // One element of the tree, actions its leaves.
  struct Element final {
    StoryboardElementType type = StoryboardElementType::STORY;
    std::string name;
    std::size_t parent = 0;
    std::vector<std::size_t> children;
    State state = State::INIT;
    int executions = 0;
    int maximum_executions = 1;  // For maneuver groups and events.
    const Trigger* start_trigger = nullptr;
    const Trigger* stop_trigger = nullptr;
    const Event* event = nullptr;
    const Action* action = nullptr;
    const PrivateAction* private_action = nullptr;  // An action's, if private.
    std::size_t entity = 0;                         // An action's actor.
  };

  // A traffic signal controller as it runs: when its cycle last began, and
  // the phase whose states it last set.
  struct ControllerState final {
    static constexpr std::size_t NONE = static_cast<std::size_t>(-1);

    const TrafficSignalController* controller = nullptr;
    double offset = 0.0;  // s.
    std::size_t phase = NONE;
  };

  // A condition's memory: its last value, for edges, and its values over
  // time, for delays.
  struct ConditionMemory final {
    bool evaluated = false;
    bool last = false;
    std::vector<std::pair<double, bool>> history;
    // For a storyboard element's transition: how much of the log it has
    // seen, a transition counting at the condition's next evaluation.
    std::size_t seen = 0;
  };

  auto add(Element element) -> std::size_t;
  auto build(const Story& story) -> void;

  auto start_element(std::size_t index, double time) -> void;
  auto start_event(std::size_t index, double time) -> void;
  auto end_element(std::size_t index, double time) -> void;
  auto stop_element(std::size_t index, double time) -> void;
  auto reset_element(std::size_t index) -> void;
  auto propagate(std::size_t index, double time) -> void;
  auto evaluate_triggers(std::size_t index, double time) -> void;
  auto finish_instant(double time) -> void;
  auto record(std::size_t index, StoryboardElementState transition, double time)
      -> void;
  auto apply(const GlobalAction& action, double time) -> void;
  auto set_signal(const std::string& signal, const std::string& state) -> void;
  auto run_controllers(double time) -> void;
  auto find_phase_index(const ControllerState& state, double time) const
      -> std::size_t;

  auto fire(const Trigger& trigger, double time) -> bool;
  auto reset_trigger(const Trigger* trigger) -> void;
  auto evaluate(const Condition& condition, double time) -> bool;
  auto check(const EntityCondition& condition, double time) -> bool;
  auto check(const ValueCondition& condition, double time) -> bool;
  auto compute_relative_distance(std::size_t from, std::size_t to,
                                 const RelativeDistance& distance) const
      -> double;
  auto compute_road_gap(std::size_t from, std::size_t to, bool freespace) const
      -> double;
  auto compute_distance_to(std::size_t from,
                           const DistanceCondition& condition) const -> double;

  const Scenario* scenario_ = nullptr;
  const road::RoadNetwork* network_ = nullptr;
  std::vector<Element> elements_;  // The storyboard first.
  std::vector<std::pair<const Condition*, ConditionMemory>> memories_;
  std::vector<Parameter> parameters_;
  std::vector<TrafficSignalState> signals_;
  std::vector<ControllerState> controllers_;
  std::span<const EntityState> entities_;
  std::vector<Transition> transitions_;
  // Every element's transitions, for storyboard element state conditions.
  std::vector<std::pair<std::size_t, StoryboardElementState>> log_;
  const ConditionMemory* checking_ = nullptr;
  StoryboardOrders orders_;
  // Actions done as they start, a teleport, a route, a parameter set or a
  // traffic signal action, which end once every trigger of the step is
  // evaluated, as esmini ends them when they run.
  std::vector<std::size_t> instant_;
};

}  // namespace simon::scenario
