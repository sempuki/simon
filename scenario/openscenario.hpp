// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

// A scenario as ASAM OpenSCENARIO 1.x describes it (see model/REFERENCES.md):
// the entities, their vehicles, and the storyboard that drives them, its
// stories, acts, maneuver groups, maneuvers and events, and the triggers that
// start and stop them. format/openscenario reads it from files.
//
// The subset is what object-level driving needs: positions on lanes,
// roads and in the world; speed, lane change, lane offset, teleport and
// parameter actions; and the conditions on time, speed, distance, headway,
// position, the end of the road, leaving the road, parameters and the
// storyboard's own states.
namespace simon::scenario {

//-- Positions ----------------------------------------------------------------

// A heading given with a position: absolute, counterclockwise from x, or
// relative to the road's direction at the position. Without one, an entity
// heads along its lane's direction of travel.
struct Orientation final {
  bool relative = true;
  double h = 0.0;
};

// A position in the world, h counterclockwise from x.
struct WorldPosition final {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double h = 0.0;
};

// A position on a road's lane: s along the road, offset from the lane's
// middle.
struct LanePosition final {
  std::string road;
  int lane = 0;
  double s = 0.0;
  double offset = 0.0;
  std::optional<Orientation> orientation;
};

// A position on a road: s along it, t across it.
struct RoadPosition final {
  std::string road;
  double s = 0.0;
  double t = 0.0;
  std::optional<Orientation> orientation;
};

// A position relative to an entity's, along and across its road.
struct RelativeRoadPosition final {
  std::string entity;
  double ds = 0.0;
  double dt = 0.0;
  std::optional<Orientation> orientation;
};

// A position relative to an entity's: lanes across and s along its road,
// and an offset in the target lane.
struct RelativeLanePosition final {
  std::string entity;
  int lanes = 0;
  double ds = 0.0;
  double offset = 0.0;
  std::optional<Orientation> orientation;
};

using Position = std::variant<WorldPosition, LanePosition, RoadPosition,
                              RelativeRoadPosition, RelativeLanePosition>;

//-- Actions ------------------------------------------------------------------

// How a value moves from where it is to its target: in a step, linearly,
// along a cubic or a half cosine, over a time, a distance, or at a rate.
enum class DynamicsShape : std::uint8_t { STEP, LINEAR, CUBIC, SINUSOIDAL };
enum class DynamicsDimension : std::uint8_t { TIME, DISTANCE, RATE };

struct TransitionDynamics final {
  DynamicsShape shape = DynamicsShape::STEP;
  DynamicsDimension dimension = DynamicsDimension::TIME;
  double value = 0.0;
};

// A speed to reach: absolute, or relative to an entity's by a difference or
// a factor, held once reached if continuous.
struct AbsoluteTargetSpeed final {
  double value = 0.0;
};

struct RelativeTargetSpeed final {
  enum class Kind : std::uint8_t { DELTA, FACTOR };

  std::string entity;
  double value = 0.0;
  Kind kind = Kind::DELTA;
  bool continuous = false;
};

struct SpeedAction final {
  TransitionDynamics dynamics;
  std::variant<AbsoluteTargetSpeed, RelativeTargetSpeed> target;
};

// A lane to change to: by its id, or relative to an entity's lane, positive
// to the left of its travel.
struct AbsoluteTargetLane final {
  int lane = 0;
};

struct RelativeTargetLane final {
  std::string entity;
  int lanes = 0;
};

struct LaneChangeAction final {
  TransitionDynamics dynamics;
  std::variant<AbsoluteTargetLane, RelativeTargetLane> target;
  double target_offset = 0.0;  // In the target lane.
};

// An offset to move to within the lane, absolute or relative to an entity,
// shaped by a limit on lateral acceleration.
struct LaneOffsetAction final {
  DynamicsShape shape = DynamicsShape::SINUSOIDAL;
  double max_lateral_acceleration = 0.0;
  std::optional<std::string> relative_to;  // An entity, if relative.
  double value = 0.0;
  bool continuous = false;
};

struct TeleportAction final {
  Position position;
};

// A global action setting a parameter, or adding to it or multiplying it.
struct ParameterAction final {
  enum class Kind : std::uint8_t { SET, ADD, MULTIPLY };

  std::string parameter;
  Kind kind = Kind::SET;
  std::string value;
};

using PrivateAction = std::variant<SpeedAction, LaneChangeAction,
                                   LaneOffsetAction, TeleportAction>;

struct Action final {
  std::string name;
  std::variant<PrivateAction, ParameterAction> action;
};

//-- Conditions ---------------------------------------------------------------

enum class Rule : std::uint8_t {
  GREATER_THAN,
  LESS_THAN,
  EQUAL_TO,
  GREATER_OR_EQUAL,
  LESS_OR_EQUAL,
  NOT_EQUAL_TO,
};

struct SimulationTimeCondition final {
  double value = 0.0;
  Rule rule = Rule::GREATER_THAN;
};

struct ParameterCondition final {
  std::string parameter;
  std::string value;
  Rule rule = Rule::EQUAL_TO;
};

enum class StoryboardElementType : std::uint8_t {
  STORY,
  ACT,
  MANEUVER_GROUP,
  MANEUVER,
  EVENT,
  ACTION,
};

// A storyboard element's state or transition: standby, running, complete,
// or the start, end, stop or skip transition into one.
enum class StoryboardElementState : std::uint8_t {
  STANDBY,
  RUNNING,
  COMPLETE,
  START_TRANSITION,
  END_TRANSITION,
  STOP_TRANSITION,
  SKIP_TRANSITION,
};

struct StoryboardElementStateCondition final {
  StoryboardElementType type = StoryboardElementType::EVENT;
  std::string element;
  StoryboardElementState state = StoryboardElementState::COMPLETE;
};

struct SpeedCondition final {
  double value = 0.0;
  Rule rule = Rule::GREATER_THAN;
};

struct AccelerationCondition final {
  double value = 0.0;
  Rule rule = Rule::GREATER_THAN;
};

// The time to the other entity at this speed: the distance along the road,
// or straight, between reference points, or between bounding boxes if
// freespace, over the triggering entity's speed.
struct TimeHeadwayCondition final {
  std::string entity;
  double value = 0.0;
  bool freespace = false;
  bool along_road = false;
  Rule rule = Rule::GREATER_THAN;
};

// The distance to the other entity, along the road's s, across it, or
// straight, between reference points or bounding boxes.
struct RelativeDistanceCondition final {
  enum class Kind : std::uint8_t { LONGITUDINAL, LATERAL, CARTESIAN };

  std::string entity;
  Kind kind = Kind::CARTESIAN;
  double value = 0.0;
  bool freespace = false;
  bool along_road = false;
  Rule rule = Rule::GREATER_THAN;
};

struct ReachPositionCondition final {
  Position position;
  double tolerance = 0.0;
};

// At the end of a road, or off it, for at least `duration` seconds.
struct EndOfRoadCondition final {
  double duration = 0.0;
};

struct OffroadCondition final {
  double duration = 0.0;
};

using EntityConditionKind =
    std::variant<SpeedCondition, AccelerationCondition, TimeHeadwayCondition,
                 RelativeDistanceCondition, ReachPositionCondition,
                 EndOfRoadCondition, OffroadCondition>;

// A condition on entities: on any or all of the triggering entities.
struct EntityCondition final {
  std::vector<std::string> triggering;
  bool all = false;
  EntityConditionKind condition;
};

using ValueCondition = std::variant<SimulationTimeCondition, ParameterCondition,
                                    StoryboardElementStateCondition>;

// A condition and when it counts: on its rising edge, its falling edge,
// either, or whenever it holds, after a delay.
struct Condition final {
  enum class Edge : std::uint8_t { NONE, RISING, FALLING, RISING_OR_FALLING };

  std::string name;
  double delay = 0.0;
  Edge edge = Edge::NONE;
  std::variant<EntityCondition, ValueCondition> condition;
};

// Fires when every condition of any of its groups holds.
struct Trigger final {
  std::vector<std::vector<Condition>> groups;
};

//-- Storyboard ---------------------------------------------------------------

struct Event final {
  // What an event does when it starts while another of its maneuver's runs:
  // stops the others, waits its turn, or runs beside them.
  enum class Priority : std::uint8_t { OVERWRITE, SKIP, PARALLEL };

  std::string name;
  Priority priority = Priority::OVERWRITE;
  int maximum_executions = 1;
  std::vector<Action> actions;
  std::optional<Trigger> start;  // None: starts with its maneuver.
};

struct Maneuver final {
  std::string name;
  std::vector<Event> events;
};

struct ManeuverGroup final {
  std::string name;
  int maximum_executions = 1;
  std::vector<std::string> actors;
  std::vector<Maneuver> maneuvers;
};

struct Act final {
  std::string name;
  std::vector<ManeuverGroup> groups;
  std::optional<Trigger> start;
  std::optional<Trigger> stop;
};

struct Story final {
  std::string name;
  std::vector<Act> acts;
};

// Each entity's actions at the start.
struct InitActions final {
  std::string entity;
  std::vector<PrivateAction> actions;
};

struct Storyboard final {
  std::vector<InitActions> init;
  std::vector<ParameterAction> global_init;
  std::vector<Story> stories;
  std::optional<Trigger> stop;
};

//-- Entities -----------------------------------------------------------------

// A vehicle's box about its reference point, the rear axle's middle on the
// ground, and how fast it may go and change speed.
struct Vehicle final {
  std::string name;
  std::string category;
  std::array<double, 3> center{};      // m, from the reference point.
  std::array<double, 3> dimensions{};  // Length, width, height, m.
  double max_speed = 0.0;
  double max_acceleration = 0.0;
  double max_deceleration = 0.0;
  double wheelbase = 0.0;  // From the axles.
};

struct Entity final {
  std::string name;
  Vehicle vehicle;
};

// A parameter's value, as text, as parameters are written.
struct Parameter final {
  std::string name;
  std::string type;
  std::string value;
};

struct Scenario final {
  // The entity named `name`, if there is one.
  auto find_entity(std::string_view name) const -> const Entity*;

  std::string road_network;  // The OpenDRIVE file, as a path.
  std::vector<Parameter> parameters;
  std::vector<Entity> entities;
  Storyboard storyboard;
};

}  // namespace simon::scenario
