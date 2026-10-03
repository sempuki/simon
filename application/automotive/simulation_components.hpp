// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/step.hpp"
#include "framework/vocabulary.hpp"
#include "framework/world.hpp"
#include "model/kinematics.hpp"
#include "model/lane_graph.hpp"
#include "model/road.hpp"
#include "model/road_placement.hpp"
#include "model/traffic.hpp"
#include "model/units.hpp"
#include "scenario/storyboard.hpp"
#include "scenario/transition.hpp"

// Vehicles drive a network of roads, each following its lane by the
// Intelligent Driver Model and changing lanes by MOBIL. A vehicle's state is
// in lane coordinates: its lane, its place along the lane and its speed. It
// follows the lane's middle, which is the kinematic single-track model with
// its steering set by the lane's curvature, and its place in the world
// follows from the road's geometry.
namespace simon::automotive {

using framework::Duration;
using framework::Entity;
using framework::Step;
using framework::TimePoint;
using model::AccelerationMagnitude;
using model::Angle;
using model::LaneKey;
using model::Length;
using model::Position;
using model::Speed;

// The roads vehicles drive on and how their lanes link, read once and shared
// by the systems. It outlives the world.
struct Network final {
  model::RoadNetwork roads;
  model::LaneGraph graph;
};

// Where a vehicle is in the network and how fast it goes: its lane, the s of
// its front bumper on the lane's road, and its speed along the lane. `turns`
// counts the lanes it has entered, which with its driver's seed picks its way
// at each fork.
struct LaneState final {
  LaneKey lane;
  Length s = 0.0 * model::meter;
  Speed speed = 0.0 * model::meter_per_second;
  std::uint32_t turns = 0;
};

// A vehicle's driver, and the vehicle's length.
struct Driver final {
  model::IntelligentDriver following;
  model::LaneChanger changing;
  Length length = 4.5 * model::meter;
  std::uint64_t seed = 0;  // Picks its way at forks.
};

// What a vehicle's driver decided this step: its acceleration, and the lane
// it changes to, if any.
struct DriveCommand final {
  AccelerationMagnitude acceleration = 0.0 * model::meter_per_second_squared;
  std::optional<LaneKey> change;
};

// A vehicle in the world: its front bumper's position, and its heading,
// counterclockwise from east.
struct VehiclePose final {
  Position position = model::meters(0.0, 0.0, 0.0);
  Angle heading = 0.0 * model::radian;
};

//-- As a spatial component ---------------------------------------------------

inline auto distance(const VehiclePose& a, const VehiclePose& b) -> Length {
  return norm(a.position - b.position);
}
inline auto coordinates(const VehiclePose& pose) -> framework::Coordinates {
  return model::coordinates(pose.position);
}
inline auto coordinate_length(const VehiclePose&, Length length) -> double {
  return length.numerical_value_in(model::meter);
}
inline auto pose(const VehiclePose& vehicle) -> model::Pose {
  return model::Pose{.position = vehicle.position,
                     .orientation = Quaternion{AngleAxis{
                         model::radians(vehicle.heading), Vector3::UnitZ()}}};
}

//-- Scenarios ----------------------------------------------------------------

// What a scenario's systems share, outside the world: the roads, the
// scenario and the player running its storyboard, and each entity's state
// and orders as the storyboard saw and gave them this step. It outlives the
// world.
struct ScenarioContext final {
  const model::RoadNetwork* roads = nullptr;
  const scenario::Scenario* scenario = nullptr;
  scenario::StoryboardPlayer* player = nullptr;
  std::vector<scenario::EntityState> states;  // By entity.
  bool started = false;
};

// A vehicle an OpenSCENARIO scenario drives: its entity's index in the
// scenario. The storyboard drives it, by its actions on speed and on the
// lateral position, and between them it holds its speed and keeps its lane.
struct ScenarioActor final {
  std::size_t entity = 0;
};

// What the storyboard asked of a vehicle this step: actions to stop, by
// handle, and actions to start.
struct ScenarioOrders final {
  std::vector<std::uint32_t> stops;
  std::vector<scenario::ActionOrder> starts;
  // Where each teleport puts the vehicle, in the order given, each found
  // after the teleports before it, as a relative position needs.
  std::vector<model::RoadPlacement> teleports;
  // Whether the storyboard teleported the vehicle this step, which then
  // stays where it was put; esmini's init teleports come before the first
  // step, and do not hold it.
  bool held = false;
};

// A running speed action: its transition over time, and the entity its
// target follows, for a relative target.
struct SpeedChange final {
  std::uint32_t handle = 0;
  scenario::Transition transition;
  std::optional<std::size_t> relative_to;
  scenario::RelativeTargetSpeed::Kind kind =
      scenario::RelativeTargetSpeed::Kind::DELTA;
  double relative_value = 0.0;
  bool continuous = false;
  bool reached = false;
};

// A vehicle's speed, the action changing it, and the speed actions it
// finished this step.
struct ScenarioSpeed final {
  double speed = 0.0;         // m/s, along its heading.
  double acceleration = 0.0;  // m/s^2.
  std::optional<SpeedChange> change;
  std::vector<std::uint32_t> finished;
  // The speed before its action ran this step, and that action's handle:
  // actions run in the storyboard's order, so a lateral action before the
  // speed action moves at the speed before it.
  double unstepped = 0.0;
  std::optional<std::uint32_t> stepped_by;
};

// A running lateral action: a lane change to `lane`, or a lane offset, its
// offset from the target lane's middle, positive to the left of the lane's
// travel, moving by its transition over time or distance.
struct LateralChange final {
  std::uint32_t handle = 0;
  bool lane_change = true;
  int lane = 0;
  scenario::DynamicsDimension dimension = scenario::DynamicsDimension::TIME;
  scenario::Transition transition;
};

// A vehicle on the road: where it is, the lateral action moving it, how long
// it has been at the end of its road, and the lateral actions it finished
// this step.
struct ScenarioMotion final {
  model::RoadPlacement placement;
  std::optional<LateralChange> change;
  double end_of_road = -1.0;  // s, -1 if not there.
  std::vector<std::uint32_t> finished;
};

namespace archetype {

using framework::Archetype;
using framework::Requires;

struct Vehicle final                                                         //
    : Archetype<"vehicle",                                                   //
                Requires<VehiclePose, LaneState, Driver, DriveCommand>> {};  //

struct ScenarioVehicle final                             //
    : Archetype<"scenario vehicle",                      //
                Requires<VehiclePose, ScenarioActor,     //
                         ScenarioOrders, ScenarioSpeed,  //
                         ScenarioMotion>> {};            //

}  // namespace archetype

using World =
    framework::World<VehiclePose,
                     framework::TypeList<LaneState, Driver, DriveCommand>,
                     framework::TypeList<archetype::Vehicle>>;

// The world an OpenSCENARIO scenario plays in.
using ScenarioWorld =
    framework::World<VehiclePose,
                     framework::TypeList<ScenarioActor, ScenarioOrders,
                                         ScenarioSpeed, ScenarioMotion>,
                     framework::TypeList<archetype::ScenarioVehicle>>;

}  // namespace simon::automotive
