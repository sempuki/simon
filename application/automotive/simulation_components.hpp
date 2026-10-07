// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "core/math.hpp"
#include "core/time.hpp"
#include "core/units.hpp"
#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/world.hpp"
#include "model/kinematics.hpp"
#include "model/road/lane_graph.hpp"
#include "model/road/polyline.hpp"
#include "model/road/road.hpp"
#include "model/road/road_placement.hpp"
#include "model/road/walking_graph.hpp"
#include "model/traffic/right_of_way.hpp"
#include "model/traffic/traffic.hpp"
#include "model/traffic/traffic_control.hpp"
#include "scenario/storyboard.hpp"
#include "scenario/transition.hpp"

// Vehicles drive a network of roads, each following its lane by the
// Intelligent Driver Model and changing lanes by MOBIL. A vehicle's state is
// in lane coordinates: its lane, its place along the lane and its speed. It
// follows the lane's middle, which is the kinematic single-track model with
// its steering set by the lane's curvature, and its place in the world
// follows from the road's geometry.
namespace simon::automotive {

using framework::Entity;
using road::LaneKey;

// The roads vehicles drive on, how their lanes link, and the signal groups
// and stop lines on them, read once and shared by the systems. It outlives
// the world.
struct Network final {
  road::RoadNetwork roads;
  road::LaneGraph graph;
  road::LaneGraph driving;  // The driving lanes' graph alone.
  traffic::TrafficControl control;
  traffic::RightOfWay rights;
  road::WalkingGraph walking;
  std::vector<std::uint32_t> walking_components;  // Each node's.
  // Each crosswalk's signal group, if a light stands just before it, and
  // whether vehicles yield to pedestrians at crosswalks without one.
  std::vector<std::optional<std::uint32_t>> crosswalk_groups;
  bool vehicles_yield = true;
};

// Where a vehicle is in the network and how fast it goes: its lane, the s of
// its front bumper on the lane's road, and its speed along the lane. `turns`
// counts the lanes it has entered, which with its driver's seed picks its way
// at each fork.
struct LaneState final {
  LaneKey lane;
  Length s = 0.0 * meter;
  Speed speed = 0.0 * meter_per_second;
  std::uint32_t turns = 0;
};

// A vehicle's driver, and the vehicle's length.
struct Driver final {
  traffic::IntelligentDriver following;
  traffic::LaneChanger changing;
  Length length = 4.5 * meter;
  std::uint64_t seed = 0;  // Picks its way at forks.
};

// How a driver treats what it must stop for, and what it has resolved to do
// ahead: the stop line, by its place in the network's stop lines, that it
// passes whatever the light shows, having decided to go on yellow or been
// too near to stop on red; and the junction's connecting lane it has decided
// to enter, having found a gap or been too near to stop. It gives way where
// the first vehicle with priority would reach the conflict within
// `critical_gap` of its reaching the junction. Only vehicles on a network
// with lights or junctions have it.
struct Tactical final {
  static constexpr std::uint32_t NONE = ~std::uint32_t{0};

  traffic::LightBraking braking;
  Time critical_gap = 6.0 * second;
  std::optional<LaneKey> entering;
  std::uint32_t committed = NONE;
};

// Since when a vehicle has stood still, or never if it is moving, and
// whether its driver has committed to the junction ahead or the one it is
// in, for deciding who goes first among drivers that wait for each other.
struct Stopped final {
  TimePoint since = TimePoint::max();
  bool committed = false;
};

// A signal group, by its place in the network's groups, and what it shows.
struct SignalState final {
  std::uint32_t group = 0;
  traffic::Aspect aspect = traffic::Aspect::RED;
};

// What a vehicle's driver decided this step: its acceleration, and the lane
// it changes to, if any.
struct DriveCommand final {
  AccelerationMagnitude acceleration = 0.0 * meter_per_second_squared;
  std::optional<LaneKey> change;
};

// Where a road user is in the world, the world's spatial component: a
// vehicle's front bumper or a pedestrian's middle, and its heading,
// counterclockwise from east.
struct RoadPose final {
  Position position = meters(0.0, 0.0, 0.0);
  Angle heading = 0.0 * radian;
};

//-- As a spatial component ---------------------------------------------------

inline auto distance(const RoadPose& a, const RoadPose& b) -> Length {
  return norm(a.position - b.position);
}
inline auto coordinates(const RoadPose& pose) -> Coordinates {
  return model::coordinates(pose.position);
}
inline auto coordinate_length(const RoadPose&, Length length) -> double {
  return length.numerical_value_in(meter);
}
inline auto pose(const RoadPose& vehicle) -> model::Pose {
  return model::Pose{.position = vehicle.position,
                     .orientation = Quaternion{AngleAxis{
                         radians(vehicle.heading), Vector3::UnitZ()}}};
}

//-- Scenarios ----------------------------------------------------------------

// What a scenario's systems share, outside the world: the roads, the
// scenario and the player running its storyboard, and each entity's state
// and orders as the storyboard saw and gave them this step. It outlives the
// world.
struct ScenarioContext final {
  const road::RoadNetwork* roads = nullptr;
  const road::LaneGraph* lanes = nullptr;
  const scenario::Scenario* scenario = nullptr;
  scenario::StoryboardPlayer* player = nullptr;
  std::vector<scenario::EntityState> states;  // By entity.
  bool started = false;
};

// A vehicle or pedestrian an OpenSCENARIO scenario drives: its entity's
// index in the scenario. The storyboard drives it, by its actions on speed,
// on the lateral position and on its route or trajectory, and between them
// it holds its speed and keeps its lane.
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
  std::vector<road::RoadPlacement> teleports;
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

// A trajectory being followed: its polyline, how far along it the entity
// is, and whether the entity faces back along it, as it does if it was
// moving backward when it started.
struct TrajectoryRun final {
  std::uint32_t handle = 0;
  road::Polyline polyline;
  double along = 0.0;  // m.
  bool backward = false;
};

// An entity on the road: where it is, the lateral action or trajectory
// moving it, the roads its route runs through, how long it has been at the
// end of its road, and the lateral actions it finished this step.
struct ScenarioMotion final {
  road::RoadPlacement placement;
  std::optional<LateralChange> change;
  std::optional<TrajectoryRun> trajectory;
  // Where a trajectory put it this step, which its placement only nears.
  std::optional<road::PlacementPose> pose;
  std::vector<std::size_t> route;  // By index.
  double end_of_road = -1.0;       // s, -1 if not there.
  std::vector<std::uint32_t> finished;
};

//-- Pedestrians ---------------------------------------------------------------

// A pedestrian's route on the walking graph, and how many it has walked.
struct WalkRoute final {
  std::vector<road::Leg> legs;
  std::uint32_t trips = 0;
};

// Where a pedestrian is on its route: the leg it walks, how far along it,
// and its speed.
struct WalkState final {
  std::uint32_t leg = 0;
  Length along = 0.0 * meter;
  Speed speed = 0.0 * meter_per_second;
};

// A pedestrian's walking speed, as it would walk alone; its start-up time,
// which with the time to walk across makes the gap it needs to cross; its
// seed, which picks where it goes; and whether it waits for a light that
// tells it to.
struct Walker final {
  Speed desired_speed = 1.34 * meter_per_second;
  Time start_up = 2.0 * second;
  std::uint64_t seed = 0;
  bool complies = true;
};

// How fast a pedestrian walks this step, the crosswalk, by its place, it
// has decided to cross or is crossing, if any, and how soon it will be off
// it.
struct WalkCommand final {
  static constexpr std::uint32_t NONE = ~std::uint32_t{0};

  Speed speed = 0.0 * meter_per_second;
  Time clear = 0.0 * second;
  std::uint32_t crossing = NONE;
  bool on = false;  // On the crosswalk already.
};

namespace archetype {

using framework::Archetype;
using framework::Requires;

struct Vehicle final                                                      //
    : Archetype<"vehicle",                                                //
                Requires<RoadPose, LaneState, Driver, DriveCommand>> {};  //

// A vehicle on a network with lights or junctions, which it stops for and
// gives way at.
struct TacticalVehicle final                      //
    : Archetype<"tactical vehicle",               //
                Requires<RoadPose, LaneState,     //
                         Driver, DriveCommand,    //
                         Tactical, Stopped>> {};  //

// A pedestrian on the walking graph.
struct Pedestrian final                                //
    : Archetype<"pedestrian",                          //
                Requires<RoadPose, WalkState, Walker,  //
                         WalkRoute, WalkCommand>> {};  //

// A signal group's controller, running its plan.
struct SignalController final                                    //
    : Archetype<"signal controller",                             //
                Requires<traffic::SignalPlan, SignalState>> {};  //

struct ScenarioEntity final                              //
    : Archetype<"scenario entity",                       //
                Requires<RoadPose, ScenarioActor,        //
                         ScenarioOrders, ScenarioSpeed,  //
                         ScenarioMotion>> {};            //

}  // namespace archetype

using World = framework::World<
    RoadPose,
    framework::TypeList<LaneState, Driver, DriveCommand, Tactical, Stopped,
                        traffic::SignalPlan, SignalState, WalkState, Walker,
                        WalkRoute, WalkCommand>,
    framework::TypeList<archetype::Vehicle, archetype::TacticalVehicle,
                        archetype::Pedestrian, archetype::SignalController>>;

// The world an OpenSCENARIO scenario plays in.
using ScenarioWorld =
    framework::World<RoadPose,
                     framework::TypeList<ScenarioActor, ScenarioOrders,
                                         ScenarioSpeed, ScenarioMotion>,
                     framework::TypeList<archetype::ScenarioEntity>>;

}  // namespace simon::automotive
