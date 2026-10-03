// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>

#include "framework/archetype.hpp"
#include "framework/entity.hpp"
#include "framework/step.hpp"
#include "framework/vocabulary.hpp"
#include "framework/world.hpp"
#include "model/kinematics.hpp"
#include "model/lane_graph.hpp"
#include "model/road.hpp"
#include "model/traffic.hpp"
#include "model/units.hpp"

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

namespace archetype {

using framework::Archetype;
using framework::Requires;

struct Vehicle final                                                         //
    : Archetype<"vehicle",                                                   //
                Requires<VehiclePose, LaneState, Driver, DriveCommand>> {};  //

}  // namespace archetype

using World =
    framework::World<VehiclePose,
                     framework::TypeList<LaneState, Driver, DriveCommand>,
                     framework::TypeList<archetype::Vehicle>>;

}  // namespace simon::automotive
