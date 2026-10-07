// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "base/core.hpp"
#include "model/road/lane_graph.hpp"
#include "model/road/road.hpp"

// A vehicle placed on a road by lane rather than by lane graph, as a scenario
// places it (see model/REFERENCES.md, ASAM OpenSCENARIO): its road, its lane,
// s along the road, its offset from the lane's middle, and its heading
// relative to the road's direction at s. It may drive either way in any lane.
// Plain SI numbers, as the road's data are.
namespace simon::model {

struct RoadPlacement final {
  std::size_t road = 0;  // Index into the network's roads.
  int lane = 0;
  double s = 0.0;
  double offset = 0.0;   // m, along t from the lane's middle.
  double heading = 0.0;  // rad, from the road's direction.
};

// Where a placement is in the world: x, y and z, and its heading,
// counterclockwise from x.
struct PlacementPose final {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double heading = 0.0;
};

// Whether a placement drives with s: its heading within a quarter turn of
// the road's direction.
auto runs_forward(const RoadPlacement& placement) -> bool;

// The t of `lane`'s middle on `road` at `s`, midway between its borders, in
// the lane section in force at s.
auto compute_lane_center(const Road& road, double s, int lane) -> double;

// The placement's t: its lane's middle and its offset.
auto compute_placement_t(const Road& road, const RoadPlacement& placement)
    -> double;

auto compute_placement_pose(const RoadNetwork& network,
                            const RoadPlacement& placement) -> PlacementPose;

// The change in s that moves the placement `distance` meters along its path
// at its t, the path's length to the reference line's in the ratio
// 1 - kappa t, kappa sampled at the placement.
auto convert_distance_to_ds(const RoadNetwork& network,
                            const RoadPlacement& placement, double distance)
    -> double;

enum class RoadMove : std::uint8_t { ALONG, END_OF_ROAD };

// Moves the placement `ds` along its road in the direction it heads, onto
// the road a link joins past either end, its lane continued by the lane's
// link and its heading and offset turned if the next road runs the other
// way. Into a junction it takes the connecting road `route` takes next, its
// lane by the connection's lane link; without a link to follow it stops at
// the end.
auto move_along_road(const RoadNetwork& network, InOut<RoadPlacement> placement,
                     double ds, std::span<const std::size_t> route = {})
    -> RoadMove;

// The roads, by index and in order, of the shortest way by lane length on
// `graph` from each of `waypoints` to the next, as traffic keeps right; as
// far as it gets if a waypoint cannot be reached.
auto find_route(const RoadNetwork& network, const LaneGraph& graph,
                std::span<const RoadPlacement> waypoints)
    -> std::vector<std::size_t>;

// The placement nearest the world's (`x`, `y`): the road whose lane holds
// it, nearest by t, and its lane, offset and s, heading `heading`; road
// `staying` if one of its lanes holds it, as an entity keeps to its road.
// None off every road's lanes.
auto find_placement(const RoadNetwork& network, double x, double y,
                    double heading,
                    std::optional<std::size_t> staying = std::nullopt)
    -> std::optional<RoadPlacement>;

}  // namespace simon::model
