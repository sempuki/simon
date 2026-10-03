// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road_placement.hpp"

#include <cmath>
#include <numbers>

namespace simon::model {

namespace {

constexpr double TINY = 1e-12;

// The angle in (-pi, pi].
auto wrap(double angle) -> double {
  return std::remainder(angle, 2.0 * std::numbers::pi);
}

}  // namespace

auto runs_forward(const RoadPlacement& placement) -> bool {
  return std::abs(wrap(placement.heading)) < 0.5 * std::numbers::pi;
}

auto compute_lane_center(const Road& road, double s, int lane) -> double {
  const LaneSection& section = find_lane_section(road, s * meter);
  int inner = lane > 0 ? lane - 1 : lane < 0 ? lane + 1 : 0;
  return 0.5 * (compute_lane_border(road, section, s * meter, lane) +
                compute_lane_border(road, section, s * meter, inner))
                   .numerical_value_in(meter);
}

auto compute_placement_t(const Road& road, const RoadPlacement& placement)
    -> double {
  return compute_lane_center(road, placement.s, placement.lane) +
         placement.offset;
}

auto compute_placement_pose(const RoadNetwork& network,
                            const RoadPlacement& placement) -> PlacementPose {
  const Road& road = network.roads[placement.road];
  Length s = placement.s * meter;
  PlanPoint point = compute_plan_point(road, s);
  Position position = compute_road_position(
      road, point, s, compute_placement_t(road, placement) * meter);
  Vector3 at = eigen(position);
  return {.x = at.x(),
          .y = at.y(),
          .z = at.z(),
          .heading = wrap(point.heading + placement.heading)};
}

// A path at t from a reference line of curvature kappa is 1 - kappa t times
// as long, so s changes by distance / (1 - kappa t).
auto convert_distance_to_ds(const RoadNetwork& network,
                            const RoadPlacement& placement, double distance)
    -> double {
  const Road& road = network.roads[placement.road];
  double curvature = compute_plan_point(road, placement.s * meter).curvature;
  if (std::abs(curvature) < TINY) {
    return distance;
  }
  return distance / (1.0 - curvature * compute_placement_t(road, placement));
}

auto move_along_road(const RoadNetwork& network, InOut<RoadPlacement> placement,
                     double ds) -> RoadMove {
  double remaining = runs_forward(*placement) ? ds : -ds;
  for (int links = 0; links < 8; ++links) {
    const Road& road = network.roads[placement->road];
    double s = placement->s + remaining;
    if (s >= 0.0 && s <= road.length) {
      placement->s = s;
      return RoadMove::ALONG;
    }
    bool past_end = s > road.length;
    const RoadLink& link = past_end ? road.successor : road.predecessor;
    const Road* next = link.kind == RoadLink::Kind::ROAD
                           ? network.find_road(link.id)
                           : nullptr;
    const LaneSection& section =
        past_end ? road.lane_sections.back() : road.lane_sections.front();
    const std::vector<Lane>& side =
        placement->lane > 0 ? section.left : section.right;
    auto index = static_cast<std::size_t>(std::abs(placement->lane));
    std::optional<int> lane = index >= 1 && index <= side.size()
                                  ? (past_end ? side[index - 1].successor
                                              : side[index - 1].predecessor)
                                  : std::nullopt;
    if (next == nullptr) {
      placement->s = past_end ? road.length : 0.0;
      return RoadMove::END_OF_ROAD;
    }
    double beyond = past_end ? s - road.length : -s;
    placement->road = static_cast<std::size_t>(next - network.roads.data());
    placement->lane = lane.value_or(placement->lane);
    if (link.contact == RoadLink::Contact::START) {
      placement->s = 0.0;
      remaining = beyond;
    } else {
      placement->s = next->length;
      remaining = -beyond;
    }
    // Entering at the end, the next road runs the other way: s and t turn,
    // and so does the heading relative to it.
    if (past_end == (link.contact == RoadLink::Contact::END)) {
      placement->offset = -placement->offset;
      placement->heading = wrap(placement->heading + std::numbers::pi);
    }
  }
  return RoadMove::END_OF_ROAD;
}

auto find_placement(const RoadNetwork& network, double x, double y,
                    double heading) -> std::optional<RoadPlacement> {
  std::optional<RoadPlacement> nearest;
  double nearest_apart = 0.0;
  for (std::size_t i = 0; i < network.roads.size(); ++i) {
    const Road& road = network.roads[i];
    RoadCoordinates at = find_road_coordinates(road, x * meter, y * meter);
    double s = at.s.numerical_value_in(meter);
    double t = at.t.numerical_value_in(meter);
    if (s < 0.0 || s > road.length) {
      continue;
    }
    std::optional<int> lane = find_lane(road, at.s, at.t);
    if (!lane || *lane == 0) {
      continue;
    }
    double center = compute_lane_center(road, s, *lane);
    PlanPoint point = compute_plan_point(road, at.s);
    RoadPlacement placement{.road = i,
                            .lane = *lane,
                            .s = s,
                            .offset = t - center,
                            .heading = wrap(heading - point.heading)};
    double apart = std::abs(t - center);
    if (!nearest || apart < nearest_apart) {
      nearest = placement;
      nearest_apart = apart;
    }
  }
  return nearest;
}

}  // namespace simon::model
