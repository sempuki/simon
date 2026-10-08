// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road/placement.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <numbers>
#include <queue>

#include "core/units.hpp"

namespace simon::road {

namespace {

constexpr double TINY = 1e-12;

}  // namespace

auto runs_forward(const Placement& placement) -> bool {
  return std::abs(wrap(placement.heading)) < 0.5 * std::numbers::pi;
}

auto compute_lane_center(const Road& road, double s, int lane) -> double {
  const LaneSection& section = find_lane_section(road, s * meter);
  int inner = lane > 0 ? lane - 1 : lane < 0 ? lane + 1 : 0;
  return 0.5 * (compute_lane_border(road, section, s * meter, lane) +
                compute_lane_border(road, section, s * meter, inner))
                   .numerical_value_in(meter);
}

auto compute_placement_t(const Road& road, const Placement& placement)
    -> double {
  return compute_lane_center(road, placement.s, placement.lane) +
         placement.offset;
}

auto compute_placement_pose(const Map& network, const Placement& placement)
    -> PlacementPose {
  const Road& road = network.roads[placement.road];
  Length s = placement.s * meter;
  PlanPoint point = compute_plan_point(road, s);
  Position position = compute_position(
      road, point, s, compute_placement_t(road, placement) * meter);
  Vector3 at = eigen(position);
  return {.x = at.x(),
          .y = at.y(),
          .z = at.z(),
          .heading = wrap(point.heading + placement.heading)};
}

// A path at t from a reference line of curvature kappa is 1 - kappa t times
// as long, so s changes by distance / (1 - kappa t).
auto convert_distance_to_ds(const Map& network, const Placement& placement,
                            double distance) -> double {
  const Road& road = network.roads[placement.road];
  double curvature = compute_plan_point(road, placement.s * meter).curvature;
  if (std::abs(curvature) < TINY) {
    return distance;
  }
  return distance / (1.0 - curvature * compute_placement_t(road, placement));
}

auto move_along(const Map& network, InOut<Placement> placement, double ds,
                std::span<const std::size_t> route) -> Move {
  double remaining = runs_forward(*placement) ? ds : -ds;
  for (int links = 0; links < 8; ++links) {
    const Road& road = network.roads[placement->road];
    double s = placement->s + remaining;
    if (s >= 0.0 && s <= road.length) {
      placement->s = s;
      return Move::ALONG;
    }
    bool past_end = s > road.length;
    Link link = past_end ? road.successor : road.predecessor;
    const Road* next =
        link.kind == Link::Kind::ROAD ? network.find_road(link.id) : nullptr;
    const LaneSection& section =
        past_end ? road.lane_sections.back() : road.lane_sections.front();
    const std::vector<Lane>& side =
        placement->lane > 0 ? section.left : section.right;
    auto index = static_cast<std::size_t>(std::abs(placement->lane));
    std::optional<int> lane = index >= 1 && index <= side.size()
                                  ? (past_end ? side[index - 1].successor
                                              : side[index - 1].predecessor)
                                  : std::nullopt;
    if (link.kind == Link::Kind::JUNCTION) {
      // The connecting road the route takes next, and the lane it links.
      lane.reset();
      auto here = std::ranges::find(route, placement->road);
      const Junction* junction = network.find_junction(link.id);
      if (junction != nullptr && here != route.end() &&
          here + 1 != route.end()) {
        const std::string& through = network.roads[*(here + 1)].id;
        for (const JunctionConnection& connection : junction->connections) {
          if (connection.incoming_road != road.id ||
              connection.connecting_road != through) {
            continue;
          }
          auto linked =
              std::ranges::find(connection.lane_links, placement->lane,
                                &JunctionConnection::LaneLink::from);
          if (linked != connection.lane_links.end()) {
            next = network.find_road(through);
            lane = linked->to;
            link.contact = connection.contact;
            break;
          }
        }
      }
    }
    if (next == nullptr) {
      placement->s = past_end ? road.length : 0.0;
      return Move::END_OF_ROAD;
    }
    double beyond = past_end ? s - road.length : -s;
    placement->road = static_cast<std::size_t>(next - network.roads.data());
    placement->lane = lane.value_or(placement->lane);
    if (link.contact == Link::Contact::START) {
      placement->s = 0.0;
      remaining = beyond;
    } else {
      placement->s = next->length;
      remaining = -beyond;
    }
    // Entering at the end, the next road runs the other way: s and t turn,
    // and so does the heading relative to it.
    if (past_end == (link.contact == Link::Contact::END)) {
      placement->offset = -placement->offset;
      placement->heading = wrap(placement->heading + std::numbers::pi);
    }
  }
  return Move::END_OF_ROAD;
}

auto find_route(const Map& network, const LaneGraph& graph,
                std::span<const Placement> waypoints)
    -> std::vector<std::size_t> {
  auto key_of = [&](const Placement& placement) {
    const std::vector<LaneSection>& sections =
        network.roads[placement.road].lane_sections;
    std::uint32_t section = 0;
    while (section + 1 < sections.size() &&
           sections[section + 1].s0 <= placement.s) {
      ++section;
    }
    return LaneKey{.road = static_cast<std::uint32_t>(placement.road),
                   .section = section,
                   .lane = placement.lane};
  };
  std::vector<std::size_t> roads;
  auto append = [&](std::size_t road) {
    if (roads.empty() || roads.back() != road) {
      roads.push_back(road);
    }
  };
  if (waypoints.empty()) {
    return roads;
  }
  append(waypoints.front().road);
  for (std::size_t w = 1; w < waypoints.size(); ++w) {
    LaneKey from = key_of(waypoints[w - 1]);
    LaneKey to = key_of(waypoints[w]);
    // Dijkstra's algorithm, each lane costing its section's length.
    using Entry = std::pair<double, LaneKey>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
    std::map<LaneKey, double> best{{from, 0.0}};
    std::map<LaneKey, LaneKey> came;
    open.emplace(0.0, from);
    while (!open.empty()) {
      auto [cost, at] = open.top();
      open.pop();
      if (at == to || cost > best[at]) {
        continue;
      }
      const Road& road = network.roads[at.road];
      double length =
          find_section_end(network, at) - road.lane_sections[at.section].s0;
      for (const LaneKey& next : graph.successors_of(at)) {
        auto known = best.find(next);
        if (known == best.end() || cost + length < known->second) {
          best[next] = cost + length;
          came[next] = at;
          open.emplace(cost + length, next);
        }
      }
    }
    if (!best.contains(to)) {
      break;
    }
    std::vector<LaneKey> path{to};
    while (path.back() != from) {
      path.push_back(came.at(path.back()));
    }
    for (auto it = path.rbegin(); it != path.rend(); ++it) {
      append(it->road);
    }
  }
  return roads;
}

auto find_placement(const Map& network, double x, double y, double heading,
                    std::optional<std::size_t> staying)
    -> std::optional<Placement> {
  // Road `i`'s lane holding (x, y), if one does.
  auto place_on = [&](std::size_t i) -> std::optional<Placement> {
    const Road& road = network.roads[i];
    RoadCoordinates at = find_road_coordinates(road, x * meter, y * meter);
    double s = at.s.numerical_value_in(meter);
    double t = at.t.numerical_value_in(meter);
    if (s < 0.0 || s > road.length) {
      return std::nullopt;
    }
    std::optional<int> lane = find_lane(road, at.s, at.t);
    if (!lane || *lane == 0) {
      return std::nullopt;
    }
    PlanPoint point = compute_plan_point(road, at.s);
    return Placement{.road = i,
                     .lane = *lane,
                     .s = s,
                     .offset = t - compute_lane_center(road, s, *lane),
                     .heading = wrap(heading - point.heading)};
  };
  if (staying && *staying < network.roads.size()) {
    if (std::optional<Placement> kept = place_on(*staying)) {
      return kept;
    }
  }
  std::optional<Placement> nearest;
  for (std::size_t i = 0; i < network.roads.size(); ++i) {
    std::optional<Placement> placement = place_on(i);
    if (placement &&
        (!nearest || std::abs(placement->offset) < std::abs(nearest->offset))) {
      nearest = placement;
    }
  }
  return nearest;
}

}  // namespace simon::road
