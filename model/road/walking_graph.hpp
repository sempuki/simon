// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "core/units.hpp"
#include "model/road/lane_graph.hpp"
#include "model/road/road.hpp"

// Where pedestrians walk: a graph of sidewalk lanes, crossings from
// crosswalks, and links across junction corners where sidewalks end near
// each other (see model/REFERENCES.md). Edges have no direction; a
// pedestrian walks one either way.
namespace simon::road {

struct WalkPoint final {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// A piece of the graph: from node `from` to node `to` along `path`, whose
// points are every meter or so along it, `distance` the length to each.
struct WalkEdge final {
  enum class Kind : std::uint8_t { SIDEWALK, CROSSING, CORNER };

  static constexpr std::uint32_t NONE = ~std::uint32_t{0};

  std::vector<WalkPoint> path;
  std::vector<double> distance;
  std::uint32_t from = 0;
  std::uint32_t to = 0;
  std::uint32_t crosswalk = NONE;  // The crossing's, by its place.
  Kind kind = Kind::SIDEWALK;

  auto length() const -> double { return distance.back(); }
};

// A crosswalk the graph crosses: its road and object, by their places,
// where across the road its crossing runs, and its depth along the road.
struct Crosswalk final {
  std::uint32_t road = 0;
  std::uint32_t object = 0;
  double s = 0.0;
  double t_from = 0.0;  // The middle of the sidewalk on one side,
  double t_to = 0.0;    // and on the other.
  double depth = 0.0;   // Along s.
};

// Where a driving lane meets a crosswalk: `near` and `far` meters into the
// lane in its direction of travel.
struct CrosswalkZone final {
  LaneKey lane;
  double near = 0.0;
  double far = 0.0;
  std::uint32_t crosswalk = 0;
};

// One edge of a route, walked from its `from` if `forward`, else from its
// `to`.
struct Leg final {
  std::uint32_t edge = 0;
  bool forward = true;
};

class WalkingGraph final {
 public:
  auto nodes() const -> std::span<const WalkPoint> { return nodes_; }
  auto edges() const -> std::span<const WalkEdge> { return edges_; }
  auto crosswalks() const -> std::span<const Crosswalk> { return crosswalks_; }

  // Every crosswalk zone, by lane and then along it.
  auto zones() const -> std::span<const CrosswalkZone> { return zones_; }

  // `lane`'s crosswalk zones, in order along it.
  auto zones_on(const LaneKey& lane) const -> std::span<const CrosswalkZone>;

  // The edges that meet at `node`, by their places.
  auto edges_at(std::uint32_t node) const -> std::span<const std::uint32_t>;

  // The shortest route from node `from` to node `to`, by Dijkstra's
  // algorithm; none if `to` cannot be reached, or is `from`.
  auto find_route(std::uint32_t from, std::uint32_t to) const
      -> std::vector<Leg>;

  // The point `along` meters from the start of `leg`, and the heading there,
  // counterclockwise from east.
  auto locate(const Leg& leg, double along) const
      -> std::pair<WalkPoint, double>;

  // Which nodes can reach each other: each node's component, by number.
  auto find_components() const -> std::vector<std::uint32_t>;

 private:
  friend auto build_walking_graph(const Map& network, double corner_reach)
      -> WalkingGraph;

  std::vector<WalkPoint> nodes_;
  std::vector<WalkEdge> edges_;
  std::vector<Crosswalk> crosswalks_;
  std::vector<CrosswalkZone> zones_;
  LaneNumbering numbering_;
  LaneRanges zone_ranges_;            // Into zones_, by lane number.
  std::vector<std::uint32_t> first_;  // Each node's edges' start; one more.
  std::vector<std::uint32_t> at_;
};

// The walking graph of `network`: each sidewalk lane's middle, split where a
// crosswalk meets it; a crossing along each crosswalk between the sidewalks
// nearest the road on either side; and a corner link between sidewalk ends
// of different roads within `corner_reach` of each other whose straight
// line crosses no driving lane. Ends within 0.5 m are one node. Each
// crosswalk's zone is on every driving lane between its sidewalks, its depth
// its object's length, 3 m if it has none.
auto build_walking_graph(const Map& network, double corner_reach = 25.0)
    -> WalkingGraph;

}  // namespace simon::road
