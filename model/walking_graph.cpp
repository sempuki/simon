// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/walking_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <span>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "base/core.hpp"

namespace simon::model {

namespace {

constexpr double SAMPLE = 1.0;  // m between a path's points, at most.
constexpr double SAME = 0.5;    // m: ends nearer are one node.

auto point_at(const Road& road, double s, double t) -> WalkPoint {
  Vector3 p = compute_road_position(road, s * meter, t * meter)
                  .numerical_value_in(meter)
                  .eigen();
  return WalkPoint{.x = p.x(), .y = p.y(), .z = p.z()};
}

auto apart(const WalkPoint& a, const WalkPoint& b) -> double {
  return std::hypot(a.x - b.x, a.y - b.y);
}

// The middle of lane `id` of `road` at s.
auto lane_middle(const Road& road, double s, int id) -> double {
  int inner = id > 0 ? id - 1 : id + 1;
  return 0.5 *
         (compute_lane_border(road, s * meter, id).numerical_value_in(meter) +
          compute_lane_border(road, s * meter, inner)
              .numerical_value_in(meter));
}

// The sidewalk nearest the reference line on the side of `sign`, in the lane
// section in force at s, if there is one.
auto nearest_sidewalk(const LaneSection& section, int sign)
    -> std::optional<int> {
  const std::vector<Lane>& side = sign > 0 ? section.left : section.right;
  for (const Lane& lane : side) {
    if (lane.type == "sidewalk") {
      return lane.id;
    }
  }
  return std::nullopt;
}

// Whether segments a0-a1 and b0-b1 cross in the plane.
auto crosses(const WalkPoint& a0, const WalkPoint& a1, const WalkPoint& b0,
             const WalkPoint& b1) -> bool {
  auto side = [](const WalkPoint& p, const WalkPoint& q, const WalkPoint& r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
  };
  double d1 = side(b0, b1, a0);
  double d2 = side(b0, b1, a1);
  double d3 = side(a0, a1, b0);
  double d4 = side(a0, a1, b1);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

// Segments by the square cells their bounding boxes touch, so that a
// segment is tested against those near it alone; two segments that cross
// share the cell of the point where they do.
class SegmentGrid final {
 public:
  SegmentGrid(std::span<const std::pair<WalkPoint, WalkPoint>> segments,
              double cell)
      : segments_{segments}, cell_{cell} {
    for (std::uint32_t i = 0; i < segments.size(); ++i) {
      visit(segments[i].first, segments[i].second,
            [&](std::int64_t key) { cells_[key].push_back(i); });
    }
  }

  // Whether a0-a1 crosses any of the segments.
  auto crosses_any(const WalkPoint& a0, const WalkPoint& a1) const -> bool {
    bool found = false;
    visit(a0, a1, [&](std::int64_t key) {
      auto cell = cells_.find(key);
      if (found || cell == cells_.end()) {
        return;
      }
      for (std::uint32_t i : cell->second) {
        if (crosses(a0, a1, segments_[i].first, segments_[i].second)) {
          found = true;
          return;
        }
      }
    });
    return found;
  }

 private:
  // Calls `f` with the key of each cell the box of p-q touches.
  template <typename F>
  auto visit(const WalkPoint& p, const WalkPoint& q, F f) const -> void {
    auto index = [&](double v) {
      return static_cast<std::int64_t>(std::floor(v / cell_));
    };
    for (std::int64_t i = index(std::min(p.x, q.x));
         i <= index(std::max(p.x, q.x)); ++i) {
      for (std::int64_t j = index(std::min(p.y, q.y));
           j <= index(std::max(p.y, q.y)); ++j) {
        f(i * 0x100000000LL + j);
      }
    }
  }

  std::span<const std::pair<WalkPoint, WalkPoint>> segments_;
  double cell_ = 1.0;
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> cells_;
};

// Builds edges and merges their ends into nodes.
class Builder final {
 public:
  // The node at `p`: an existing one within SAME, or a new one.
  auto node_at(const WalkPoint& p) -> std::uint32_t {
    for (std::uint32_t n = 0; n < nodes_.size(); ++n) {
      if (apart(nodes_[n], p) < SAME) {
        return n;
      }
    }
    nodes_.push_back(p);
    return static_cast<std::uint32_t>(nodes_.size() - 1);
  }

  auto add(std::vector<WalkPoint> path, WalkEdge::Kind kind,
           std::uint32_t crosswalk = WalkEdge::NONE) -> void {
    WalkEdge edge{
        .path = std::move(path), .crosswalk = crosswalk, .kind = kind};
    edge.distance.push_back(0.0);
    for (std::size_t i = 1; i < edge.path.size(); ++i) {
      edge.distance.push_back(edge.distance.back() +
                              apart(edge.path[i - 1], edge.path[i]));
    }
    if (edge.length() <= 0.0) {
      return;
    }
    edge.from = node_at(edge.path.front());
    edge.to = node_at(edge.path.back());
    if (edge.from != edge.to) {
      edges_.push_back(std::move(edge));
    }
  }

  std::vector<WalkPoint> nodes_;
  std::vector<WalkEdge> edges_;
};

}  // namespace

auto WalkingGraph::edges_at(std::uint32_t node) const
    -> std::span<const std::uint32_t> {
  return std::span{at_}.subspan(first_[node], first_[node + 1] - first_[node]);
}

auto WalkingGraph::zones_on(const LaneKey& lane) const
    -> std::span<const CrosswalkZone> {
  auto [first, last] = zone_ranges_.range_of(numbering_.number_of(lane));
  return std::span{zones_}.subspan(first, last - first);
}

auto WalkingGraph::find_route(std::uint32_t from, std::uint32_t to) const
    -> std::vector<Leg> {
  constexpr double FAR = std::numeric_limits<double>::infinity();
  std::vector<double> best(nodes_.size(), FAR);
  std::vector<std::uint32_t> via(nodes_.size(), WalkEdge::NONE);
  using Entry = std::pair<double, std::uint32_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;
  best[from] = 0.0;
  open.emplace(0.0, from);
  while (!open.empty()) {
    auto [cost, node] = open.top();
    open.pop();
    if (cost > best[node]) {
      continue;
    }
    if (node == to) {
      break;
    }
    for (std::uint32_t e : edges_at(node)) {
      const WalkEdge& edge = edges_[e];
      std::uint32_t next = edge.from == node ? edge.to : edge.from;
      double reach = cost + edge.length();
      if (reach < best[next]) {
        best[next] = reach;
        via[next] = e;
        open.emplace(reach, next);
      }
    }
  }
  std::vector<Leg> route;
  if (from == to || best[to] == FAR) {
    return route;
  }
  for (std::uint32_t node = to; node != from;) {
    const WalkEdge& edge = edges_[via[node]];
    bool forward = edge.to == node;
    route.push_back(Leg{.edge = via[node], .forward = forward});
    node = forward ? edge.from : edge.to;
  }
  std::ranges::reverse(route);
  return route;
}

auto WalkingGraph::locate(const Leg& leg, double along) const
    -> std::pair<WalkPoint, double> {
  const WalkEdge& edge = edges_[leg.edge];
  double at = std::clamp(leg.forward ? along : edge.length() - along, 0.0,
                         edge.length());
  auto next = std::ranges::upper_bound(edge.distance, at);
  std::size_t i = std::clamp<std::size_t>(
      static_cast<std::size_t>(next - edge.distance.begin()), 1,
      edge.path.size() - 1);
  const WalkPoint& a = edge.path[i - 1];
  const WalkPoint& b = edge.path[i];
  double span = edge.distance[i] - edge.distance[i - 1];
  double f = span > 0.0 ? (at - edge.distance[i - 1]) / span : 0.0;
  WalkPoint p{.x = a.x + f * (b.x - a.x),
              .y = a.y + f * (b.y - a.y),
              .z = a.z + f * (b.z - a.z)};
  double heading = std::atan2(b.y - a.y, b.x - a.x);
  return {p, leg.forward ? heading : heading + std::numbers::pi};
}

auto WalkingGraph::find_components() const -> std::vector<std::uint32_t> {
  constexpr std::uint32_t UNSEEN = ~std::uint32_t{0};
  std::vector<std::uint32_t> component(nodes_.size(), UNSEEN);
  std::uint32_t count = 0;
  for (std::uint32_t start = 0; start < nodes_.size(); ++start) {
    if (component[start] != UNSEEN) {
      continue;
    }
    std::vector<std::uint32_t> stack{start};
    component[start] = count;
    while (!stack.empty()) {
      std::uint32_t node = stack.back();
      stack.pop_back();
      for (std::uint32_t e : edges_at(node)) {
        std::uint32_t next =
            edges_[e].from == node ? edges_[e].to : edges_[e].from;
        if (component[next] == UNSEEN) {
          component[next] = count;
          stack.push_back(next);
        }
      }
    }
    ++count;
  }
  return component;
}

auto build_walking_graph(const RoadNetwork& network, double corner_reach)
    -> WalkingGraph {
  WalkingGraph graph;
  Builder builder;

  // Crosswalks, between the sidewalks nearest the road on either side.
  std::map<LaneKey, std::vector<double>> splits;  // Sidewalk, s.
  for (std::uint32_t r = 0; r < network.roads.size(); ++r) {
    const Road& road = network.roads[r];
    for (std::uint32_t o = 0; o < road.objects.size(); ++o) {
      const RoadObject& object = road.objects[o];
      if (object.type != "crosswalk") {
        continue;
      }
      const LaneSection& section = find_lane_section(road, object.s * meter);
      auto k = static_cast<std::uint32_t>(&section - road.lane_sections.data());
      std::optional<int> left = nearest_sidewalk(section, 1);
      std::optional<int> right = nearest_sidewalk(section, -1);
      if (!left || !right) {
        continue;
      }
      auto index = static_cast<std::uint32_t>(graph.crosswalks_.size());
      double depth = object.length > 0.0 ? object.length : 3.0;
      graph.crosswalks_.push_back(
          Crosswalk{.road = r,
                    .object = o,
                    .s = object.s,
                    .t_from = lane_middle(road, object.s, *right),
                    .t_to = lane_middle(road, object.s, *left),
                    .depth = depth});
      double s0 = section.s0;
      double s1 = find_section_end(network, {.road = r, .section = k});
      for (const std::vector<Lane>* side : {&section.left, &section.right}) {
        for (const Lane& lane : *side) {
          if (lane.type != "driving" ||
              std::abs(lane.id) >= std::abs(lane.id > 0 ? *left : *right)) {
            continue;
          }
          LaneKey key{.road = r, .section = k, .lane = lane.id};
          // Into the lane, the way it runs.
          auto into = [&](double s) {
            return runs_with_s(key) ? s - s0 : s1 - s;
          };
          double before = object.s - depth / 2.0;
          double after = object.s + depth / 2.0;
          graph.zones_.push_back(CrosswalkZone{
              .lane = key,
              .near = runs_with_s(key) ? into(before) : into(after),
              .far = runs_with_s(key) ? into(after) : into(before),
              .crosswalk = index});
        }
      }
      splits[{.road = r, .section = k, .lane = *left}].push_back(object.s);
      splits[{.road = r, .section = k, .lane = *right}].push_back(object.s);
    }
  }

  // Each sidewalk's middle, in pieces between its ends and crosswalks.
  std::vector<std::pair<WalkPoint, std::uint32_t>> ends;  // Point, road.
  for (std::uint32_t r = 0; r < network.roads.size(); ++r) {
    const Road& road = network.roads[r];
    for (std::uint32_t k = 0; k < road.lane_sections.size(); ++k) {
      const LaneSection& section = road.lane_sections[k];
      double s0 = section.s0;
      double s1 = find_section_end(network, {.road = r, .section = k});
      for (const std::vector<Lane>* side : {&section.left, &section.right}) {
        for (const Lane& lane : *side) {
          if (lane.type != "sidewalk") {
            continue;
          }
          std::vector<double> cuts{s0, s1};
          auto found = splits.find({.road = r, .section = k, .lane = lane.id});
          if (found != splits.end()) {
            cuts.insert(cuts.end(), found->second.begin(), found->second.end());
          }
          std::ranges::sort(cuts);
          for (std::size_t c = 0; c + 1 < cuts.size(); ++c) {
            double from = cuts[c];
            double to = cuts[c + 1];
            std::vector<WalkPoint> path;
            int pieces =
                std::max(1, static_cast<int>(std::ceil((to - from) / SAMPLE)));
            for (int i = 0; i <= pieces; ++i) {
              double s = from + (to - from) * i / pieces;
              // At the section's end the lane is the section's own.
              double inside = std::min(s, s1 - 1e-9);
              path.push_back(
                  point_at(road, s, lane_middle(road, inside, lane.id)));
            }
            builder.add(std::move(path), WalkEdge::Kind::SIDEWALK);
          }
          ends.emplace_back(point_at(road, s0, lane_middle(road, s0, lane.id)),
                            r);
          ends.emplace_back(
              point_at(road, s1, lane_middle(road, s1 - 1e-9, lane.id)), r);
        }
      }
    }
  }

  // Crossings, straight across the road.
  for (std::uint32_t c = 0; c < graph.crosswalks_.size(); ++c) {
    const Crosswalk& crosswalk = graph.crosswalks_[c];
    const Road& road = network.roads[crosswalk.road];
    builder.add({point_at(road, crosswalk.s, crosswalk.t_from),
                 point_at(road, crosswalk.s, crosswalk.t_to)},
                WalkEdge::Kind::CROSSING, c);
  }

  // Corners: sidewalk ends of different roads, near each other, that no
  // sidewalk joins yet, linked where the line between them crosses no
  // driving lane.
  std::vector<std::pair<WalkPoint, WalkPoint>> driving;  // Lane middles.
  for (const Road& road : network.roads) {
    for (std::uint32_t k = 0; k < road.lane_sections.size(); ++k) {
      const LaneSection& section = road.lane_sections[k];
      double s0 = section.s0;
      double s1 = k + 1 < road.lane_sections.size()
                      ? road.lane_sections[k + 1].s0
                      : road.length;
      for (const std::vector<Lane>* side : {&section.left, &section.right}) {
        for (const Lane& lane : *side) {
          if (lane.type != "driving") {
            continue;
          }
          int pieces =
              std::max(1, static_cast<int>(std::ceil((s1 - s0) / SAMPLE)));
          WalkPoint last;
          for (int i = 0; i <= pieces; ++i) {
            double s = s0 + (s1 - s0) * i / pieces;
            WalkPoint p = point_at(
                road, s, lane_middle(road, std::min(s, s1 - 1e-9), lane.id));
            if (i > 0) {
              driving.emplace_back(last, p);
            }
            last = p;
          }
        }
      }
    }
  }
  std::vector<int> degree(builder.nodes_.size(), 0);
  for (const WalkEdge& edge : builder.edges_) {
    ++degree[edge.from];
    ++degree[edge.to];
  }
  std::vector<std::pair<std::uint32_t, std::uint32_t>> dangling;  // Node, road.
  for (const auto& [point, road] : ends) {
    std::uint32_t node = builder.node_at(point);
    if (node < degree.size() && degree[node] == 1 &&
        std::ranges::find(dangling, std::pair{node, road}) == dangling.end()) {
      dangling.emplace_back(node, road);
    }
  }
  SegmentGrid near{driving, corner_reach};
  for (std::size_t i = 0; i < dangling.size(); ++i) {
    for (std::size_t j = i + 1; j < dangling.size(); ++j) {
      auto [a, road_a] = dangling[i];
      auto [b, road_b] = dangling[j];
      const WalkPoint pa = builder.nodes_[a];
      const WalkPoint pb = builder.nodes_[b];
      if (road_a == road_b || a == b || apart(pa, pb) >= corner_reach) {
        continue;
      }
      if (!near.crosses_any(pa, pb)) {
        builder.add({pa, pb}, WalkEdge::Kind::CORNER);
      }
    }
  }

  std::ranges::sort(
      graph.zones_, [](const CrosswalkZone& a, const CrosswalkZone& b) {
        return std::tie(a.lane, a.near) < std::tie(b.lane, b.near);
      });
  graph.numbering_ = LaneNumbering{network};
  graph.zone_ranges_ =
      LaneRanges{graph.numbering_, graph.zones_.size(),
                 [&](std::size_t i) { return graph.zones_[i].lane; }};
  graph.nodes_ = std::move(builder.nodes_);
  graph.edges_ = std::move(builder.edges_);
  std::vector<std::vector<std::uint32_t>> at(graph.nodes_.size());
  for (std::uint32_t e = 0; e < graph.edges_.size(); ++e) {
    at[graph.edges_[e].from].push_back(e);
    at[graph.edges_[e].to].push_back(e);
  }
  for (const std::vector<std::uint32_t>& edges : at) {
    graph.first_.push_back(static_cast<std::uint32_t>(graph.at_.size()));
    graph.at_.insert(graph.at_.end(), edges.begin(), edges.end());
  }
  graph.first_.push_back(static_cast<std::uint32_t>(graph.at_.size()));
  return graph;
}

}  // namespace simon::model
