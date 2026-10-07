// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/road/lane_graph.hpp"

#include <algorithm>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "base/core.hpp"

namespace simon::road {

namespace {

// Finds roads by id, once.
class RoadIndex final {
 public:
  explicit RoadIndex(const Map& network) : network_{&network} {
    for (std::size_t i = 0; i < network.roads.size(); ++i) {
      by_id_.emplace(network.roads[i].id, static_cast<std::uint32_t>(i));
    }
  }

  auto find(const std::string& id) const -> std::optional<std::uint32_t> {
    auto found = by_id_.find(id);
    if (found == by_id_.end()) {
      return std::nullopt;
    }
    return found->second;
  }

  auto road(std::uint32_t index) const -> const Road& {
    return network_->roads[index];
  }

 private:
  const Map* network_ = nullptr;
  std::map<std::string, std::uint32_t, std::less<>> by_id_;
};

auto has_lane(const LaneSection& section, int id) -> bool {
  if (id == 0) {
    return false;
  }
  const std::vector<Lane>& side = id > 0 ? section.left : section.right;
  return static_cast<std::size_t>(std::abs(id)) <= side.size();
}

// The lane section next to `section` of `road`: before it in s if `before`,
// else after it, within the road or across the road's link to another road.
// None at a junction or an unlinked end.
auto adjacent_section(const RoadIndex& roads, std::uint32_t road,
                      std::uint32_t section, bool before)
    -> std::optional<std::pair<std::uint32_t, std::uint32_t>> {
  const Road& here = roads.road(road);
  if (before && section > 0) {
    return std::pair{road, section - 1};
  }
  if (!before && section + 1 < here.lane_sections.size()) {
    return std::pair{road, section + 1};
  }
  const Link& link = before ? here.predecessor : here.successor;
  if (link.kind != Link::Kind::ROAD) {
    return std::nullopt;
  }
  std::optional<std::uint32_t> next = roads.find(link.id);
  if (!next || roads.road(*next).lane_sections.empty()) {
    return std::nullopt;
  }
  auto last =
      static_cast<std::uint32_t>(roads.road(*next).lane_sections.size() - 1);
  return std::pair{
      *next, link.contact == Link::Contact::START ? std::uint32_t{0} : last};
}

}  // namespace

auto find_section_end(const Map& network, const LaneKey& key) -> double {
  const Road& road = network.roads[key.road];
  return key.section + 1 < road.lane_sections.size()
             ? road.lane_sections[key.section + 1].s0
             : road.length;
}

auto find_lane_length(const Map& network, const LaneKey& key) -> double {
  return find_section_end(network, key) -
         network.roads[key.road].lane_sections[key.section].s0;
}

auto find_s_along(const Map& network, const LaneKey& key, double along)
    -> double {
  return runs_with_s(key)
             ? network.roads[key.road].lane_sections[key.section].s0 + along
             : find_section_end(network, key) - along;
}

auto find_lane(const Map& network, const LaneKey& key) -> const Lane& {
  const LaneSection& section =
      network.roads[key.road].lane_sections[key.section];
  CHECK_PRECONDITION(has_lane(section, key.lane));
  const std::vector<Lane>& side = key.lane > 0 ? section.left : section.right;
  return side[static_cast<std::size_t>(std::abs(key.lane)) - 1];
}

auto compute_lane_middle(const Map& network, const LaneKey& key, Length s)
    -> Length {
  const Road& road = network.roads[key.road];
  const LaneSection& section = road.lane_sections[key.section];
  int inner = key.lane > 0 ? key.lane - 1 : key.lane + 1;
  return 0.5 * (compute_lane_border(road, section, s, key.lane) +
                compute_lane_border(road, section, s, inner));
}

LaneNumbering::LaneNumbering(const Map& network) {
  for (const Road& road : network.roads) {
    first_section_.push_back(static_cast<std::uint32_t>(sections_.size()));
    for (const LaneSection& section : road.lane_sections) {
      Section numbered{.first = count_,
                       .right = static_cast<std::int32_t>(section.right.size()),
                       .left = static_cast<std::int32_t>(section.left.size())};
      count_ += static_cast<std::uint32_t>(numbered.right + numbered.left);
      sections_.push_back(numbered);
    }
  }
  first_section_.push_back(static_cast<std::uint32_t>(sections_.size()));
}

auto LaneGraph::successors_of(const LaneKey& lane) const
    -> std::span<const LaneKey> {
  auto [first, last] = successors_.range_of(numbering_.number_of(lane));
  return std::span{to_}.subspan(first, last - first);
}

auto LaneGraph::predecessors_of(const LaneKey& lane) const
    -> std::span<const LaneKey> {
  auto [first, last] = predecessors_.range_of(numbering_.number_of(lane));
  return std::span{before_}.subspan(first, last - first);
}

auto LaneGraph::edges() const -> std::vector<Edge> {
  std::vector<Edge> all;
  for (std::size_t i = 0; i < from_.size(); ++i) {
    for (std::uint32_t j = first_[i]; j < first_[i + 1]; ++j) {
      all.push_back(Edge{.from = from_[i], .to = to_[j]});
    }
  }
  return all;
}

// A lane links by s to the lane in the section before it and after it. In
// the direction of travel, the lane before in s leads into a lane that runs
// with s and the lane after in s follows it; against s, the other way round.
// At a junction, each connection's lane links lead from the incoming road's
// section at the junction into the connecting road's section where it is
// entered.
auto build_graph(const LaneNumbering& numbering,
                 std::vector<std::pair<LaneKey, LaneKey>> edges) -> LaneGraph;

auto build_lane_graph(const Map& network) -> LaneGraph {
  RoadIndex roads{network};
  std::vector<std::pair<LaneKey, LaneKey>> edges;

  for (std::uint32_t r = 0; r < network.roads.size(); ++r) {
    const Road& road = network.roads[r];
    for (std::uint32_t k = 0; k < road.lane_sections.size(); ++k) {
      const LaneSection& section = road.lane_sections[k];
      for (const std::vector<Lane>* side : {&section.left, &section.right}) {
        for (const Lane& lane : *side) {
          LaneKey here{.road = r, .section = k, .lane = lane.id};
          bool with_s = runs_with_s(here);
          for (bool before : {true, false}) {
            const std::optional<int>& linked =
                before ? lane.predecessor : lane.successor;
            if (!linked) {
              continue;
            }
            auto next = adjacent_section(roads, r, k, before);
            if (!next ||
                !has_lane(roads.road(next->first).lane_sections[next->second],
                          *linked)) {
              continue;
            }
            LaneKey there{
                .road = next->first, .section = next->second, .lane = *linked};
            if (before == with_s) {
              edges.emplace_back(there, here);
            } else {
              edges.emplace_back(here, there);
            }
          }
        }
      }
    }
  }

  for (const Junction& junction : network.junctions) {
    for (const JunctionConnection& connection : junction.connections) {
      std::optional<std::uint32_t> incoming =
          roads.find(connection.incoming_road);
      std::optional<std::uint32_t> connecting =
          roads.find(connection.connecting_road);
      if (!incoming || !connecting) {
        continue;
      }
      const Road& in = roads.road(*incoming);
      const Road& through = roads.road(*connecting);
      if (in.lane_sections.empty() || through.lane_sections.empty()) {
        continue;
      }
      bool ends_here = in.successor.kind == Link::Kind::JUNCTION &&
                       in.successor.id == junction.id;
      auto in_section = static_cast<std::uint32_t>(
          ends_here ? in.lane_sections.size() - 1 : 0);
      auto through_section =
          static_cast<std::uint32_t>(connection.contact == Link::Contact::START
                                         ? 0
                                         : through.lane_sections.size() - 1);
      for (const JunctionConnection::LaneLink& link : connection.lane_links) {
        if (!has_lane(in.lane_sections[in_section], link.from) ||
            !has_lane(through.lane_sections[through_section], link.to)) {
          continue;
        }
        edges.emplace_back(
            LaneKey{
                .road = *incoming, .section = in_section, .lane = link.from},
            LaneKey{.road = *connecting,
                    .section = through_section,
                    .lane = link.to});
      }
    }
  }

  return build_graph(LaneNumbering{network}, std::move(edges));
}

auto build_lane_graph(const Map& network, std::string_view type) -> LaneGraph {
  std::vector<std::pair<LaneKey, LaneKey>> edges;
  for (const LaneGraph::Edge& edge : build_lane_graph(network).edges()) {
    if (find_lane(network, edge.from).type == type &&
        find_lane(network, edge.to).type == type) {
      edges.emplace_back(edge.from, edge.to);
    }
  }
  return build_graph(LaneNumbering{network}, std::move(edges));
}

// The graph of `edges`, each lane's successors and predecessors in order.
auto build_graph(const LaneNumbering& numbering,
                 std::vector<std::pair<LaneKey, LaneKey>> edges) -> LaneGraph {
  std::ranges::sort(edges);
  auto [end, _] = std::ranges::unique(edges);
  edges.erase(end, edges.end());

  LaneGraph graph;
  graph.numbering_ = numbering;
  graph.successors_ = LaneRanges{numbering, edges.size(),
                                 [&](std::size_t i) { return edges[i].first; }};
  for (const auto& [from, to] : edges) {
    if (graph.from_.empty() || graph.from_.back() != from) {
      graph.from_.push_back(from);
      graph.first_.push_back(static_cast<std::uint32_t>(graph.to_.size()));
    }
    graph.to_.push_back(to);
  }
  graph.first_.push_back(static_cast<std::uint32_t>(graph.to_.size()));
  std::ranges::sort(edges, {}, [](const std::pair<LaneKey, LaneKey>& edge) {
    return std::pair{edge.second, edge.first};
  });
  for (const auto& [from, to] : edges) {
    if (graph.into_.empty() || graph.into_.back() != to) {
      graph.into_.push_back(to);
      graph.first_from_.push_back(
          static_cast<std::uint32_t>(graph.before_.size()));
    }
    graph.before_.push_back(from);
  }
  graph.first_from_.push_back(static_cast<std::uint32_t>(graph.before_.size()));
  graph.predecessors_ = LaneRanges{
      numbering, edges.size(), [&](std::size_t i) { return edges[i].second; }};
  return graph;
}

}  // namespace simon::road
