// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <compare>
#include <cstdint>
#include <span>
#include <vector>

#include "model/road.hpp"

// Which lanes traffic moves into from each lane, as ASAM OpenDRIVE links them
// (see model/REFERENCES.md): across lane sections within a road, across road
// links at a road's ends, and from incoming roads into a junction's
// connecting roads. Traffic keeps right: a lane right of the reference line
// (negative id) runs with s, and one left of it against s.
namespace simon::model {

// A lane of a network: its road and lane section, by index, and its id.
struct LaneKey final {
  friend auto operator<=>(const LaneKey&, const LaneKey&) = default;

  std::uint32_t road = 0;
  std::uint32_t section = 0;
  std::int32_t lane = 0;
};

// Whether traffic in `lane` moves with s.
inline auto runs_with_s(const LaneKey& lane) -> bool { return lane.lane < 0; }

class LaneGraph final {
 public:
  // The lanes traffic moves into from `lane`, in order; none if it has none or
  // is not in the graph.
  auto successors_of(const LaneKey& lane) const -> std::span<const LaneKey>;

  // Every edge of the graph, from a lane to a successor, in order.
  struct Edge final {
    LaneKey from;
    LaneKey to;
  };
  auto edges() const -> std::vector<Edge>;

 private:
  friend auto build_lane_graph(const RoadNetwork& network) -> LaneGraph;

  std::vector<LaneKey> from_;         // Each lane with successors, in order.
  std::vector<std::uint32_t> first_;  // Its successors' start; one more.
  std::vector<LaneKey> to_;
};

// Where `key`'s lane section ends in s: at the next section's start, or at
// the road's end.
auto find_section_end(const RoadNetwork& network, const LaneKey& key) -> double;

// The lane `key` names.
auto find_lane(const RoadNetwork& network, const LaneKey& key) -> const Lane&;

// The t of the middle of `key`'s lane at `s`, between its borders.
auto compute_lane_middle(const RoadNetwork& network, const LaneKey& key,
                         Length s) -> Length;

// The lane graph of every lane of `network`, of every type. A link to a lane
// or road the network lacks adds no edge.
auto build_lane_graph(const RoadNetwork& network) -> LaneGraph;

}  // namespace simon::model
