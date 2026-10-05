// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <compare>
#include <cstdint>
#include <span>
#include <string_view>
#include <utility>
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

// Every lane of a network numbered densely, road by road and section by
// section, each section's lanes in order of id, so a lane's number comes from
// its key at once and orders lanes as their keys do. Ids in a section run
// from -1 and 1 outward without gaps, as OpenDRIVE has them.
class LaneNumbering final {
 public:
  static constexpr std::uint32_t NONE = ~std::uint32_t{0};

  LaneNumbering() = default;
  explicit LaneNumbering(const RoadNetwork& network);

  auto size() const -> std::uint32_t { return count_; }

  // `key`'s number; NONE if the network has no such lane.
  auto number_of(const LaneKey& key) const -> std::uint32_t {
    if (key.road + 1 >= first_section_.size()) {
      return NONE;
    }
    std::uint32_t at = first_section_[key.road] + key.section;
    if (at >= first_section_[key.road + 1]) {
      return NONE;
    }
    const Section& section = sections_[at];
    if (key.lane == 0 || key.lane < -section.right || key.lane > section.left) {
      return NONE;
    }
    return section.first +
           static_cast<std::uint32_t>(key.lane + section.right) -
           (key.lane > 0 ? 1 : 0);
  }

 private:
  struct Section final {
    std::uint32_t first = 0;  // Its outermost right lane's number.
    std::int32_t right = 0;   // Lanes right of the reference line.
    std::int32_t left = 0;
  };

  std::vector<std::uint32_t> first_section_;  // Each road's; one more.
  std::vector<Section> sections_;
  std::uint32_t count_ = 0;
};

// Where each lane's items lie in a list sorted by lane: for each lane by
// number, its first item and one past its last.
class LaneRanges final {
 public:
  LaneRanges() = default;

  // The ranges of `count` items, the `i`th in lane `lane_of(i)`, sorted.
  template <typename LaneOf>
  LaneRanges(const LaneNumbering& numbering, std::size_t count, LaneOf lane_of)
      : first_(numbering.size() + 1, 0) {
    for (std::size_t i = 0; i < count; ++i) {
      std::uint32_t number = numbering.number_of(lane_of(i));
      if (number != LaneNumbering::NONE) {
        ++first_[number + 1];
      }
    }
    for (std::size_t n = 1; n < first_.size(); ++n) {
      first_[n] += first_[n - 1];
    }
  }

  // Lane `number`'s first item and one past its last; none for NONE.
  auto range_of(std::uint32_t number) const
      -> std::pair<std::uint32_t, std::uint32_t> {
    if (number + 1 >= first_.size()) {
      return {0, 0};
    }
    return {first_[number], first_[number + 1]};
  }

 private:
  std::vector<std::uint32_t> first_;
};

class LaneGraph final {
 public:
  // The lanes traffic moves into from `lane`, in order; none if it has none or
  // is not in the graph.
  auto successors_of(const LaneKey& lane) const -> std::span<const LaneKey>;

  // The lanes traffic moves into `lane` from, in order; more than one where
  // lanes merge.
  auto predecessors_of(const LaneKey& lane) const -> std::span<const LaneKey>;

  // Every edge of the graph, from a lane to a successor, in order.
  struct Edge final {
    LaneKey from;
    LaneKey to;
  };
  auto edges() const -> std::vector<Edge>;

  // The network's lanes, numbered.
  auto numbering() const -> const LaneNumbering& { return numbering_; }

 private:
  friend auto build_graph(const LaneNumbering& numbering,
                          std::vector<std::pair<LaneKey, LaneKey>> edges)
      -> LaneGraph;

  LaneNumbering numbering_;
  LaneRanges successors_;    // Into to_, by lane number.
  LaneRanges predecessors_;  // Into before_, by lane number.

  std::vector<LaneKey> from_;         // Each lane with successors, in order.
  std::vector<std::uint32_t> first_;  // Its successors' start; one more.
  std::vector<LaneKey> to_;
  // The same, the other way.
  std::vector<LaneKey> into_;  // Each lane with predecessors, in order.
  std::vector<std::uint32_t> first_from_;
  std::vector<LaneKey> before_;
};

// Where `key`'s lane section ends in s: at the next section's start, or at
// the road's end.
auto find_section_end(const RoadNetwork& network, const LaneKey& key) -> double;

// The length of `key`'s lane, from its section's start to its end.
auto find_lane_length(const RoadNetwork& network, const LaneKey& key) -> double;

// The s of `along` meters along `key`'s lane, in the direction of travel.
auto find_s_along(const RoadNetwork& network, const LaneKey& key, double along)
    -> double;

// The lane `key` names.
auto find_lane(const RoadNetwork& network, const LaneKey& key) -> const Lane&;

// The t of the middle of `key`'s lane at `s`, between its borders.
auto compute_lane_middle(const RoadNetwork& network, const LaneKey& key,
                         Length s) -> Length;

// The lane graph of every lane of `network`, of every type. A link to a lane
// or road the network lacks adds no edge.
auto build_lane_graph(const RoadNetwork& network) -> LaneGraph;

// The lane graph of `network`'s lanes of `type` alone, such as "driving",
// each lane's successors in the same order as in the whole graph.
auto build_lane_graph(const RoadNetwork& network, std::string_view type)
    -> LaneGraph;

}  // namespace simon::model
