// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "core/units.hpp"
#include "model/lane_graph.hpp"
#include "model/road.hpp"
#include "model/traffic_control.hpp"

// Right of way in junctions (see model/REFERENCES.md): where two connecting
// lanes cross or merge, which of them gives way, and the lanes that lead
// into each such conflict, for finding the vehicles that will reach it.
// Traffic keeps right.
namespace simon::model {

// Why one lane gives way to another where they meet.
enum class Yielding : std::uint8_t {
  PRIORITY,  // The junction's <priority> says so.
  SIGN,      // Its approach has a give-way or stop sign, and the other none.
  TURN,      // It turns left across the other's oncoming traffic.
  RIGHT,     // The other comes from its right.
  LIGHTS,    // Lights of different groups keep them apart; each waits only
             // for the other still in the junction, and both keep it.
};

// Where connecting lane `lane` meets connecting lane `foe` of the same
// junction: `along` meters into `lane` and `foe_along` into `foe`, in their
// directions of travel. They cross there, or, merging into one lane, their
// middles first come within a car's width there. Only the lane that gives
// way keeps the conflict.
struct Conflict final {
  LaneKey lane;
  LaneKey foe;
  double along = 0.0;
  double foe_along = 0.0;
  Yielding why = Yielding::RIGHT;
  bool merge = false;
};

// A lane from which traffic reaches a conflict's foe lane: `to_conflict`
// meters from the lane's start to the conflict, and the approach it leads
// into, toward the conflict, by its place among the conflict's approaches,
// if it is not the foe lane itself.
struct Approach final {
  static constexpr std::uint32_t NONE = ~std::uint32_t{0};

  LaneKey lane;
  double to_conflict = 0.0;
  std::uint32_t toward = NONE;
};

// A network's conflicts, by the lane that gives way and then along it, and
// for each the approaches to its foe lane within `reach`.
class RightOfWay final {
 public:
  auto conflicts() const -> std::span<const Conflict> { return conflicts_; }

  // `lane`'s conflicts, in order along it.
  auto conflicts_on(const LaneKey& lane) const -> std::span<const Conflict>;

  // The conflicts whose foe is `lane`, where traffic gives way to it, by
  // their places among all the conflicts.
  auto conflicts_against(const LaneKey& lane) const
      -> std::span<const std::uint32_t>;

  // The approaches to conflict `index`'s foe lane, the foe lane first.
  auto approaches_of(std::uint32_t index) const -> std::span<const Approach>;

  // How far into `lane` its middle first comes within a car's width of
  // another lane that merges with it, if one does.
  auto find_merge(const LaneKey& lane) const -> std::optional<double>;

  // How far into `lane` its middle is first a car's width from every other
  // lane leaving the same lane, if another does.
  auto find_parting(const LaneKey& lane) const -> std::optional<double>;

 private:
  friend auto build_right_of_way(const RoadNetwork& network,
                                 const LaneGraph& graph,
                                 const TrafficControl& control, double reach)
      -> RightOfWay;

  std::vector<Conflict> conflicts_;
  std::vector<std::uint32_t> first_;  // Each conflict's approaches' start.
  std::vector<Approach> approaches_;
  std::vector<std::pair<LaneKey, double>> merges_;    // By lane.
  std::vector<std::pair<LaneKey, double>> partings_;  // By lane.
  std::vector<std::uint32_t> against_;                // Conflicts by foe lane.
  // Into conflicts_, against_, merges_ and partings_, by lane number.
  LaneNumbering numbering_;
  LaneRanges on_;
  LaneRanges against_ranges_;
  LaneRanges merge_ranges_;
  LaneRanges parting_ranges_;
};

// The conflicts of every junction of `network`: each pair of driving lanes
// on its connecting roads from different incoming lanes that cross, their
// middles sampled every 0.25 m, or that lead into the same lane. Who gives
// way is decided in order by the junction's priorities, by signals, which
// keep apart lanes whose lights are of different groups, by give-way and
// stop signs, by a left turn giving way to oncoming traffic, and by giving
// way to traffic from the right. Approaches reach back `reach` meters.
auto build_right_of_way(const RoadNetwork& network, const LaneGraph& graph,
                        const TrafficControl& control, double reach = 200.0)
    -> RightOfWay;

// How long a vehicle at `speed`, able to accelerate at `acceleration` up to
// `top_speed`, takes at the soonest to cover `distance`: speeding up to its
// top speed, and on at it.
auto compute_soonest_arrival(Length distance, Speed speed,
                             AccelerationMagnitude acceleration,
                             Speed top_speed) -> Time;

}  // namespace simon::model
