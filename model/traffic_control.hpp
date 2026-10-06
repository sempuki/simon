// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows Eclipse SUMO 1.27.1 (Copyright (C) 2001-2026 DLR and others, EPL-2.0
// OR GPL-2.0-or-later) and MovSim (GPL-3.0-or-later); translated to C++ and
// changed. See NOTICE.md.

#pragma once

#include <chrono>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "model/lane_graph.hpp"
#include "model/road.hpp"
#include "model/traffic.hpp"
#include "model/units.hpp"

// Traffic lights (see model/REFERENCES.md): fixed-time plans for groups of
// signals, the stop lines their signals put on lanes, and what a driver does
// at a light that is not green, by the rule of Treiber and Kesting's movsim.
namespace simon::model {

// What a signal group shows.
enum class Aspect : std::uint8_t { RED, YELLOW, GREEN };

// A fixed-time plan for one signal group: its phases in order, repeating
// every cycle from `offset`. Each phase holds from its start, inclusive, to
// the next's, exclusive.
struct SignalPlan final {
  struct Phase final {
    std::chrono::nanoseconds duration{};
    Aspect aspect = Aspect::RED;
  };

  // The sum of the phases' durations.
  auto cycle() const -> std::chrono::nanoseconds;

  // What the group shows at `time` from the start of the run.
  auto aspect_at(std::chrono::nanoseconds time) const -> Aspect;

  // How long after `time` the group goes on showing what it shows then: to
  // the end of the run of phases with its aspect, at most a cycle.
  auto keeps_aspect(std::chrono::nanoseconds time) const
      -> std::chrono::nanoseconds;

  std::vector<Phase> phases;
  std::chrono::nanoseconds offset{};
};

// Plans for `groups` groups that take turns: each in its turn shows green for
// `green` and yellow for `yellow`, then every group shows red for `all_red`
// before the next turn, and the first turn starts at 0.
auto plan_in_turn(std::size_t groups, std::chrono::nanoseconds green,
                  std::chrono::nanoseconds yellow,
                  std::chrono::nanoseconds all_red) -> std::vector<SignalPlan>;

// Signals that change together: an OpenDRIVE controller, the junction that
// lists it, if any, and its place in that junction's order.
struct SignalGroup final {
  std::string controller;
  std::string junction;
  std::uint32_t sequence = 0;
};

// Where a lane's traffic stops for a signal group: `along` meters into the
// lane in its direction of travel.
struct StopLine final {
  LaneKey lane;
  double along = 0.0;
  std::uint32_t group = 0;
};

// A network's signal groups and their stop lines.
class TrafficControl final {
 public:
  auto groups() const -> std::span<const SignalGroup> { return groups_; }

  // Every stop line, by lane and then along it.
  auto stop_lines() const -> std::span<const StopLine> { return stop_lines_; }

  // `lane`'s stop lines, in order along it.
  auto stop_lines_on(const LaneKey& lane) const -> std::span<const StopLine>;

 private:
  friend auto build_traffic_control(const RoadNetwork& network)
      -> TrafficControl;

  std::vector<SignalGroup> groups_;
  std::vector<StopLine> stop_lines_;
  LaneNumbering numbering_;
  LaneRanges lines_;  // Into stop_lines_, by lane number.
};

// The signal groups of `network`, one per controller, in its order, and a
// stop line for each driving lane each controlled traffic light holds for:
// the lanes its orientation runs on, those its validities name if it has any.
// A light no controller lists stops no one.
auto build_traffic_control(const RoadNetwork& network) -> TrafficControl;

// How hard a driver will brake for a light: up to `yellow` to stop at a
// yellow one, if it could stop at all at `kinematic`; and at a red one, up to
// `maximum`, all it can; movsim's values. It stops `line_gap` short of the
// stop line, SUMO's default.
struct LightBraking final {
  AccelerationMagnitude yellow = 4.0 * meter_per_second_squared;
  AccelerationMagnitude kinematic = 6.0 * meter_per_second_squared;
  AccelerationMagnitude maximum = 9.0 * meter_per_second_squared;
  Length line_gap = 1.0 * meter;
};

// The acceleration of a driver at `speed` stopping at a line `distance`
// ahead: the Intelligent Driver Model's behind a standing leader at the line,
// with no minimum gap, as SUMO stops its IDM drivers at lines. Within a
// centimeter of the line it stops at once, its braking unbounded, and stays.
// It comes to rest with its front at the line and never past it.
auto compute_stop_acceleration(const IntelligentDriver& driver, Speed speed,
                               Length distance) -> AccelerationMagnitude;

// Whether a driver at `speed`, `distance` before the stop line of a light
// showing `aspect`, stops for it. It stops for red, unless it cannot stop
// even at its maximum braking, and for yellow, if its braking to the line
// stays below `braking.yellow` and it could stop at the kinematic rate. Green
// stops no one.
auto stops_at_light(const IntelligentDriver& driver,
                    const LightBraking& braking, Aspect aspect, Speed speed,
                    Length distance) -> bool;

}  // namespace simon::model
