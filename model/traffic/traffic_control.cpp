// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows Eclipse SUMO 1.27.1 (Copyright (C) 2001-2026 DLR and others, EPL-2.0
// OR GPL-2.0-or-later) and MovSim (GPL-3.0-or-later); translated to C++ and
// changed. See NOTICE.md.

#include "model/traffic/traffic_control.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <tuple>

#include "base/core.hpp"

namespace simon::traffic {

namespace {

// How far into `lane`, in its direction of travel, `s` is.
auto along_lane(const road::Map& network, const road::LaneKey& lane, double s)
    -> double {
  return road::runs_with_s(lane)
             ? s - network.roads[lane.road].lane_sections[lane.section].s0
             : road::find_section_end(network, lane) - s;
}

// Whether `signal` holds for traffic in lane `id`.
auto holds_for(const road::Signal& signal, int id) -> bool {
  bool direction =
      signal.orientation == road::Orientation::BOTH ||
      (signal.orientation == road::Orientation::POSITIVE) == (id < 0);
  if (!direction) {
    return false;
  }
  return signal.validities.empty() ||
         std::ranges::any_of(signal.validities,
                             [&](const road::LaneValidity& valid) {
                               return valid.from <= id && id <= valid.to;
                             });
}

}  // namespace

auto SignalPlan::cycle() const -> std::chrono::nanoseconds {
  std::chrono::nanoseconds total{};
  for (const Phase& phase : phases) {
    total += phase.duration;
  }
  return total;
}

auto SignalPlan::aspect_at(std::chrono::nanoseconds time) const -> Aspect {
  std::chrono::nanoseconds length = cycle();
  CHECK_PRECONDITION(length > std::chrono::nanoseconds{});
  std::chrono::nanoseconds into = (time - offset) % length;
  if (into < std::chrono::nanoseconds{}) {
    into += length;
  }
  for (const Phase& phase : phases) {
    if (into < phase.duration) {
      return phase.aspect;
    }
    into -= phase.duration;
  }
  return phases.back().aspect;
}

auto SignalPlan::keeps_aspect(std::chrono::nanoseconds time) const
    -> std::chrono::nanoseconds {
  std::chrono::nanoseconds length = cycle();
  CHECK_PRECONDITION(length > std::chrono::nanoseconds{});
  std::chrono::nanoseconds into = (time - offset) % length;
  if (into < std::chrono::nanoseconds{}) {
    into += length;
  }
  // Find the phase in force, then add phases while the aspect holds.
  std::size_t at = 0;
  while (into >= phases[at].duration) {
    into -= phases[at].duration;
    ++at;
  }
  Aspect aspect = phases[at].aspect;
  std::chrono::nanoseconds left = phases[at].duration - into;
  for (std::size_t k = 1; k < phases.size(); ++k) {
    const Phase& next = phases[(at + k) % phases.size()];
    if (next.aspect != aspect) {
      return left;
    }
    left += next.duration;
  }
  return length;  // It always shows this.
}

auto plan_in_turn(std::size_t groups, std::chrono::nanoseconds green,
                  std::chrono::nanoseconds yellow,
                  std::chrono::nanoseconds all_red) -> std::vector<SignalPlan> {
  std::chrono::nanoseconds turn = green + yellow + all_red;
  auto count = static_cast<std::int64_t>(groups);
  std::vector<SignalPlan> plans;
  for (std::int64_t k = 0; k < count; ++k) {
    plans.push_back(
        SignalPlan{.phases = {{.duration = green, .aspect = Aspect::GREEN},
                              {.duration = yellow, .aspect = Aspect::YELLOW},
                              {.duration = turn * (count - 1) + all_red,
                               .aspect = Aspect::RED}},
                   .offset = turn * k});
  }
  return plans;
}

auto Signals::stop_lines_on(const road::LaneKey& lane) const
    -> std::span<const StopLine> {
  auto [first, last] = lines_.range_of(numbering_.number_of(lane));
  return std::span{stop_lines_}.subspan(first, last - first);
}

auto build_signals(const road::Map& network) -> Signals {
  Signals signals;
  std::map<std::string, std::uint32_t, std::less<>> group_of_signal;
  for (const road::SignalController& controller : network.controllers) {
    SignalGroup group{.controller = controller.id,
                      .sequence = controller.sequence};
    for (const road::Junction& junction : network.junctions) {
      auto listed = std::ranges::find(junction.controllers, controller.id,
                                      &road::JunctionController::id);
      if (listed != junction.controllers.end()) {
        group.junction = junction.id;
        group.sequence = listed->sequence;
        break;
      }
    }
    auto index = static_cast<std::uint32_t>(signals.groups_.size());
    for (const road::SignalController::Control& controlled :
         controller.controls) {
      group_of_signal.emplace(controlled.signal, index);
    }
    signals.groups_.push_back(std::move(group));
  }

  for (std::uint32_t r = 0; r < network.roads.size(); ++r) {
    const road::Road& road = network.roads[r];
    for (const road::Signal& signal : road.signals) {
      auto group = group_of_signal.find(signal.id);
      if (!signal.dynamic || group == group_of_signal.end()) {
        continue;
      }
      const road::LaneSection& section =
          find_lane_section(road, signal.s * meter);
      auto k = static_cast<std::uint32_t>(&section - road.lane_sections.data());
      for (const std::vector<road::Lane>* side :
           {&section.left, &section.right}) {
        for (const road::Lane& lane : *side) {
          if (lane.type != "driving" || !holds_for(signal, lane.id)) {
            continue;
          }
          road::LaneKey key{.road = r, .section = k, .lane = lane.id};
          signals.stop_lines_.push_back(
              StopLine{.lane = key,
                       .along = along_lane(network, key, signal.s),
                       .group = group->second});
        }
      }
    }
  }
  std::ranges::sort(signals.stop_lines_,
                    [](const StopLine& a, const StopLine& b) {
                      return std::tie(a.lane, a.along, a.group) <
                             std::tie(b.lane, b.along, b.group);
                    });
  signals.numbering_ = road::LaneNumbering{network};
  signals.lines_ = road::LaneRanges{
      signals.numbering_, signals.stop_lines_.size(),
      [&](std::size_t i) { return signals.stop_lines_[i].lane; }};
  return signals;
}

// SUMO's MSCFModel_IDM::stopSpeed, which leaves the minimum gap out of the
// desired gap for a stop, and stops within a centimeter of the line (see
// model/REFERENCES.md). With no minimum gap a driver at rest wants no gap at
// all, so it would creep to the line and over it.
auto compute_stop_acceleration(const IntelligentDriver& driver, Speed speed,
                               Length distance) -> AccelerationMagnitude {
  constexpr Length CENTIMETER = 0.01 * meter;
  if (distance < CENTIMETER) {
    // Stops at once: no braking is too hard, so a step stops it.
    return (speed > 0.0 * meter_per_second
                ? -std::numeric_limits<double>::infinity()
                : 0.0) *
           meter_per_second_squared;
  }
  IntelligentDriver stopping = driver;
  stopping.minimum_gap = 0.0 * meter;
  return compute_idm_acceleration(
      stopping, speed,
      Leader{.gap = distance, .speed = 0.0 * meter_per_second});
}

// movsim's TrafficLightApproaching for yellow: a driver passes when its
// braking to the line is at least the yellow braking or it could not stop at
// the kinematic rate (see model/REFERENCES.md). At red it passes only when it
// could not stop at its maximum braking; movsim also passes when the IDM's
// braking passes that, which near the line, with no minimum gap, a creeping
// driver's does.
auto stops_at_light(const IntelligentDriver& driver,
                    const LightBraking& braking, Aspect aspect, Speed speed,
                    Length distance) -> bool {
  if (aspect == Aspect::GREEN) {
    return false;
  }
  double v = speed.numerical_value_in(meter_per_second);
  double d = distance.numerical_value_in(meter);
  double toward =
      std::min(0.0, compute_stop_acceleration(driver, speed, distance)
                        .numerical_value_in(meter_per_second_squared));
  double limit = (aspect == Aspect::YELLOW ? braking.yellow : braking.maximum)
                     .numerical_value_in(meter_per_second_squared);
  double rate = (aspect == Aspect::YELLOW ? braking.kinematic : braking.maximum)
                    .numerical_value_in(meter_per_second_squared);
  bool too_hard = aspect == Aspect::YELLOW && toward <= -limit;
  bool too_close = v * v / (2.0 * rate) >= d;
  return !too_hard && !too_close;
}

}  // namespace simon::traffic
