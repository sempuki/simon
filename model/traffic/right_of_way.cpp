// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows Eclipse SUMO 1.27.1, Copyright (C) 2001-2026 DLR and others,
// EPL-2.0 OR GPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#include "model/traffic/right_of_way.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <tuple>
#include <utility>

namespace simon::traffic {

namespace {

constexpr double SAMPLE = 0.25;    // m between a lane's sampled middles.
constexpr double CLEARANCE = 2.0;  // m: two cars' bodies touch nearer.

struct Point final {
  double x = 0.0;
  double y = 0.0;
};

// A connecting lane: where traffic enters it from, where it leads, its
// middle sampled along it, and its heading in and out.
struct Movement final {
  road::LaneKey lane;
  road::LaneKey incoming;
  std::vector<road::LaneKey> exits;
  std::vector<Point> middle;  // Every SAMPLE m along, and at its end.
  double length = 0.0;
  double heading_in = 0.0;  // rad, in the direction of travel.
  double heading_out = 0.0;
};

auto build_movement(const road::Map& network, const road::LaneGraph& graph,
                    const road::LaneKey& lane, const road::LaneKey& incoming)
    -> Movement {
  Movement movement{.lane = lane, .incoming = incoming};
  std::span<const road::LaneKey> next = graph.successors_of(lane);
  movement.exits.assign(next.begin(), next.end());
  movement.length = road::find_lane_length(network, lane);
  const road::Road& road = network.roads[lane.road];
  auto point_at = [&](double along) {
    Length s = road::find_s_along(network, lane, along) * meter;
    Length t = road::compute_lane_middle(network, lane, s);
    Vector3 p =
        road::compute_position(road, s, t).numerical_value_in(meter).eigen();
    return Point{.x = p.x(), .y = p.y()};
  };
  for (double along = 0.0; along < movement.length; along += SAMPLE) {
    movement.middle.push_back(point_at(along));
  }
  movement.middle.push_back(point_at(movement.length));
  auto heading = [&](double along) {
    double h = road::compute_plan_point(
                   road, road::find_s_along(network, lane, along) * meter)
                   .heading;
    return road::runs_with_s(lane) ? h : h + std::numbers::pi;
  };
  movement.heading_in = heading(0.0);
  movement.heading_out = heading(movement.length);
  return movement;
}

// Where segments p0-p1 and q0-q1 cross, as fractions along each.
auto cross(Point p0, Point p1, Point q0, Point q1)
    -> std::optional<std::pair<double, double>> {
  double rx = p1.x - p0.x;
  double ry = p1.y - p0.y;
  double sx = q1.x - q0.x;
  double sy = q1.y - q0.y;
  double denominator = rx * sy - ry * sx;
  if (denominator == 0.0) {
    return std::nullopt;
  }
  double qpx = q0.x - p0.x;
  double qpy = q0.y - p0.y;
  double u = (qpx * sy - qpy * sx) / denominator;
  double v = (qpx * ry - qpy * rx) / denominator;
  if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) {
    return std::nullopt;
  }
  return std::pair{u, v};
}

// How far into `a` its middle first comes within CLEARANCE of `b`'s.
auto find_meeting(const Movement& a, const Movement& b) -> double {
  for (std::size_t i = 0; i < a.middle.size(); ++i) {
    for (const Point& q : b.middle) {
      if (std::hypot(a.middle[i].x - q.x, a.middle[i].y - q.y) < CLEARANCE) {
        return std::min(static_cast<double>(i) * SAMPLE, a.length);
      }
    }
  }
  return a.length;
}

// How far into `a` its middle is first CLEARANCE from all of `b`'s.
auto find_parting_along(const Movement& a, const Movement& b) -> double {
  for (std::size_t i = 0; i < a.middle.size(); ++i) {
    bool apart = std::ranges::all_of(b.middle, [&](const Point& q) {
      return std::hypot(a.middle[i].x - q.x, a.middle[i].y - q.y) >= CLEARANCE;
    });
    if (apart) {
      return std::min(static_cast<double>(i) * SAMPLE, a.length);
    }
  }
  return a.length;
}

// Where two movements' middles first cross, as along each.
auto find_crossing(const Movement& a, const Movement& b)
    -> std::optional<std::pair<double, double>> {
  auto along_of = [](const Movement& m, std::size_t i, double fraction) {
    double start = std::min(static_cast<double>(i) * SAMPLE, m.length);
    double end = std::min(static_cast<double>(i + 1) * SAMPLE, m.length);
    return start + fraction * (end - start);
  };
  for (std::size_t i = 0; i + 1 < a.middle.size(); ++i) {
    for (std::size_t j = 0; j + 1 < b.middle.size(); ++j) {
      if (auto at = cross(a.middle[i], a.middle[i + 1], b.middle[j],
                          b.middle[j + 1])) {
        return std::pair{along_of(a, i, at->first), along_of(b, j, at->second)};
      }
    }
  }
  return std::nullopt;
}

enum class Turn : std::uint8_t { LEFT, STRAIGHT, RIGHT };

auto classify_turn(const Movement& movement) -> Turn {
  double change = wrap(movement.heading_out - movement.heading_in);
  if (change > std::numbers::pi / 6.0) {
    return Turn::LEFT;
  }
  if (change < -std::numbers::pi / 6.0) {
    return Turn::RIGHT;
  }
  return Turn::STRAIGHT;
}

// Whether `road` has a give-way or stop sign for traffic in lane `lane`:
// Germany's 205 or 206, the catalog OpenDRIVE's examples use.
auto has_yield_sign(const road::Map& network, const road::LaneKey& lane)
    -> bool {
  return std::ranges::any_of(
      network.roads[lane.road].signals, [&](const road::Signal& signal) {
        bool kind =
            !signal.dynamic && (signal.type == "205" || signal.type == "206");
        bool direction = signal.orientation == road::Direction::BOTH ||
                         (signal.orientation == road::Direction::POSITIVE) ==
                             (lane.lane < 0);
        bool valid = signal.validities.empty() ||
                     std::ranges::any_of(
                         signal.validities, [&](const road::LaneValidity& v) {
                           return v.from <= lane.lane && lane.lane <= v.to;
                         });
        return kind && direction && valid;
      });
}

// The signal group of the first stop line on `lane`, if it has one.
auto find_group(const Control& control, const road::LaneKey& lane)
    -> std::optional<std::uint32_t> {
  std::span<const StopLine> lines = control.stop_lines_on(lane);
  if (lines.empty()) {
    return std::nullopt;
  }
  return lines.front().group;
}

// Whether `a` gives way to `b`, and why.
auto who_yields(const road::Map& network, const road::Junction& junction,
                const Movement& a, const Movement& b)
    -> std::pair<bool, Yielding> {
  const std::string& road_a = network.roads[a.lane.road].id;
  const std::string& road_b = network.roads[b.lane.road].id;
  for (const road::JunctionPriority& priority : junction.priorities) {
    if (priority.high == road_b && priority.low == road_a) {
      return std::pair{true, Yielding::PRIORITY};
    }
    if (priority.high == road_a && priority.low == road_b) {
      return std::pair{false, Yielding::PRIORITY};
    }
  }
  bool sign_a = has_yield_sign(network, a.incoming);
  bool sign_b = has_yield_sign(network, b.incoming);
  if (sign_a != sign_b) {
    return std::pair{sign_a, Yielding::SIGN};
  }
  double from = wrap(b.heading_in - a.heading_in);  // b's way from a's.
  bool opposite = std::abs(from) > 3.0 * std::numbers::pi / 4.0;
  if (opposite && classify_turn(a) == Turn::LEFT &&
      classify_turn(b) != Turn::LEFT) {
    return std::pair{true, Yielding::TURN};
  }
  if (opposite && classify_turn(b) == Turn::LEFT &&
      classify_turn(a) != Turn::LEFT) {
    return std::pair{false, Yielding::TURN};
  }
  if (from > std::numbers::pi / 4.0 && from < 3.0 * std::numbers::pi / 4.0) {
    return std::pair{true, Yielding::RIGHT};  // b comes from a's right.
  }
  if (from < -std::numbers::pi / 4.0 && from > -3.0 * std::numbers::pi / 4.0) {
    return std::pair{false, Yielding::RIGHT};
  }
  // Neither from the other's right, as two lanes from one side: the later
  // in order gives way, so the choice repeats.
  return std::pair{b.lane < a.lane, Yielding::RIGHT};
}

}  // namespace

auto RightOfWay::conflicts_on(const road::LaneKey& lane) const
    -> std::span<const Conflict> {
  auto [first, last] = on_.range_of(numbering_.number_of(lane));
  return std::span{conflicts_}.subspan(first, last - first);
}

auto RightOfWay::conflicts_against(const road::LaneKey& lane) const
    -> std::span<const std::uint32_t> {
  auto [first, last] = against_ranges_.range_of(numbering_.number_of(lane));
  return std::span{against_}.subspan(first, last - first);
}

auto RightOfWay::find_parting(const road::LaneKey& lane) const
    -> std::optional<double> {
  auto [first, last] = parting_ranges_.range_of(numbering_.number_of(lane));
  if (first == last) {
    return std::nullopt;
  }
  return partings_[first].second;
}

auto RightOfWay::find_merge(const road::LaneKey& lane) const
    -> std::optional<double> {
  auto [first, last] = merge_ranges_.range_of(numbering_.number_of(lane));
  if (first == last) {
    return std::nullopt;
  }
  return merges_[first].second;
}

auto RightOfWay::approaches_of(std::uint32_t index) const
    -> std::span<const Approach> {
  return std::span{approaches_}.subspan(first_[index],
                                        first_[index + 1] - first_[index]);
}

auto build_right_of_way(const road::Map& network, const road::LaneGraph& graph,
                        const Control& control, double reach) -> RightOfWay {
  std::vector<road::LaneGraph::Edge> edges = graph.edges();
  std::map<road::LaneKey, std::vector<road::LaneKey>> predecessors;
  for (const road::LaneGraph::Edge& edge : edges) {
    predecessors[edge.to].push_back(edge.from);
  }
  auto is_driving = [&](const road::LaneKey& lane) {
    return find_lane(network, lane).type == "driving";
  };

  RightOfWay right;
  for (const road::Junction& junction : network.junctions) {
    // Each driving lane entering the junction, from outside it.
    std::vector<Movement> movements;
    for (const road::LaneGraph::Edge& edge : edges) {
      const road::Road& to = network.roads[edge.to.road];
      const road::Road& from = network.roads[edge.from.road];
      if (to.junction == junction.id && from.junction != junction.id &&
          is_driving(edge.to) && is_driving(edge.from)) {
        movements.push_back(build_movement(network, graph, edge.to, edge.from));
      }
    }
    for (std::size_t i = 0; i < movements.size(); ++i) {
      for (std::size_t j = i + 1; j < movements.size(); ++j) {
        const Movement& a = movements[i];
        const Movement& b = movements[j];
        if (a.incoming == b.incoming) {
          // They part, and never meet.
          right.partings_.emplace_back(a.lane, find_parting_along(a, b));
          right.partings_.emplace_back(b.lane, find_parting_along(b, a));
          continue;
        }
        bool merge =
            std::ranges::any_of(a.exits, [&](const road::LaneKey& exit) {
              return std::ranges::find(b.exits, exit) != b.exits.end();
            });
        std::optional<std::pair<double, double>> at =
            merge ? std::pair{find_meeting(a, b), find_meeting(b, a)}
                  : find_crossing(a, b);
        if (!at) {
          continue;
        }
        if (merge) {
          right.merges_.emplace_back(a.lane, at->first);
          right.merges_.emplace_back(b.lane, at->second);
        }
        std::optional<std::uint32_t> group_a = find_group(control, a.incoming);
        std::optional<std::uint32_t> group_b = find_group(control, b.incoming);
        if (group_a && group_b && *group_a != *group_b) {
          for (bool a_keeps : {true, false}) {
            right.conflicts_.push_back(
                Conflict{.lane = a_keeps ? a.lane : b.lane,
                         .foe = a_keeps ? b.lane : a.lane,
                         .along = a_keeps ? at->first : at->second,
                         .foe_along = a_keeps ? at->second : at->first,
                         .why = Yielding::LIGHTS,
                         .merge = merge});
          }
          continue;
        }
        auto [a_yields, why] = who_yields(network, junction, a, b);
        const Movement& yielder = a_yields ? a : b;
        const Movement& foe = a_yields ? b : a;
        right.conflicts_.push_back(
            Conflict{.lane = yielder.lane,
                     .foe = foe.lane,
                     .along = a_yields ? at->first : at->second,
                     .foe_along = a_yields ? at->second : at->first,
                     .why = why,
                     .merge = merge});
      }
    }
  }
  // Each lane's farthest parting.
  std::ranges::sort(right.partings_, [](const auto& x, const auto& y) {
    return std::tie(x.first, y.second) < std::tie(y.first, x.second);
  });
  auto [last, __] = std::ranges::unique(
      right.partings_, {}, &std::pair<road::LaneKey, double>::first);
  right.partings_.erase(last, right.partings_.end());
  // Each lane's earliest meeting.
  std::ranges::sort(right.merges_);
  auto [end, _] = std::ranges::unique(right.merges_, {},
                                      &std::pair<road::LaneKey, double>::first);
  right.merges_.erase(end, right.merges_.end());
  std::ranges::sort(right.conflicts_, [](const Conflict& x, const Conflict& y) {
    return std::tie(x.lane, x.along, x.foe) < std::tie(y.lane, y.along, y.foe);
  });

  for (std::uint32_t c = 0; c < right.conflicts_.size(); ++c) {
    right.against_.push_back(c);
  }
  std::ranges::stable_sort(right.against_, {}, [&](std::uint32_t c) {
    return right.conflicts_[c].foe;
  });
  right.numbering_ = graph.numbering();
  right.on_ =
      road::LaneRanges{right.numbering_, right.conflicts_.size(),
                       [&](std::size_t i) { return right.conflicts_[i].lane; }};
  right.against_ranges_ = road::LaneRanges{
      right.numbering_, right.against_.size(),
      [&](std::size_t i) { return right.conflicts_[right.against_[i]].foe; }};
  right.merge_ranges_ =
      road::LaneRanges{right.numbering_, right.merges_.size(),
                       [&](std::size_t i) { return right.merges_[i].first; }};
  right.parting_ranges_ =
      road::LaneRanges{right.numbering_, right.partings_.size(),
                       [&](std::size_t i) { return right.partings_[i].first; }};

  // Each conflict's approaches, back from its foe lane, breadth first.
  for (const Conflict& conflict : right.conflicts_) {
    right.first_.push_back(
        static_cast<std::uint32_t>(right.approaches_.size()));
    auto root = static_cast<std::uint32_t>(right.approaches_.size());
    right.approaches_.push_back(
        Approach{.lane = conflict.foe, .to_conflict = conflict.foe_along});
    std::set<road::LaneKey> seen{conflict.foe};
    for (std::uint32_t k = root; k < right.approaches_.size(); ++k) {
      Approach here = right.approaches_[k];
      if (here.to_conflict >= reach) {
        continue;
      }
      auto found = predecessors.find(here.lane);
      if (found == predecessors.end()) {
        continue;
      }
      for (const road::LaneKey& before : found->second) {
        if (!is_driving(before) || !seen.insert(before).second) {
          continue;
        }
        right.approaches_.push_back(Approach{
            .lane = before,
            .to_conflict = find_lane_length(network, before) + here.to_conflict,
            .toward = k - root});
      }
    }
  }
  right.first_.push_back(static_cast<std::uint32_t>(right.approaches_.size()));
  return right;
}

auto compute_soonest_arrival(Length distance, Speed speed,
                             AccelerationMagnitude acceleration,
                             Speed top_speed) -> Time {
  double d = std::max(distance.numerical_value_in(meter), 0.0);
  double v = std::max(speed.numerical_value_in(meter_per_second), 0.0);
  double a = acceleration.numerical_value_in(meter_per_second_squared);
  double top = std::max(top_speed.numerical_value_in(meter_per_second), v);
  if (a <= 0.0 || v >= top) {
    return (v > 0.0 ? d / v : std::numeric_limits<double>::infinity()) * second;
  }
  // Speeding up to the top speed covers (top^2 - v^2) / 2a.
  double speeding = (top * top - v * v) / (2.0 * a);
  if (d <= speeding) {
    return (-v + std::sqrt(v * v + 2.0 * a * d)) / a * second;
  }
  return ((top - v) / a + (d - speeding) / top) * second;
}

}  // namespace simon::traffic
