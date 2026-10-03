// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <span>
#include <tuple>
#include <vector>

#include "application/automotive/simulation_components.hpp"
#include "engine/rate_gate.hpp"
#include "framework/system.hpp"
#include "framework/vocabulary.hpp"
#include "model/lane_graph.hpp"
#include "model/road.hpp"
#include "model/traffic.hpp"

namespace simon::automotive {

using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

//-- Lanes --------------------------------------------------------------------

// The lane's length, from its section's start to its end.
inline auto lane_length(const Network& network, const LaneKey& lane) -> double {
  return model::find_section_end(network.roads, lane) -
         network.roads.roads[lane.road].lane_sections[lane.section].s0;
}

// How far along its lane, in the direction of travel, `s` is.
inline auto along_lane(const Network& network, const LaneKey& lane, Length s)
    -> double {
  double at = s.numerical_value_in(model::meter);
  return model::runs_with_s(lane)
             ? at -
                   network.roads.roads[lane.road].lane_sections[lane.section].s0
             : model::find_section_end(network.roads, lane) - at;
}

// The s of `along` meters along `lane`, in the direction of travel.
inline auto s_along(const Network& network, const LaneKey& lane, double along)
    -> Length {
  double s =
      model::runs_with_s(lane)
          ? network.roads.roads[lane.road].lane_sections[lane.section].s0 +
                along
          : model::find_section_end(network.roads, lane) - along;
  return s * model::meter;
}

// The driving lane a vehicle enters from `lane`, its `turns`th: at a fork,
// one picked by its `seed`, and the same each time it is asked. None at a
// dead end.
inline auto choose_next_lane(const Network& network, const LaneKey& lane,
                             std::uint64_t seed, std::uint32_t turns)
    -> std::optional<LaneKey> {
  std::span<const LaneKey> next = network.graph.successors_of(lane);
  std::vector<LaneKey> driving;
  for (const LaneKey& key : next) {
    if (model::find_lane(network.roads, key).type == "driving") {
      driving.push_back(key);
    }
  }
  if (driving.empty()) {
    return std::nullopt;
  }
  // SplitMix64 of the seed and the turn (see model/REFERENCES.md).
  std::uint64_t z = seed + (std::uint64_t{turns} + 1) * 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  z ^= z >> 31;
  return driving[z % driving.size()];
}

// The driving lane beside `lane` in its section, to the right of travel if
// `right`, else to the left, on the same side of the reference line.
inline auto find_neighbor(const Network& network, const LaneKey& lane,
                          bool right) -> std::optional<LaneKey> {
  // Outward is to the right of travel on either side of the road.
  int outward = lane.lane > 0 ? 1 : -1;
  LaneKey beside = lane;
  beside.lane += right ? outward : -outward;
  const model::LaneSection& section =
      network.roads.roads[lane.road].lane_sections[lane.section];
  const std::vector<model::Lane>& side =
      beside.lane > 0 ? section.left : section.right;
  if (beside.lane == 0 ||
      static_cast<std::size_t>(std::abs(beside.lane)) > side.size() ||
      model::find_lane(network.roads, beside).type != "driving") {
    return std::nullopt;
  }
  return beside;
}

// Every vehicle by lane, in order along it, for finding leaders and
// followers. Collected before the step's decisions; the pointers stay valid
// until the next sync point.
class LaneOccupancy final {
 public:
  struct Occupant final {
    LaneKey lane;
    double along = 0.0;  // m along the lane, its front bumper.
    double speed = 0.0;  // m/s.
    std::uint32_t turns = 0;
    const Driver* driver = nullptr;
    Entity entity;
  };

  template <typename ProjectedWorldType>
  auto collect(ProjectedWorldType& world, const Network& network) -> void {
    occupants_.clear();
    const auto& drivers = world.template store_of<Driver>();
    world.template store_of<LaneState>().for_each(
        [&](Entity owner, const LaneState& state) {
          const Driver* driver = drivers.maybe_component_of(owner);
          if (!driver) {
            return;
          }
          occupants_.push_back(Occupant{
              .lane = state.lane,
              .along = along_lane(network, state.lane, state.s),
              .speed = state.speed.numerical_value_in(model::meter_per_second),
              .turns = state.turns,
              .driver = driver,
              .entity = owner});
        });
    std::ranges::sort(occupants_, [](const Occupant& a, const Occupant& b) {
      return std::tie(a.lane, a.along, a.entity) <
             std::tie(b.lane, b.along, b.entity);
    });
  }

  // The first vehicle in `lane` ahead of `along`, but `self`.
  auto find_ahead(const LaneKey& lane, double along, Entity self) const
      -> const Occupant* {
    auto [first, last] = in_lane(lane);
    for (auto it = std::upper_bound(first, last, along, by_along); it != last;
         ++it) {
      if (it->entity != self) {
        return &*it;
      }
    }
    return nullptr;
  }

  // The last vehicle in `lane` behind `along`, but `self`.
  auto find_behind(const LaneKey& lane, double along, Entity self) const
      -> const Occupant* {
    auto [first, last] = in_lane(lane);
    for (auto it = std::lower_bound(first, last, along, after_along);
         it != first;) {
      --it;
      if (it->entity != self) {
        return &*it;
      }
    }
    return nullptr;
  }

  // The vehicle nearest the start of `lane`.
  auto find_first(const LaneKey& lane) const -> const Occupant* {
    auto [first, last] = in_lane(lane);
    return first == last ? nullptr : &*first;
  }

 private:
  using Iterator = std::vector<Occupant>::const_iterator;

  static auto by_along(double along, const Occupant& occupant) -> bool {
    return along < occupant.along;
  }
  static auto after_along(const Occupant& occupant, double along) -> bool {
    return occupant.along < along;
  }

  auto in_lane(const LaneKey& lane) const -> std::pair<Iterator, Iterator> {
    auto first =
        std::ranges::lower_bound(occupants_, lane, {}, &Occupant::lane);
    auto last = std::ranges::upper_bound(occupants_, lane, {}, &Occupant::lane);
    return {first, last};
  }

  std::vector<Occupant> occupants_;
};

//-- Systems -------------------------------------------------------------------

// Each step, each driver finds its leader, in its lane or the lanes its way
// leads into, and accelerates by the Intelligent Driver Model; once a second
// it weighs changing to a lane beside its own by MOBIL. A dead end ahead is a
// leader standing still.
struct Decide final            //
    : System<DriveCommand,     //
             const LaneState,  //
             const Driver> {
  using SystemWorld = ProjectedWorld<Decide>;
  using AllowComponentList = TypeList<LaneState, Driver>;

  // How far ahead a driver looks for a leader across its lane's end.
  static constexpr double LOOKAHEAD = 250.0;  // m.

  explicit Decide(const Network& network) : network_{&network} {}

  auto prepare(SystemWorld& world, Step step) -> bool {
    occupancy_.collect(world, *network_);
    changing_ = gate_.fire(step).has_value();
    return true;
  }

  auto operator()(SystemWorld&, Entity self,  //
                  DriveCommand& command,      //
                  const LaneState* state,     //
                  const Driver* driver) const -> void {
    if (!state || !driver) {
      return;
    }
    double along = along_lane(*network_, state->lane, state->s);
    std::optional<model::Leader> leader =
        find_leader(state->lane, along, state->turns, driver->seed, self);
    auto now = model::compute_idm_acceleration(driver->following, state->speed,
                                               leader);
    command = DriveCommand{.acceleration = now};
    if (!changing_ || state->speed < 1.0 * model::meter_per_second) {
      return;
    }
    for (bool right : {true, false}) {
      std::optional<LaneKey> target =
          find_neighbor(*network_, state->lane, right);
      if (target && worth_changing(*state, *driver, self, along, now, leader,
                                   *target, right)) {
        command.change = target;
        command.acceleration = model::compute_idm_acceleration(
            driver->following, state->speed,
            find_leader(*target, along, state->turns, driver->seed, self));
        return;
      }
    }
  }

 private:
  // The leader of a vehicle `along` meters into `lane`: the next vehicle
  // ahead in the lane, else the first in the lanes its way leads into, within
  // the lookahead.
  auto find_leader(const LaneKey& lane, double along, std::uint32_t turns,
                   std::uint64_t seed, Entity self) const
      -> std::optional<model::Leader> {
    if (const LaneOccupancy::Occupant* ahead =
            occupancy_.find_ahead(lane, along, self)) {
      return gap_to(*ahead, ahead->along - along);
    }
    double distance = lane_length(*network_, lane) - along;
    LaneKey key = lane;
    while (distance < LOOKAHEAD) {
      std::optional<LaneKey> next =
          choose_next_lane(*network_, key, seed, turns++);
      if (!next) {
        return model::Leader{.gap = distance * model::meter,
                             .speed = 0.0 * model::meter_per_second};
      }
      key = *next;
      if (const LaneOccupancy::Occupant* first = occupancy_.find_first(key)) {
        return gap_to(*first, distance + first->along);
      }
      distance += lane_length(*network_, key);
    }
    return std::nullopt;
  }

  // A leader `apart` meters ahead, front bumper to front bumper.
  static auto gap_to(const LaneOccupancy::Occupant& leader, double apart)
      -> model::Leader {
    return model::Leader{
        .gap =
            (apart - leader.driver->length.numerical_value_in(model::meter)) *
            model::meter,
        .speed = leader.speed * model::meter_per_second};
  }

  // MOBIL's accelerations for changing from `state`'s lane to `target`: this
  // driver's, its old follower's and its new follower's, now and after. A
  // change onto a vehicle, or with one just behind, is never worth it.
  auto worth_changing(const LaneState& state, const Driver& driver, Entity self,
                      double along, AccelerationMagnitude now,
                      const std::optional<model::Leader>& leader,
                      const LaneKey& target, bool right) const -> bool {
    double length = driver.length.numerical_value_in(model::meter);
    model::LaneChangeAccelerations accelerations{.self_now = now};
    std::optional<model::Leader> new_leader =
        find_leader(target, along, state.turns, driver.seed, self);
    if (new_leader && new_leader->gap <= 0.0 * model::meter) {
      return false;
    }
    accelerations.self_after = model::compute_idm_acceleration(
        driver.following, state.speed, new_leader);

    if (const LaneOccupancy::Occupant* follower =
            occupancy_.find_behind(target, along, self)) {
      double gap = along - length - follower->along;
      if (gap <= 0.0) {
        return false;
      }
      Speed speed = follower->speed * model::meter_per_second;
      const model::IntelligentDriver& following = follower->driver->following;
      accelerations.new_follower_now = model::compute_idm_acceleration(
          following, speed,
          find_leader(target, follower->along, follower->turns,
                      follower->driver->seed, follower->entity));
      accelerations.new_follower_after = model::compute_idm_acceleration(
          following, speed,
          model::Leader{.gap = gap * model::meter, .speed = state.speed});
    }
    if (const LaneOccupancy::Occupant* follower =
            occupancy_.find_behind(state.lane, along, self)) {
      Speed speed = follower->speed * model::meter_per_second;
      const model::IntelligentDriver& following = follower->driver->following;
      accelerations.old_follower_now = model::compute_idm_acceleration(
          following, speed,
          model::Leader{
              .gap = (along - length - follower->along) * model::meter,
              .speed = state.speed});
      std::optional<model::Leader> after = leader;
      if (after) {
        after->gap += (along - follower->along) * model::meter;
      }
      accelerations.old_follower_after =
          model::compute_idm_acceleration(following, speed, after);
    }
    return model::decide_lane_change(driver.changing, accelerations, right);
  }

  const Network* network_ = nullptr;
  LaneOccupancy occupancy_;
  engine::RateGate gate_{1s};
  bool changing_ = false;
};

// Each step, each vehicle changes lane if its driver decided to, and moves
// along its lane at the acceleration it decided, held over the step, never
// backward. Past its lane's end it enters the next lane on its way, and at a
// dead end it stops.
struct Drive final                //
    : System<LaneState,           //
             const DriveCommand,  //
             const Driver> {
  using SystemWorld = ProjectedWorld<Drive>;
  using SequenceAfterSystemList = SystemList<Decide>;

  explicit Drive(const Network& network) : network_{&network} {}

  auto operator()(SystemWorld&, Entity,         //
                  LaneState& state,             //
                  const DriveCommand* command,  //
                  const Driver* driver,         //
                  Step step) const -> void {
    if (!command || !driver) {
      return;
    }
    if (command->change) {
      state.lane = *command->change;
    }
    double dt = std::chrono::duration<double>(step.dt).count();
    double v = state.speed.numerical_value_in(model::meter_per_second);
    double a = command->acceleration.numerical_value_in(
        model::meter_per_second_squared);
    double travel = 0.0;
    if (v + a * dt < 0.0) {  // Stops within the step.
      travel = v * v / (2.0 * -a);
      v = 0.0;
    } else {
      travel = v * dt + 0.5 * a * dt * dt;
      v += a * dt;
    }
    double along = along_lane(*network_, state.lane, state.s) + travel;
    double length = lane_length(*network_, state.lane);
    while (along > length) {
      std::optional<LaneKey> next =
          choose_next_lane(*network_, state.lane, driver->seed, state.turns);
      if (!next) {
        along = length;
        v = 0.0;
        break;
      }
      along -= length;
      ++state.turns;
      state.lane = *next;
      length = lane_length(*network_, state.lane);
    }
    state.s = s_along(*network_, state.lane, along);
    state.speed = v * model::meter_per_second;
  }

 private:
  const Network* network_ = nullptr;
};

// After Drive, each vehicle's place in the world follows its lane state: the
// middle of its lane at its s, heading along the road, or against it on the
// left.
struct FollowLane final    //
    : System<VehiclePose,  //
             const LaneState> {
  using SystemWorld = ProjectedWorld<FollowLane>;
  using SequenceAfterSystemList = SystemList<Drive>;

  explicit FollowLane(const Network& network) : network_{&network} {}

  auto operator()(SystemWorld&, Entity,  //
                  VehiclePose& pose,     //
                  const LaneState* state) const -> void {
    if (!state) {
      return;
    }
    pose = locate_vehicle(*network_, *state);
  }

  // The pose of a vehicle in `state`.
  static auto locate_vehicle(const Network& network, const LaneState& state)
      -> VehiclePose {
    const model::Road& road = network.roads.roads[state.lane.road];
    Length middle =
        model::compute_lane_middle(network.roads, state.lane, state.s);
    double heading = model::compute_plan_point(road, state.s).heading;
    if (!model::runs_with_s(state.lane)) {
      heading += std::numbers::pi;
    }
    return VehiclePose{
        .position = model::compute_road_position(road, state.s, middle),
        .heading = heading * model::radian};
  }

 private:
  const Network* network_ = nullptr;
};

using Schedule = SystemList<Decide, Drive, FollowLane>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::automotive
