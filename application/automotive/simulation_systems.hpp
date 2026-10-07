// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

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
#include "core/random.hpp"
#include "engine/rate_gate.hpp"
#include "framework/component_store.hpp"
#include "framework/system.hpp"
#include "model/road/lane_graph.hpp"
#include "model/road/road.hpp"
#include "model/road/walking_graph.hpp"
#include "model/traffic/traffic.hpp"
#include "model/traffic/traffic_control.hpp"

namespace simon::automotive {

using framework::ComponentStore;
using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

//-- Lanes --------------------------------------------------------------------

// The lane's length, from its section's start to its end.
inline auto find_lane_length(const Network& network, const LaneKey& lane)
    -> double {
  return road::find_lane_length(network.map, lane);
}

// How far along its lane, in the direction of travel, `s` is.
inline auto along_lane(const Network& network, const LaneKey& lane, Length s)
    -> double {
  double at = s.numerical_value_in(meter);
  return road::runs_with_s(lane)
             ? at - network.map.roads[lane.road].lane_sections[lane.section].s0
             : road::find_section_end(network.map, lane) - at;
}

// The s of `along` meters along `lane`, in the direction of travel.
inline auto find_s_along(const Network& network, const LaneKey& lane,
                         double along) -> Length {
  return road::find_s_along(network.map, lane, along) * meter;
}

// The driving lane a vehicle enters from `lane`, its `turns`th: at a fork,
// one picked by its `seed`, and the same each time it is asked. None at a
// dead end.
inline auto choose_next_lane(const Network& network, const LaneKey& lane,
                             std::uint64_t seed, std::uint32_t turns)
    -> std::optional<LaneKey> {
  std::span<const LaneKey> driving = network.driving.successors_of(lane);
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
  const road::LaneSection& section =
      network.map.roads[lane.road].lane_sections[lane.section];
  const std::vector<road::Lane>& side =
      beside.lane > 0 ? section.left : section.right;
  if (beside.lane == 0 ||
      static_cast<std::size_t>(std::abs(beside.lane)) > side.size() ||
      road::find_lane(network.map, beside).type != "driving") {
    return std::nullopt;
  }
  return beside;
}

// Every vehicle by lane, in order along it, for finding leaders and
// followers. Collected before the step's decisions; the pointers stay valid
// until the next sync point.
class LaneOccupancy final {
  static constexpr std::uint32_t NOWHERE = ~std::uint32_t{0};

 public:
  struct Occupant final {
    LaneKey lane;
    double along = 0.0;   // m along the lane, its front bumper.
    double speed = 0.0;   // m/s.
    double length = 0.0;  // m.
    std::uint32_t turns = 0;
    const Driver* driver = nullptr;
    Entity entity;
  };

  // Since when a vehicle has stood still, and whether it has committed to
  // the junction ahead or the one it is in: kept beside the occupants, so
  // the sorted index carries no more than it did without junctions.
  struct Stop final {
    TimePoint since = TimePoint::max();  // Never, if moving.
    bool committed = false;
  };

  // Every vehicle with a LaneState in `states` and a Driver in `drivers`,
  // and its stop if `stopped` holds them.
  auto collect(const ComponentStore<LaneState>& states,
               const ComponentStore<Driver>& drivers, const Network& network,
               const ComponentStore<Stopped>* stopped = nullptr) -> void {
    occupants_.clear();
    states.for_each([&](Entity owner, const LaneState& state) {
      const Driver* driver = drivers.maybe_component_of(owner);
      if (!driver) {
        return;
      }
      occupants_.push_back(
          Occupant{.lane = state.lane,
                   .along = along_lane(network, state.lane, state.s),
                   .speed = state.speed.numerical_value_in(meter_per_second),
                   .length = driver->length.numerical_value_in(meter),
                   .turns = state.turns,
                   .driver = driver,
                   .entity = owner});
      if (const Stopped* since =
              stopped ? stopped->maybe_component_of(owner) : nullptr) {
        if (owner.index >= stops_.size()) {
          stops_.resize(owner.index + 1);
        }
        stops_[owner.index] =
            Stop{.since = since->since, .committed = since->committed};
      }
    });
    std::ranges::sort(occupants_, [](const Occupant& a, const Occupant& b) {
      return std::tie(a.lane, a.along, a.entity) <
             std::tie(b.lane, b.along, b.entity);
    });
    numbering_ = &network.graph.numbering();
    lanes_ =
        road::LaneRanges{*numbering_, occupants_.size(),
                         [&](std::size_t i) { return occupants_[i].lane; }};
    places_.assign(places_.size(), NOWHERE);
    for (std::uint32_t i = 0; i < occupants_.size(); ++i) {
      std::uint32_t index = occupants_[i].entity.index;
      if (index >= places_.size()) {
        places_.resize(index + 1, NOWHERE);
      }
      places_[index] = i;
    }
  }

  // The first vehicle in `lane` ahead of `along`, but `self`.
  auto find_ahead(const LaneKey& lane, double along, Entity self) const
      -> const Occupant* {
    // In its own lane, the vehicles ahead of `self` follow it, those level
    // with it ahead by entity, as sorted.
    if (std::uint32_t at = place_of(self, lane); at != NOWHERE) {
      ++at;
      return at < occupants_.size() && occupants_[at].lane == lane
                 ? &occupants_[at]
                 : nullptr;
    }
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
    // In its own lane, the vehicles behind `self` precede it.
    if (std::uint32_t at = place_of(self, lane); at != NOWHERE) {
      while (at > 0 && occupants_[--at].lane == lane) {
        if (occupants_[at].along < along) {
          return &occupants_[at];
        }
      }
      return nullptr;
    }
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

  // `occupant`'s stop, if they were collected.
  auto stop_of(const Occupant& occupant) const -> Stop {
    return occupant.entity.index < stops_.size() ? stops_[occupant.entity.index]
                                                 : Stop{};
  }

  // The vehicles in `lane`, in order along it.
  auto occupants_of(const LaneKey& lane) const -> std::span<const Occupant> {
    auto [first, last] = in_lane(lane);
    return {first, last};
  }

  // `self`, if it is in `lane`.
  auto find(Entity self, const LaneKey& lane) const -> const Occupant* {
    std::uint32_t at = place_of(self, lane);
    return at == NOWHERE ? nullptr : &occupants_[at];
  }

 private:
  using Iterator = std::vector<Occupant>::const_iterator;

  static auto by_along(double along, const Occupant& occupant) -> bool {
    return along < occupant.along;
  }
  static auto after_along(const Occupant& occupant, double along) -> bool {
    return occupant.along < along;
  }

  // Where `self` is among the occupants, if it is in `lane`.
  auto place_of(Entity self, const LaneKey& lane) const -> std::uint32_t {
    if (self.index >= places_.size()) {
      return NOWHERE;
    }
    std::uint32_t at = places_[self.index];
    return at != NOWHERE && occupants_[at].entity == self &&
                   occupants_[at].lane == lane
               ? at
               : NOWHERE;
  }

  auto in_lane(const LaneKey& lane) const -> std::pair<Iterator, Iterator> {
    if (numbering_ == nullptr) {
      return {occupants_.end(), occupants_.end()};
    }
    auto [first, last] = lanes_.range_of(numbering_->number_of(lane));
    return {occupants_.begin() + first, occupants_.begin() + last};
  }

  std::vector<Occupant> occupants_;  // By lane, then along it.
  const road::LaneNumbering* numbering_ = nullptr;
  road::LaneRanges lanes_;             // Into occupants_, by lane number.
  std::vector<std::uint32_t> places_;  // Each entity's place, by its index.
  std::vector<Stop> stops_;            // Each entity's stop, by its index.
};

//-- Systems -------------------------------------------------------------------

// Each step, each signal group shows what its plan gives for the step's time.
struct RunSignals final    //
    : System<SignalState,  //
             const traffic::SignalPlan> {
  using SystemWorld = ProjectedWorld<RunSignals>;

  auto operator()(SystemWorld&, Entity,             //
                  SignalState& state,               //
                  const traffic::SignalPlan* plan,  //
                  Step step) const -> void {
    if (plan) {
      state.aspect = plan->aspect_at(step.time.time_since_epoch());
    }
  }
};

// Each step, each driver finds its leader, in its lane or the lanes its way
// leads into, and accelerates by the Intelligent Driver Model; once a second
// it weighs changing to a lane beside its own by MOBIL. A dead end ahead is a
// leader standing still. At a light it stops for, or a junction where it
// gives way and finds no gap, it brakes to stop its line gap short of the
// line or the junction, and its acceleration is the lesser of the two.
struct Decide final            //
    : System<DriveCommand,     //
             const LaneState,  //
             const Driver,     //
             Tactical> {
  using SystemWorld = ProjectedWorld<Decide>;
  using AllowComponentList =
      TypeList<LaneState, Driver, SignalState, Stopped, WalkCommand>;
  using SequenceAfterSystemList = SystemList<RunSignals>;

  // How far ahead a driver looks for a leader across its lane's end, and for
  // a light or a junction.
  static constexpr double LOOKAHEAD = 250.0;  // m.

  explicit Decide(const Network& network) : network_{&network} {}

  auto prepare(SystemWorld& world, Step step) -> bool {
    lights_ = !network_->control.stop_lines().empty();
    walks_ = !network_->walking.zones().empty();
    if (walks_) {
      std::size_t crosswalks = network_->walking.crosswalks().size();
      on_.assign(crosswalks, 0.0);
      coming_.assign(crosswalks, 0.0);
      world.store_of<WalkCommand>().for_each(
          [&](Entity, const WalkCommand& walking) {
            if (walking.crossing < crosswalks) {
              double& until = (walking.on ? on_ : coming_)[walking.crossing];
              until = std::max(until, walking.clear.numerical_value_in(second));
            }
          });
    }
    yields_ = !network_->rights.conflicts().empty();
    occupancy_.collect(world.store_of<LaneState>(), world.store_of<Driver>(),
                       *network_,
                       yields_ ? &world.store_of<Stopped>() : nullptr);
    changing_ = gate_.fire(step).has_value();
    aspects_.assign(network_->control.groups().size(), traffic::Aspect::GREEN);
    world.store_of<SignalState>().for_each(
        [&](Entity, const SignalState& signal) {
          if (signal.group < aspects_.size()) {
            aspects_[signal.group] = signal.aspect;
          }
        });
    return true;
  }

  auto operator()(SystemWorld&, Entity self,  //
                  DriveCommand& command,      //
                  const LaneState* state,     //
                  const Driver* driver,       //
                  Tactical* tactical) const -> void {
    if (!state || !driver) {
      return;
    }
    double along = along_lane(*network_, state->lane, state->s);
    std::optional<Length> light =
        lights_ && tactical ? find_light(*state, *driver, along, *tactical)
                            : std::nullopt;
    if (std::optional<Length> wait =
            yields_ && tactical
                ? find_wait(*state, *driver, along, self, *tactical)
                : std::nullopt) {
      light = light ? std::min(*light, *wait) : *wait;
    }
    if (std::optional<Length> walkers =
            walks_ && tactical
                ? find_crosswalk(*state, *driver, along, *tactical)
                : std::nullopt) {
      light = light ? std::min(*light, *walkers) : *walkers;
    }
    auto obey = [&](AccelerationMagnitude acceleration) {
      return light ? std::min(acceleration,
                              traffic::compute_stop_acceleration(
                                  driver->following, state->speed, *light))
                   : acceleration;
    };
    std::optional<traffic::Leader> leader =
        find_leader(state->lane, along, state->turns, driver->seed, self);
    auto now = traffic::compute_idm_acceleration(driver->following,
                                                 state->speed, leader);
    command = DriveCommand{.acceleration = obey(now)};
    if (!changing_ || state->speed < 1.0 * meter_per_second) {
      return;
    }
    for (bool right : {true, false}) {
      std::optional<LaneKey> target =
          find_neighbor(*network_, state->lane, right);
      if (target && worth_changing(*state, *driver, self, along, now, leader,
                                   *target, right)) {
        command.change = target;
        command.acceleration = obey(traffic::compute_idm_acceleration(
            driver->following, state->speed,
            find_leader(*target, along, state->turns, driver->seed, self)));
        return;
      }
    }
  }

 private:
  // How far ahead the driver waits at the next junction on its way, within
  // the lookahead, its line gap short of the junction; none if it may go.
  // It goes when there is room for it past the junction, so it never stops
  // in it, and every vehicle with priority would reach each conflict on its
  // connecting lane at least its critical gap after it reaches the junction;
  // once it would have to brake to wait it holds to that, as it does when it
  // is too near to stop. Of two drivers waiting at the junction for each
  // other, the one that stopped first goes.
  [[gnu::noinline]] auto find_wait(const LaneState& state, const Driver& driver,
                                   double along, Entity self,
                                   Tactical& tactical) const
      -> std::optional<Length> {
    const traffic::RightOfWay& rights = network_->rights;
    double before = -along;  // From the vehicle to the lane's start.
    LaneKey key = state.lane;
    std::uint32_t turns = state.turns;
    while (network_->map.roads[key.road].junction == "-1") {
      before += find_lane_length(*network_, key);
      std::optional<LaneKey> next =
          choose_next_lane(*network_, key, driver.seed, turns++);
      if (!next || before >= LOOKAHEAD) {
        tactical.entering.reset();
        return std::nullopt;
      }
      key = *next;
    }
    if (before <= 0.0 || tactical.entering == key) {
      tactical.entering = key;  // In the junction, or decided.
      return std::nullopt;
    }
    tactical.entering.reset();
    // Kept clear: room past the junction for the whole vehicle.
    bool room = true;
    if (std::optional<LaneKey> exit =
            choose_next_lane(*network_, key, driver.seed, turns)) {
      if (const LaneOccupancy::Occupant* last = occupancy_.find_first(*exit)) {
        room = last->along - last->length >=
               (driver.length + driver.following.minimum_gap)
                   .numerical_value_in(meter);
      }
    }
    double v = state.speed.numerical_value_in(meter_per_second);
    double to_wait = std::max(
        before - tactical.braking.line_gap.numerical_value_in(meter), 0.0);
    // s, at the soonest, to where it waits.
    double arriving =
        traffic::compute_soonest_arrival(to_wait * meter, state.speed,
                                         driver.following.acceleration,
                                         driver.following.desired_speed)
            .numerical_value_in(second);
    const LaneOccupancy::Occupant* me = occupancy_.find(self, state.lane);
    LaneOccupancy::Stop my_stop =
        me ? occupancy_.stop_of(*me) : LaneOccupancy::Stop{};
    bool waiting = me && my_stop.since != TimePoint::max() && to_wait < 0.5;
    double gap = tactical.critical_gap.numerical_value_in(second);
    bool clear = room;
    // Nor does it enter while one that gives way to it is in the junction
    // and has not cleared where their ways meet, or waits to enter it,
    // having stopped first, as that one then goes.
    for (std::uint32_t index : rights.conflicts_against(key)) {
      const traffic::Conflict& conflict = rights.conflicts()[index];
      for (const LaneOccupancy::Occupant& other :
           occupancy_.occupants_of(conflict.lane)) {
        clear = clear && !(other.along - other.length < conflict.along);
      }
      if (!waiting || conflict.why == traffic::Yielding::LIGHTS) {
        continue;
      }
      for (const LaneKey& before_lane :
           network_->graph.predecessors_of(conflict.lane)) {
        std::span<const LaneOccupancy::Occupant> in =
            occupancy_.occupants_of(before_lane);
        if (in.empty()) {
          continue;
        }
        const LaneOccupancy::Occupant& other = in.back();
        LaneOccupancy::Stop other_stop = occupancy_.stop_of(other);
        bool first = std::tie(other_stop.since, other.entity) <
                     std::tie(my_stop.since, me->entity);
        bool waits =
            other_stop.since != TimePoint::max() && !other_stop.committed &&
            other.along >=
                find_lane_length(*network_, before_lane) -
                    tactical.braking.line_gap.numerical_value_in(meter) - 0.5;
        bool heading =
            choose_next_lane(*network_, before_lane, other.driver->seed,
                             other.turns) == conflict.lane;
        clear = clear && !(first && waits && heading);
      }
    }
    for (const traffic::Conflict& conflict : rights.conflicts_on(key)) {
      if (conflict.why == traffic::Yielding::LIGHTS) {
        continue;  // Kept apart by lights: only who is in the junction.
      }
      auto index =
          static_cast<std::uint32_t>(&conflict - rights.conflicts().data());
      std::span<const traffic::Approach> approaches =
          rights.approaches_of(index);
      for (const traffic::Approach& approach : approaches) {
        if (const LaneOccupancy::Occupant* foe =
                find_foe(approaches, approach, self)) {
          TimePoint foe_since = occupancy_.stop_of(*foe).since;
          if (waiting && waits_at_entry(approach, *foe, tactical) &&
              std::tie(my_stop.since, me->entity) <
                  std::tie(foe_since, foe->entity)) {
            continue;  // It stopped first.
          }
          double time = traffic::compute_soonest_arrival(
                            (approach.to_conflict - foe->along) * meter,
                            foe->speed * meter_per_second,
                            foe->driver->following.acceleration,
                            foe->driver->following.desired_speed)
                            .numerical_value_in(second);
          clear = clear && time - arriving >= gap;
        }
      }
    }
    double comfortable = driver.following.deceleration.numerical_value_in(
        meter_per_second_squared);
    double maximum =
        tactical.braking.maximum.numerical_value_in(meter_per_second_squared);
    if (clear) {
      if (to_wait <= v * v / (2.0 * comfortable) + 1.0) {
        tactical.entering = key;
      }
      return std::nullopt;
    }
    if (to_wait > 0.0 && v * v / (2.0 * maximum) >= to_wait) {
      tactical.entering = key;  // Too near to stop.
      return std::nullopt;
    }
    return to_wait * meter;
  }

  // The nearest vehicle in `approach`'s lane, but `self`, whose way leads to
  // the conflict and that has not passed it, unless one going elsewhere
  // stands still before it.
  auto find_foe(std::span<const traffic::Approach> approaches,
                const traffic::Approach& approach, Entity self) const
      -> const LaneOccupancy::Occupant* {
    std::span<const LaneOccupancy::Occupant> in =
        occupancy_.occupants_of(approach.lane);
    for (auto it = in.rbegin(); it != in.rend(); ++it) {
      // Past the conflict only once its tail is.
      if (it->entity == self || it->along - it->length > approach.to_conflict) {
        continue;
      }
      LaneKey key = approach.lane;
      std::uint32_t turns = it->turns;
      std::uint32_t toward = approach.toward;
      bool heading = true;
      while (heading && toward != traffic::Approach::NONE) {
        std::optional<LaneKey> next =
            choose_next_lane(*network_, key, it->driver->seed, turns++);
        heading = next == approaches[toward].lane;
        key = approaches[toward].lane;
        toward = approaches[toward].toward;
      }
      if (heading) {
        return &*it;
      }
      if (occupancy_.stop_of(*it).since != TimePoint::max()) {
        return nullptr;  // Those behind wait for it.
      }
    }
    return nullptr;
  }

  // Whether `foe` stands waiting at its own way into the junction, at the end
  // of the lane leading into the conflict's foe lane.
  auto waits_at_entry(const traffic::Approach& approach,
                      const LaneOccupancy::Occupant& foe,
                      const Tactical& tactical) const -> bool {
    double line_gap = tactical.braking.line_gap.numerical_value_in(meter);
    LaneOccupancy::Stop stop = occupancy_.stop_of(foe);
    return approach.toward == 0 && stop.since != TimePoint::max() &&
           !stop.committed &&
           foe.along >=
               find_lane_length(*network_, approach.lane) - line_gap - 0.5;
  }

  // How far ahead the driver stops for the first crosswalk on its way,
  // within the lookahead, that a pedestrian is on, or, where vehicles yield,
  // is about to step onto: its line gap short of the crosswalk; none if it
  // need not stop, or cannot.
  [[gnu::noinline]] auto find_crosswalk(const LaneState& state,
                                        const Driver& driver, double along,
                                        const Tactical& tactical) const
      -> std::optional<Length> {
    double before = -along;  // From the vehicle to the lane's start.
    LaneKey key = state.lane;
    std::uint32_t turns = state.turns;
    double v = state.speed.numerical_value_in(meter_per_second);
    double maximum =
        tactical.braking.maximum.numerical_value_in(meter_per_second_squared);
    double line_gap = tactical.braking.line_gap.numerical_value_in(meter);
    while (before < LOOKAHEAD) {
      for (const road::CrosswalkZone& zone : network_->walking.zones_on(key)) {
        double distance = before + zone.near;
        if (distance <= 0.0 || distance >= LOOKAHEAD) {
          continue;
        }
        // It stops for one it would reach the crosswalk before, with a
        // second to spare, or, where vehicles yield, one about to step on.
        double reaching = distance / std::max(v, 1.0);
        bool blocked =
            (on_[zone.crosswalk] > 0.0 &&
             reaching < on_[zone.crosswalk] + 1.0) ||
            (network_->vehicles_yield && coming_[zone.crosswalk] > 0.0);
        if (!blocked) {
          continue;
        }
        double to_stop = std::max(distance - line_gap, 0.0);
        if (v * v / (2.0 * maximum) >= to_stop && to_stop > 0.0) {
          continue;  // Too near to stop.
        }
        return to_stop * meter;
      }
      before += find_lane_length(*network_, key);
      std::optional<LaneKey> next =
          choose_next_lane(*network_, key, driver.seed, turns++);
      if (!next) {
        break;
      }
      key = *next;
    }
    return std::nullopt;
  }

  // How far ahead the driver stops for the first light, within the lookahead
  // along the vehicle's way, it stops for: its line gap short of the line. A
  // driver that will not stop for a light that is not green commits to
  // passing it, and holds to that until the line is behind it.
  [[gnu::noinline]] auto find_light(const LaneState& state,
                                    const Driver& driver, double along,
                                    Tactical& tactical) const
      -> std::optional<Length> {
    std::span<const traffic::StopLine> all = network_->control.stop_lines();
    bool held = false;
    std::optional<Length> light;
    double before = -along;  // From the vehicle to the lane's start.
    LaneKey key = state.lane;
    std::uint32_t turns = state.turns;
    while (!light && before < LOOKAHEAD) {
      for (const traffic::StopLine& line :
           network_->control.stop_lines_on(key)) {
        double distance = before + line.along;
        if (distance <= 0.0 || distance >= LOOKAHEAD) {
          continue;
        }
        auto index = static_cast<std::uint32_t>(&line - all.data());
        if (index == tactical.committed) {
          held = true;
          continue;
        }
        traffic::Aspect aspect = aspects_[line.group];
        if (aspect == traffic::Aspect::GREEN) {
          continue;
        }
        // Where it stops, its line gap short of the line.
        Length to_stop =
            std::max(distance * meter - tactical.braking.line_gap, 0.0 * meter);
        if (traffic::stops_at_light(driver.following, tactical.braking, aspect,
                                    state.speed, to_stop)) {
          light = to_stop;
          break;
        }
        tactical.committed = index;
        held = true;
      }
      before += find_lane_length(*network_, key);
      std::optional<LaneKey> next =
          choose_next_lane(*network_, key, driver.seed, turns++);
      if (!next) {
        break;
      }
      key = *next;
    }
    if (!held) {
      tactical.committed = Tactical::NONE;
    }
    return light;
  }

  // The leader of a vehicle `along` meters into `lane`: the next vehicle
  // ahead in the lane, else the first in the lanes its way leads into, within
  // the lookahead.
  auto find_leader(const LaneKey& lane, double along, std::uint32_t turns,
                   std::uint64_t seed, Entity self) const
      -> std::optional<traffic::Leader> {
    if (yields_) {
      return find_leader_near_junctions(lane, along, turns, seed, self);
    }
    if (const LaneOccupancy::Occupant* ahead =
            occupancy_.find_ahead(lane, along, self)) {
      return gap_to(*ahead, ahead->along - along);
    }
    double distance = find_lane_length(*network_, lane) - along;
    LaneKey key = lane;
    while (distance < LOOKAHEAD) {
      std::optional<LaneKey> next =
          choose_next_lane(*network_, key, seed, turns++);
      if (!next) {
        return traffic::Leader{.gap = distance * meter,
                               .speed = 0.0 * meter_per_second};
      }
      key = *next;
      if (const LaneOccupancy::Occupant* first = occupancy_.find_first(key)) {
        return gap_to(*first, distance + first->along);
      }
      distance += find_lane_length(*network_, key);
    }
    return std::nullopt;
  }

  // find_leader where lanes merge and part in junctions: the vehicles on
  // lanes merging into or parting from the driver's way lead it too.
  [[gnu::noinline]] auto find_leader_near_junctions(
      const LaneKey& lane, double along, std::uint32_t turns,
      std::uint64_t seed, Entity self) const -> std::optional<traffic::Leader> {
    auto nearer = [](std::optional<traffic::Leader> a,
                     std::optional<traffic::Leader> b) {
      return !a || (b && b->gap < a->gap) ? b : a;
    };
    // Where lanes part, those leaving the same lane share it at first.
    std::optional<traffic::Leader> parting;
    if (yields_) {
      for (const LaneKey& before : network_->graph.predecessors_of(lane)) {
        for (const LaneKey& sibling : network_->graph.successors_of(before)) {
          if (sibling != lane) {
            parting = nearer(parting, find_parting(sibling, -along, self));
          }
        }
      }
    }
    if (const LaneOccupancy::Occupant* ahead =
            occupancy_.find_ahead(lane, along, self)) {
      return nearer(parting, gap_to(*ahead, ahead->along - along));
    }
    double distance = find_lane_length(*network_, lane) - along;
    LaneKey key = lane;
    while (distance < LOOKAHEAD) {
      std::optional<LaneKey> next =
          choose_next_lane(*network_, key, seed, turns++);
      if (!next) {
        return nearer(parting,
                      traffic::Leader{.gap = distance * meter,
                                      .speed = 0.0 * meter_per_second});
      }
      LaneKey from = key;
      key = *next;
      if (yields_) {
        parting = nearer(parting, find_merging(from, key, distance, self));
        for (const LaneKey& sibling : network_->graph.successors_of(from)) {
          if (sibling != key) {
            parting = nearer(parting, find_parting(sibling, distance, self));
          }
        }
      }
      if (const LaneOccupancy::Occupant* first = occupancy_.find_first(key)) {
        return nearer(parting, gap_to(*first, distance + first->along));
      }
      if (parting) {
        return parting;
      }
      distance += find_lane_length(*network_, key);
    }
    return parting;
  }

  // The last vehicle on `lane`, which parts from the driver's way, still
  // where the two share the road: a leader `distance` plus its place along
  // `lane` ahead of the driver, if that is ahead.
  auto find_parting(const LaneKey& lane, double distance, Entity self) const
      -> std::optional<traffic::Leader> {
    std::optional<double> parts = network_->rights.find_parting(lane);
    if (!parts) {
      return std::nullopt;
    }
    for (const LaneOccupancy::Occupant& other : occupancy_.occupants_of(lane)) {
      if (other.entity != self && distance + other.along > 0.0 &&
          other.along - other.length < *parts) {
        return gap_to(other, distance + other.along);
      }
    }
    return std::nullopt;
  }

  // Of the vehicles about to merge into `into` from lanes other than `from`,
  // the nearest to where their ways meet that is nearer it than the driver,
  // `distance` from `into`: a leader as if in the driver's lane, as SUMO
  // treats a vehicle on a merging lane. Each is measured to where its lane
  // first comes within a car's width of another; ties go to the lower
  // entity.
  auto find_merging(const LaneKey& from, const LaneKey& into, double distance,
                    Entity self) const -> std::optional<traffic::Leader> {
    std::span<const LaneKey> feeding = network_->graph.predecessors_of(into);
    if (feeding.size() < 2) {
      return std::nullopt;
    }
    auto before_end = [&](const LaneKey& lane) {
      double length = find_lane_length(*network_, lane);
      return length - network_->rights.find_merge(lane).value_or(length);
    };
    double mine = distance - before_end(from);  // To where ways meet.
    std::optional<traffic::Leader> nearest;
    for (const LaneKey& lane : feeding) {
      if (lane == from) {
        continue;
      }
      std::span<const LaneOccupancy::Occupant> in =
          occupancy_.occupants_of(lane);
      double meets = find_lane_length(*network_, lane) - before_end(lane);
      for (auto it = in.rbegin(); it != in.rend(); ++it) {
        double theirs = meets - it->along;
        if (std::tie(theirs, it->entity) >= std::tie(mine, self)) {
          break;  // It and those behind it follow the driver.
        }
        if (choose_next_lane(*network_, lane, it->driver->seed, it->turns) !=
            into) {
          continue;
        }
        traffic::Leader leader = gap_to(*it, mine - theirs);
        if (!nearest || leader.gap < nearest->gap) {
          nearest = leader;
        }
        break;
      }
    }
    return nearest;
  }

  // A leader `apart` meters ahead, front bumper to front bumper.
  static auto gap_to(const LaneOccupancy::Occupant& leader, double apart)
      -> traffic::Leader {
    return traffic::Leader{.gap = (apart - leader.length) * meter,
                           .speed = leader.speed * meter_per_second};
  }

  // MOBIL's accelerations for changing from `state`'s lane to `target`: this
  // driver's, its old follower's and its new follower's, now and after. A
  // change onto a vehicle, or with one just behind, is never worth it.
  auto worth_changing(const LaneState& state, const Driver& driver, Entity self,
                      double along, AccelerationMagnitude now,
                      const std::optional<traffic::Leader>& leader,
                      const LaneKey& target, bool right) const -> bool {
    double length = driver.length.numerical_value_in(meter);
    traffic::LaneChangeAccelerations accelerations{.self_now = now};
    std::optional<traffic::Leader> new_leader =
        find_leader(target, along, state.turns, driver.seed, self);
    if (new_leader && new_leader->gap <= 0.0 * meter) {
      return false;
    }
    accelerations.self_after = traffic::compute_idm_acceleration(
        driver.following, state.speed, new_leader);

    if (const LaneOccupancy::Occupant* follower =
            occupancy_.find_behind(target, along, self)) {
      double gap = along - length - follower->along;
      if (gap <= 0.0) {
        return false;
      }
      Speed speed = follower->speed * meter_per_second;
      const traffic::IntelligentDriver& following = follower->driver->following;
      accelerations.new_follower_now = traffic::compute_idm_acceleration(
          following, speed,
          find_leader(target, follower->along, follower->turns,
                      follower->driver->seed, follower->entity));
      accelerations.new_follower_after = traffic::compute_idm_acceleration(
          following, speed,
          traffic::Leader{.gap = gap * meter, .speed = state.speed});
    }
    if (const LaneOccupancy::Occupant* follower =
            occupancy_.find_behind(state.lane, along, self)) {
      Speed speed = follower->speed * meter_per_second;
      const traffic::IntelligentDriver& following = follower->driver->following;
      accelerations.old_follower_now = traffic::compute_idm_acceleration(
          following, speed,
          traffic::Leader{.gap = (along - length - follower->along) * meter,
                          .speed = state.speed});
      std::optional<traffic::Leader> after = leader;
      if (after) {
        after->gap += (along - follower->along) * meter;
      }
      accelerations.old_follower_after =
          traffic::compute_idm_acceleration(following, speed, after);
    }
    return traffic::decide_lane_change(driver.changing, accelerations, right);
  }

  const Network* network_ = nullptr;
  LaneOccupancy occupancy_;
  std::vector<traffic::Aspect> aspects_;  // By signal group.
  engine::RateGate gate_{1s};
  bool changing_ = false;
  bool lights_ = false;  // Whether the network has any.
  bool yields_ = false;  // Whether its junctions have any conflicts.
  bool walks_ = false;   // Whether its roads have any crosswalks.
  // By crosswalk: when, in s from now, the last pedestrian on it will be
  // off it, and the last that has decided to step onto it; 0 if none.
  std::vector<double> on_;
  std::vector<double> coming_;
};

// Each step, each vehicle changes lane if its driver decided to, and moves
// along its lane at the acceleration it decided, held over the step, never
// backward. Past its lane's end it enters the next lane on its way, and at a
// dead end it stops.
struct Drive final                //
    : System<LaneState,           //
             const DriveCommand,  //
             const Driver,        //
             const Tactical,      //
             Stopped> {
  using SystemWorld = ProjectedWorld<Drive>;
  using SequenceAfterSystemList = SystemList<Decide>;

  explicit Drive(const Network& network) : network_{&network} {}

  auto operator()(SystemWorld&, Entity,         //
                  LaneState& state,             //
                  const DriveCommand* command,  //
                  const Driver* driver,         //
                  const Tactical* tactical,     //
                  Stopped* stopped,             //
                  Step step) const -> void {
    if (!command || !driver) {
      return;
    }
    if (command->change) {
      state.lane = *command->change;
    }
    double dt = std::chrono::duration<double>(step.dt).count();
    double v = state.speed.numerical_value_in(meter_per_second);
    double a =
        command->acceleration.numerical_value_in(meter_per_second_squared);
    double travel = 0.0;
    if (v + a * dt < 0.0) {  // Stops within the step.
      travel = v * v / (2.0 * -a);
      v = 0.0;
    } else {
      travel = v * dt + 0.5 * a * dt * dt;
      v += a * dt;
    }
    double along = along_lane(*network_, state.lane, state.s) + travel;
    double length = find_lane_length(*network_, state.lane);
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
      length = find_lane_length(*network_, state.lane);
    }
    state.s = find_s_along(*network_, state.lane, along);
    state.speed = v * meter_per_second;
    if (stopped) {
      stopped->committed = tactical && tactical->entering;
      if (v > 0.0) {
        stopped->since = TimePoint::max();
      } else if (stopped->since == TimePoint::max()) {
        stopped->since = step.time + step.dt;
      }
    }
  }

 private:
  const Network* network_ = nullptr;
};

// After Drive, each vehicle's place in the world follows its lane state: the
// middle of its lane at its s, heading along the road, or against it on the
// left.
struct FollowLane final  //
    : System<RoadPose,   //
             const LaneState> {
  using SystemWorld = ProjectedWorld<FollowLane>;
  using SequenceAfterSystemList = SystemList<Drive>;

  explicit FollowLane(const Network& network) : network_{&network} {}

  auto operator()(SystemWorld&, Entity,  //
                  RoadPose& pose,        //
                  const LaneState* state) const -> void {
    if (!state) {
      return;
    }
    pose = locate_vehicle(*network_, *state);
  }

  // The pose of a vehicle in `state`.
  static auto locate_vehicle(const Network& network, const LaneState& state)
      -> RoadPose {
    const road::Road& road = network.map.roads[state.lane.road];
    Length middle = road::compute_lane_middle(network.map, state.lane, state.s);
    road::PlanPoint point = road::compute_plan_point(road, state.s);
    double heading = point.heading;
    if (!road::runs_with_s(state.lane)) {
      heading += std::numbers::pi;
    }
    return RoadPose{
        .position = road::compute_position(road, point, state.s, middle),
        .heading = heading * radian};
  }

 private:
  const Network* network_ = nullptr;
};

//-- Pedestrians ---------------------------------------------------------------

// The node a pedestrian on `legs` reaches at their end.
inline auto find_route_end(const Network& network,
                           std::span<const road::Leg> legs) -> std::uint32_t {
  const road::WalkEdge& edge = network.walking.edges()[legs.back().edge];
  return legs.back().forward ? edge.to : edge.from;
}

// A route from `node` to a node `seed` and `trip` pick at random among those
// `node` can reach; none if it can reach none.
inline auto plan_walk(const Network& network, std::uint32_t node,
                      std::uint64_t seed, std::uint32_t trip)
    -> std::vector<road::Leg> {
  // SplitMix64 of the seed and the trip (see model/REFERENCES.md).
  std::uint64_t z = seed + (std::uint64_t{trip} + 1) * 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  z ^= z >> 31;
  Random random{z};
  std::span<const std::uint32_t> component = network.walking_components;
  for (int attempt = 0; attempt < 100; ++attempt) {
    auto goal = static_cast<std::uint32_t>(
        random.uniform(0.0, static_cast<double>(component.size())));
    if (goal != node && component[goal] == component[node]) {
      return network.walking.find_route(node, goal);
    }
  }
  return {};
}

// Where pedestrians walk, each step: every pedestrian by edge and direction,
// in order along it, for finding who walks ahead of whom.
class WalkOccupancy final {
 public:
  struct Walking final {
    std::uint32_t edge = 0;
    bool forward = true;
    double along = 0.0;  // m along the edge, as walked.
    double speed = 0.0;  // m/s.
    Entity entity;
  };

  // Every pedestrian with a WalkState in `states` on a WalkRoute in
  // `routes`.
  auto collect(const ComponentStore<WalkState>& states,
               const ComponentStore<WalkRoute>& routes) -> void {
    walking_.clear();
    states.for_each([&](Entity owner, const WalkState& state) {
      const WalkRoute* route = routes.maybe_component_of(owner);
      if (!route || state.leg >= route->legs.size()) {
        return;
      }
      const road::Leg& leg = route->legs[state.leg];
      walking_.push_back(
          Walking{.edge = leg.edge,
                  .forward = leg.forward,
                  .along = state.along.numerical_value_in(meter),
                  .speed = state.speed.numerical_value_in(meter_per_second),
                  .entity = owner});
    });
    std::ranges::sort(walking_, [](const Walking& a, const Walking& b) {
      return std::tie(a.edge, a.forward, a.along, a.entity) <
             std::tie(b.edge, b.forward, b.along, b.entity);
    });
  }

  // The first pedestrian walking `leg` ahead of `along`, but `self`.
  auto find_ahead(const road::Leg& leg, double along, Entity self) const
      -> const Walking* {
    auto first = std::ranges::lower_bound(
        walking_, std::tuple{leg.edge, leg.forward, along}, {},
        [](const Walking& w) {
          return std::tuple{w.edge, w.forward, w.along};
        });
    for (auto it = first; it != walking_.end() && it->edge == leg.edge &&
                          it->forward == leg.forward;
         ++it) {
      if (it->entity != self && (it->along > along || it->entity > self)) {
        return &*it;
      }
    }
    return nullptr;
  }

 private:
  std::vector<Walking> walking_;  // By edge, direction, then along.
};

// Each step, each pedestrian walks at its own speed, slowing behind the one
// ahead on its edge, or on its next leg's: so as never to come nearer than
// half a meter, it walks the gap beyond that in a second at most. At the
// kerb before a crosswalk it decides whether to cross, and waits there until
// it may. Where a light stands just before the crosswalk, a pedestrian that
// complies crosses while the traffic it crosses has red, and only if the red
// lasts as long as it takes to cross. Elsewhere, or if it does not comply,
// it crosses when every vehicle coming to the crosswalk would reach it no
// sooner than its critical gap, its time to cross and its start-up time, as
// the Highway Capacity Manual has it, or, where vehicles yield, could still
// stop before it comfortably. Once it decides, it goes.
struct Pace final              //
    : System<WalkCommand,      //
             const WalkState,  //
             const Walker,     //
             const WalkRoute> {
  using SystemWorld = ProjectedWorld<Pace>;
  using AllowComponentList = TypeList<WalkState, WalkRoute, LaneState, Driver,
                                      SignalState, traffic::SignalPlan>;

  static constexpr double SPACE = 0.5;      // m kept to the one ahead.
  static constexpr double HEADWAY = 1.0;    // s to close the rest.
  static constexpr double LOOKAHEAD = 5.0;  // m into the next leg.
  static constexpr double KERB = 1.5;       // m from the kerb it decides.
  static constexpr double WAIT = 0.2;       // m from the kerb it waits.
  static constexpr double GROUP = 3.0;      // m behind one it crosses with.
  static constexpr double REACH = 200.0;    // m it looks up the road.

  explicit Pace(const Network& network) : network_{&network} {
    // Each zone's lanes, back up the road: a lane and how far its start is
    // from the zone's near edge.
    std::span<const road::CrosswalkZone> zones = network.walking.zones();
    for (std::uint32_t z = 0; z < zones.size(); ++z) {
      first_.push_back(static_cast<std::uint32_t>(upstream_.size()));
      upstream_.emplace_back(zones[z].lane, zones[z].near);
      for (std::size_t k = first_.back(); k < upstream_.size(); ++k) {
        auto [lane, to] = upstream_[k];
        if (to >= REACH) {
          continue;
        }
        for (const LaneKey& before : network.graph.predecessors_of(lane)) {
          if (road::find_lane(network.map, before).type == "driving" &&
              std::none_of(upstream_.begin() + first_.back(), upstream_.end(),
                           [&](const auto& u) { return u.first == before; })) {
            upstream_.emplace_back(before,
                                   to + find_lane_length(network, before));
          }
        }
      }
      by_crosswalk_.emplace_back(zones[z].crosswalk, z);
    }
    first_.push_back(static_cast<std::uint32_t>(upstream_.size()));
    std::ranges::sort(by_crosswalk_);
  }

  auto prepare(SystemWorld& world, Step step) -> bool {
    occupancy_.collect(world.store_of<WalkState>(),
                       world.store_of<WalkRoute>());
    crossings_ = !network_->walking.zones().empty();
    if (crossings_) {
      vehicles_.collect(world.store_of<LaneState>(), world.store_of<Driver>(),
                        *network_);
      now_ = step.time.time_since_epoch();
      std::size_t groups = network_->control.groups().size();
      aspects_.assign(groups, traffic::Aspect::GREEN);
      plans_.assign(groups, nullptr);
      const auto& plans = world.store_of<traffic::SignalPlan>();
      world.store_of<SignalState>().for_each(
          [&](Entity owner, const SignalState& signal) {
            if (signal.group < groups) {
              aspects_[signal.group] = signal.aspect;
              plans_[signal.group] = plans.maybe_component_of(owner);
            }
          });
    }
    return true;
  }

  auto operator()(SystemWorld&, Entity self,  //
                  WalkCommand& command,       //
                  const WalkState* state,     //
                  const Walker* walker,       //
                  const WalkRoute* route) const -> void {
    if (!state || !walker || !route || state->leg >= route->legs.size()) {
      return;
    }
    std::span<const road::WalkEdge> edges = network_->walking.edges();
    double along = state->along.numerical_value_in(meter);
    const road::Leg& leg = route->legs[state->leg];
    double left = edges[leg.edge].length() - along;
    std::optional<double> gap;
    const WalkOccupancy::Walking* ahead =
        occupancy_.find_ahead(leg, along, self);
    if (ahead) {
      gap = ahead->along - along;
    } else if (state->leg + 1 < route->legs.size() && left < LOOKAHEAD) {
      ahead = occupancy_.find_ahead(route->legs[state->leg + 1], -1.0, self);
      if (ahead) {
        gap = left + ahead->along;
      }
    }
    double desired = walker->desired_speed.numerical_value_in(meter_per_second);
    double speed =
        gap ? std::clamp((*gap - SPACE) / HEADWAY, 0.0, desired) : desired;

    std::uint32_t decided = command.crossing;
    command = WalkCommand{};
    if (edges[leg.edge].kind == road::WalkEdge::Kind::CROSSING) {
      command.crossing = edges[leg.edge].crosswalk;
      command.on = true;
      command.clear = left / desired * second;
    } else if (crossings_ && state->leg + 1 < route->legs.size() &&
               edges[route->legs[state->leg + 1].edge].kind ==
                   road::WalkEdge::Kind::CROSSING) {
      const road::WalkEdge& crossing = edges[route->legs[state->leg + 1].edge];
      // One waiting close behind another goes with it as it steps on.
      bool go = decided == crossing.crosswalk ||
                (ahead && *gap <= GROUP &&
                 ahead->edge == route->legs[state->leg + 1].edge);
      if (!go && left <= KERB) {
        go = may_cross(crossing, *walker);
      }
      if (go) {
        command.crossing = crossing.crosswalk;
        command.clear = (left + crossing.length()) / desired * second;
      } else {
        // Waits at the kerb.
        speed =
            std::min(speed, std::clamp((left - WAIT) / HEADWAY, 0.0, desired));
      }
    }
    command.speed = speed * meter_per_second;
  }

 private:
  // Whether a pedestrian may step onto `crossing` now.
  auto may_cross(const road::WalkEdge& crossing, const Walker& walker) const
      -> bool {
    double speed = walker.desired_speed.numerical_value_in(meter_per_second);
    double across = crossing.length() / speed;  // s.
    const std::optional<std::uint32_t>& group =
        network_->crosswalk_groups[crossing.crosswalk];
    if (group && walker.complies) {
      const traffic::SignalPlan* plan = plans_[*group];
      return aspects_[*group] == traffic::Aspect::RED && plan &&
             std::chrono::duration<double>(plan->keeps_aspect(now_)).count() >=
                 across;
    }
    double critical = across + walker.start_up.numerical_value_in(second);
    auto [first, last] = std::ranges::equal_range(
        by_crosswalk_, crossing.crosswalk, {},
        &std::pair<std::uint32_t, std::uint32_t>::first);
    for (auto it = first; it != last; ++it) {
      const road::CrosswalkZone& zone = network_->walking.zones()[it->second];
      for (std::uint32_t k = first_[it->second]; k < first_[it->second + 1];
           ++k) {
        auto [lane, to] = upstream_[k];
        for (const LaneOccupancy::Occupant& vehicle :
             vehicles_.occupants_of(lane)) {
          double distance = to - vehicle.along;  // To the near edge.
          if (lane == zone.lane && distance <= 0.0) {
            if (vehicle.along - vehicle.length < zone.far) {
              return false;  // On the crosswalk.
            }
            continue;  // Past it.
          }
          const traffic::IntelligentDriver& driver = vehicle.driver->following;
          double arrival =
              traffic::compute_soonest_arrival(
                  distance * meter, vehicle.speed * meter_per_second,
                  driver.acceleration, driver.desired_speed)
                  .numerical_value_in(second);
          double comfortable =
              driver.deceleration.numerical_value_in(meter_per_second_squared);
          bool stops = network_->vehicles_yield &&
                       vehicle.speed * vehicle.speed / (2.0 * comfortable) <=
                           distance - 1.0;
          if (arrival < critical && !stops) {
            return false;
          }
        }
      }
    }
    return true;
  }

  const Network* network_ = nullptr;
  WalkOccupancy occupancy_;
  LaneOccupancy vehicles_;
  std::vector<traffic::Aspect> aspects_;  // By signal group.
  std::vector<const traffic::SignalPlan*> plans_;
  std::vector<std::pair<LaneKey, double>> upstream_;  // By zone.
  std::vector<std::uint32_t> first_;                  // Each zone's start.
  std::vector<std::pair<std::uint32_t, std::uint32_t>> by_crosswalk_;
  std::chrono::nanoseconds now_{};
  bool crossings_ = false;
};

// Each step, each pedestrian walks its route at the speed it set, leg after
// leg; at the route's end it sets out for a new place.
struct Walk final                //
    : System<WalkState,          //
             const WalkCommand,  //
             const Walker,       //
             WalkRoute> {
  using SystemWorld = ProjectedWorld<Walk>;
  using SequenceAfterSystemList = SystemList<Pace>;

  explicit Walk(const Network& network) : network_{&network} {}

  auto operator()(SystemWorld&, Entity,        //
                  WalkState& state,            //
                  const WalkCommand* command,  //
                  const Walker* walker,        //
                  WalkRoute* route,            //
                  Step step) const -> void {
    if (!command || !walker || !route || route->legs.empty()) {
      return;
    }
    double dt = std::chrono::duration<double>(step.dt).count();
    state.speed = command->speed;
    double along = state.along.numerical_value_in(meter) +
                   command->speed.numerical_value_in(meter_per_second) * dt;
    std::span<const road::WalkEdge> edges = network_->walking.edges();
    while (along >= edges[route->legs[state.leg].edge].length()) {
      along -= edges[route->legs[state.leg].edge].length();
      if (++state.leg < route->legs.size()) {
        continue;
      }
      // There: on to somewhere new, from where it stands.
      std::uint32_t node = find_route_end(*network_, route->legs);
      std::vector<road::Leg> next =
          plan_walk(*network_, node, walker->seed, ++route->trips);
      state.leg = 0;
      if (next.empty()) {
        state.leg = static_cast<std::uint32_t>(route->legs.size() - 1);
        along = edges[route->legs[state.leg].edge].length();
        break;
      }
      route->legs = std::move(next);
    }
    state.along = along * meter;
  }

 private:
  const Network* network_ = nullptr;
};

// After Walk, each pedestrian's place in the world follows its place on its
// route, facing the way it walks.
struct PlaceWalker final       //
    : System<RoadPose,         //
             const WalkState,  //
             const WalkRoute> {
  using SystemWorld = ProjectedWorld<PlaceWalker>;
  using SequenceAfterSystemList = SystemList<Walk>;

  explicit PlaceWalker(const Network& network) : network_{&network} {}

  auto operator()(SystemWorld&, Entity,    //
                  RoadPose& pose,          //
                  const WalkState* state,  //
                  const WalkRoute* route) const -> void {
    if (!state || !route || state->leg >= route->legs.size()) {
      return;
    }
    pose = locate_walker(*network_, *route, *state);
  }

  static auto locate_walker(const Network& network, const WalkRoute& route,
                            const WalkState& state) -> RoadPose {
    auto [point, heading] = network.walking.locate(
        route.legs[state.leg], state.along.numerical_value_in(meter));
    return RoadPose{.position = meters(point.x, point.y, point.z),
                    .heading = heading * radian};
  }

 private:
  const Network* network_ = nullptr;
};

using Schedule =
    SystemList<RunSignals, Pace, Decide, Drive, FollowLane, Walk, PlaceWalker>;

// The schedule's systems on `network`.
inline auto make_schedule(const Network& network) -> Schedule {
  return Schedule{RunSignals{},        Pace{network},       Decide{network},
                  Drive{network},      FollowLane{network}, Walk{network},
                  PlaceWalker{network}};
}
using Scheduler = framework::Scheduler<World, Schedule>;

//-- Scenarios ----------------------------------------------------------------

template <typename SystemType>
using ScenarioProjectedWorld =
    framework::ProjectedWorld<SystemType, ScenarioWorld>;

// Each step, the storyboard's turn: it learns which actions every vehicle
// finished last step and where every vehicle is, evaluates its triggers, and
// orders each vehicle's actions to stop and start. The first step starts the
// storyboard, its init actions, and then evaluates it as any other.
struct RunStoryboard final    //
    : System<ScenarioOrders,  //
             const ScenarioActor> {
  using SystemWorld = ScenarioProjectedWorld<RunStoryboard>;
  using AllowComponentList =
      TypeList<ScenarioActor, ScenarioSpeed, ScenarioMotion>;

  explicit RunStoryboard(ScenarioContext& context) : context_{&context} {}

  auto prepare(SystemWorld& world, Step step) -> bool {
    ScenarioContext& context = *context_;
    double time =
        std::chrono::duration<double>(step.time.time_since_epoch()).count();
    std::size_t count = context.scenario->entities.size();
    context.states.resize(count);
    std::vector<std::uint32_t> finished;
    const auto& speeds = world.store_of<ScenarioSpeed>();
    const auto& motions = world.store_of<ScenarioMotion>();
    world.store_of<ScenarioActor>().for_each(
        [&](Entity owner, const ScenarioActor& actor) {
          const ScenarioSpeed* speed = speeds.maybe_component_of(owner);
          const ScenarioMotion* motion = motions.maybe_component_of(owner);
          if (speed == nullptr || motion == nullptr || actor.entity >= count) {
            return;
          }
          context.states[actor.entity] = scenario::EntityState{
              .placement = motion->placement,
              .pose = motion->pose.value_or(road::compute_placement_pose(
                  *context.roads, motion->placement)),
              .speed = speed->speed,
              .acceleration = speed->acceleration,
              .end_of_road = motion->end_of_road};
          finished.insert(finished.end(), speed->finished.begin(),
                          speed->finished.end());
          finished.insert(finished.end(), motion->finished.begin(),
                          motion->finished.end());
        });
    orders_.assign(count, {});
    if (!context.started) {
      context.started = true;
      deal(context.player->start(time), false);
    }
    deal(context.player->step(time, context.states, finished), true);
    return true;
  }

  auto operator()(SystemWorld&, Entity,    //
                  ScenarioOrders& orders,  //
                  const ScenarioActor* actor) const -> void {
    if (actor == nullptr || actor->entity >= orders_.size()) {
      orders = {};
      return;
    }
    orders = orders_[actor->entity];
  }

 private:
  // Hands each order to its entity, a stop to every entity, since a handle
  // names its action alone.
  // A teleport is found where it is given, so that a position relative to
  // an entity teleported before it finds that entity where it went.
  auto deal(scenario::StoryboardOrders given, bool holding) -> void {
    ScenarioContext& context = *context_;
    for (ScenarioOrders& orders : orders_) {
      orders.stops.insert(orders.stops.end(), given.stops.begin(),
                          given.stops.end());
    }
    for (const scenario::ActionOrder& order : given.starts) {
      if (order.entity >= orders_.size()) {
        continue;
      }
      orders_[order.entity].starts.push_back(order);
      if (const auto* teleport =
              std::get_if<scenario::TeleportAction>(order.action)) {
        road::Placement placement =
            context.player->locate(teleport->position, context.states);
        orders_[order.entity].teleports.push_back(placement);
        orders_[order.entity].held = orders_[order.entity].held || holding;
        context.states[order.entity].placement = placement;
        context.states[order.entity].pose =
            road::compute_placement_pose(*context.roads, placement);
      }
    }
  }

  ScenarioContext* context_ = nullptr;
  std::vector<ScenarioOrders> orders_;
};

// Each step, a vehicle's speed: its speed action started, stopped or run, as
// esmini runs it. The transition moves on by the step and gives the speed,
// within the vehicle's acceleration and deceleration, the transition held
// back where they limit it; once the transition ends the speed closes on the
// target within the same limits. A relative target follows its entity's
// speed at the step's start. A vehicle stuck at the end of its road stops.
struct ControlSpeed final    //
    : System<ScenarioSpeed,  //
             const ScenarioActor, const ScenarioOrders, const ScenarioMotion> {
  using SystemWorld = ScenarioProjectedWorld<ControlSpeed>;
  using SequenceAfterSystemList = SystemList<RunStoryboard>;

  explicit ControlSpeed(ScenarioContext& context) : context_{&context} {}

  auto operator()(SystemWorld&, Entity,          //
                  ScenarioSpeed& speed,          //
                  const ScenarioActor* actor,    //
                  const ScenarioOrders* orders,  //
                  const ScenarioMotion* motion,  //
                  Step step) const -> void;

 private:
  ScenarioContext* context_ = nullptr;
};

// Each step, an entity's place on the road: teleported, moved by its lateral
// action, a lane change or lane offset, or along its trajectory, as esmini
// moves it, or else carried along its lane at its speed, into a junction by
// its route. A lateral action keeps the entity's path as long as its speed
// allows, its lateral motion taken from it, and turns its heading to its
// path. A trajectory holds the entity to its polyline, and at its end the
// entity goes on along its heading for the rest of the step.
struct MoveOnRoad final       //
    : System<ScenarioMotion,  //
             const ScenarioActor, const ScenarioOrders, const ScenarioSpeed> {
  using SystemWorld = ScenarioProjectedWorld<MoveOnRoad>;
  using SequenceAfterSystemList = SystemList<ControlSpeed>;

  explicit MoveOnRoad(ScenarioContext& context) : context_{&context} {}

  auto operator()(SystemWorld&, Entity,          //
                  ScenarioMotion& motion,        //
                  const ScenarioActor* actor,    //
                  const ScenarioOrders* orders,  //
                  const ScenarioSpeed* speed,    //
                  Step step) const -> void;

 private:
  ScenarioContext* context_ = nullptr;
};

// After MoveOnRoad, each entity's place in the world: its reference point,
// the middle of a vehicle's rear axle, where its trajectory put it or at its
// road placement.
struct PlaceOnRoad final  //
    : System<RoadPose,    //
             const ScenarioMotion> {
  using SystemWorld = ScenarioProjectedWorld<PlaceOnRoad>;
  using SequenceAfterSystemList = SystemList<MoveOnRoad>;

  explicit PlaceOnRoad(ScenarioContext& context) : context_{&context} {}

  auto operator()(SystemWorld&, Entity,  //
                  RoadPose& pose,        //
                  const ScenarioMotion* motion) const -> void {
    if (motion == nullptr) {
      return;
    }
    road::PlacementPose at = motion->pose.value_or(
        road::compute_placement_pose(*context_->roads, motion->placement));
    pose = RoadPose{.position = meters(at.x, at.y, at.z),
                    .heading = at.heading * radian};
  }

 private:
  ScenarioContext* context_ = nullptr;
};

using ScenarioSchedule =
    SystemList<RunStoryboard, ControlSpeed, MoveOnRoad, PlaceOnRoad>;
using ScenarioScheduler = framework::Scheduler<ScenarioWorld, ScenarioSchedule>;

}  // namespace simon::automotive
