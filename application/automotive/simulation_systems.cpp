// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows esmini 3.8.2, Copyright (c) partners of Simulation Scenarios,
// MPL-2.0; translated to C++ and changed. See NOTICE.md.

#include "application/automotive/simulation_systems.hpp"
#include "core/argument.hpp"
#include "core/units.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <variant>

namespace simon::automotive {

namespace {

namespace osc = scenario;

constexpr double SMALL = 1e-10;

auto convert_to_seconds(const Step& step) -> double {
  return std::chrono::duration<double>(step.dt).count();
}

auto stopped(const ScenarioOrders* orders, std::uint32_t handle) -> bool {
  return orders != nullptr &&
         std::ranges::find(orders->stops, handle) != orders->stops.end();
}

}  // namespace

//-- ControlSpeed -------------------------------------------------------------

auto ControlSpeed::operator()(SystemWorld&, Entity,          //
                              ScenarioSpeed& speed,          //
                              const ScenarioActor* actor,    //
                              const ScenarioOrders* orders,  //
                              const ScenarioMotion* motion,  //
                              Step step) const -> void {
  speed.finished.clear();
  // Once the storyboard stops, nothing moves.
  if (actor == nullptr || !context_->player->running()) {
    return;
  }
  const ScenarioContext& context = *context_;
  const osc::Vehicle& vehicle =
      context.scenario->entities[actor->entity].vehicle;
  double dt = convert_to_seconds(step);
  double before = speed.speed;

  // A vehicle stuck at the end of its road has stopped.
  if (motion != nullptr && motion->end_of_road > 0.0) {
    speed.speed = 0.0;
  }
  if (speed.change && stopped(orders, speed.change->handle)) {
    speed.change.reset();
  }

  // The target a relative speed action follows: its entity's speed at the
  // step's start, by a difference or a factor.
  auto relative_target = [&](const SpeedChange& change) {
    double reference = context.states[*change.relative_to].speed;
    return change.kind == osc::RelativeTargetSpeed::Kind::FACTOR
               ? reference * change.relative_value
               : reference + change.relative_value;
  };
  auto limit = [&](double value) {
    return std::clamp(value, -vehicle.max_speed, vehicle.max_speed);
  };

  if (orders != nullptr) {
    for (const osc::ActionOrder& order : orders->starts) {
      const auto* action = std::get_if<osc::SpeedAction>(order.action);
      if (action == nullptr) {
        continue;
      }
      SpeedChange change{.handle = order.handle};
      change.transition.shape = action->dynamics.shape;
      change.transition.start = speed.speed;
      double target = 0.0;
      if (const auto* absolute =
              std::get_if<osc::AbsoluteTargetSpeed>(&action->target)) {
        target = absolute->value;
      } else {
        const auto& relative =
            std::get<osc::RelativeTargetSpeed>(action->target);
        change.relative_to = context.player->find_entity(relative.entity);
        change.kind = relative.kind;
        change.relative_value = relative.value;
        change.continuous = relative.continuous;
        target = relative_target(change);
      }
      change.transition.target = limit(target);
      double b = change.transition.target - change.transition.start;
      switch (action->dynamics.dimension) {
        case osc::DynamicsDimension::TIME:
          change.transition.end = action->dynamics.value;
          break;
        case osc::DynamicsDimension::RATE:
          change.transition.end =
              change.transition.compute_end_at_peak(action->dynamics.value);
          break;
        case osc::DynamicsDimension::DISTANCE: {
          // Speed changes over time, so a distance becomes the time to
          // cover it: 2 d / |v0 + v1| without a turn of direction, else the
          // two halves either side of standing still.
          double v0 = change.transition.start;
          double v1 = change.transition.target;
          double distance = action->dynamics.value;
          if (std::abs(b) < SMALL) {
            change.transition.end = 0.0;
          } else if (std::abs(v0) > SMALL && std::abs(v1) > SMALL &&
                     sign(v0) != sign(v1)) {
            double share = v0 * v0 / (v0 * v0 + v1 * v1);
            change.transition.end =
                2.0 * share * distance / std::abs(v0) +
                2.0 * (1.0 - share) * distance / std::abs(v1);
          } else {
            change.transition.end = 2.0 * distance / std::abs(v0 + v1);
          }
          break;
        }
      }
      // No faster than the vehicle can speed up or slow down: the
      // transition is lengthened until its rate peaks within the limit.
      change.transition.stretch_to(b > 0.0 ? vehicle.max_acceleration
                                           : vehicle.max_deceleration);
      speed.speed = change.transition.evaluate();
      speed.change = change;
    }
  }

  speed.unstepped = speed.speed;
  speed.stepped_by.reset();
  if (speed.change) {
    SpeedChange& change = *speed.change;
    speed.stepped_by = change.handle;
    if (change.relative_to) {
      change.transition.target = limit(relative_target(change));
    }
    double next = 0.0;
    if (change.transition.end > 0.0 && change.transition.done()) {
      double acceleration =
          std::clamp((change.transition.target - speed.speed) / dt,
                     -vehicle.max_deceleration, vehicle.max_acceleration);
      next = limit(speed.speed + acceleration * dt);
    } else {
      change.transition.advance(dt);
      next = change.transition.evaluate();
      double wanted = (next - speed.speed) / dt;
      if (wanted > vehicle.max_acceleration + SMALL ||
          wanted < -(vehicle.max_deceleration + SMALL)) {
        // Too sharp a change for the vehicle: it changes as fast as it
        // can, and the transition is held back by as much.
        double acceleration = std::clamp(wanted, -vehicle.max_deceleration,
                                         vehicle.max_acceleration);
        next = limit(speed.speed + acceleration * dt);
        double reached = (next - speed.speed) / wanted;
        change.transition.advance(-(dt - reached));
      }
    }
    if (change.transition.done() &&
        std::abs(next - change.transition.target) < SMALL) {
      change.reached = true;
      next = change.transition.target;
    }
    speed.speed = next;
    if (change.reached && !(change.relative_to && change.continuous)) {
      speed.finished.push_back(change.handle);
      speed.change.reset();
    }
  }
  speed.acceleration = (speed.speed - before) / dt;
}

//-- MoveOnRoad ---------------------------------------------------------------

auto MoveOnRoad::operator()(SystemWorld&, Entity,          //
                            ScenarioMotion& motion,        //
                            const ScenarioActor* actor,    //
                            const ScenarioOrders* orders,  //
                            const ScenarioSpeed* speed,    //
                            Step step) const -> void {
  motion.finished.clear();
  if (actor == nullptr || speed == nullptr || !context_->player->running()) {
    return;
  }
  const ScenarioContext& context = *context_;
  const road::Map& roads = *context.roads;
  double dt = convert_to_seconds(step);
  road::Placement& placement = motion.placement;

  if (motion.change && stopped(orders, motion.change->handle)) {
    motion.change.reset();
  }
  if (motion.trajectory && stopped(orders, motion.trajectory->handle)) {
    motion.trajectory.reset();
  }
  if (!motion.trajectory) {
    motion.pose.reset();
  }

  // The lateral offset from `lane`'s middle, positive to the left of the
  // lane's travel: esmini's offset agnostic of the side of the road.
  auto agnostic_offset = [&](int lane) {
    const road::Road& road = roads.roads[placement.road];
    return sign(lane) * (road::compute_placement_t(road, placement) -
                         road::compute_lane_center(road, placement.s, lane));
  };

  if (orders != nullptr) {
    std::size_t teleports = 0;
    for (const osc::ActionOrder& order : orders->starts) {
      if (std::holds_alternative<osc::TeleportAction>(*order.action)) {
        placement = orders->teleports[teleports++];
        motion.end_of_road = -1.0;
      } else if (const auto* change =
                     std::get_if<osc::LaneChangeAction>(order.action)) {
        int lane = 0;
        if (const auto* absolute =
                std::get_if<osc::AbsoluteTargetLane>(&change->target)) {
          lane = absolute->lane;
        } else {
          const auto& relative =
              std::get<osc::RelativeTargetLane>(change->target);
          const road::Placement& reference =
              context.states[context.player->find_entity(relative.entity)]
                  .placement;
          lane = reference.lane +
                 relative.lanes * (road::runs_forward(reference) ? 1 : -1);
          if (lane == 0 || (lane > 0) != (reference.lane > 0)) {
            lane = static_cast<int>(sign(lane - reference.lane)) *
                   (std::abs(lane) + 1);
          }
        }
        // The heading turns back to the road's, and the offset counts from
        // the target lane's middle.
        placement.heading =
            road::runs_forward(placement) ? 0.0 : std::numbers::pi;
        LateralChange lateral{.handle = order.handle,
                              .lane_change = true,
                              .lane = lane,
                              .dimension = change->dynamics.dimension};
        lateral.transition.shape = change->dynamics.shape;
        lateral.transition.start = agnostic_offset(lane);
        lateral.transition.target = sign(lane) * change->target_offset;
        lateral.transition.end =
            change->dynamics.dimension == osc::DynamicsDimension::RATE
                ? lateral.transition.compute_end_at_peak(change->dynamics.value)
                : change->dynamics.value;
        motion.change = lateral;
      } else if (const auto* offset =
                     std::get_if<osc::LaneOffsetAction>(order.action)) {
        LateralChange lateral{.handle = order.handle,
                              .lane_change = false,
                              .lane = placement.lane,
                              .dimension = osc::DynamicsDimension::TIME};
        lateral.transition.shape = offset->shape;
        lateral.transition.start = agnostic_offset(placement.lane);
        double target = offset->value;
        if (offset->relative_to) {
          const road::Placement& reference =
              context.states[context.player->find_entity(*offset->relative_to)]
                  .placement;
          const road::Road& road = roads.roads[placement.road];
          target =
              road::compute_placement_t(road, reference) +
              offset->value * (road::runs_forward(reference) ? 1.0 : -1.0) -
              road::compute_lane_center(road, placement.s, placement.lane);
        }
        lateral.transition.target = sign(placement.lane) * target;
        // The time a shape takes to move b at the given peak lateral
        // acceleration (esmini's GetTargetParamValByPrimPrimPeak).
        double b =
            std::abs(lateral.transition.target - lateral.transition.start);
        double a = std::max(offset->max_lateral_acceleration, SMALL);
        lateral.transition.end =
            offset->shape == osc::DynamicsShape::SINUSOIDAL
                ? std::numbers::pi * std::sqrt(b / (2.0 * a))
            : offset->shape == osc::DynamicsShape::STEP
                ? 0.0
                : std::sqrt(6.0 * b / a);
        motion.change = lateral;
      } else if (const auto* assign =
                     std::get_if<osc::AssignRouteAction>(order.action)) {
        std::vector<road::Placement> waypoints;
        for (const osc::Waypoint& waypoint : assign->route.waypoints) {
          waypoints.push_back(
              context.player->locate(waypoint.position, context.states));
        }
        motion.route = road::find_route(roads, *context.lanes, waypoints);
      } else if (const auto* follow =
                     std::get_if<osc::FollowTrajectoryAction>(order.action)) {
        std::vector<road::PolylinePoint> points;
        for (const osc::Vertex& vertex : follow->vertices) {
          road::PlacementPose at = road::compute_placement_pose(
              roads, context.player->locate(vertex.position, context.states));
          points.push_back({.x = at.x, .y = at.y});
        }
        // It faces back along the polyline if moving backward as it starts
        // (esmini's FollowTrajectoryAction::Start).
        motion.trajectory =
            TrajectoryRun{.handle = order.handle,
                          .polyline = road::Polyline{std::move(points)},
                          .along = follow->initial_distance_offset,
                          .backward = speed->unstepped < 0.0};
      }
    }
  }

  double v = speed->speed;
  std::optional<std::uint32_t> lateral_handle;
  if (motion.change) {
    lateral_handle = motion.change->handle;
  } else if (motion.trajectory) {
    lateral_handle = motion.trajectory->handle;
  }
  if (lateral_handle && speed->stepped_by &&
      *speed->stepped_by > *lateral_handle) {
    v = speed->unstepped;
  }
  road::Move moved = road::Move::ALONG;
  if (motion.trajectory) {
    // esmini's FollowTrajectoryAction::Step without timing: along the
    // polyline at the speed, the way it faced as it started.
    TrajectoryRun& run = *motion.trajectory;
    double before = run.along;
    double direction = (v < 0.0 ? -1.0 : 1.0) * (run.backward ? -1.0 : 1.0);
    double length = run.polyline.length();
    run.along = std::clamp(before + direction * std::abs(v) * dt, 0.0, length);
    road::PolylinePoint at = run.polyline.evaluate(run.along);
    double moving = direction * std::abs(v);
    bool ended = dt > 0.0 && ((moving > 0.0 && run.along > length - 1e-6) ||
                              (moving < 0.0 && run.along < 1e-6));
    if (ended) {
      double rest = std::abs(v) * dt - std::abs(run.along - before);
      at.x += rest * std::cos(at.heading);
      at.y += rest * std::sin(at.heading);
      motion.finished.push_back(run.handle);
    }
    double heading = at.heading + (run.backward ? std::numbers::pi : 0.0);
    placement = road::find_placement(roads, at.x, at.y, heading, placement.road)
                    .value_or(placement);
    motion.pose = road::PlacementPose{
        .x = at.x,
        .y = at.y,
        .z = road::compute_placement_pose(roads, placement).z,
        .heading = wrap(heading)};
    if (ended) {
      motion.trajectory.reset();
    }
    motion.end_of_road = -1.0;
    return;
  }
  if (motion.change && std::abs(v) > SMALL) {
    LateralChange& change = *motion.change;
    double rate = change.transition.compute_slope();
    double length = std::abs(v) * dt;
    double along = 0.0;
    double offset = 0.0;
    double heading = 0.0;
    if (change.dimension != osc::DynamicsDimension::DISTANCE) {
      double across = dt * std::abs(rate);
      along =
          std::sqrt(length * length - std::pow(std::min(across, length), 2));
      if (across < length + SMALL) {
        change.transition.advance(dt);
        offset = change.transition.evaluate();
        heading = std::atan2(sign(v) * sign(rate) * across, along);
      } else {
        // Too much lateral motion for the speed: all of the step goes
        // sideways, linearly.
        change.transition.advance(
            length * change.transition.end /
            std::abs(change.transition.target - change.transition.start));
        offset = change.transition.evaluate(osc::DynamicsShape::LINEAR);
        heading = sign(v) * sign(rate) * (0.5 * std::numbers::pi - SMALL);
      }
    } else {
      along = std::sqrt(length * length / (1.0 + rate * rate));
      change.transition.advance(along);
      offset = change.transition.evaluate();
      heading = std::atan(rate);
    }
    if (change.transition.shape == osc::DynamicsShape::STEP ||
        change.transition.end < SMALL) {
      along = length;
    }
    // Into the target lane at the new offset, then along the road.
    bool forward = road::runs_forward(placement);
    placement.lane = change.lane;
    placement.offset = offset * sign(change.lane);
    double ds = road::convert_distance_to_ds(roads, placement, sign(v) * along);
    moved = road::move_along(roads, InOut(placement), ds, motion.route);
    forward = road::runs_forward(placement);
    bool ended = change.transition.done() ||
                 std::abs(offset - change.transition.target) < SMALL ||
                 (change.transition.parameter > 0.0 &&
                  sign(offset - change.transition.target) !=
                      sign(change.transition.start - change.transition.target));
    if (ended || moved == road::Move::END_OF_ROAD) {
      placement.heading = forward ? 0.0 : std::numbers::pi;
      motion.finished.push_back(change.handle);
      motion.change.reset();
    } else {
      placement.heading = (forward ? 0.0 : std::numbers::pi) +
                          (forward ? 1.0 : -1.0) * sign(change.lane) * heading;
    }
  } else if (std::abs(v) > SMALL && !(orders != nullptr && orders->held)) {
    double ds = road::convert_distance_to_ds(roads, placement, v * dt);
    moved = road::move_along(roads, InOut(placement), ds, motion.route);
  }

  if (moved == road::Move::END_OF_ROAD) {
    motion.end_of_road =
        motion.end_of_road < 0.0 ? 0.0 : motion.end_of_road + dt;
  } else if (std::abs(v) > SMALL) {
    motion.end_of_road = -1.0;
  } else if (motion.end_of_road >= 0.0) {
    motion.end_of_road += dt;
  }
}

//-- Decide
//---------------------------------------------------------------------

auto Decide::prepare(SystemWorld& world, Step step) -> bool {
  lights_ = !network_->signals.stop_lines().empty();
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
                     *network_, yields_ ? &world.store_of<Stopped>() : nullptr);
  changing_ = gate_.fire(step).has_value();
  aspects_.assign(network_->signals.groups().size(), traffic::Aspect::GREEN);
  world.store_of<SignalState>().for_each(
      [&](Entity, const SignalState& signal) {
        if (signal.group < aspects_.size()) {
          aspects_[signal.group] = signal.aspect;
        }
      });
  return true;
}

auto Decide::operator()(SystemWorld&, Entity self,  //
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
          walks_ && tactical ? find_crosswalk(*state, *driver, along, *tactical)
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
  auto now = traffic::compute_idm_acceleration(driver->following, state->speed,
                                               leader);
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

auto Decide::find_wait(const LaneState& state, const Driver& driver,
                       double along, Entity self, Tactical& tactical) const
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
    std::span<const traffic::Approach> approaches = rights.approaches_of(index);
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

auto Decide::find_foe(std::span<const traffic::Approach> approaches,
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

auto Decide::waits_at_entry(const traffic::Approach& approach,
                            const LaneOccupancy::Occupant& foe,
                            const Tactical& tactical) const -> bool {
  double line_gap = tactical.braking.line_gap.numerical_value_in(meter);
  LaneOccupancy::Stop stop = occupancy_.stop_of(foe);
  return approach.toward == 0 && stop.since != TimePoint::max() &&
         !stop.committed &&
         foe.along >=
             find_lane_length(*network_, approach.lane) - line_gap - 0.5;
}

auto Decide::find_crosswalk(const LaneState& state, const Driver& driver,
                            double along, const Tactical& tactical) const
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
          (on_[zone.crosswalk] > 0.0 && reaching < on_[zone.crosswalk] + 1.0) ||
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

auto Decide::find_light(const LaneState& state, const Driver& driver,
                        double along, Tactical& tactical) const
    -> std::optional<Length> {
  std::span<const traffic::StopLine> all = network_->signals.stop_lines();
  bool held = false;
  std::optional<Length> light;
  double before = -along;  // From the vehicle to the lane's start.
  LaneKey key = state.lane;
  std::uint32_t turns = state.turns;
  while (!light && before < LOOKAHEAD) {
    for (const traffic::StopLine& line : network_->signals.stop_lines_on(key)) {
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

auto Decide::find_leader(const LaneKey& lane, double along, std::uint32_t turns,
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

auto Decide::find_leader_near_junctions(const LaneKey& lane, double along,
                                        std::uint32_t turns, std::uint64_t seed,
                                        Entity self) const
    -> std::optional<traffic::Leader> {
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
      return nearer(parting, traffic::Leader{.gap = distance * meter,
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

auto Decide::find_parting(const LaneKey& lane, double distance,
                          Entity self) const -> std::optional<traffic::Leader> {
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

auto Decide::find_merging(const LaneKey& from, const LaneKey& into,
                          double distance, Entity self) const
    -> std::optional<traffic::Leader> {
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
    std::span<const LaneOccupancy::Occupant> in = occupancy_.occupants_of(lane);
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

auto Decide::gap_to(const LaneOccupancy::Occupant& leader, double apart)
    -> traffic::Leader {
  return traffic::Leader{.gap = (apart - leader.length) * meter,
                         .speed = leader.speed * meter_per_second};
}

auto Decide::worth_changing(const LaneState& state, const Driver& driver,
                            Entity self, double along,
                            AccelerationMagnitude now,
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

}  // namespace simon::automotive
