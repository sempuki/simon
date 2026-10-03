// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/automotive/simulation_systems.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <variant>

namespace simon::automotive {

namespace {

namespace osc = scenario;

constexpr double SMALL = 1e-10;

auto seconds_of(const Step& step) -> double {
  return std::chrono::duration<double>(step.dt).count();
}

auto sign(double x) -> double { return x > 0.0 ? 1.0 : x < 0.0 ? -1.0 : 0.0; }

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
  double dt = seconds_of(step);
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
  const model::RoadNetwork& roads = *context.roads;
  double dt = seconds_of(step);
  model::RoadPlacement& placement = motion.placement;

  if (motion.change && stopped(orders, motion.change->handle)) {
    motion.change.reset();
  }

  // The lateral offset from `lane`'s middle, positive to the left of the
  // lane's travel: esmini's offset agnostic of the side of the road.
  auto agnostic_offset = [&](int lane) {
    const model::Road& road = roads.roads[placement.road];
    return sign(lane) * (model::compute_placement_t(road, placement) -
                         model::compute_lane_center(road, placement.s, lane));
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
          const model::RoadPlacement& reference =
              context.states[context.player->find_entity(relative.entity)]
                  .placement;
          lane = reference.lane +
                 relative.lanes * (model::runs_forward(reference) ? 1 : -1);
          if (lane == 0 || (lane > 0) != (reference.lane > 0)) {
            lane = static_cast<int>(sign(lane - reference.lane)) *
                   (std::abs(lane) + 1);
          }
        }
        // The heading turns back to the road's, and the offset counts from
        // the target lane's middle.
        placement.heading =
            model::runs_forward(placement) ? 0.0 : std::numbers::pi;
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
          const model::RoadPlacement& reference =
              context.states[context.player->find_entity(*offset->relative_to)]
                  .placement;
          const model::Road& road = roads.roads[placement.road];
          target =
              model::compute_placement_t(road, reference) +
              offset->value * (model::runs_forward(reference) ? 1.0 : -1.0) -
              model::compute_lane_center(road, placement.s, placement.lane);
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
      }
    }
  }

  double v = speed->speed;
  if (motion.change && speed->stepped_by &&
      *speed->stepped_by > motion.change->handle) {
    v = speed->unstepped;
  }
  model::RoadMove moved = model::RoadMove::ALONG;
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
    bool forward = model::runs_forward(placement);
    placement.lane = change.lane;
    placement.offset = offset * sign(change.lane);
    double ds =
        model::convert_distance_to_ds(roads, placement, sign(v) * along);
    moved = model::move_along_road(roads, InOut(placement), ds);
    forward = model::runs_forward(placement);
    bool ended = change.transition.done() ||
                 std::abs(offset - change.transition.target) < SMALL ||
                 (change.transition.parameter > 0.0 &&
                  sign(offset - change.transition.target) !=
                      sign(change.transition.start - change.transition.target));
    if (ended || moved == model::RoadMove::END_OF_ROAD) {
      placement.heading = forward ? 0.0 : std::numbers::pi;
      motion.finished.push_back(change.handle);
      motion.change.reset();
    } else {
      placement.heading = (forward ? 0.0 : std::numbers::pi) +
                          (forward ? 1.0 : -1.0) * sign(change.lane) * heading;
    }
  } else if (std::abs(v) > SMALL && !(orders != nullptr && orders->held)) {
    double ds = model::convert_distance_to_ds(roads, placement, v * dt);
    moved = model::move_along_road(roads, InOut(placement), ds);
  }

  if (moved == model::RoadMove::END_OF_ROAD) {
    motion.end_of_road =
        motion.end_of_road < 0.0 ? 0.0 : motion.end_of_road + dt;
  } else if (std::abs(v) > SMALL) {
    motion.end_of_road = -1.0;
  } else if (motion.end_of_road >= 0.0) {
    motion.end_of_road += dt;
  }
}

}  // namespace simon::automotive
