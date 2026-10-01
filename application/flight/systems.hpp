// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>

#include "application/flight/components.hpp"
#include "engine/rate_gate.hpp"
#include "framework/continuous.hpp"
#include "framework/system.hpp"
#include "model/atmosphere.hpp"
#include "model/control.hpp"
#include "model/flight_path.hpp"

namespace simon::flight {

using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

template <typename SystemType>
using WorldAccess = framework::WorldAccess<SystemType, World>;

//-- Guidance and control: discrete, at their own rates ------------------------

// Once a second, each aircraft steers its autopilot at its route's next
// waypoint, and moves on to the one after when it is within capture range.
struct FollowRoute final : System<Route, const AirState, Autopilot> {
  using LocalWorld = WorldAccess<FollowRoute>;

  static constexpr Length CAPTURE = 3000.0 * model::meter;

  auto prepare(LocalWorld&, Step step) -> bool {
    return gate_.fire(step).has_value();
  }

  auto operator()(LocalWorld&, Entity, Route& route, const AirState* state,
                  Autopilot* autopilot) const -> void {
    if (!state || !autopilot) {
      return;
    }
    if (model::ground_distance(state->position, route.waypoints[route.next]) <
        CAPTURE) {
      route.next = (route.next + 1) % Route::SIZE;
      ++route.reached;
    }
    const Position& waypoint = route.waypoints[route.next];
    autopilot->heading = model::bearing(state->position, waypoint);
    autopilot->altitude = waypoint.numerical_value_in(model::meter).z() *
                          model::meter;
    autopilot->speed = route.speed;
  }

 private:
  engine::RateGate gate_{1s};
};

// The autopilot's gains, shared by every aircraft.
struct AutopilotGains final {
  Rate altitude = 0.2 * model::per_second;  // Climb rate per meter of error.
  Angle steepest_climb = 0.25 * model::radian;
  // How slow an aircraft may be before it stops climbing: the steepest climb
  // shrinks to nothing as the speed falls this far below its target, so a
  // climb never trades away more speed than that.
  Speed speed_margin = 20.0 * model::meter_per_second;
  Rate climb = 1.0 * model::per_second;  // Of the flight-path angle error.
  Rate heading = 0.5 * model::per_second;
  model::PiGains speed{.proportional = 0.05, .integral = 0.02};  // Per m/s.
};

// Ten times a second, each aircraft's autopilot turns its targets into
// commands: a bank for the heading, a load factor for the altitude, and a
// throttle for the speed.
struct FlyAutopilot final : System<Commands, const AirState, const Handling,
                                   const FlightControls, Autopilot> {
  using LocalWorld = WorldAccess<FlyAutopilot>;
  using SequenceAfterSystemList = SystemList<FollowRoute>;

  auto prepare(LocalWorld&, Step step) -> bool {
    auto firing = gate_.fire(step);
    elapsed_ = firing ? model::seconds(firing->elapsed) : 0.0 * model::second;
    return firing.has_value();
  }

  auto operator()(LocalWorld&, Entity, Commands& commands,
                  const AirState* state, const Handling* handling,
                  const FlightControls* controls, Autopilot* autopilot) const
      -> void {
    if (!state || !handling || !controls || !autopilot) {
      return;
    }
    double speed_error =
        (autopilot->speed - state->speed).numerical_value_in(
            model::meter_per_second);
    commands.bank = model::bank_command(*state, autopilot->heading,
                                        gains_.heading, handling->max_bank);
    Angle climb = model::climb_command(*state, autopilot->altitude,
                                       gains_.altitude, gains_.steepest_climb);
    // Speed comes first: a slow aircraft climbs less steeply, or not at all.
    double slow = std::clamp(
        1.0 - speed_error / gains_.speed_margin.numerical_value_in(
                                model::meter_per_second),
        0.0, 1.0);
    climb = std::min(climb, gains_.steepest_climb * slow);
    commands.load_factor =
        std::clamp(model::load_factor_command(*state, climb, controls->bank,
                                              gains_.climb),
                   handling->min_load_factor, handling->max_load_factor);
    commands.throttle = model::pi_control(
        speed_error, gains_.speed, elapsed_,
        lib::InOut(autopilot->throttle_integral));
  }

 private:
  engine::RateGate gate_{100ms};
  Time elapsed_ = 0.0 * model::second;  // Since the gate last fired.
  AutopilotGains gains_;
};

// Every step, each airframe follows its commands: the load factor and
// throttle through first-order lags, the bank at no more than its roll rate.
struct Actuate final
    : System<FlightControls, const Commands, const Handling> {
  using LocalWorld = WorldAccess<Actuate>;
  using SequenceAfterSystemList = SystemList<FlyAutopilot>;

  auto operator()(LocalWorld&, Entity, FlightControls& controls,
                  const Commands* commands, const Handling* handling,
                  Step step) const -> void {
    if (!commands || !handling) {
      return;
    }
    Time dt = model::seconds(step.dt);
    controls.load_factor = model::lag(controls.load_factor,
                                      commands->load_factor,
                                      handling->load_factor_lag, dt);
    controls.bank =
        model::approach(controls.bank, commands->bank, handling->roll_rate * dt);
    controls.throttle = model::lag(controls.throttle, commands->throttle,
                                   handling->throttle_lag, dt);
  }
};

//-- Dynamics -----------------------------------------------------------------

// The rate of each aircraft's AirState under its controls, in the air at its
// altitude.
inline auto rate_of(const AirState& state, const FlightControls& controls,
                    const Airframe& airframe,
                    const model::StandardAirTable& air) -> AirStateRate {
  return model::point_mass_rate(state, controls, airframe,
                                air(model::altitude_of(state)));
}

// The default: each aircraft advances in one semi-implicit pass per step.
// Aircraft that have an AirStateRate are left to Precise; for the default
// archetype the rate is absent, so that check compiles away.
struct Fly final : System<AirState, const FlightControls, const Airframe,
                          const AirStateRate> {
  using LocalWorld = WorldAccess<Fly>;
  using SequenceAfterSystemList = SystemList<Actuate>;

  auto operator()(LocalWorld&, Entity, AirState& state,
                  const FlightControls* controls, const Airframe* airframe,
                  const AirStateRate* precise, Step step) const -> void {
    if (precise || !controls || !airframe) {
      return;
    }
    state =
        model::fly(state, rate_of(state, *controls, *airframe, air_), step.dt);
  }

 private:
  model::StandardAirTable air_;
};

// The opt-in: the rate of each precise aircraft's AirState, for Continuous.
struct PointMassRates final
    : System<AirStateRate, const AirState, const FlightControls,
             const Airframe> {
  using LocalWorld = WorldAccess<PointMassRates>;

  auto operator()(LocalWorld&, Entity, AirStateRate& rate,
                  const AirState* state, const FlightControls* controls,
                  const Airframe* airframe) const -> void {
    if (!state || !controls || !airframe) {
      return;
    }
    rate = rate_of(*state, *controls, *airframe, air_);
  }

 private:
  model::StandardAirTable air_;
};

using Precise = framework::Continuous<framework::RungeKutta4,
                                      TypeList<AirState>,
                                      SystemList<PointMassRates>>;

//-- Schedule -----------------------------------------------------------------

using Schedule = SystemList<FollowRoute, FlyAutopilot, Actuate, Fly, Precise>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::flight
