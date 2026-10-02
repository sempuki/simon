// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include "application/flight/components.hpp"
#include "engine/rate_gate.hpp"
#include "framework/continuous.hpp"
#include "framework/system.hpp"
#include "model/atmosphere.hpp"
#include "model/control.hpp"
#include "model/flight_path.hpp"
#include "model/rigid_aircraft.hpp"

namespace simon::flight {

using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

//-- Guidance and control: discrete, at their own rates ------------------------

// Once a second, each aircraft steers its autopilot at its route's next
// waypoint, and moves on to the one after when it is within capture range.
struct FollowRoute final      //
    : System<Route,           //
             const AirState,  //
             Autopilot> {
  using SystemWorld = ProjectedWorld<FollowRoute>;

  static constexpr Length CAPTURE = 3000.0 * model::meter;

  auto prepare(SystemWorld&, Step step) -> bool {
    return gate_.fire(step).has_value();
  }

  auto operator()(SystemWorld&, Entity,   //
                  Route& route,           //
                  const AirState* state,  //
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
    autopilot->altitude = model::altitude_of(waypoint);
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
  // Throttle, from none to full, for the speed error.
  model::PiGains<Speed> speed{
      .proportional = 0.05 * model::second / model::meter,
      .integral = 0.02 / model::meter,
      .low = 0.0,
      .high = 1.0};
};

// Ten times a second, each aircraft's autopilot turns its targets into
// commands: a bank for the heading, a load factor for the altitude, and a
// throttle for the speed.
struct FlyAutopilot final           //
    : System<Commands,              //
             const AirState,        //
             const Handling,        //
             const FlightControls,  //
             Autopilot> {
  using SystemWorld = ProjectedWorld<FlyAutopilot>;
  using SequenceAfterSystemList = SystemList<FollowRoute>;

  auto prepare(SystemWorld&, Step step) -> bool {
    auto firing = gate_.fire(step);
    elapsed_ = firing ? model::seconds(firing->elapsed) : 0.0 * model::second;
    return firing.has_value();
  }

  auto operator()(SystemWorld&, Entity,            //
                  Commands& commands,              //
                  const AirState* state,           //
                  const Handling* handling,        //
                  const FlightControls* controls,  //
                  Autopilot* autopilot) const -> void {
    if (!state || !handling || !controls || !autopilot) {
      return;
    }
    Speed speed_error = autopilot->speed - state->speed;
    commands.bank = model::bank_command(*state, autopilot->heading,
                                        gains_.heading, handling->max_bank);

    // Speed comes first: a slow aircraft climbs less steeply, or not at all.
    Angle climb = model::climb_command(*state, autopilot->altitude,
                                       gains_.altitude, gains_.steepest_climb);
    double slow = std::clamp(
        1.0 - model::number_of(speed_error / gains_.speed_margin), 0.0, 1.0);
    climb = model::min(climb, gains_.steepest_climb * slow);

    commands.load_factor = std::clamp(
        model::load_factor_command(*state, climb, controls->bank, gains_.climb),
        handling->min_load_factor, handling->max_load_factor);
    commands.throttle =
        model::pi_control(speed_error, gains_.speed, elapsed_,
                          lib::InOut(autopilot->throttle_integral));
  }

 private:
  engine::RateGate gate_{100ms};
  Time elapsed_ = 0.0 * model::second;  // Since the gate last fired.
  AutopilotGains gains_;
};

// Every step, each airframe follows its commands: the load factor and
// throttle through first-order lags, the bank at no more than its roll rate.
struct Actuate final          //
    : System<FlightControls,  //
             const Commands,  //
             const Handling> {
  using SystemWorld = ProjectedWorld<Actuate>;
  using SequenceAfterSystemList = SystemList<FlyAutopilot>;

  auto operator()(SystemWorld&, Entity,      //
                  FlightControls& controls,  //
                  const Commands* commands,  //
                  const Handling* handling,  //
                  Step step) const -> void {
    if (!commands || !handling) {
      return;
    }
    Time dt = model::seconds(step.dt);
    controls.load_factor =
        model::lag(controls.load_factor, commands->load_factor,
                   handling->load_factor_lag, dt);
    controls.bank = model::approach(controls.bank, commands->bank,
                                    handling->roll_rate * dt);
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
// Aircraft that have an AirStateRate are Precise's, and those that have a
// RigidBody are Rigid's, so Fly excludes them; the runner skips their
// archetypes' segments whole.
struct Fly final                    //
    : System<AirState,              //
             const FlightControls,  //
             const Airframe> {
  using SystemWorld = ProjectedWorld<Fly>;
  using SequenceAfterSystemList = SystemList<Actuate>;
  using ExcludeComponentList = TypeList<AirStateRate, RigidBody>;

  auto operator()(SystemWorld&, Entity,            //
                  AirState& state,                 //
                  const FlightControls* controls,  //
                  const Airframe* airframe,        //
                  Step step) const -> void {
    if (!controls || !airframe) {
      return;
    }
    state =
        model::fly(state, rate_of(state, *controls, *airframe, air_), step.dt);
  }

 private:
  model::StandardAirTable air_;
};

// The opt-in: the rate of each precise aircraft's AirState, for Continuous.
struct PointMassRates final         //
    : System<AirStateRate,          //
             const AirState,        //
             const FlightControls,  //
             const Airframe> {
  using SystemWorld = ProjectedWorld<PointMassRates>;

  auto operator()(SystemWorld&, Entity,            //
                  AirStateRate& rate,              //
                  const AirState* state,           //
                  const FlightControls* controls,  //
                  const Airframe* airframe) const -> void {
    if (!state || !controls || !airframe) {
      return;
    }
    rate = rate_of(*state, *controls, *airframe, air_);
  }

 private:
  model::StandardAirTable air_;
};

using Precise =
    framework::Continuous<framework::RungeKutta4, TypeList<AirState>,
                          SystemList<PointMassRates>>;

// A rigid aircraft's autopilot gains. The targets come from the point-mass
// autopilot's laws; these fly the surfaces to them.
struct SurfaceGains final {
  double bank = 1.0;      // Aileron per radian of bank error.
  double roll = 2.0;      // Aileron per rad/s of roll rate.
  double climb = 1.2;     // Elevator per radian of flight-path angle error.
  double pitch = 4.0;     // Elevator per rad/s of pitch rate.
  double integral = 1.0;  // Elevator per radian second of the same error.
  double speed = 0.05;    // Throttle per m/s of speed error.
};

// Each step, each rigid aircraft's autopilot flies its surfaces toward the
// targets FollowRoute sets, as it sets every aircraft's: aileron for the bank
// its heading needs, elevator for the flight-path angle its altitude needs,
// and throttle for its speed. Bank, climb and speed laws are the point-mass
// autopilot's.
struct FlySurfaces final       //
    : System<FlightSignals,    //
             const AirState,   //
             const RigidBody,  //
             const Autopilot,  //
             EngineControls,   //
             SurfaceAutopilot> {
  using SystemWorld = ProjectedWorld<FlySurfaces>;
  using SequenceAfterSystemList = SystemList<FollowRoute>;

  explicit FlySurfaces(model::Earth earth = model::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,        //
                  FlightSignals& signals,      //
                  const AirState* state,       //
                  const RigidBody* body,       //
                  const Autopilot* autopilot,  //
                  EngineControls* controls,    //
                  SurfaceAutopilot* trim,      //
                  Step step) const -> void {
    if (!state || !body || !autopilot || !controls || !trim) {
      return;
    }
    Eigen::Matrix3d attitude = earth_.body_to_north_east_down(
        *body, model::seconds(step.time.time_since_epoch()));
    double bank = std::atan2(attitude(2, 1), attitude(2, 2));
    Eigen::Vector3d rates = earth_.air_rate(*body)
                                .numerical_value_in(model::radian_per_second)
                                .eigen();

    // Up to 45 degrees of bank: a 737 turns on a radius of about 4 km at
    // 200 m/s, near the 3 km FollowRoute captures a waypoint at, so it seldom
    // circles one.
    double bank_error = model::radians(model::bank_command(
                            *state, autopilot->heading, 0.5 * model::per_second,
                            0.79 * model::radian)) -
                        bank;
    // Speed comes first, as it does for point-mass aircraft: a slow
    // aircraft climbs less steeply, or not at all.
    double speed_error = (autopilot->speed - state->speed)
                             .numerical_value_in(model::meter_per_second);
    double steepest = 0.08 * std::clamp(1.0 - speed_error / 20.0, 0.0, 1.0);
    double climb_error =
        std::min(model::radians(model::climb_command(
                     *state, autopilot->altitude, 0.05 * model::per_second,
                     0.08 * model::radian)),
                 steepest) -
        model::radians(state->flight_path_angle);
    trim->climb_integral = std::clamp(
        trim->climb_integral +
            climb_error *
                model::seconds(step.dt).numerical_value_in(model::second),
        -0.1, 0.1);
    // A turn needs a pitch rate of g/V sin(bank) tan(bank) to hold its
    // flight path.
    double turn = model::STANDARD_GRAVITY.numerical_value_in(
                      model::meter_per_second_squared) /
                  state->speed.numerical_value_in(model::meter_per_second) *
                  std::sin(bank) * std::tan(bank);

    using enum model::FlightSignal;
    // The 737's elevator command is positive nose down.
    signals[AILERON_COMMAND] = std::clamp(
        gains_.bank * bank_error - gains_.roll * rates.x(), -1.0, 1.0);
    signals[ELEVATOR_COMMAND] = std::clamp(
        -gains_.climb * climb_error - gains_.pitch * (turn - rates.y()) -
            gains_.integral * trim->climb_integral,
        -1.0, 1.0);
    signals[PITCH_TRIM_COMMAND] = trim->pitch_trim;
    controls->throttle.fill(
        std::clamp(trim->throttle_trim + gains_.speed * speed_error, 0.0, 1.0));
  }

 private:
  model::Earth earth_;
  SurfaceGains gains_;
};

// Each step, each rigid aircraft's flight controls read its state and its
// pilot's commands, and set its control surfaces.
struct RunFlightControls final    //
    : System<FlightSignals,       //
             const RigidBody,     //
             const AircraftType,  //
             ControlSurfaces> {
  using SystemWorld = ProjectedWorld<RunFlightControls>;

  explicit RunFlightControls(model::Earth earth = model::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,       //
                  FlightSignals& signals,     //
                  const RigidBody* body,      //
                  const AircraftType* type,   //
                  ControlSurfaces* surfaces,  //
                  Step step) const -> void {
    if (!body || !type || !type->data || !surfaces) {
      return;
    }
    model::Time time = model::seconds(step.time.time_since_epoch());
    Eigen::Vector3d uvw = earth_.air_velocity(*body)
                              .numerical_value_in(model::meter_per_second)
                              .eigen();
    Eigen::Vector3d rates = earth_.air_rate(*body)
                                .numerical_value_in(model::radian_per_second)
                                .eigen();
    using enum model::FlightSignal;
    signals[MACH] = uvw.norm() / air_(earth_.altitude(*body, time))
                                     .speed_of_sound.numerical_value_in(
                                         model::meter_per_second);
    signals[ROLL_RATE] = rates.x();
    signals[PITCH_RATE] = rates.y();
    signals[YAW_RATE] = rates.z();
    signals[ALPHA] = std::atan2(uvw.z(), uvw.x());
    signals[BETA] = std::atan2(uvw.y(), std::hypot(uvw.x(), uvw.z()));

    model::run_flight_controls(type->data->flight_controls, signals,
                               model::seconds(step.dt));
    *surfaces = ControlSurfaces{
        .elevator = signals[ELEVATOR],
        .left_aileron = signals[LEFT_AILERON],
        .right_aileron = signals[RIGHT_AILERON],
        .rudder = signals[RUDDER],
        .flaps = signals[FLAPS],
        .gear = signals[GEAR],
        .speedbrake = signals[SPEEDBRAKE],
        .spoilers = signals[SPOILERS],
    };
  }

 private:
  model::Earth earth_;
  model::StandardAirTable air_;
};

// Each step, each rigid aircraft's engines run at their throttles in the air
// they breathe, and set the thrust and fuel flow the step holds.
struct RunEngines final             //
    : System<Engines,               //
             const RigidBody,       //
             const EngineControls,  //
             const FuelTanks,       //
             const AircraftType> {
  using SystemWorld = ProjectedWorld<RunEngines>;
  using SequenceAfterSystemList = SystemList<RunFlightControls>;

  explicit RunEngines(model::Earth earth = model::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,            //
                  Engines& engines,                //
                  const RigidBody* body,           //
                  const EngineControls* controls,  //
                  const FuelTanks* tanks,          //
                  const AircraftType* type,        //
                  Step step) const -> void {
    if (!body || !controls || !tanks || !type || !type->data) {
      return;
    }
    model::EngineAir air = model::engine_air_of(
        *body, earth_, air_, model::seconds(step.time.time_since_epoch()));
    model::run_engines(*type->data, engines, *controls, *tanks, air,
                       model::seconds(step.dt));
  }

 private:
  model::Earth earth_;
  model::StandardAirTable air_;
};

// The rate of each rigid aircraft's body, over a flat Earth unless
// constructed with a round one.
struct RigidAircraftRates final      //
    : System<RigidBodyRate,          //
             const RigidBody,        //
             const ControlSurfaces,  //
             const Engines,          //
             const MassBalance,      //
             const AircraftType> {
  using SystemWorld = ProjectedWorld<RigidAircraftRates>;

  explicit RigidAircraftRates(model::Earth earth = model::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,             //
                  RigidBodyRate& rate,              //
                  const RigidBody* body,            //
                  const ControlSurfaces* surfaces,  //
                  const Engines* engines,           //
                  const MassBalance* mass,          //
                  const AircraftType* type,         //
                  Step step) const -> void {
    if (!body || !surfaces || !engines || !mass || !type || !type->data) {
      return;
    }
    rate = model::rigid_aircraft_rate(
        *body, *surfaces, *engines, *mass, *type->data, earth_, air_,
        model::seconds(step.time.time_since_epoch()));
  }

 private:
  model::Earth earth_;
  model::StandardAirTable air_;
};

using Rigid = framework::Continuous<framework::RungeKutta4, TypeList<RigidBody>,
                                    SystemList<RigidAircraftRates>>;

// After Rigid, each rigid aircraft's engines burn the step's fuel, and its
// mass balance follows.
struct BurnFuel final             //
    : System<FuelTanks,           //
             const Engines,       //
             const AircraftType,  //
             MassBalance> {
  using SystemWorld = ProjectedWorld<BurnFuel>;
  using SequenceAfterSystemList = SystemList<Rigid>;

  auto operator()(SystemWorld&, Entity,      //
                  FuelTanks& tanks,          //
                  const Engines* engines,    //
                  const AircraftType* type,  //
                  MassBalance* mass,         //
                  Step step) const -> void {
    if (!engines || !type || !type->data || !mass) {
      return;
    }
    model::burn_fuel(*type->data, *engines, tanks, model::seconds(step.dt));
    *mass = model::mass_balance_of(*type->data, tanks);
  }
};

// After Rigid, each rigid aircraft's AirState follows its body, so spatial
// queries and the rest of the world see it as they see any aircraft. Aircraft
// with FlightControls fly the point-mass model, so their segments are
// skipped whole.
struct FollowRigidBody final  //
    : System<AirState,        //
             const RigidBody> {
  using SystemWorld = ProjectedWorld<FollowRigidBody>;
  using SequenceAfterSystemList = SystemList<Rigid>;
  using ExcludeComponentList = TypeList<FlightControls>;

  explicit FollowRigidBody(model::Earth earth = model::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,   //
                  AirState& state,        //
                  const RigidBody* body,  //
                  Step step) const -> void {
    if (!body) {
      return;
    }
    // The body has moved on by the step.
    state = earth_.air_state(
        *body, model::seconds((step.time + step.dt).time_since_epoch()));
  }

 private:
  model::Earth earth_;
};

//-- Schedule -----------------------------------------------------------------

using Schedule =
    SystemList<FollowRoute, FlyAutopilot, Actuate, Fly, Precise, FlySurfaces,
               RunFlightControls, RunEngines, Rigid, BurnFuel, FollowRigidBody>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::flight
