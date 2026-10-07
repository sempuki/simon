// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include "application/aeronautic/simulation_components.hpp"
#include "core/argument.hpp"
#include "core/math.hpp"
#include "engine/rate_gate.hpp"
#include "framework/continuous.hpp"
#include "framework/system.hpp"
#include "model/aircraft/flight_path.hpp"
#include "model/aircraft/rigid_aircraft.hpp"
#include "model/control.hpp"
#include "model/earth/atmosphere.hpp"
#include "model/earth/wind.hpp"

namespace simon::aeronautic {

using framework::System;
using framework::SystemList;
using framework::TypeList;
using namespace std::chrono_literals;

template <typename SystemType>
using ProjectedWorld = framework::ProjectedWorld<SystemType, World>;

//-- Wind ----------------------------------------------------------------------

// Each step, the air moves at each aircraft that has a Wind: by the field's
// steady wind, and, for a rigid aircraft that has Gusts, by the turbulence it
// meets along its path. The wind holds over the step.
struct MoveAir final          //
    : System<Wind,            //
             const AirState,  //
             Gusts,           //
             const AircraftType> {
  using SystemWorld = ProjectedWorld<MoveAir>;

  explicit MoveAir(earth::WindField field = {}) : field_{field} {}

  auto operator()(SystemWorld&, Entity,      //
                  Wind& wind,                //
                  const AirState* state,     //
                  Gusts* gusts,              //
                  const AircraftType* type,  //
                  Step step) const -> void {
    if (!gusts || !state || !type || !type->data) {
      wind = earth::compute_wind(field_);
      return;
    }
    earth::advance_gusts(field_.turbulence, aircraft::altitude_of(*state),
                         state->speed, type->data->wing_span, seconds(step.dt),
                         InOut(*gusts));
    wind = earth::compute_wind(field_, *gusts, state->heading);
  }

 private:
  earth::WindField field_;
};

//-- Guidance and control: discrete, at their own rates ------------------------

// Once a second, each aircraft steers its autopilot at its route's next
// waypoint, and moves on to the one after when it is within capture range.
struct FollowRoute final      //
    : System<Route,           //
             const AirState,  //
             Autopilot> {
  using SystemWorld = ProjectedWorld<FollowRoute>;

  static constexpr Length CAPTURE = 3000.0 * meter;

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
    if (aircraft::ground_distance(state->position,
                                  route.waypoints[route.next]) < CAPTURE) {
      route.next = (route.next + 1) % Route::SIZE;
      ++route.reached;
    }
    const Position& waypoint = route.waypoints[route.next];
    autopilot->heading = aircraft::compute_bearing(state->position, waypoint);
    autopilot->altitude = aircraft::altitude_of(waypoint);
    autopilot->speed = route.speed;
  }

 private:
  engine::RateGate gate_{1s};
};

// The autopilot's gains, shared by every aircraft.
struct AutopilotGains final {
  Rate altitude = 0.2 * per_second;  // Climb rate per meter of error.
  Angle steepest_climb = 0.25 * radian;
  // The speed deficit at which an aircraft stops climbing: the steepest climb
  // shrinks to nothing as the speed falls this far below its target, so a
  // climb never trades away more speed than that.
  Speed speed_margin = 20.0 * meter_per_second;
  Rate climb = 1.0 * per_second;  // Of the flight-path angle error.
  Rate heading = 0.5 * per_second;
  // Throttle, from none to full, for the speed error.
  model::PiGains<Speed> speed{.proportional = 0.05 * second / meter,
                              .integral = 0.02 / meter,
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
    elapsed_ = firing ? seconds(firing->elapsed) : 0.0 * second;
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
    commands.bank = aircraft::compute_bank_command(
        *state, autopilot->heading, gains_.heading, handling->max_bank);

    // Speed comes first: a slow aircraft climbs less steeply, or not at all.
    Angle climb = aircraft::compute_climb_command(
        *state, autopilot->altitude, gains_.altitude, gains_.steepest_climb);
    double slow = std::clamp(1.0 - number_of(speed_error / gains_.speed_margin),
                             0.0, 1.0);
    climb = min(climb, gains_.steepest_climb * slow);

    commands.load_factor =
        std::clamp(aircraft::compute_load_factor_command(
                       *state, climb, controls->bank, gains_.climb),
                   handling->min_load_factor, handling->max_load_factor);
    commands.throttle = model::pi_control(speed_error, gains_.speed, elapsed_,
                                          InOut(autopilot->throttle_integral));
  }

 private:
  engine::RateGate gate_{100ms};
  Time elapsed_ = 0.0 * second;  // Since the gate last fired.
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
    Time dt = seconds(step.dt);
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
inline auto compute_rate(const AirState& state, const FlightControls& controls,
                         const Airframe& airframe,
                         const earth::StandardAirTable& air) -> AirStateRate {
  return aircraft::compute_point_mass_rate(state, controls, airframe,
                                           air(aircraft::altitude_of(state)));
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
    state = aircraft::fly(
        state, compute_rate(state, *controls, *airframe, air_), step.dt);
  }

 private:
  earth::StandardAirTable air_;
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
    rate = compute_rate(*state, *controls, *airframe, air_);
  }

 private:
  earth::StandardAirTable air_;
};

using Precise =
    framework::Continuous<framework::RungeKutta4, TypeList<AirState>,
                          SystemList<PointMassRates>>;

// After Fly and Precise, the wind carries each point-mass aircraft that has
// one. Its AirState is its motion through the air, so the air's own motion
// adds to its position alone. Rigid aircraft fly through the wind in their
// dynamics, so DriftWithWind excludes them.
struct DriftWithWind final  //
    : System<const Wind,    //
             AirState> {
  using SystemWorld = ProjectedWorld<DriftWithWind>;
  using SequenceAfterSystemList = SystemList<Fly, Precise>;
  using ExcludeComponentList = TypeList<RigidBody>;

  auto operator()(SystemWorld&, Entity,  //
                  const Wind& wind,      //
                  AirState* state,       //
                  Step step) const -> void {
    if (!state) {
      return;
    }
    // North, east and down to the local frame's east, north and up.
    Vector3 ned =
        wind.north_east_down.numerical_value_in(meter_per_second).eigen();
    state->position +=
        meters_per_second(ned.y(), ned.x(), -ned.z()) * seconds(step.dt);
  }
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
             SurfaceAutopilot> {
  using SystemWorld = ProjectedWorld<FlySurfaces>;
  using SequenceAfterSystemList = SystemList<FollowRoute>;

  explicit FlySurfaces(aircraft::Earth earth = aircraft::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,        //
                  FlightSignals& signals,      //
                  const AirState* state,       //
                  const RigidBody* body,       //
                  const Autopilot* autopilot,  //
                  SurfaceAutopilot* trim,      //
                  Step step) const -> void {
    if (!state || !body || !autopilot || !trim) {
      return;
    }
    const SurfaceGains& gains = trim->gains;
    Matrix3 attitude = earth_.convert_body_to_north_east_down(
        *body, seconds(step.time.time_since_epoch()));
    double bank = std::atan2(attitude(2, 1), attitude(2, 2));
    Vector3 rates =
        earth_.air_rate(*body).numerical_value_in(radian_per_second).eigen();

    double bank_error =
        radians(aircraft::compute_bank_command(
            *state, autopilot->heading, 0.5 * per_second, gains.max_bank)) -
        bank;
    // Speed comes first, as it does for point-mass aircraft: a slow
    // aircraft climbs less steeply, or not at all.
    double speed_error =
        (autopilot->speed - state->speed).numerical_value_in(meter_per_second);
    double steepest = 0.08 * std::clamp(1.0 - speed_error / 20.0, 0.0, 1.0);
    double climb_error =
        std::min(
            radians(aircraft::compute_climb_command(
                *state, autopilot->altitude, 0.05 * per_second, 0.08 * radian)),
            steepest) -
        radians(state->flight_path_angle);
    trim->climb_integral = std::clamp(
        trim->climb_integral +
            climb_error * seconds(step.dt).numerical_value_in(second),
        -0.1, 0.1);
    // A turn needs a pitch rate of g/V sin(bank) tan(bank) to hold its
    // flight path.
    double turn =
        earth::STANDARD_GRAVITY.numerical_value_in(meter_per_second_squared) /
        state->speed.numerical_value_in(meter_per_second) * std::sin(bank) *
        std::tan(bank);

    using enum aircraft::FlightSignal;
    // The 737's elevator command is positive nose down.
    signals[AILERON_COMMAND] =
        std::clamp(gains.bank * bank_error - gains.roll * rates.x(), -1.0, 1.0);
    signals[ELEVATOR_COMMAND] = std::clamp(
        -gains.climb * climb_error - gains.pitch * (turn - rates.y()) -
            gains.integral * trim->climb_integral,
        -1.0, 1.0);
    signals[PITCH_TRIM_COMMAND] = trim->pitch_trim;
    double throttle =
        std::clamp(trim->throttle_trim + gains.speed * speed_error, 0.0, 1.0);
    for (std::size_t i = 0; i < aircraft::MAX_ENGINES; ++i) {
      signals.values[aircraft::index_of(THROTTLE_COMMAND_0) + i] = throttle;
    }
  }

 private:
  aircraft::Earth earth_;
};

// Each rigid aircraft's flight controls read its state and its pilot's
// commands, and set its control surfaces: every step, or, given a period, at
// that rate whatever the step, as a digital flight control computer runs. A
// computer's blocks then step by its period, so its control laws do not change
// with the step the dynamics are integrated at.
struct RunFlightControls final        //
    : System<FlightSignals,           //
             const RigidBody,         //
             const BodyAcceleration,  //
             const MassBalance,       //
             const AircraftType,      //
             const Wind> {
  using SystemWorld = ProjectedWorld<RunFlightControls>;

  explicit RunFlightControls(aircraft::Earth earth = aircraft::Earth::flat(),
                             std::optional<Duration> period = std::nullopt)
      : earth_{earth} {
    if (period) {
      gate_ = engine::RateGate{*period};
    }
  }

  auto prepare(SystemWorld&, Step step) -> bool {
    if (!gate_) {
      dt_ = seconds(step.dt);
      return true;
    }
    dt_ = seconds(gate_->period());
    return gate_->fire(step).has_value();
  }

  auto operator()(SystemWorld&, Entity,          //
                  FlightSignals& signals,        //
                  const RigidBody* body,         //
                  const BodyAcceleration* felt,  //
                  const MassBalance* mass,       //
                  const AircraftType* type,      //
                  const Wind* wind,              //
                  Step step) const -> void {
    if (!body || !felt || !mass || !type || !type->data) {
      return;
    }
    aircraft::sense_flight_state(
        *body, *felt, *mass, *type->data, earth_, air_, wind ? *wind : still_,
        seconds(step.time.time_since_epoch()), InOut(signals));
    aircraft::run_flight_controls(type->data->flight_controls, InOut(signals),
                                  dt_);
  }

 private:
  aircraft::Earth earth_;
  earth::StandardAirTable air_;
  Wind still_;
  std::optional<engine::RateGate> gate_;
  Time dt_ = 0.0 * second;  // The blocks' step.
};

// Each step, each rigid aircraft's engines run at their throttles in the air
// they breathe, and set the thrust and fuel flow the step holds.
struct RunEngines final            //
    : System<Engines,              //
             const RigidBody,      //
             const FlightSignals,  //
             const FuelTanks,      //
             const AircraftType,   //
             const Wind> {
  using SystemWorld = ProjectedWorld<RunEngines>;
  using SequenceAfterSystemList = SystemList<RunFlightControls>;

  explicit RunEngines(aircraft::Earth earth = aircraft::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,          //
                  Engines& engines,              //
                  const RigidBody* body,         //
                  const FlightSignals* signals,  //
                  const FuelTanks* tanks,        //
                  const AircraftType* type,      //
                  const Wind* wind,              //
                  Step step) const -> void {
    if (!body || !signals || !tanks || !type || !type->data) {
      return;
    }
    aircraft::EngineAir air =
        aircraft::compute_engine_air(*body, earth_, air_, wind ? *wind : still_,
                                     seconds(step.time.time_since_epoch()));
    aircraft::run_engines(*type->data, InOut(engines), *signals, *tanks, air,
                          seconds(step.dt));
  }

 private:
  aircraft::Earth earth_;
  earth::StandardAirTable air_;
  Wind still_;
};

// The rate of each rigid aircraft's body, over a flat Earth unless
// constructed with a round one. It also keeps what each body feels, so after
// Rigid each holds what its body felt at the last stage, which the flight
// controls read the next step.
struct RigidAircraftRates final    //
    : System<RigidBodyRate,        //
             const RigidBody,      //
             const FlightSignals,  //
             const Engines,        //
             const MassBalance,    //
             const AircraftType,   //
             BodyAcceleration,     //
             const Wind> {
  using SystemWorld = ProjectedWorld<RigidAircraftRates>;

  explicit RigidAircraftRates(aircraft::Earth earth = aircraft::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,          //
                  RigidBodyRate& rate,           //
                  const RigidBody* body,         //
                  const FlightSignals* signals,  //
                  const Engines* engines,        //
                  const MassBalance* mass,       //
                  const AircraftType* type,      //
                  BodyAcceleration* felt,        //
                  const Wind* wind,              //
                  Step step) const -> void {
    if (!body || !signals || !engines || !mass || !type || !type->data ||
        !felt) {
      return;
    }
    rate = aircraft::compute_rigid_aircraft_rate(
        *body, *signals, *engines, *mass, *type->data, earth_, air_,
        wind ? *wind : still_, seconds(step.time.time_since_epoch()),
        Out(*felt));
  }

 private:
  aircraft::Earth earth_;
  earth::StandardAirTable air_;
  Wind still_;
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
    aircraft::burn_fuel(*type->data, *engines, InOut(tanks), seconds(step.dt));
    *mass = aircraft::compute_mass_balance(*type->data, tanks);
  }
};

// After Rigid, each rigid aircraft's AirState follows its body, so spatial
// queries and the rest of the world see it as they see any aircraft. Aircraft
// with FlightControls fly the point-mass model, so their segments are
// skipped whole.
struct FollowRigidBody final   //
    : System<AirState,         //
             const RigidBody,  //
             const Wind> {
  using SystemWorld = ProjectedWorld<FollowRigidBody>;
  using SequenceAfterSystemList = SystemList<Rigid>;
  using ExcludeComponentList = TypeList<FlightControls>;

  explicit FollowRigidBody(aircraft::Earth earth = aircraft::Earth::flat())
      : earth_{earth} {}

  auto operator()(SystemWorld&, Entity,   //
                  AirState& state,        //
                  const RigidBody* body,  //
                  const Wind* wind,       //
                  Step step) const -> void {
    if (!body) {
      return;
    }
    // The body has moved on by the step.
    state = earth_.air_state(*body,
                             seconds((step.time + step.dt).time_since_epoch()),
                             wind ? *wind : still_);
  }

 private:
  aircraft::Earth earth_;
  Wind still_;
};

//-- Schedule -----------------------------------------------------------------

using Schedule =
    SystemList<MoveAir, FollowRoute, FlyAutopilot, Actuate, Fly, Precise,
               DriftWithWind, FlySurfaces, RunFlightControls, RunEngines, Rigid,
               BurnFuel, FollowRigidBody>;
using Scheduler = framework::Scheduler<World, Schedule>;

}  // namespace simon::aeronautic
