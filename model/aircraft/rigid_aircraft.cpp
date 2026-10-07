// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows JSBSim 1.3.1, Copyright (C) 2000 Jon S. Berndt and the JSBSim
// authors, LGPL-2.0-or-later; translated to C++ and changed. See NOTICE.md.

#include "model/aircraft/rigid_aircraft.hpp"

#include <cmath>
#include <cstddef>

namespace simon::model {

auto compute_aero_inputs(const RigidBody& body, const FlightSignals& signals,
                         const AircraftData& aircraft,
                         const Displacement& reference, const Earth& earth,
                         const StandardAirTable& air, Time time) -> AeroInputs {
  return compute_aero_inputs(body, signals, aircraft, reference, earth,
                             earth.place(body, time), air);
}

namespace {

auto compute_aero_inputs(const BodyMotion& motion, const FlightSignals& signals,
                         const AircraftData& aircraft,
                         const Displacement& reference, const Place& place,
                         const StandardAirTable& air) -> AeroInputs {
  const Vector3& uvw = motion.air_velocity;
  const Vector3& rates = motion.air_rate;
  Length altitude = place.altitude;
  Air here = air(altitude);

  double speed = uvw.norm();
  double along_and_down = std::sqrt(uvw.x() * uvw.x() + uvw.z() * uvw.z());
  double density = here.density.numerical_value_in(kilogram_per_cubic_meter);

  AeroInputs inputs;
  inputs.read(aircraft.aero.signals, signals.values);
  using enum AeroVariable;
  inputs[ALPHA] = along_and_down > 0.0 ? std::atan2(uvw.z(), uvw.x()) : 0.0;
  inputs[BETA] = speed > 0.0 ? std::atan2(uvw.y(), along_and_down) : 0.0;
  inputs[MACH] =
      speed / here.speed_of_sound.numerical_value_in(meter_per_second);
  inputs[DYNAMIC_PRESSURE] = 0.5 * density * speed * speed;
  double twice = 2.0 * std::max(speed, 1e-3);
  inputs[SPAN_OVER_TWICE_SPEED] =
      aircraft.wing_span.numerical_value_in(meter) / twice;
  inputs[CHORD_OVER_TWICE_SPEED] =
      aircraft.chord.numerical_value_in(meter) / twice;
  inputs[ROLL_RATE] = rates.x();
  inputs[PITCH_RATE] = rates.y();
  inputs[YAW_RATE] = rates.z();
  // The reference point's height: the body's, less how far down from it the
  // point lies.
  double below = (place.north_east_down.transpose() *
                  (motion.to_inertial * eigen(reference)))
                     .z();
  inputs[HEIGHT_OVER_SPAN] = (altitude.numerical_value_in(meter) - below) /
                             aircraft.wing_span.numerical_value_in(meter);
  return inputs;
}

}  // namespace

auto compute_aero_inputs(const RigidBody& body, const FlightSignals& signals,
                         const AircraftData& aircraft,
                         const Displacement& reference, const Earth& earth,
                         const Place& place, const StandardAirTable& air)
    -> AeroInputs {
  return compute_aero_inputs(compute_body_motion<false>(body, earth, place, {}),
                             signals, aircraft, reference, place, air);
}

namespace {

constexpr auto index(AeroAxis axis) -> std::size_t {
  return static_cast<std::size_t>(axis);
}

auto sum(const AeroModel& model, AeroAxis axis, const AeroInputs& inputs)
    -> double {
  double total = 0.0;
  for (const AeroTerm& term : model.axes[index(axis)]) {
    total += term(inputs);
  }
  return total;
}

}  // namespace

namespace {

// The rate, compiled for still air apart, so that it costs what it would
// without wind.
template <bool WINDY>
auto compute_rate(const RigidBody& body, const FlightSignals& signals,
                  const Engines& engines, const MassBalance& mass,
                  const AircraftData& aircraft, const Earth& earth,
                  const StandardAirTable& air, const Wind& wind, Time time,
                  Out<BodyAcceleration> felt) -> RigidBodyRate {
  const AeroModel& model = aircraft.aero;
  Displacement reference =
      compute_body_offset(aircraft.aero_reference, mass.center_of_mass);
  Place place = earth.place(body, time);
  BodyMotion motion = compute_body_motion<WINDY>(body, earth, place, wind);
  AeroInputs inputs =
      compute_aero_inputs(motion, signals, aircraft, reference, place, air);
  Acceleration gravity = earth.gravity(place);
  WindAngles angles = compute_wind_angles(motion.air_velocity);
  double kilograms = mass.properties.mass.numerical_value_in(kilogram);

  // Lift first, so the induced drag reads its coefficient.
  auto forces = [&]() -> AeroSums {
    AeroSums sums{};
    sums[index(AeroAxis::LIFT)] = sum(model, AeroAxis::LIFT, inputs);
    double area = inputs[AeroVariable::DYNAMIC_PRESSURE] *
                  aircraft.wing_area.numerical_value_in(square_meter);
    double coefficient = area > 0.0 ? sums[index(AeroAxis::LIFT)] / area : 0.0;
    inputs[AeroVariable::LIFT_COEFFICIENT_SQUARED] = coefficient * coefficient;
    sums[index(AeroAxis::DRAG)] = sum(model, AeroAxis::DRAG, inputs);
    sums[index(AeroAxis::SIDE)] = sum(model, AeroAxis::SIDE, inputs);
    return sums;
  };

  // The engines' thrust, along body x from where each is mounted.
  Vector3 thrust_force = Vector3::Zero();
  Vector3 thrust_moment = Vector3::Zero();
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    Vector3 thrust{engines.turbines[i].thrust.numerical_value_in(newton), 0.0,
                   0.0};
    Vector3 arm = eigen(
        compute_body_offset(aircraft.engines[i].location, mass.center_of_mass));
    thrust_force += thrust;
    thrust_moment += arm.cross(thrust);
  }

  // The rate of angle of attack follows from the body's acceleration, which
  // the forces set. In body axes the air velocity changes at the force per
  // unit mass and the rest, which the stage fixes (see
  // compute_air_acceleration). With alpha = atan2(w, u), its rate is
  // (u w' - w u') / (u^2 + w^2), as JSBSim's FGAuxiliary has it.
  const Vector3& uvw = motion.air_velocity;
  Vector3 rest = compute_air_acceleration<WINDY>(
      body, motion, earth, place, wind, Vector3::Zero(), eigen(gravity));
  auto alpha_rate = [&](const AeroSums& sums) {
    Vector3 force = compute_aero_loads(sums, angles, reference)
                        .force.numerical_value_in(newton)
                        .eigen() +
                    thrust_force;
    Vector3 uvw_rate = force / kilograms + rest;
    double along_and_down = uvw.x() * uvw.x() + uvw.z() * uvw.z();
    return along_and_down > 0.0
               ? (uvw.x() * uvw_rate.z() - uvw.z() * uvw_rate.x()) /
                     along_and_down
               : 0.0;
  };

  AeroSums sums = forces();
  inputs[AeroVariable::ALPHA_RATE] = alpha_rate(sums);
  if (model.forces_read_alpha_rate) {
    sums = forces();
    inputs[AeroVariable::ALPHA_RATE] = alpha_rate(sums);
  }
  sums[index(AeroAxis::ROLL)] = sum(model, AeroAxis::ROLL, inputs);
  sums[index(AeroAxis::PITCH)] = sum(model, AeroAxis::PITCH, inputs);
  sums[index(AeroAxis::YAW)] = sum(model, AeroAxis::YAW, inputs);

  AeroLoads loads = compute_aero_loads(sums, angles, reference);
  Vector3 force = loads.force.numerical_value_in(newton).eigen() + thrust_force;
  Vector3 moment =
      loads.moment.numerical_value_in(newton_meter).eigen() + thrust_moment;
  RigidBodyRate rate = compute_rigid_body_rate(
      body, motion.to_inertial, QuantityVector{force} * newton,
      QuantityVector{moment} * newton_meter, mass.properties, gravity);
  if (felt) {
    *felt = BodyAcceleration{
        .specific_force =
            QuantityVector{force / kilograms} * meter_per_second_squared,
        .angular = rate.angular_acceleration,
    };
  }
  return rate;
}

}  // namespace

auto compute_rigid_aircraft_rate(
    const RigidBody& body, const FlightSignals& signals, const Engines& engines,
    const MassBalance& mass, const AircraftData& aircraft, const Earth& earth,
    const StandardAirTable& air, const Wind& wind, Time time,
    Out<BodyAcceleration> felt) -> RigidBodyRate {
  if (is_still(wind)) {
    return compute_rate<false>(body, signals, engines, mass, aircraft, earth,
                               air, wind, time, felt);
  }
  return compute_rate<true>(body, signals, engines, mass, aircraft, earth, air,
                            wind, time, felt);
}

}  // namespace simon::model
