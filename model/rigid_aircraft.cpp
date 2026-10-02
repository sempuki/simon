// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/rigid_aircraft.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace simon::model {

namespace {

auto eigen(const Displacement& d) -> Eigen::Vector3d {
  return d.numerical_value_in(meter).eigen();
}
auto eigen(const Velocity& v) -> Eigen::Vector3d {
  return v.numerical_value_in(meter_per_second).eigen();
}
auto eigen(const Acceleration& a) -> Eigen::Vector3d {
  return a.numerical_value_in(meter_per_second_squared).eigen();
}
auto eigen(const AngularVelocity& w) -> Eigen::Vector3d {
  return w.numerical_value_in(radian_per_second).eigen();
}

// The Earth's rotation, in the inertial frame, or none.
auto spin(bool round) -> Eigen::Vector3d {
  return round ? eigen(wgs84::rotation()) : Eigen::Vector3d::Zero();
}

// From the north-east-down frame at a geodetic latitude and longitude to
// ECEF: its columns are north, east and down.
auto north_east_down_to_fixed(Angle latitude, Angle longitude)
    -> Eigen::Matrix3d {
  double sin_lat = sin(latitude);
  double cos_lat = cos(latitude);
  double sin_lon = sin(longitude);
  double cos_lon = cos(longitude);
  Eigen::Matrix3d basis;
  basis << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,  //
      -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,        //
      cos_lat, 0.0, -sin_lat;
  return basis;
}

// The point mass inertia of `mass` at `offset` from the center of mass.
auto point_inertia(double mass, const Eigen::Vector3d& offset)
    -> Eigen::Matrix3d {
  return mass * (offset.squaredNorm() * Eigen::Matrix3d::Identity() -
                 offset * offset.transpose());
}

}  // namespace

//-- Earth ---------------------------------------------------------------------

auto Earth::round(const wgs84::Geodetic& origin) -> Earth {
  Earth earth;
  earth.round_ = true;
  earth.origin_fixed_ = wgs84::fixed_of(origin);
  // East, north and up, as rows.
  Eigen::Matrix3d ned =
      north_east_down_to_fixed(origin.latitude, origin.longitude);
  earth.fixed_to_local_.row(0) = ned.col(1).transpose();
  earth.fixed_to_local_.row(1) = ned.col(0).transpose();
  earth.fixed_to_local_.row(2) = -ned.col(2).transpose();
  return earth;
}

auto Earth::angle(Time time) const -> Angle {
  return round_
             ? wgs84::ROTATION_RATE * time.numerical_value_in(second) * radian
             : 0.0 * radian;
}

auto Earth::fixed_of(const RigidBody& body, Time time) const -> Position {
  if (!round_) {
    return body.position;
  }
  return Vector3d{wgs84::inertial_to_fixed(angle(time)) *
                  eigen(body.position)} *
         meter;
}

auto Earth::place(const RigidBody& body, Time time) const -> Place {
  if (!round_) {
    Place flat{
        .fixed = body.position,
        .altitude = altitude_of(body.position),
    };
    // North is y, east x, and down -z.
    flat.north_east_down << 0.0, 1.0, 0.0,  //
        1.0, 0.0, 0.0,                      //
        0.0, 0.0, -1.0;
    return flat;
  }
  Eigen::Matrix3d to_fixed = wgs84::inertial_to_fixed(angle(time));
  Position fixed = Vector3d{to_fixed * eigen(body.position)} * meter;
  wgs84::Geodetic where = wgs84::geodetic_of(fixed);
  return Place{
      .inertial_to_fixed = to_fixed,
      .fixed = fixed,
      .altitude = where.altitude,
      .north_east_down =
          to_fixed.transpose() *
          north_east_down_to_fixed(where.latitude, where.longitude),
  };
}

auto Earth::gravity(const RigidBody& body, Time time) const -> Acceleration {
  return gravity(place(body, time));
}

auto Earth::gravity(const Place& place) const -> Acceleration {
  if (!round_) {
    return meters_per_second_squared(
        0.0, 0.0,
        -STANDARD_GRAVITY.numerical_value_in(meter_per_second_squared));
  }
  return Vector3d{place.inertial_to_fixed.transpose() *
                  eigen(wgs84::gravitation(place.fixed))} *
         meter_per_second_squared;
}
auto Earth::air_velocity(const RigidBody& body) const -> Velocity {
  Eigen::Vector3d relative =
      eigen(body.velocity) - spin(round_).cross(eigen(body.position));
  return Vector3d{body.attitude.conjugate() * relative} * meter_per_second;
}

auto Earth::air_acceleration(const RigidBody& body,
                             const Acceleration& acceleration) const
    -> Acceleration {
  // d/dt R^T (v - W x r) = R^T (a - W x v) - w x R^T (v - W x r).
  Eigen::Vector3d along =
      body.attitude.conjugate() *
      (eigen(acceleration) - spin(round_).cross(eigen(body.velocity)));
  Eigen::Vector3d uvw = eigen(air_velocity(body));
  return Vector3d{along - eigen(body.rate).cross(uvw)} *
         meter_per_second_squared;
}

auto Earth::air_rate(const RigidBody& body) const -> AngularVelocity {
  return Vector3d{eigen(body.rate) - body.attitude.conjugate() * spin(round_)} *
         radian_per_second;
}

auto Earth::altitude(const RigidBody& body, Time time) const -> Length {
  if (!round_) {
    return altitude_of(body.position);
  }
  return wgs84::geodetic_of(fixed_of(body, time)).altitude;
}

auto Earth::north_east_down(const Position& fixed, Time time) const
    -> Eigen::Matrix3d {
  if (!round_) {
    // North is y, east x, and down -z.
    Eigen::Matrix3d basis;
    basis << 0.0, 1.0, 0.0,  //
        1.0, 0.0, 0.0,       //
        0.0, 0.0, -1.0;
    return basis;
  }
  wgs84::Geodetic where = wgs84::geodetic_of(fixed);
  return wgs84::inertial_to_fixed(angle(time)).transpose() *
         north_east_down_to_fixed(where.latitude, where.longitude);
}

auto Earth::body_to_north_east_down(const RigidBody& body, Time time) const
    -> Eigen::Matrix3d {
  return north_east_down(fixed_of(body, time), time).transpose() *
         body.attitude.toRotationMatrix();
}

auto Earth::air_state(const RigidBody& body, Time time) const -> AirState {
  Position fixed = fixed_of(body, time);
  Position local =
      round_
          ? Vector3d{fixed_to_local_ * (eigen(fixed) - eigen(origin_fixed_))} *
                meter
          : body.position;

  Eigen::Vector3d relative =
      eigen(body.velocity) - spin(round_).cross(eigen(body.position));
  Eigen::Vector3d ned = north_east_down(fixed, time).transpose() * relative;
  double speed = ned.norm();
  return AirState{
      .position = local,
      .speed = speed * meter_per_second,
      .flight_path_angle =
          (speed > 0.0 ? std::asin(-ned.z() / speed) : 0.0) * radian,
      .heading = std::atan2(ned.y(), ned.x()) * radian,
  };
}

auto Earth::body_at(const Position& position, Angle roll, Angle pitch,
                    Angle yaw, const Velocity& air_velocity,
                    const AngularVelocity& air_rate, Time time) const
    -> RigidBody {
  Eigen::Vector3d inertial = eigen(position);
  if (round_) {
    Eigen::Vector3d fixed =
        eigen(origin_fixed_) + fixed_to_local_.transpose() * eigen(position);
    inertial = wgs84::inertial_to_fixed(angle(time)).transpose() * fixed;
  }
  Position where = Vector3d{inertial} * meter;
  Position fixed =
      round_ ? fixed_of(RigidBody{.position = where}, time) : where;

  Quaternion body_to_local =
      Quaternion{Eigen::AngleAxisd{radians(yaw), Eigen::Vector3d::UnitZ()}} *
      Quaternion{Eigen::AngleAxisd{radians(pitch), Eigen::Vector3d::UnitY()}} *
      Quaternion{Eigen::AngleAxisd{radians(roll), Eigen::Vector3d::UnitX()}};
  Quaternion attitude =
      Quaternion{north_east_down(fixed, time)} * body_to_local;
  attitude.normalize();

  return RigidBody{
      .position = where,
      .velocity = Vector3d{attitude * eigen(air_velocity) +
                           spin(round_).cross(inertial)} *
                  meter_per_second,
      .attitude = attitude,
      .rate = Vector3d{eigen(air_rate) + attitude.conjugate() * spin(round_)} *
              radian_per_second,
  };
}

//-- Mass balance --------------------------------------------------------------

auto body_offset(const Displacement& structural,
                 const Displacement& center_of_mass) -> Displacement {
  Eigen::Vector3d apart = eigen(structural) - eigen(center_of_mass);
  // Structural x aft and z up; body x forward and z down.
  return meters(-apart.x(), apart.y(), -apart.z());
}

auto mass_balance_of(const AircraftData& aircraft,
                     std::span<const Mass> contents) -> MassBalance {
  CHECK_PRECONDITION(contents.size() == aircraft.tanks.size());
  double empty = aircraft.empty_mass.numerical_value_in(kilogram);
  double total = empty;
  Eigen::Vector3d moment = empty * eigen(aircraft.empty_center_of_mass);
  for (std::size_t i = 0; i < contents.size(); ++i) {
    double fuel = contents[i].numerical_value_in(kilogram);
    total += fuel;
    moment += fuel * eigen(aircraft.tanks[i].location);
  }
  Displacement center = Vector3d{moment / total} * meter;

  const std::array<double, 6>& j = aircraft.empty_inertia;
  Eigen::Matrix3d inertia;
  inertia << j[0], j[3], j[4],  //
      j[3], j[1], j[5],         //
      j[4], j[5], j[2];
  inertia += point_inertia(
      empty, eigen(body_offset(aircraft.empty_center_of_mass, center)));
  for (std::size_t i = 0; i < contents.size(); ++i) {
    inertia +=
        point_inertia(contents[i].numerical_value_in(kilogram),
                      eigen(body_offset(aircraft.tanks[i].location, center)));
  }
  return MassBalance{
      .properties = MassProperties::of(total * kilogram, inertia),
      .center_of_mass = center,
  };
}

auto mass_balance_of(const AircraftData& aircraft) -> MassBalance {
  return mass_balance_of(aircraft, fuel_tanks_of(aircraft));
}

auto mass_balance_of(const AircraftData& aircraft, const FuelTanks& tanks)
    -> MassBalance {
  return mass_balance_of(
      aircraft, std::span{tanks.contents.data(), aircraft.tanks.size()});
}

//-- Engines and fuel ----------------------------------------------------------

auto fuel_tanks_of(const AircraftData& aircraft) -> FuelTanks {
  FuelTanks tanks;
  for (std::size_t i = 0; i < aircraft.tanks.size(); ++i) {
    tanks.contents[i] = aircraft.tanks[i].contents;
  }
  return tanks;
}

auto settled_engines(const AircraftData& aircraft,
                     const EngineControls& controls, const EngineAir& air)
    -> Engines {
  Engines engines;
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    engines.turbines[i] =
        steady_turbine(aircraft.engines[i], controls.throttle[i], air);
  }
  return engines;
}

auto engine_air_of(const RigidBody& body, const Earth& earth,
                   const StandardAirTable& air, Time time) -> EngineAir {
  Length altitude = earth.altitude(body, time);
  Air here = air(altitude);
  double sound = here.speed_of_sound.numerical_value_in(meter_per_second);
  double speed =
      magnitude(earth.air_velocity(body).numerical_value_in(meter_per_second));
  // The speed of sound is sqrt(gamma R T).
  double temperature =
      sound * sound /
      (internal::HEAT_RATIO *
       internal::GAS_CONSTANT.numerical_value_in(joule_per_kilogram_kelvin));
  return EngineAir{
      .mach = speed / sound,
      .density_altitude = altitude,
      .density_ratio =
          number_of(here.density / standard_air(0.0 * meter).density),
      .temperature = units::delta<kelvin>(temperature),
  };
}

namespace {

auto has_fuel(const TurbineData& turbine, const FuelTanks& tanks) -> bool {
  return std::ranges::any_of(turbine.feeds, [&](std::size_t tank) {
    return tanks.contents[tank] > 0.0 * kilogram;
  });
}

}  // namespace

auto run_engines(const AircraftData& aircraft, Engines& engines,
                 const EngineControls& controls, const FuelTanks& tanks,
                 const EngineAir& air, Time dt) -> void {
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    const TurbineData& turbine = aircraft.engines[i];
    TurbineState& state = engines.turbines[i];
    if (!has_fuel(turbine, tanks)) {
      state.thrust = 0.0 * newton;
      state.fuel_flow = 0.0;
      continue;
    }
    state = run_turbine(turbine, state, controls.throttle[i], air, dt);
  }
}

auto burn_fuel(const AircraftData& aircraft, const Engines& engines,
               FuelTanks& tanks, Time dt) -> void {
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    const TurbineData& turbine = aircraft.engines[i];
    auto feeding = std::ranges::count_if(turbine.feeds, [&](std::size_t tank) {
      return tanks.contents[tank] > 0.0 * kilogram;
    });
    if (feeding == 0) {
      continue;
    }
    Mass share = engines.turbines[i].fuel_flow * dt.numerical_value_in(second) /
                 static_cast<double>(feeding) * kilogram;
    for (std::size_t tank : turbine.feeds) {
      if (tanks.contents[tank] > 0.0 * kilogram) {
        tanks.contents[tank] =
            max(tanks.contents[tank] - share, 0.0 * kilogram);
      }
    }
  }
}

//-- Aerodynamics --------------------------------------------------------------

auto aero_inputs_of(const RigidBody& body, const ControlSurfaces& surfaces,
                    const AircraftData& aircraft, const Displacement& reference,
                    const Earth& earth, const StandardAirTable& air, Time time)
    -> AeroInputs {
  return aero_inputs_of(body, surfaces, aircraft, reference, earth,
                        earth.place(body, time), air);
}

auto aero_inputs_of(const RigidBody& body, const ControlSurfaces& surfaces,
                    const AircraftData& aircraft, const Displacement& reference,
                    const Earth& earth, const Place& place,
                    const StandardAirTable& air) -> AeroInputs {
  Eigen::Vector3d uvw = eigen(earth.air_velocity(body));
  Eigen::Vector3d rates = eigen(earth.air_rate(body));
  Length altitude = place.altitude;
  Air here = air(altitude);

  double speed = uvw.norm();
  double along_and_down = std::hypot(uvw.x(), uvw.z());
  double density = here.density.numerical_value_in(kilogram_per_cubic_meter);

  AeroInputs inputs;
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
  inputs[ELEVATOR] = surfaces.elevator;
  inputs[ELEVATOR_MAGNITUDE] = std::abs(surfaces.elevator);
  inputs[LEFT_AILERON] = surfaces.left_aileron;
  inputs[RIGHT_AILERON] = surfaces.right_aileron;
  inputs[RUDDER] = surfaces.rudder;
  inputs[FLAPS] = surfaces.flaps;
  inputs[GEAR] = surfaces.gear;
  inputs[SPEEDBRAKE] = surfaces.speedbrake;
  inputs[SPOILERS] = surfaces.spoilers;
  // The reference point's height: the body's, less how far down from it the
  // point lies.
  double below =
      (place.north_east_down.transpose() * (body.attitude * eigen(reference)))
          .z();
  inputs[HEIGHT_OVER_SPAN] = (altitude.numerical_value_in(meter) - below) /
                             aircraft.wing_span.numerical_value_in(meter);
  return inputs;
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

auto reads(const AeroModel& model, AeroAxis axis, AeroVariable variable)
    -> bool {
  for (const AeroTerm& term : model.axes[index(axis)]) {
    for (AeroVariable factor : term.factors) {
      if (factor == variable) {
        return true;
      }
    }
    for (const AeroTable& table : term.tables) {
      if (table.row == variable || table.column == variable) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

auto rigid_aircraft_rate(const RigidBody& body, const ControlSurfaces& surfaces,
                         const Engines& engines, const MassBalance& mass,
                         const AircraftData& aircraft, const Earth& earth,
                         const StandardAirTable& air, Time time)
    -> RigidBodyRate {
  const AeroModel& model = aircraft.aero;
  Displacement reference =
      body_offset(aircraft.aero_reference, mass.center_of_mass);
  Place place = earth.place(body, time);
  AeroInputs inputs =
      aero_inputs_of(body, surfaces, aircraft, reference, earth, place, air);
  Acceleration gravity = earth.gravity(place);
  Angle alpha = inputs[AeroVariable::ALPHA] * radian;
  Angle beta = inputs[AeroVariable::BETA] * radian;

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

  // The rate of angle of attack follows from the body's acceleration, which
  // the forces set.
  // The engines' thrust, along body x from where each is mounted.
  Eigen::Vector3d thrust_force = Eigen::Vector3d::Zero();
  Eigen::Vector3d thrust_moment = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < aircraft.engines.size(); ++i) {
    Eigen::Vector3d thrust{
        engines.turbines[i].thrust.numerical_value_in(newton), 0.0, 0.0};
    Eigen::Vector3d arm =
        eigen(body_offset(aircraft.engines[i].location, mass.center_of_mass));
    thrust_force += thrust;
    thrust_moment += arm.cross(thrust);
  }

  auto alpha_rate = [&](const AeroSums& sums) {
    Eigen::Vector3d force = aero_loads(sums, alpha, beta, reference)
                                .force.numerical_value_in(newton)
                                .eigen() +
                            thrust_force;
    RigidBodyRate translation =
        rigid_body_rate(body, Vector3d{force} * newton,
                        Vector3d{} * newton_meter, mass.properties, gravity);
    Eigen::Vector3d uvw = eigen(earth.air_velocity(body));
    Eigen::Vector3d uvw_rate =
        eigen(earth.air_acceleration(body, translation.acceleration));
    double along_and_down = uvw.x() * uvw.x() + uvw.z() * uvw.z();
    return along_and_down > 0.0
               ? (uvw.x() * uvw_rate.z() - uvw.z() * uvw_rate.x()) /
                     along_and_down
               : 0.0;
  };

  AeroSums sums = forces();
  inputs[AeroVariable::ALPHA_RATE] = alpha_rate(sums);
  bool forces_read_alpha_rate =
      reads(model, AeroAxis::LIFT, AeroVariable::ALPHA_RATE) ||
      reads(model, AeroAxis::DRAG, AeroVariable::ALPHA_RATE) ||
      reads(model, AeroAxis::SIDE, AeroVariable::ALPHA_RATE);
  if (forces_read_alpha_rate) {
    sums = forces();
    inputs[AeroVariable::ALPHA_RATE] = alpha_rate(sums);
  }
  sums[index(AeroAxis::ROLL)] = sum(model, AeroAxis::ROLL, inputs);
  sums[index(AeroAxis::PITCH)] = sum(model, AeroAxis::PITCH, inputs);
  sums[index(AeroAxis::YAW)] = sum(model, AeroAxis::YAW, inputs);

  AeroLoads loads = aero_loads(sums, alpha, beta, reference);
  Eigen::Vector3d force =
      loads.force.numerical_value_in(newton).eigen() + thrust_force;
  Eigen::Vector3d moment =
      loads.moment.numerical_value_in(newton_meter).eigen() + thrust_moment;
  return rigid_body_rate(body, Vector3d{force} * newton,
                         Vector3d{moment} * newton_meter, mass.properties,
                         gravity);
}

}  // namespace simon::model
