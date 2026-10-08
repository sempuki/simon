// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/aircraft/frames.hpp"
#include "core/math.hpp"

#include <cmath>

namespace simon::aircraft {

namespace {

// From the north-east-down frame at a place on the ellipsoid to ECEF: its
// columns are north, east and down.
auto convert_north_east_down_to_fixed(double sin_lat, double cos_lat,
                                      double sin_lon, double cos_lon)
    -> Matrix3 {
  Matrix3 basis;
  basis << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon,  //
      -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,        //
      cos_lat, 0.0, -sin_lat;
  return basis;
}

auto convert_north_east_down_to_fixed(const earth::wgs84::Location& where)
    -> Matrix3 {
  return convert_north_east_down_to_fixed(
      where.sin_latitude, where.cos_latitude, where.sin_longitude,
      where.cos_longitude);
}

}  // namespace

auto Earth::round(const earth::wgs84::Geodetic& origin) -> Earth {
  Earth earth;
  earth.round_ = true;
  earth.origin_fixed_ = earth::wgs84::convert_geodetic_to_fixed(origin);
  // East, north and up, as rows.
  Matrix3 ned = convert_north_east_down_to_fixed(
      sin(origin.latitude), cos(origin.latitude), sin(origin.longitude),
      cos(origin.longitude));
  earth.fixed_to_local_.row(0) = ned.col(1).transpose();
  earth.fixed_to_local_.row(1) = ned.col(0).transpose();
  earth.fixed_to_local_.row(2) = -ned.col(2).transpose();
  return earth;
}

auto Earth::angle(Time time) const -> Angle {
  return round_ ? earth::wgs84::ROTATION_RATE *
                      time.numerical_value_in(second) * radian
                : 0.0 * radian;
}

auto Earth::find_fixed(const model::RigidBody& body, Time time) const
    -> Position {
  if (!round_) {
    return body.position;
  }
  return QuantityVector{earth::wgs84::convert_inertial_to_fixed(angle(time)) *
                        eigen(body.position)} *
         meter;
}

auto Earth::place(const model::RigidBody& body, Time time) const -> Place {
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
  Matrix3 to_fixed = earth::wgs84::convert_inertial_to_fixed(angle(time));
  Position fixed = QuantityVector{to_fixed * eigen(body.position)} * meter;
  earth::wgs84::Location where = earth::wgs84::locate(fixed);
  return Place{
      .convert_inertial_to_fixed = to_fixed,
      .fixed = fixed,
      .altitude = where.altitude,
      .north_east_down =
          to_fixed.transpose() * convert_north_east_down_to_fixed(where),
      .sin_latitude = where.sin_latitude,
      .cos_latitude = where.cos_latitude,
  };
}

auto Earth::gravity(const model::RigidBody& body, Time time) const
    -> Acceleration {
  return gravity(place(body, time));
}

auto Earth::gravity(const Place& place) const -> Acceleration {
  if (!round_) {
    return meters_per_second_squared(
        0.0, 0.0,
        -earth::STANDARD_GRAVITY.numerical_value_in(meter_per_second_squared));
  }
  return QuantityVector{place.convert_inertial_to_fixed.transpose() *
                        eigen(earth::wgs84::compute_gravitation(place.fixed))} *
         meter_per_second_squared;
}
auto Earth::air_velocity(const model::RigidBody& body) const -> Velocity {
  Vector3 relative =
      eigen(body.velocity) - spin(*this).cross(eigen(body.position));
  return QuantityVector{body.attitude.conjugate() * relative} *
         meter_per_second;
}

auto Earth::air_acceleration(const model::RigidBody& body,
                             const Acceleration& acceleration) const
    -> Acceleration {
  // d/dt R^T (v - W x r) = R^T (a - W x v) - w x R^T (v - W x r).
  Vector3 along =
      body.attitude.conjugate() *
      (eigen(acceleration) - spin(*this).cross(eigen(body.velocity)));
  Vector3 uvw = eigen(air_velocity(body));
  return QuantityVector{along - eigen(body.rate).cross(uvw)} *
         meter_per_second_squared;
}

auto Earth::air_rate(const model::RigidBody& body) const -> AngularVelocity {
  return QuantityVector{eigen(body.rate) -
                        body.attitude.conjugate() * spin(*this)} *
         radian_per_second;
}

auto Earth::altitude(const model::RigidBody& body, Time time) const -> Length {
  if (!round_) {
    return altitude_of(body.position);
  }
  return earth::wgs84::locate(find_fixed(body, time)).altitude;
}

auto Earth::north_east_down(const Position& fixed, Time time) const -> Matrix3 {
  if (!round_) {
    // North is y, east x, and down -z.
    Matrix3 basis;
    basis << 0.0, 1.0, 0.0,  //
        1.0, 0.0, 0.0,       //
        0.0, 0.0, -1.0;
    return basis;
  }
  return earth::wgs84::convert_inertial_to_fixed(angle(time)).transpose() *
         convert_north_east_down_to_fixed(earth::wgs84::locate(fixed));
}

auto Earth::level_rate(const model::RigidBody& body, Time time) const
    -> AngularVelocity {
  if (!round_) {
    return QuantityVector{} * radian_per_second;
  }
  Place here = place(body, time);
  Matrix3 to_north_east_down =
      here.north_east_down.transpose() * body.attitude.toRotationMatrix();
  Vector3 velocity = to_north_east_down * eigen(air_velocity(body));

  // The ellipsoid's radii of curvature: in the prime vertical, and along
  // the meridian.
  double sin_lat = here.sin_latitude;
  double w = 1.0 - earth::wgs84::ECCENTRICITY_SQUARED * sin_lat * sin_lat;
  double height = here.altitude.numerical_value_in(meter);
  double prime = earth::wgs84::SEMIMAJOR_AXIS / std::sqrt(w) + height;
  double meridian = earth::wgs84::SEMIMAJOR_AXIS *
                        (1.0 - earth::wgs84::ECCENTRICITY_SQUARED) /
                        (w * std::sqrt(w)) +
                    height;
  Vector3 turning{velocity.y() / prime, -velocity.x() / meridian,
                  -velocity.y() * sin_lat / here.cos_latitude / prime};
  return QuantityVector{to_north_east_down.transpose() * turning} *
         radian_per_second;
}

auto Earth::convert_body_to_north_east_down(const model::RigidBody& body,
                                            Time time) const -> Matrix3 {
  return north_east_down(find_fixed(body, time), time).transpose() *
         body.attitude.toRotationMatrix();
}

auto Earth::air_state(const model::RigidBody& body, Time time,
                      const earth::Wind& wind) const -> AirState {
  Position fixed = find_fixed(body, time);
  Position local = round_
                       ? QuantityVector{fixed_to_local_ *
                                        (eigen(fixed) - eigen(origin_fixed_))} *
                             meter
                       : body.position;

  Vector3 relative =
      eigen(body.velocity) - spin(*this).cross(eigen(body.position));
  Vector3 ned = north_east_down(fixed, time).transpose() * relative -
                eigen(wind.north_east_down);
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
    -> model::RigidBody {
  Vector3 inertial = eigen(position);
  if (round_) {
    Vector3 fixed =
        eigen(origin_fixed_) + fixed_to_local_.transpose() * eigen(position);
    inertial =
        earth::wgs84::convert_inertial_to_fixed(angle(time)).transpose() *
        fixed;
  }
  Position where = QuantityVector{inertial} * meter;
  Position fixed =
      round_ ? find_fixed(model::RigidBody{.position = where}, time) : where;

  Quaternion body_to_local =
      Quaternion{AngleAxis{radians(yaw), Vector3::UnitZ()}} *
      Quaternion{AngleAxis{radians(pitch), Vector3::UnitY()}} *
      Quaternion{AngleAxis{radians(roll), Vector3::UnitX()}};
  Quaternion attitude =
      Quaternion{north_east_down(fixed, time)} * body_to_local;
  attitude.normalize();

  return model::RigidBody{
      .position = where,
      .velocity = QuantityVector{attitude * eigen(air_velocity) +
                                 spin(*this).cross(inertial)} *
                  meter_per_second,
      .attitude = attitude,
      .rate =
          QuantityVector{eigen(air_rate) + attitude.conjugate() * spin(*this)} *
          radian_per_second,
  };
}

auto compute_air_velocity(const model::RigidBody& body, const Earth& earth,
                          const Place& place, const earth::Wind& wind)
    -> Velocity {
  return QuantityVector{
             compute_body_motion<true>(body, earth, place, wind).air_velocity} *
         meter_per_second;
}

auto compute_air_acceleration(const model::RigidBody& body, const Earth& earth,
                              const Place& place, const earth::Wind& wind,
                              const Acceleration& specific_force,
                              const Acceleration& gravity) -> Acceleration {
  return QuantityVector{compute_air_acceleration<true>(
             body, compute_body_motion<true>(body, earth, place, wind), earth,
             place, wind, eigen(specific_force), eigen(gravity))} *
         meter_per_second_squared;
}

}  // namespace simon::aircraft
