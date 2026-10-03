// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/sensing.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace simon::model {

auto sense_flight_state(const RigidBody& body, const BodyAcceleration& felt,
                        const MassBalance& mass, const AircraftData& aircraft,
                        const Earth& earth, const StandardAirTable& air,
                        const Wind& wind, Time time,
                        InOut<FlightSignals> signals) -> void {
  const FlightControlData& controls = aircraft.flight_controls;
  using enum FlightSignal;
  FlightSignals& values = *signals;

  // The body's place on the Earth, found once, if anything needs it.
  bool still = is_still(wind);
  bool air_data = controls.reads(MACH) || controls.reads(CALIBRATED_AIRSPEED);
  bool attitude = controls.reads(PITCH) || controls.reads(ROLL) ||
                  controls.reads(GROUND_SPEED);
  std::optional<Place> place;
  if (!still || air_data || attitude) {
    place = earth.place(body, time);
  }

  bool reads_rates = controls.reads(ROLL_RATE) || controls.reads(PITCH_RATE) ||
                     controls.reads(YAW_RATE);
  Vector3 uvw = Vector3::Zero();
  Vector3 rates = Vector3::Zero();
  if (still) {
    uvw = eigen(earth.air_velocity(body));
    if (reads_rates) {
      rates = eigen(earth.air_rate(body));
    }
  } else {
    BodyMotion motion = compute_body_motion<true>(body, earth, *place, wind);
    uvw = motion.air_velocity;
    rates = motion.air_rate;
  }
  values[BODY_VELOCITY_X] = uvw.x();
  values[BODY_VELOCITY_Y] = uvw.y();
  values[ALPHA] = std::atan2(uvw.z(), uvw.x());
  values[BETA] = std::atan2(uvw.y(), std::hypot(uvw.x(), uvw.z()));
  if (reads_rates) {
    values[ROLL_RATE] = rates.x();
    values[PITCH_RATE] = rates.y();
    values[YAW_RATE] = rates.z();
  }

  if (air_data || attitude) {
    if (air_data) {
      Air here = air(place->altitude);
      double mach =
          uvw.norm() / here.speed_of_sound.numerical_value_in(meter_per_second);
      values[MACH] = mach;
      if (controls.reads(CALIBRATED_AIRSPEED)) {
        values[CALIBRATED_AIRSPEED] = calibrated_airspeed(mach, here)
                                          .numerical_value_in(meter_per_second);
      }
    }
    if (attitude) {
      Matrix3 to_north_east_down =
          place->north_east_down.transpose() * body.attitude.toRotationMatrix();
      // Over the ground: through the air, and with it.
      Vector3 north_east_down =
          to_north_east_down * uvw + eigen(wind.north_east_down);
      values[GROUND_SPEED] =
          std::hypot(north_east_down.x(), north_east_down.y());
      values[PITCH] =
          -std::asin(std::clamp(to_north_east_down(2, 0), -1.0, 1.0));
      values[ROLL] =
          std::atan2(to_north_east_down(2, 1), to_north_east_down(2, 2));
    }
  }

  // The pilot's acceleration: the body's, and the eye point's about the
  // center of mass, in g.
  if (controls.reads(PILOT_ACCELERATION_Y) ||
      controls.reads(PILOT_ACCELERATION_Z)) {
    Vector3 eye = eigen(body_offset(aircraft.eye_point, mass.center_of_mass));
    Vector3 turning = eigen(body.rate);
    Vector3 pilot = eigen(felt.specific_force) +
                    eigen(felt.angular).cross(eye) +
                    turning.cross(turning.cross(eye));
    pilot /= STANDARD_GRAVITY.numerical_value_in(meter_per_second_squared);
    values[PILOT_ACCELERATION_Y] = pilot.y();
    values[PILOT_ACCELERATION_Z] = pilot.z();
  }
  values[WEIGHT_ON_WHEELS] = 0.0;
}

}  // namespace simon::model
