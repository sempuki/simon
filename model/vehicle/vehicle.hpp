// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the CommonRoad vehicle models 3.0.2, Copyright 2020 Technical
// University of Munich, BSD-3-Clause; translated to C++ and changed. See
// NOTICE.md.

#pragma once

#include "core/units.hpp"
#include "model/vehicle/tire.hpp"

// A road vehicle's parameters, as CommonRoad's vehicle models define them
// (Althoff and Wuersching, "CommonRoad: Vehicle Models"; see
// model/REFERENCES.md), and the limits on how it is driven. The defaults are
// CommonRoad's vehicle 1, a Ford Escort, whose multibody parameters come from
// the US Department of Transportation's vehicle dynamics data. Each field
// names CommonRoad's symbol for it.
namespace simon::vehicle {

// The steering a vehicle can do: its angle's range, and how fast the angle
// can change.
struct SteeringLimits final {
  Angle min = -0.91 * radian;
  Angle max = 0.91 * radian;
  AngularRate min_rate = -0.4 * radian_per_second;
  AngularRate max_rate = 0.4 * radian_per_second;
};

// The longitudinal acceleration and speed a vehicle can reach. Above
// `switch_speed` the engine's power limits acceleration to
// max_acceleration switch_speed / v.
struct LongitudinalLimits final {
  AccelerationMagnitude max_acceleration = 11.5 * meter_per_second_squared;
  Speed switch_speed = 4.755 * meter_per_second;
  Speed min_speed = -13.9 * meter_per_second;
  Speed max_speed = 45.8 * meter_per_second;
};

// The suspension between the sprung body and each axle's unsprung mass, and
// the tires' compliance, for the multibody model.
struct Suspension final {
  Stiffness front_spring = 21898.332429625985 * newton_per_meter;  // K_sf.
  Damping front_damping = 1459.3902937206362 * newton_second_per_meter;
  Stiffness rear_spring = 21898.332429625985 * newton_per_meter;  // K_sr.
  Damping rear_damping = 1459.3902937206362 * newton_second_per_meter;
  // The compliant pin joint between body and axle at the roll axis.
  Stiffness roll_axis_spring = 175186.65943700788 * newton_per_meter;  // K_ras.
  Damping roll_axis_damping =
      10215.732056044453 * newton_second_per_meter;  // K_rad.
  // Each axle's auxiliary roll stiffness, normally negative.
  TorsionalStiffness front_roll_stiffness =
      -12880.270509148304 * newton_meter_per_radian;  // K_tsf.
  TorsionalStiffness rear_roll_stiffness =
      0.0 * newton_meter_per_radian;                             // K_tsr.
  Stiffness tire_spring = 189785.5477234252 * newton_per_meter;  // K_zt.
  Compliance tire_compliance =
      1.0278264878518764e-05 * meter_per_newton;  // K_lt.
  // Camber with suspension travel, linear and quadratic.
  units::quantity<radian_per_meter, double> front_camber =
      -0.62335958005249337 * radian_per_meter;  // D_f.
  units::quantity<radian_per_meter, double> rear_camber =
      -0.20997375328083986 * radian_per_meter;  // D_r.
  units::quantity<radian_per_square_meter, double> front_camber_squared =
      0.0 * radian_per_square_meter;  // E_f.
  units::quantity<radian_per_square_meter, double> rear_camber_squared =
      0.0 * radian_per_square_meter;  // E_r.
};

// What the vehicle models read of a vehicle.
struct Parameters final {
  // The wheelbase.
  auto wheelbase() const -> Length { return front + rear; }

  Length length = 4.298 * meter;  // l.
  Length width = 1.674 * meter;   // w.
  Length front =
      0.88392 * meter;            // a: from the center of gravity to each axle.
  Length rear = 1.50876 * meter;  // b.
  Length front_track = 1.389888 * meter;               // T_f.
  Length rear_track = 1.423416 * meter;                // T_r.
  Length center_of_gravity_height = 0.557784 * meter;  // h_cg.
  Length sprung_height = 0.59436 * meter;       // h_s, the sprung body's.
  Length front_roll_axis_height = 0.0 * meter;  // h_raf.
  Length rear_roll_axis_height = 0.0 * meter;   // h_rar.
  Length wheel_radius = 0.344 * meter;          // R_w.

  Mass mass = 1225.8878467253344 * kilogram;                 // m.
  Mass sprung_mass = 1094.5427202904771 * kilogram;          // m_s.
  Mass front_unsprung_mass = 65.672563217428632 * kilogram;  // m_uf.
  Mass rear_unsprung_mass = 65.672563217428632 * kilogram;   // m_ur.

  // The sprung body's, but for yaw, the whole vehicle's.
  MomentOfInertia roll_inertia = 244.04723069965206 * kilogram_square_meter;
  MomentOfInertia pitch_inertia = 1342.2597688480864 * kilogram_square_meter;
  MomentOfInertia yaw_inertia = 1538.8533713561394 * kilogram_square_meter;
  MomentOfInertia roll_yaw_product = 0.0 * kilogram_square_meter;  // I_xz_s.
  // Each axle's unsprung mass in roll, and each wheel's in spin.
  MomentOfInertia front_unsprung_roll_inertia =
      32.53963075995361 * kilogram_square_meter;  // I_uf.
  MomentOfInertia rear_unsprung_roll_inertia =
      32.53963075995361 * kilogram_square_meter;                // I_ur.
  MomentOfInertia wheel_inertia = 1.7 * kilogram_square_meter;  // I_y_w.

  // The front axle's shares of braking and of drive.
  double front_brake_share = 0.76;  // T_sb.
  double front_drive_share = 1.0;   // T_se.

  SteeringLimits steering;
  LongitudinalLimits longitudinal;
  Suspension suspension;
  Tire tire;  // CommonRoad's by default.
};

// What drives a vehicle: the steering angle's rate, and the longitudinal
// acceleration asked of the engine or brakes.
struct Input final {
  AngularRate steering_rate = 0.0 * radian_per_second;
  AccelerationMagnitude acceleration = 0.0 * meter_per_second_squared;
};

// The steering rate the vehicle reaches for `wanted` at `steering`: none
// pressing past either end of the steering's range, and within the rate's
// bounds otherwise.
inline auto limit_steering_rate(Angle steering, AngularRate wanted,
                                const SteeringLimits& limits) -> AngularRate {
  if ((steering <= limits.min && wanted <= 0.0 * radian_per_second) ||
      (steering >= limits.max && wanted >= 0.0 * radian_per_second)) {
    return 0.0 * radian_per_second;
  }
  return clamp(wanted, limits.min_rate, limits.max_rate);
}

// The acceleration the vehicle reaches for `wanted` at `speed`: none pressing
// past either end of the speed's range, at most max_acceleration braking, and
// at most the engine's power allows speeding up.
inline auto limit_acceleration(Speed speed, AccelerationMagnitude wanted,
                               const LongitudinalLimits& limits)
    -> AccelerationMagnitude {
  AccelerationMagnitude most =
      speed > limits.switch_speed
          ? limits.max_acceleration * number_of(limits.switch_speed / speed)
          : limits.max_acceleration;
  if ((speed <= limits.min_speed && wanted <= 0.0 * meter_per_second_squared) ||
      (speed >= limits.max_speed && wanted >= 0.0 * meter_per_second_squared)) {
    return 0.0 * meter_per_second_squared;
  }
  return clamp(wanted, -limits.max_acceleration, most);
}

// The input after the vehicle's limits, at `steering` and `speed`.
inline auto limit_input(const Input& input, Angle steering, Speed speed,
                        const Parameters& vehicle) -> Input {
  return {.steering_rate = limit_steering_rate(steering, input.steering_rate,
                                               vehicle.steering),
          .acceleration = limit_acceleration(speed, input.acceleration,
                                             vehicle.longitudinal)};
}

}  // namespace simon::vehicle
