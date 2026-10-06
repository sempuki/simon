// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the CommonRoad vehicle models 3.0.2, Copyright 2020 Technical
// University of Munich, BSD-3-Clause; translated to C++ and changed. See
// NOTICE.md.

#include "model/single_track.hpp"

#include <algorithm>
#include <cmath>

namespace simon::model {
namespace {

constexpr double GRAVITY = 9.81;  // m/s^2, as CommonRoad has it.

// The kinematic model about the center of gravity (CommonRoad's
// vehicle_dynamics_ks_cog), with the rates of its slip angle and yaw rate,
// which the dynamic models take at a crawl. beta = atan(tan(delta) b / l), so
// by the chain rule
// beta' = b / l delta' / (cos(delta)^2 (1 + (tan(delta) b / l)^2)), and the
// yaw rate v cos(beta) tan(delta) / l changes at its derivative, with the
// state's slip angle for beta.
struct Crawl final {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double yaw_rate = 0.0;
  double slip_angle = 0.0;
};

auto compute_crawl(double steering, double speed, double heading,
                   double slip_angle, double steering_rate, double acceleration,
                   double b, double l) -> Crawl {
  double tangent = std::tan(steering);
  double cosine = std::cos(steering);
  double beta = std::atan(tangent * b / l);
  double ratio = tangent * b / l;
  double slip_rate =
      b * steering_rate / (l * cosine * cosine * (1.0 + ratio * ratio));
  return Crawl{
      .x = speed * std::cos(beta + heading),
      .y = speed * std::sin(beta + heading),
      .heading = speed * std::cos(beta) * tangent / l,
      .yaw_rate =
          1.0 / l *
          (acceleration * std::cos(slip_angle) * tangent -
           speed * std::sin(slip_angle) * slip_rate * tangent +
           speed * std::cos(slip_angle) * steering_rate / (cosine * cosine)),
      .slip_angle = slip_rate,
  };
}

}  // namespace

//-- Dynamic single-track model ------------------------------------------------

auto operator+(const DynamicSingleTrackRate& a, const DynamicSingleTrackRate& b)
    -> DynamicSingleTrackRate {
  return {.x = a.x + b.x,
          .y = a.y + b.y,
          .steering = a.steering + b.steering,
          .speed = a.speed + b.speed,
          .heading = a.heading + b.heading,
          .yaw_rate = a.yaw_rate + b.yaw_rate,
          .slip_angle = a.slip_angle + b.slip_angle};
}

auto operator*(double weight, const DynamicSingleTrackRate& rate)
    -> DynamicSingleTrackRate {
  return {.x = weight * rate.x,
          .y = weight * rate.y,
          .steering = weight * rate.steering,
          .speed = weight * rate.speed,
          .heading = weight * rate.heading,
          .yaw_rate = weight * rate.yaw_rate,
          .slip_angle = weight * rate.slip_angle};
}

auto advance(const DynamicSingleTrack& state,
             const DynamicSingleTrackRate& rate, framework::Duration dt)
    -> DynamicSingleTrack {
  Time seconds = model::seconds(dt);
  return {.x = state.x + rate.x * seconds,
          .y = state.y + rate.y * seconds,
          .steering = state.steering + rate.steering * seconds,
          .speed = state.speed + rate.speed * seconds,
          .heading = state.heading + rate.heading * seconds,
          .yaw_rate = state.yaw_rate + rate.yaw_rate * seconds,
          .slip_angle = state.slip_angle + rate.slip_angle * seconds};
}

// The single-track model as CommonRoad's vehicle_dynamics_st writes it: each
// axle's lateral force C_S mu F_z alpha, with F_z the axle's static load
// shifted by the acceleration through the center of gravity's height, and the
// cornering stiffness C_S and friction mu the tire's, -p_ky1 / p_dy1 and p_dy1.
auto compute_dynamic_single_track_rate(const DynamicSingleTrack& state,
                                       const VehicleInput& input,
                                       const VehicleParameters& vehicle)
    -> DynamicSingleTrackRate {
  VehicleInput limited =
      limit_input(input, state.steering, state.speed, vehicle);
  double u0 = limited.steering_rate.numerical_value_in(radian_per_second);
  double u1 = limited.acceleration.numerical_value_in(meter_per_second_squared);
  double steering = radians(state.steering);
  double v = state.speed.numerical_value_in(meter_per_second);
  double heading = radians(state.heading);
  double yaw_rate = state.yaw_rate.numerical_value_in(radian_per_second);
  double beta = radians(state.slip_angle);

  double lf = vehicle.front.numerical_value_in(meter);
  double lr = vehicle.rear.numerical_value_in(meter);
  double l = lf + lr;

  DynamicSingleTrackRate rate{
      .steering = limited.steering_rate,
      .speed = limited.acceleration,
      .heading = state.yaw_rate,
  };
  if (std::abs(v) < 0.1) {
    Crawl crawl = compute_crawl(steering, v, heading, beta, u0, u1, lr, l);
    rate.x = crawl.x * meter_per_second;
    rate.y = crawl.y * meter_per_second;
    rate.heading = crawl.heading * radian_per_second;
    rate.yaw_rate = crawl.yaw_rate * radian_per_second_squared;
    rate.slip_angle = crawl.slip_angle * radian_per_second;
    return rate;
  }

  LinearTire linear = compute_linear_tire(vehicle.tire);
  double mu = linear.friction;
  double c_sf = linear.cornering_stiffness;
  double c_sr = c_sf;
  double h = vehicle.sprung_height.numerical_value_in(meter);
  double m = vehicle.mass.numerical_value_in(kilogram);
  double inertia =
      vehicle.yaw_inertia.numerical_value_in(kilogram_square_meter);
  double front_load = GRAVITY * lr - u1 * h;  // Per unit mass and wheelbase.
  double rear_load = GRAVITY * lf + u1 * h;

  rate.x = v * std::cos(beta + heading) * meter_per_second;
  rate.y = v * std::sin(beta + heading) * meter_per_second;
  rate.yaw_rate =
      (-mu * m / (v * inertia * l) *
           (lf * lf * c_sf * front_load + lr * lr * c_sr * rear_load) *
           yaw_rate +
       mu * m / (inertia * l) *
           (lr * c_sr * rear_load - lf * c_sf * front_load) * beta +
       mu * m / (inertia * l) * lf * c_sf * front_load * steering) *
      radian_per_second_squared;
  rate.slip_angle =
      ((mu / (v * v * l) * (c_sr * rear_load * lr - c_sf * front_load * lf) -
        1.0) *
           yaw_rate -
       mu / (v * l) * (c_sr * rear_load + c_sf * front_load) * beta +
       mu / (v * l) * c_sf * front_load * steering) *
      radian_per_second;
  return rate;
}

//-- Drift single-track model --------------------------------------------------

auto operator+(const DriftSingleTrackRate& a, const DriftSingleTrackRate& b)
    -> DriftSingleTrackRate {
  return {.x = a.x + b.x,
          .y = a.y + b.y,
          .steering = a.steering + b.steering,
          .speed = a.speed + b.speed,
          .heading = a.heading + b.heading,
          .yaw_rate = a.yaw_rate + b.yaw_rate,
          .slip_angle = a.slip_angle + b.slip_angle,
          .front_wheel = a.front_wheel + b.front_wheel,
          .rear_wheel = a.rear_wheel + b.rear_wheel};
}

auto operator*(double weight, const DriftSingleTrackRate& rate)
    -> DriftSingleTrackRate {
  return {.x = weight * rate.x,
          .y = weight * rate.y,
          .steering = weight * rate.steering,
          .speed = weight * rate.speed,
          .heading = weight * rate.heading,
          .yaw_rate = weight * rate.yaw_rate,
          .slip_angle = weight * rate.slip_angle,
          .front_wheel = weight * rate.front_wheel,
          .rear_wheel = weight * rate.rear_wheel};
}

auto advance(const DriftSingleTrack& state, const DriftSingleTrackRate& rate,
             framework::Duration dt) -> DriftSingleTrack {
  Time seconds = model::seconds(dt);
  return {.x = state.x + rate.x * seconds,
          .y = state.y + rate.y * seconds,
          .steering = state.steering + rate.steering * seconds,
          .speed = state.speed + rate.speed * seconds,
          .heading = state.heading + rate.heading * seconds,
          .yaw_rate = state.yaw_rate + rate.yaw_rate * seconds,
          .slip_angle = state.slip_angle + rate.slip_angle * seconds,
          .front_wheel = state.front_wheel + rate.front_wheel * seconds,
          .rear_wheel = state.rear_wheel + rate.rear_wheel * seconds};
}

auto start_drift_single_track(Speed speed, const VehicleParameters& vehicle)
    -> DriftSingleTrack {
  AngularRate rolling = speed / vehicle.wheel_radius * radian;
  return {.speed = speed, .front_wheel = rolling, .rear_wheel = rolling};
}

// The drift model as CommonRoad's vehicle_dynamics_std writes it. Each axle's
// slip angle and longitudinal slip, from its ground velocity and its wheel's
// speed, give the Magic Formula's forces at the axle's load; they move and turn
// the body, and the wheel spins up under drive or brake torque and down under
// its tire's force. Below 0.2 m/s the model blends into the kinematic one by
// tanh((v - 0.2) / 0.05), its wheels then rolling to the ground's speed with a
// time constant of 0.02 s.
auto compute_drift_single_track_rate(const DriftSingleTrack& state,
                                     const VehicleInput& input,
                                     const VehicleParameters& vehicle)
    -> DriftSingleTrackRate {
  constexpr double BLEND_SPEED = 0.2;               // v_s, m/s.
  constexpr double BLEND_WIDTH = 0.05;              // v_b, m/s.
  constexpr double SLIP_SPEED = BLEND_SPEED / 2.0;  // v_min, m/s.
  constexpr double WHEEL_TIME = 0.02;               // s.

  VehicleInput limited =
      limit_input(input, state.steering, state.speed, vehicle);
  double u0 = limited.steering_rate.numerical_value_in(radian_per_second);
  double u1 = limited.acceleration.numerical_value_in(meter_per_second_squared);
  double steering = radians(state.steering);
  double v = state.speed.numerical_value_in(meter_per_second);
  double heading = radians(state.heading);
  double yaw_rate = state.yaw_rate.numerical_value_in(radian_per_second);
  double beta = radians(state.slip_angle);
  double front_wheel = state.front_wheel.numerical_value_in(radian_per_second);
  double rear_wheel = state.rear_wheel.numerical_value_in(radian_per_second);

  double lf = vehicle.front.numerical_value_in(meter);
  double lr = vehicle.rear.numerical_value_in(meter);
  double l = lf + lr;
  double m = vehicle.mass.numerical_value_in(kilogram);
  double inertia =
      vehicle.yaw_inertia.numerical_value_in(kilogram_square_meter);
  double r_w = vehicle.wheel_radius.numerical_value_in(meter);
  double i_w = vehicle.wheel_inertia.numerical_value_in(kilogram_square_meter);
  double h = vehicle.sprung_height.numerical_value_in(meter);

  // Each axle's slip angle, load and ground speed along its wheel.
  bool moving = v > SLIP_SPEED;
  double alpha_f = moving ? std::atan((v * std::sin(beta) + yaw_rate * lf) /
                                      (v * std::cos(beta))) -
                                steering
                          : 0.0;
  double alpha_r = moving ? std::atan((v * std::sin(beta) - yaw_rate * lr) /
                                      (v * std::cos(beta)))
                          : 0.0;
  double f_zf = m * (-u1 * h + GRAVITY * lr) / l;
  double f_zr = m * (u1 * h + GRAVITY * lf) / l;
  double u_wf = std::max(
      0.0, v * std::cos(beta) * std::cos(steering) +
               (v * std::sin(beta) + lf * yaw_rate) * std::sin(steering));
  double u_wr = std::max(0.0, v * std::cos(beta));

  // kappa = (R omega - u) / u, the negative of CommonRoad's slip.
  double kappa_f = r_w * front_wheel / std::max(u_wf, SLIP_SPEED) - 1.0;
  double kappa_r = r_w * rear_wheel / std::max(u_wr, SLIP_SPEED) - 1.0;
  // Each axle's left and right tires, each at half the axle's load.
  // CommonRoad's tire is linear in its load and the same on either side, so
  // to it this is one tire at the whole load; the Magic Formula's is not.
  auto compute_axle_force = [&](double kappa, double alpha, double load) {
    TireSlip slip{.longitudinal = kappa, .lateral = alpha * radian};
    TireForce left = compute_tire_force(vehicle.tire, slip, 0.5 * load * newton,
                                        TireSide::LEFT);
    TireForce right = compute_tire_force(vehicle.tire, slip,
                                         0.5 * load * newton, TireSide::RIGHT);
    return TireForce{.longitudinal = left.longitudinal + right.longitudinal,
                     .lateral = left.lateral + right.lateral,
                     .aligning = left.aligning + right.aligning};
  };
  TireForce front = compute_axle_force(kappa_f, alpha_f, f_zf);
  TireForce rear = compute_axle_force(kappa_r, alpha_r, f_zr);
  double f_xf = front.longitudinal.numerical_value_in(newton);
  double f_yf = front.lateral.numerical_value_in(newton);
  double f_xr = rear.longitudinal.numerical_value_in(newton);
  double f_yr = rear.lateral.numerical_value_in(newton);
  double m_z =
      (front.aligning + rear.aligning).numerical_value_in(newton_meter);

  double brake = u1 > 0.0 ? 0.0 : m * r_w * u1;  // T_B.
  double drive = u1 > 0.0 ? m * r_w * u1 : 0.0;  // T_E.

  double d_v = 1.0 / m *
               (-f_yf * std::sin(steering - beta) + f_yr * std::sin(beta) +
                f_xr * std::cos(beta) + f_xf * std::cos(steering - beta));
  // The tires' aligning moments yaw it too; CommonRoad's tire has none.
  double dd_psi = 1.0 / inertia *
                  (f_yf * std::cos(steering) * lf - f_yr * lr +
                   f_xf * std::sin(steering) * lf + m_z);
  double d_beta =
      moving ? -yaw_rate + 1.0 / (m * v) *
                               (f_yf * std::cos(steering - beta) +
                                f_yr * std::cos(beta) - f_xr * std::sin(beta) +
                                f_xf * std::sin(steering - beta))
             : 0.0;

  // A wheel spinning backward stops, and spins as if stopped.
  double t_sb = vehicle.front_brake_share;
  double t_se = vehicle.front_drive_share;
  double d_omega_f =
      front_wheel >= 0.0
          ? 1.0 / i_w * (-r_w * f_xf + t_sb * brake + t_se * drive)
          : 0.0;
  double d_omega_r =
      rear_wheel >= 0.0
          ? 1.0 / i_w *
                (-r_w * f_xr + (1.0 - t_sb) * brake + (1.0 - t_se) * drive)
          : 0.0;
  front_wheel = std::max(0.0, front_wheel);
  rear_wheel = std::max(0.0, rear_wheel);

  Crawl crawl = compute_crawl(steering, v, heading, beta, u0, u1, lr, l);
  double d_omega_f_ks = (u_wf / r_w - front_wheel) / WHEEL_TIME;
  double d_omega_r_ks = (u_wr / r_w - rear_wheel) / WHEEL_TIME;

  double w_std = 0.5 * (std::tanh((v - BLEND_SPEED) / BLEND_WIDTH) + 1.0);
  double w_ks = 1.0 - w_std;
  return DriftSingleTrackRate{
      .x = v * std::cos(beta + heading) * meter_per_second,
      .y = v * std::sin(beta + heading) * meter_per_second,
      .steering = limited.steering_rate,
      .speed = (w_std * d_v + w_ks * u1) * meter_per_second_squared,
      .heading = (w_std * yaw_rate + w_ks * crawl.heading) * radian_per_second,
      .yaw_rate =
          (w_std * dd_psi + w_ks * crawl.yaw_rate) * radian_per_second_squared,
      .slip_angle =
          (w_std * d_beta + w_ks * crawl.slip_angle) * radian_per_second,
      .front_wheel =
          (w_std * d_omega_f + w_ks * d_omega_f_ks) * radian_per_second_squared,
      .rear_wheel =
          (w_std * d_omega_r + w_ks * d_omega_r_ks) * radian_per_second_squared,
  };
}

}  // namespace simon::model
