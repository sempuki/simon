// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows the CommonRoad vehicle models 3.0.2, Copyright 2020 Technical
// University of Munich, BSD-3-Clause; translated to C++ and changed. See
// NOTICE.md.

#include "model/vehicle/multibody.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "model/vehicle/tire.hpp"

namespace simon::vehicle {
namespace {

constexpr double GRAVITY = 9.81;  // m/s^2, as CommonRoad has it.

// The speed below which the body moves as the kinematic model, and the least
// ground speed a wheel's slip is divided by, as the drift model bounds it.
constexpr double CRAWL = 0.1;  // m/s.

auto compute_rate_numbers(const MultibodyNumbers& x, double u0, double u1,
                          const WheelSteer& toe, const Parameters& vehicle)
    -> MultibodyNumbers;

}  // namespace

auto convert_multibody_to_numbers(const Multibody& state) -> MultibodyNumbers {
  auto axle = [](const UnsprungAxle& axle) {
    return std::array{radians(axle.roll),
                      axle.roll_rate.numerical_value_in(radian_per_second),
                      axle.lateral_speed.numerical_value_in(meter_per_second),
                      axle.height.numerical_value_in(meter),
                      axle.vertical_speed.numerical_value_in(meter_per_second)};
  };
  std::array front = axle(state.front);
  std::array rear = axle(state.rear);
  const SprungBody& body = state.body;
  return {
      state.x.numerical_value_in(meter),
      state.y.numerical_value_in(meter),
      radians(state.steering),
      state.speed.numerical_value_in(meter_per_second),
      radians(state.heading),
      state.yaw_rate.numerical_value_in(radian_per_second),
      radians(body.roll),
      body.roll_rate.numerical_value_in(radian_per_second),
      radians(body.pitch),
      body.pitch_rate.numerical_value_in(radian_per_second),
      body.lateral_speed.numerical_value_in(meter_per_second),
      body.height.numerical_value_in(meter),
      body.vertical_speed.numerical_value_in(meter_per_second),
      front[0],
      front[1],
      front[2],
      front[3],
      front[4],
      rear[0],
      rear[1],
      rear[2],
      rear[3],
      rear[4],
      state.wheels[0].numerical_value_in(radian_per_second),
      state.wheels[1].numerical_value_in(radian_per_second),
      state.wheels[2].numerical_value_in(radian_per_second),
      state.wheels[3].numerical_value_in(radian_per_second),
      state.front_pin.numerical_value_in(meter),
      state.rear_pin.numerical_value_in(meter),
  };
}

auto convert_numbers_to_multibody(const MultibodyNumbers& x) -> Multibody {
  auto axle = [&](std::size_t at) {
    return UnsprungAxle{.roll = x[at] * radian,
                        .roll_rate = x[at + 1] * radian_per_second,
                        .lateral_speed = x[at + 2] * meter_per_second,
                        .height = x[at + 3] * meter,
                        .vertical_speed = x[at + 4] * meter_per_second};
  };
  return Multibody{
      .x = x[0] * meter,
      .y = x[1] * meter,
      .steering = x[2] * radian,
      .speed = x[3] * meter_per_second,
      .heading = x[4] * radian,
      .yaw_rate = x[5] * radian_per_second,
      .body = {.roll = x[6] * radian,
               .roll_rate = x[7] * radian_per_second,
               .pitch = x[8] * radian,
               .pitch_rate = x[9] * radian_per_second,
               .lateral_speed = x[10] * meter_per_second,
               .height = x[11] * meter,
               .vertical_speed = x[12] * meter_per_second},
      .front = axle(13),
      .rear = axle(18),
      .wheels = {x[23] * radian_per_second, x[24] * radian_per_second,
                 x[25] * radian_per_second, x[26] * radian_per_second},
      .front_pin = x[27] * meter,
      .rear_pin = x[28] * meter,
  };
}

auto convert_multibody_rate_to_numbers(const MultibodyRate& rate)
    -> MultibodyNumbers {
  auto axle = [](const UnsprungAxleRate& axle) {
    return std::array{
        axle.roll.numerical_value_in(radian_per_second),
        axle.roll_rate.numerical_value_in(radian_per_second_squared),
        axle.lateral_speed.numerical_value_in(meter_per_second_squared),
        axle.height.numerical_value_in(meter_per_second),
        axle.vertical_speed.numerical_value_in(meter_per_second_squared)};
  };
  std::array front = axle(rate.front);
  std::array rear = axle(rate.rear);
  const SprungBodyRate& body = rate.body;
  return {
      rate.x.numerical_value_in(meter_per_second),
      rate.y.numerical_value_in(meter_per_second),
      rate.steering.numerical_value_in(radian_per_second),
      rate.speed.numerical_value_in(meter_per_second_squared),
      rate.heading.numerical_value_in(radian_per_second),
      rate.yaw_rate.numerical_value_in(radian_per_second_squared),
      body.roll.numerical_value_in(radian_per_second),
      body.roll_rate.numerical_value_in(radian_per_second_squared),
      body.pitch.numerical_value_in(radian_per_second),
      body.pitch_rate.numerical_value_in(radian_per_second_squared),
      body.lateral_speed.numerical_value_in(meter_per_second_squared),
      body.height.numerical_value_in(meter_per_second),
      body.vertical_speed.numerical_value_in(meter_per_second_squared),
      front[0],
      front[1],
      front[2],
      front[3],
      front[4],
      rear[0],
      rear[1],
      rear[2],
      rear[3],
      rear[4],
      rate.wheels[0].numerical_value_in(radian_per_second_squared),
      rate.wheels[1].numerical_value_in(radian_per_second_squared),
      rate.wheels[2].numerical_value_in(radian_per_second_squared),
      rate.wheels[3].numerical_value_in(radian_per_second_squared),
      rate.front_pin.numerical_value_in(meter_per_second),
      rate.rear_pin.numerical_value_in(meter_per_second),
  };
}

auto convert_numbers_to_multibody_rate(const MultibodyNumbers& f)
    -> MultibodyRate {
  auto axle = [&](std::size_t at) {
    return UnsprungAxleRate{
        .roll = f[at] * radian_per_second,
        .roll_rate = f[at + 1] * radian_per_second_squared,
        .lateral_speed = f[at + 2] * meter_per_second_squared,
        .height = f[at + 3] * meter_per_second,
        .vertical_speed = f[at + 4] * meter_per_second_squared};
  };
  return MultibodyRate{
      .x = f[0] * meter_per_second,
      .y = f[1] * meter_per_second,
      .steering = f[2] * radian_per_second,
      .speed = f[3] * meter_per_second_squared,
      .heading = f[4] * radian_per_second,
      .yaw_rate = f[5] * radian_per_second_squared,
      .body = {.roll = f[6] * radian_per_second,
               .roll_rate = f[7] * radian_per_second_squared,
               .pitch = f[8] * radian_per_second,
               .pitch_rate = f[9] * radian_per_second_squared,
               .lateral_speed = f[10] * meter_per_second_squared,
               .height = f[11] * meter_per_second,
               .vertical_speed = f[12] * meter_per_second_squared},
      .front = axle(13),
      .rear = axle(18),
      .wheels = {f[23] * radian_per_second_squared,
                 f[24] * radian_per_second_squared,
                 f[25] * radian_per_second_squared,
                 f[26] * radian_per_second_squared},
      .front_pin = f[27] * meter_per_second,
      .rear_pin = f[28] * meter_per_second,
  };
}

auto operator+(const MultibodyRate& a, const MultibodyRate& b)
    -> MultibodyRate {
  MultibodyNumbers sum = convert_multibody_rate_to_numbers(a);
  MultibodyNumbers other = convert_multibody_rate_to_numbers(b);
  for (std::size_t i = 0; i < sum.size(); ++i) {
    sum[i] += other[i];
  }
  return convert_numbers_to_multibody_rate(sum);
}

auto operator*(double weight, const MultibodyRate& rate) -> MultibodyRate {
  MultibodyNumbers scaled = convert_multibody_rate_to_numbers(rate);
  for (double& value : scaled) {
    value *= weight;
  }
  return convert_numbers_to_multibody_rate(scaled);
}

auto advance(const Multibody& state, const MultibodyRate& rate, Duration dt)
    -> Multibody {
  double seconds = simon::seconds(dt).numerical_value_in(second);
  MultibodyNumbers x = convert_multibody_to_numbers(state);
  MultibodyNumbers f = convert_multibody_rate_to_numbers(rate);
  for (std::size_t i = 0; i < x.size(); ++i) {
    x[i] += f[i] * seconds;
  }
  return convert_numbers_to_multibody(x);
}

auto start_multibody(Speed speed, const Parameters& vehicle) -> Multibody {
  double l = vehicle.wheelbase().numerical_value_in(meter);
  double k_zt =
      vehicle.suspension.tire_spring.numerical_value_in(newton_per_meter);
  double m_s = vehicle.sprung_mass.numerical_value_in(kilogram);
  // Each axle's static load, on its two tires' springs.
  double front_load =
      m_s * GRAVITY * vehicle.rear.numerical_value_in(meter) / l +
      vehicle.front_unsprung_mass.numerical_value_in(kilogram) * GRAVITY;
  double rear_load =
      m_s * GRAVITY * vehicle.front.numerical_value_in(meter) / l +
      vehicle.rear_unsprung_mass.numerical_value_in(kilogram) * GRAVITY;
  AngularRate rolling = speed / vehicle.wheel_radius * radian;
  return Multibody{
      .speed = speed,
      .front = {.height = front_load / (2.0 * k_zt) * meter},
      .rear = {.height = rear_load / (2.0 * k_zt) * meter},
      .wheels = {rolling, rolling, rolling, rolling},
  };
}

auto compute_multibody_rate(const Multibody& state, const Input& input,
                            const Parameters& vehicle, const WheelSteer& toe)
    -> MultibodyRate {
  Input limited = limit_input(input, state.steering, state.speed, vehicle);
  return convert_numbers_to_multibody_rate(compute_rate_numbers(
      convert_multibody_to_numbers(state),
      limited.steering_rate.numerical_value_in(radian_per_second),
      limited.acceleration.numerical_value_in(meter_per_second_squared), toe,
      vehicle));
}

namespace {

// vehicle_dynamics_mb, line for line, in CommonRoad's numbering from 0 and
// its names for the parameters, with each wheel steered on its own and the
// tires' aligning moments in the yaw, and two bounds of simon's: a wheel's slip
// is divided by at least 0.1 m/s of ground speed, as the drift model divides
// it, where CommonRoad divides by a speed that may be zero; and a tire off
// the ground pushes nothing (see model/tire.hpp).
auto compute_rate_numbers(const MultibodyNumbers& x, double u0, double u1,
                          const WheelSteer& toe, const Parameters& vehicle)
    -> MultibodyNumbers {
  const double g = GRAVITY;
  const Suspension& s = vehicle.suspension;
  const double a = vehicle.front.numerical_value_in(meter);
  const double b = vehicle.rear.numerical_value_in(meter);
  const double m = vehicle.mass.numerical_value_in(kilogram);
  const double m_s = vehicle.sprung_mass.numerical_value_in(kilogram);
  const double m_uf = vehicle.front_unsprung_mass.numerical_value_in(kilogram);
  const double m_ur = vehicle.rear_unsprung_mass.numerical_value_in(kilogram);
  const double I_z =
      vehicle.yaw_inertia.numerical_value_in(kilogram_square_meter);
  const double I_Phi_s =
      vehicle.roll_inertia.numerical_value_in(kilogram_square_meter);
  const double I_y_s =
      vehicle.pitch_inertia.numerical_value_in(kilogram_square_meter);
  const double I_xz_s =
      vehicle.roll_yaw_product.numerical_value_in(kilogram_square_meter);
  const double I_uf = vehicle.front_unsprung_roll_inertia.numerical_value_in(
      kilogram_square_meter);
  const double I_ur = vehicle.rear_unsprung_roll_inertia.numerical_value_in(
      kilogram_square_meter);
  const double I_y_w =
      vehicle.wheel_inertia.numerical_value_in(kilogram_square_meter);
  const double R_w = vehicle.wheel_radius.numerical_value_in(meter);
  const double T_f = vehicle.front_track.numerical_value_in(meter);
  const double T_r = vehicle.rear_track.numerical_value_in(meter);
  const double h_s = vehicle.sprung_height.numerical_value_in(meter);
  const double h_raf = vehicle.front_roll_axis_height.numerical_value_in(meter);
  const double h_rar = vehicle.rear_roll_axis_height.numerical_value_in(meter);
  const double K_zt = s.tire_spring.numerical_value_in(newton_per_meter);
  const double K_lt = s.tire_compliance.numerical_value_in(meter_per_newton);
  const double K_sf = s.front_spring.numerical_value_in(newton_per_meter);
  const double K_sr = s.rear_spring.numerical_value_in(newton_per_meter);
  const double K_sdf =
      s.front_damping.numerical_value_in(newton_second_per_meter);
  const double K_sdr =
      s.rear_damping.numerical_value_in(newton_second_per_meter);
  const double K_ras = s.roll_axis_spring.numerical_value_in(newton_per_meter);
  const double K_rad =
      s.roll_axis_damping.numerical_value_in(newton_second_per_meter);
  const double K_tsf =
      s.front_roll_stiffness.numerical_value_in(newton_meter_per_radian);
  const double K_tsr =
      s.rear_roll_stiffness.numerical_value_in(newton_meter_per_radian);
  const double D_f = s.front_camber.numerical_value_in(radian_per_meter);
  const double D_r = s.rear_camber.numerical_value_in(radian_per_meter);
  const double E_f =
      s.front_camber_squared.numerical_value_in(radian_per_square_meter);
  const double E_r =
      s.rear_camber_squared.numerical_value_in(radian_per_square_meter);
  const double T_sb = vehicle.front_brake_share;
  const double T_se = vehicle.front_drive_share;

  bool crawling = std::abs(x[3]) < CRAWL;
  double beta = crawling ? 0.0 : std::atan(x[10] / x[3]);
  double vel = std::sqrt(x[3] * x[3] + x[10] * x[10]);

  // Vertical tire forces.
  double F_z_LF =
      (x[16] + R_w * (std::cos(x[13]) - 1) - 0.5 * T_f * std::sin(x[13])) *
      K_zt;
  double F_z_RF =
      (x[16] + R_w * (std::cos(x[13]) - 1) + 0.5 * T_f * std::sin(x[13])) *
      K_zt;
  double F_z_LR =
      (x[21] + R_w * (std::cos(x[18]) - 1) - 0.5 * T_r * std::sin(x[18])) *
      K_zt;
  double F_z_RR =
      (x[21] + R_w * (std::cos(x[18]) - 1) + 0.5 * T_r * std::sin(x[18])) *
      K_zt;

  // Each wheel's steer: the steering and its toe at the front, its toe at
  // the rear.
  double delta_LF = x[2] + radians(toe.left_front);
  double delta_RF = x[2] + radians(toe.right_front);
  double delta_LR = radians(toe.left_rear);
  double delta_RR = radians(toe.right_rear);

  // Each tire's ground speed along its wheel, never negative.
  auto along = [&](double u, double v, double delta) {
    return std::max(0.0, u * std::cos(delta) + v * std::sin(delta));
  };
  double u_w_lf = along(x[3] + 0.5 * T_f * x[5], x[10] + a * x[5], delta_LF);
  double u_w_rf = along(x[3] - 0.5 * T_f * x[5], x[10] + a * x[5], delta_RF);
  double u_w_lr = along(x[3] + 0.5 * T_r * x[5], x[10] - b * x[5], delta_LR);
  double u_w_rr = along(x[3] - 0.5 * T_r * x[5], x[10] - b * x[5], delta_RR);

  // Longitudinal slip as kappa = (R omega - u) / u, the negative of
  // CommonRoad's s, and slip angles; none at a crawl.
  auto slip = [&](double omega, double u) {
    return crawling ? 0.0 : R_w * omega / std::max(u, CRAWL) - 1.0;
  };
  double kappa_lf = slip(x[23], u_w_lf);
  double kappa_rf = slip(x[24], u_w_rf);
  double kappa_lr = slip(x[25], u_w_lr);
  double kappa_rr = slip(x[26], u_w_rr);
  double alpha_LF = 0.0;
  double alpha_RF = 0.0;
  double alpha_LR = 0.0;
  double alpha_RR = 0.0;
  if (!crawling) {
    alpha_LF = std::atan((x[10] + a * x[5] - x[14] * (R_w - x[16])) /
                         (x[3] + 0.5 * T_f * x[5])) -
               delta_LF;
    alpha_RF = std::atan((x[10] + a * x[5] - x[14] * (R_w - x[16])) /
                         (x[3] - 0.5 * T_f * x[5])) -
               delta_RF;
    alpha_LR = std::atan((x[10] - b * x[5] - x[19] * (R_w - x[21])) /
                         (x[3] + 0.5 * T_r * x[5])) -
               delta_LR;
    alpha_RR = std::atan((x[10] - b * x[5] - x[19] * (R_w - x[21])) /
                         (x[3] - 0.5 * T_r * x[5])) -
               delta_RR;
  }

  // Suspension travel at each corner, and its speed.
  double z_SLF = (h_s - R_w + x[16] - x[11]) / std::cos(x[6]) - h_s + R_w +
                 a * x[8] + 0.5 * (x[6] - x[13]) * T_f;
  double z_SRF = (h_s - R_w + x[16] - x[11]) / std::cos(x[6]) - h_s + R_w +
                 a * x[8] - 0.5 * (x[6] - x[13]) * T_f;
  double z_SLR = (h_s - R_w + x[21] - x[11]) / std::cos(x[6]) - h_s + R_w -
                 b * x[8] + 0.5 * (x[6] - x[18]) * T_r;
  double z_SRR = (h_s - R_w + x[21] - x[11]) / std::cos(x[6]) - h_s + R_w -
                 b * x[8] - 0.5 * (x[6] - x[18]) * T_r;

  double dz_SLF = x[17] - x[12] + a * x[9] + 0.5 * (x[7] - x[14]) * T_f;
  double dz_SRF = x[17] - x[12] + a * x[9] - 0.5 * (x[7] - x[14]) * T_f;
  double dz_SLR = x[22] - x[12] - b * x[9] + 0.5 * (x[7] - x[19]) * T_r;
  double dz_SRR = x[22] - x[12] - b * x[9] - 0.5 * (x[7] - x[19]) * T_r;

  // Camber.
  double gamma_LF = x[6] + D_f * z_SLF + E_f * z_SLF * z_SLF;
  double gamma_RF = x[6] - D_f * z_SRF - E_f * z_SRF * z_SRF;
  double gamma_LR = x[6] + D_r * z_SLR + E_r * z_SLR * z_SLR;
  double gamma_RR = x[6] - D_r * z_SRR - E_r * z_SRR * z_SRR;

  // Tire forces, combined slip.
  auto tire = [&](double kappa, double alpha, double gamma, double load,
                  TireSide side) {
    TireForce force = compute_tire_force(vehicle.tire,
                                         {.longitudinal = kappa,
                                          .lateral = alpha * radian,
                                          .camber = gamma * radian},
                                         load * newton, side);
    return std::array{force.longitudinal.numerical_value_in(newton),
                      force.lateral.numerical_value_in(newton),
                      force.aligning.numerical_value_in(newton_meter)};
  };
  auto [F_x_LF, F_y_LF, M_z_LF] =
      tire(kappa_lf, alpha_LF, gamma_LF, F_z_LF, TireSide::LEFT);
  auto [F_x_RF, F_y_RF, M_z_RF] =
      tire(kappa_rf, alpha_RF, gamma_RF, F_z_RF, TireSide::RIGHT);
  auto [F_x_LR, F_y_LR, M_z_LR] =
      tire(kappa_lr, alpha_LR, gamma_LR, F_z_LR, TireSide::LEFT);
  auto [F_x_RR, F_y_RR, M_z_RR] =
      tire(kappa_rr, alpha_RR, gamma_RR, F_z_RR, TireSide::RIGHT);

  // Each tire's forces in the body's axes, turned by its wheel's steer.
  auto body_x = [](double f_x, double f_y, double delta) {
    return f_x * std::cos(delta) - f_y * std::sin(delta);
  };
  auto body_y = [](double f_x, double f_y, double delta) {
    return f_x * std::sin(delta) + f_y * std::cos(delta);
  };
  double X_LF = body_x(F_x_LF, F_y_LF, delta_LF);
  double X_RF = body_x(F_x_RF, F_y_RF, delta_RF);
  double X_LR = body_x(F_x_LR, F_y_LR, delta_LR);
  double X_RR = body_x(F_x_RR, F_y_RR, delta_RR);
  double Y_LF = body_y(F_x_LF, F_y_LF, delta_LF);
  double Y_RF = body_y(F_x_RF, F_y_RF, delta_RF);
  double Y_LR = body_y(F_x_LR, F_y_LR, delta_LR);
  double Y_RR = body_y(F_x_RR, F_y_RR, delta_RR);

  // The compliant pins' travel, and their forces.
  double delta_z_f = h_s - R_w + x[16] - x[11];
  double delta_z_r = h_s - R_w + x[21] - x[11];
  double delta_phi_f = x[6] - x[13];
  double delta_phi_r = x[6] - x[18];
  double dot_delta_phi_f = x[7] - x[14];
  double dot_delta_phi_r = x[7] - x[19];
  double dot_delta_z_f = x[17] - x[12];
  double dot_delta_z_r = x[22] - x[12];
  double dot_delta_y_f = x[10] + a * x[5] - x[15];
  double dot_delta_y_r = x[10] - b * x[5] - x[20];

  double delta_f = delta_z_f * std::sin(x[6]) - x[27] * std::cos(x[6]) -
                   (h_raf - R_w) * std::sin(delta_phi_f);
  double delta_r = delta_z_r * std::sin(x[6]) - x[28] * std::cos(x[6]) -
                   (h_rar - R_w) * std::sin(delta_phi_r);
  double dot_delta_f =
      (delta_z_f * std::cos(x[6]) + x[27] * std::sin(x[6])) * x[7] +
      dot_delta_z_f * std::sin(x[6]) - dot_delta_y_f * std::cos(x[6]) -
      (h_raf - R_w) * std::cos(delta_phi_f) * dot_delta_phi_f;
  double dot_delta_r =
      (delta_z_r * std::cos(x[6]) + x[28] * std::sin(x[6])) * x[7] +
      dot_delta_z_r * std::sin(x[6]) - dot_delta_y_r * std::cos(x[6]) -
      (h_rar - R_w) * std::cos(delta_phi_r) * dot_delta_phi_r;
  double F_RAF = delta_f * K_ras + dot_delta_f * K_rad;
  double F_RAR = delta_r * K_ras + dot_delta_r * K_rad;

  // Suspension forces, bump stops and squat and lift left out.
  double F_SLF = m_s * g * b / (2 * (a + b)) - z_SLF * K_sf - dz_SLF * K_sdf +
                 (x[6] - x[13]) * K_tsf / T_f;
  double F_SRF = m_s * g * b / (2 * (a + b)) - z_SRF * K_sf - dz_SRF * K_sdf -
                 (x[6] - x[13]) * K_tsf / T_f;
  double F_SLR = m_s * g * a / (2 * (a + b)) - z_SLR * K_sr - dz_SLR * K_sdr +
                 (x[6] - x[18]) * K_tsr / T_r;
  double F_SRR = m_s * g * a / (2 * (a + b)) - z_SRR * K_sr - dz_SRR * K_sdr -
                 (x[6] - x[18]) * K_tsr / T_r;

  // The sprung body's forces and moments. Its axes are SAE's, x forward, y
  // right and z down, the left wheels at -y.
  double sumX = X_LF + X_RF + X_LR + X_RR;
  // The tires' aligning moments yaw it too; CommonRoad's tire has none.
  double sumN = (Y_LF + Y_RF) * a + (X_LF - X_RF) * 0.5 * T_f +
                (X_LR - X_RR) * 0.5 * T_r - (Y_LR + Y_RR) * b + M_z_LF +
                M_z_RF + M_z_LR + M_z_RR;
  double sumY_s = (F_RAF + F_RAR) * std::cos(x[6]) +
                  (F_SLF + F_SLR + F_SRF + F_SRR) * std::sin(x[6]);
  double sumL =
      0.5 * F_SLF * T_f + 0.5 * F_SLR * T_r - 0.5 * F_SRF * T_f -
      0.5 * F_SRR * T_r -
      F_RAF / std::cos(x[6]) *
          (h_s - x[11] - R_w + x[16] - (h_raf - R_w) * std::cos(x[13])) -
      F_RAR / std::cos(x[6]) *
          (h_s - x[11] - R_w + x[21] - (h_rar - R_w) * std::cos(x[18]));
  double sumZ_s = (F_SLF + F_SLR + F_SRF + F_SRR) * std::cos(x[6]) -
                  (F_RAF + F_RAR) * std::sin(x[6]);
  double sumM_s =
      a * (F_SLF + F_SRF) - b * (F_SLR + F_SRR) + sumX * (h_s - x[11]);

  // The unsprung axles' forces and moments.
  double sumL_uf = 0.5 * F_SRF * T_f - 0.5 * F_SLF * T_f -
                   F_RAF * (h_raf - R_w) +
                   F_z_LF * (R_w * std::sin(x[13]) +
                             0.5 * T_f * std::cos(x[13]) - K_lt * F_y_LF) -
                   F_z_RF * (-R_w * std::sin(x[13]) +
                             0.5 * T_f * std::cos(x[13]) + K_lt * F_y_RF) -
                   (Y_LF + Y_RF) * (R_w - x[16]);
  double sumL_ur = 0.5 * F_SRR * T_r - 0.5 * F_SLR * T_r -
                   F_RAR * (h_rar - R_w) +
                   F_z_LR * (R_w * std::sin(x[18]) +
                             0.5 * T_r * std::cos(x[18]) - K_lt * F_y_LR) -
                   F_z_RR * (-R_w * std::sin(x[18]) +
                             0.5 * T_r * std::cos(x[18]) + K_lt * F_y_RR) -
                   (Y_LR + Y_RR) * (R_w - x[21]);
  double sumZ_uf = F_z_LF + F_z_RF + F_RAF * std::sin(x[6]) -
                   (F_SLF + F_SRF) * std::cos(x[6]);
  double sumZ_ur = F_z_LR + F_z_RR + F_RAR * std::sin(x[6]) -
                   (F_SLR + F_SRR) * std::cos(x[6]);
  double sumY_uf =
      Y_LF + Y_RF - F_RAF * std::cos(x[6]) - (F_SLF + F_SRF) * std::sin(x[6]);
  double sumY_ur =
      Y_LR + Y_RR - F_RAR * std::cos(x[6]) - (F_SLR + F_SRR) * std::sin(x[6]);

  MultibodyNumbers f{};
  if (crawling) {
    // The kinematic model about the center of gravity, as the single-track
    // models crawl (see model/single_track.cpp), with the slip angle, zero
    // at a crawl, where CommonRoad reads the roll angle.
    double l = a + b;
    double cos_steering = std::cos(x[2]);
    double tangent = std::tan(x[2]);
    double kinematic_beta = std::atan(tangent * b / l);
    double ratio = tangent * b / l;
    double d_beta =
        b * u0 / (l * cos_steering * cos_steering * (1.0 + ratio * ratio));
    f[0] = x[3] * std::cos(kinematic_beta + x[4]);
    f[1] = x[3] * std::sin(kinematic_beta + x[4]);
    f[2] = u0;
    f[3] = u1;
    f[4] = x[3] * std::cos(kinematic_beta) * tangent / l;
    f[5] = 1.0 / l *
           (u1 * std::cos(beta) * tangent -
            x[3] * std::sin(beta) * d_beta * tangent +
            x[3] * std::cos(beta) * u0 / (cos_steering * cos_steering));
  } else {
    f[0] = std::cos(beta + x[4]) * vel;
    f[1] = std::sin(beta + x[4]) * vel;
    f[2] = u0;
    f[3] = 1.0 / m * sumX + x[5] * x[10];
    f[4] = x[5];
    f[5] = 1.0 / (I_z - I_xz_s * I_xz_s / I_Phi_s) *
           (sumN + I_xz_s / I_Phi_s * sumL);
  }

  // The sprung body.
  f[6] = x[7];
  f[7] = 1.0 / (I_Phi_s - I_xz_s * I_xz_s / I_z) * (I_xz_s / I_z * sumN + sumL);
  f[8] = x[9];
  f[9] = 1.0 / I_y_s * sumM_s;
  f[10] = 1.0 / m_s * sumY_s - x[5] * x[3];
  f[11] = x[12];
  f[12] = g - 1.0 / m_s * sumZ_s;

  // The front and rear axles.
  f[13] = x[14];
  f[14] = 1.0 / I_uf * sumL_uf;
  f[15] = 1.0 / m_uf * sumY_uf - x[5] * x[3];
  f[16] = x[17];
  f[17] = g - 1.0 / m_uf * sumZ_uf;
  f[18] = x[19];
  f[19] = 1.0 / I_ur * sumL_ur;
  f[20] = 1.0 / m_ur * sumY_ur - x[5] * x[3];
  f[21] = x[22];
  f[22] = g - 1.0 / m_ur * sumZ_ur;

  // The wheels, under brake and engine torque split front to rear and evenly
  // side to side. A wheel spinning backward stops.
  double T_B = u1 > 0.0 ? 0.0 : m * R_w * u1;
  double T_E = u1 > 0.0 ? m * R_w * u1 : 0.0;
  f[23] = 1.0 / I_y_w * (-R_w * F_x_LF + 0.5 * T_sb * T_B + 0.5 * T_se * T_E);
  f[24] = 1.0 / I_y_w * (-R_w * F_x_RF + 0.5 * T_sb * T_B + 0.5 * T_se * T_E);
  f[25] = 1.0 / I_y_w *
          (-R_w * F_x_LR + 0.5 * (1 - T_sb) * T_B + 0.5 * (1 - T_se) * T_E);
  f[26] = 1.0 / I_y_w *
          (-R_w * F_x_RR + 0.5 * (1 - T_sb) * T_B + 0.5 * (1 - T_se) * T_E);
  for (std::size_t i = 23; i < 27; ++i) {
    if (x[i] < 0.0) {
      f[i] = 0.0;
    }
  }

  // The pins.
  f[27] = dot_delta_y_f;
  f[28] = dot_delta_y_r;
  return f;
}

}  // namespace
}  // namespace simon::vehicle
