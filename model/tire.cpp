// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/tire.hpp"

#include <cmath>

namespace simon::model {
namespace {

// The Magic Formula's curve, C atan(B x - E (B x - atan(B x))).
auto compute_magic_angle(double b, double c, double e, double x) -> double {
  double bx = b * x;
  return c * std::atan(bx - e * (bx - std::atan(bx)));
}

auto sign(double x) -> double { return x > 0.0 ? 1.0 : x < 0.0 ? -1.0 : 0.0; }

}  // namespace

// Pacejka's Magic Formula 5.2 for pure and combined slip (Pacejka, chapter
// 4), as CommonRoad's tire_model writes it. The lateral shifts at pure slip
// take the camber's sign, so that a left and a right tire mirror each other
// and a vehicle with no camber does not pull. Pacejka adds the vertical
// shift F_z p_vx1 after the sine, and turns the side force longitudinal slip
// induces with kappa. CommonRoad adds the shift inside the sine, as an angle,
// which brakes a tire rolling free, and turns the side force with -kappa,
// its own slip (see application/automotive/reference/commonroad_dynamic.py).
auto compute_tire_force(const MagicFormulaTire& p, const TireSlip& slip,
                        Force load) -> TireForce {
  double f_z = load.numerical_value_in(newton);
  if (!(f_z > 0.0)) {
    return {};
  }
  double kappa = slip.longitudinal;
  double alpha = radians(slip.lateral);
  double gamma = radians(slip.camber);

  // Longitudinal, pure slip.
  double kappa_x = kappa + p.p_hx1;
  double d_x = p.p_dx1 * (1.0 - p.p_dx3 * gamma * gamma) * f_z;
  double b_x = f_z * p.p_kx1 / (p.p_cx1 * d_x);
  double f0_x =
      d_x * std::sin(compute_magic_angle(b_x, p.p_cx1, p.p_ex1, kappa_x)) +
      f_z * p.p_vx1;

  // Lateral, pure slip.
  double s_hy = sign(gamma) * (p.p_hy1 + p.p_hy3 * std::abs(gamma));
  double s_vy = sign(gamma) * f_z * (p.p_vy1 + p.p_vy3 * std::abs(gamma));
  double mu_y = p.p_dy1 * (1.0 - p.p_dy3 * gamma * gamma);
  double d_y = mu_y * f_z;
  double b_y = f_z * p.p_ky1 / (p.p_cy1 * d_y);
  double f0_y =
      d_y * std::sin(compute_magic_angle(b_y, p.p_cy1, p.p_ey1, alpha + s_hy)) +
      s_vy;

  // Longitudinal, combined: weighted by the slip angle.
  double b_xa = p.r_bx1 * std::cos(std::atan(p.r_bx2 * kappa));
  double g_xa =
      std::cos(compute_magic_angle(b_xa, p.r_cx1, p.r_ex1, alpha + p.r_hx1)) /
      std::cos(compute_magic_angle(b_xa, p.r_cx1, p.r_ex1, p.r_hx1));

  // Lateral, combined: weighted by kappa, with the side force it induces.
  double b_yk = p.r_by1 * std::cos(std::atan(p.r_by2 * (alpha - p.r_by3)));
  double g_yk =
      std::cos(compute_magic_angle(b_yk, p.r_cy1, p.r_ey1, kappa + p.r_hy1)) /
      std::cos(compute_magic_angle(b_yk, p.r_cy1, p.r_ey1, p.r_hy1));
  double d_vyk = mu_y * f_z * (p.r_vy1 + p.r_vy3 * gamma) *
                 std::cos(std::atan(p.r_vy4 * alpha));
  double s_vyk = d_vyk * std::sin(p.r_vy5 * std::atan(p.r_vy6 * kappa));

  return TireForce{.longitudinal = g_xa * f0_x * newton,
                   .lateral = (g_yk * f0_y + s_vyk) * newton};
}

}  // namespace simon::model
