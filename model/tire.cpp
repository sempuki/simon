// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/tire.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <variant>

namespace simon::model {
namespace {

// The Magic Formula's curve, C atan(B x - E (B x - atan(B x))).
auto compute_magic_angle(double b, double c, double e, double x) -> double {
  double bx = b * x;
  return c * std::atan(bx - e * (bx - std::atan(bx)));
}

}  // namespace

// Pacejka's Magic Formula 5.2 for pure and combined slip (Pacejka, chapter
// 4), as CommonRoad's tire_model writes it. The lateral shifts at pure slip
// take the camber's sign, so that a left and a right tire mirror each other
// and a vehicle with no camber does not pull. Pacejka adds the vertical
// shift F_z p_vx1 after the sine, and turns the side force longitudinal slip
// induces with kappa. CommonRoad adds the shift inside the sine, as an angle,
// which brakes a tire rolling free, and turns the side force with -kappa,
// its own slip (see application/automotive/reference/commonroad_dynamic.py).
auto compute_tire_force(const CommonRoadTire& p, const TireSlip& slip,
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

// The Magic Formula 5.2's steady state, as Pacejka gives it (chapter 4),
// turn slip left out: the pure-slip forces, their weighting in combined
// slip, and the aligning moment from the pneumatic trail and the residual
// moment, with the slip angles at which they act made equivalent in combined
// slip. Each curvature factor E is at most 1. The load enters through
// dfz = (Fz - Fz0') / Fz0', Fz0' the nominal load scaled.
auto compute_tire_force(const MagicFormulaTire& p, const TireSlip& slip,
                        Force load) -> TireForce {
  double f_z = load.numerical_value_in(newton);
  if (!(f_z > 0.0)) {
    return {};
  }
  double kappa = slip.longitudinal;
  double alpha = radians(slip.lateral);
  double gamma = radians(slip.camber);
  double f_z0 = p.fnomin * p.lfzo;
  double dfz = (f_z - f_z0) / f_z0;

  // Longitudinal, pure slip.
  double s_hx = (p.phx1 + p.phx2 * dfz) * p.lhx;
  double kappa_x = kappa + s_hx;
  double gamma_x = gamma * p.lgax;
  double c_x = p.pcx1 * p.lcx;
  double mu_x =
      (p.pdx1 + p.pdx2 * dfz) * (1.0 - p.pdx3 * gamma_x * gamma_x) * p.lmux;
  double d_x = mu_x * f_z;
  double e_x = std::min(1.0, (p.pex1 + p.pex2 * dfz + p.pex3 * dfz * dfz) *
                                 (1.0 - p.pex4 * sign(kappa_x)) * p.lex);
  double k_x = f_z * (p.pkx1 + p.pkx2 * dfz) * std::exp(p.pkx3 * dfz) * p.lkx;
  double b_x = k_x / (c_x * d_x);
  double s_vx = f_z * (p.pvx1 + p.pvx2 * dfz) * p.lvx * p.lmux;
  double f0_x =
      d_x * std::sin(compute_magic_angle(b_x, c_x, e_x, kappa_x)) + s_vx;

  // Lateral, pure slip.
  double gamma_y = gamma * p.lgay;
  double s_hy = (p.phy1 + p.phy2 * dfz) * p.lhy + p.phy3 * gamma_y;
  double alpha_y = alpha + s_hy;
  double c_y = p.pcy1 * p.lcy;
  double mu_y =
      (p.pdy1 + p.pdy2 * dfz) * (1.0 - p.pdy3 * gamma_y * gamma_y) * p.lmuy;
  double d_y = mu_y * f_z;
  double e_y = std::min(
      1.0, (p.pey1 + p.pey2 * dfz) *
               (1.0 - (p.pey3 + p.pey4 * gamma_y) * sign(alpha_y)) * p.ley);
  double k_y0 =
      p.pky1 * f_z0 * std::sin(2.0 * std::atan(f_z / (p.pky2 * f_z0))) * p.lky;
  double k_y = k_y0 * (1.0 - p.pky3 * std::abs(gamma_y));
  double b_y = k_y / (c_y * d_y);
  double s_vy =
      f_z *
      ((p.pvy1 + p.pvy2 * dfz) * p.lvy + (p.pvy3 + p.pvy4 * dfz) * gamma_y) *
      p.lmuy;
  double f0_y =
      d_y * std::sin(compute_magic_angle(b_y, c_y, e_y, alpha_y)) + s_vy;

  // Longitudinal, combined: weighted by the slip angle.
  double s_hxa = p.rhx1;
  double b_xa = p.rbx1 * std::cos(std::atan(p.rbx2 * kappa)) * p.lxal;
  double e_xa = std::min(1.0, p.rex1 + p.rex2 * dfz);
  double g_xa =
      std::cos(compute_magic_angle(b_xa, p.rcx1, e_xa, alpha + s_hxa)) /
      std::cos(compute_magic_angle(b_xa, p.rcx1, e_xa, s_hxa));
  double f_x = g_xa * f0_x;

  // Lateral, combined: weighted by kappa, with the side force it induces.
  double s_hyk = p.rhy1 + p.rhy2 * dfz;
  double b_yk =
      p.rby1 * std::cos(std::atan(p.rby2 * (alpha - p.rby3))) * p.lyka;
  double e_yk = std::min(1.0, p.rey1 + p.rey2 * dfz);
  double g_yk =
      std::cos(compute_magic_angle(b_yk, p.rcy1, e_yk, kappa + s_hyk)) /
      std::cos(compute_magic_angle(b_yk, p.rcy1, e_yk, s_hyk));
  double d_vyk = mu_y * f_z * (p.rvy1 + p.rvy2 * dfz + p.rvy3 * gamma) *
                 std::cos(std::atan(p.rvy4 * alpha));
  double s_vyk = d_vyk * std::sin(p.rvy5 * std::atan(p.rvy6 * kappa)) * p.lvyka;
  double f_y = g_yk * f0_y + s_vyk;

  // Aligning moment: the pneumatic trail t and the residual moment M_zr, at
  // slip angles made equivalent in combined slip, and the moment of F_x
  // about the contact patch's lateral offset s.
  double r0 = p.unloaded_radius;
  double gamma_z = gamma * p.lgaz;
  double s_ht = p.qhz1 + p.qhz2 * dfz + (p.qhz3 + p.qhz4 * dfz) * gamma_z;
  double s_hf = s_hy + s_vy / k_y;
  double alpha_t = alpha + s_ht;
  double alpha_r = alpha + s_hf;
  double b_t = (p.qbz1 + p.qbz2 * dfz + p.qbz3 * dfz * dfz) *
               (1.0 + p.qbz4 * gamma_z + p.qbz5 * std::abs(gamma_z)) * p.lky /
               p.lmuy;
  double c_t = p.qcz1;
  double d_t = f_z * (p.qdz1 + p.qdz2 * dfz) *
               (1.0 + p.qdz3 * gamma_z + p.qdz4 * gamma_z * gamma_z) * r0 /
               f_z0 * p.ltr;
  double e_t = std::min(
      1.0, (p.qez1 + p.qez2 * dfz + p.qez3 * dfz * dfz) *
               (1.0 + (p.qez4 + p.qez5 * gamma_z) * (2.0 / std::numbers::pi) *
                          std::atan(b_t * c_t * alpha_t)));
  double b_r = p.qbz9 * p.lky / p.lmuy + p.qbz10 * b_y * c_y;
  double d_r =
      f_z *
      ((p.qdz6 + p.qdz7 * dfz) * p.lres + (p.qdz8 + p.qdz9 * dfz) * gamma_z) *
      r0 * p.lmuy;
  double stiffness = k_x / k_y;
  auto equivalent = [&](double angle) {
    double tangent = std::tan(angle);
    return std::atan(std::sqrt(tangent * tangent +
                               stiffness * stiffness * kappa * kappa)) *
           sign(angle);
  };
  double t = d_t *
             std::cos(compute_magic_angle(b_t, c_t, e_t, equivalent(alpha_t))) *
             std::cos(alpha);
  double m_zr =
      d_r * std::cos(std::atan(b_r * equivalent(alpha_r))) * std::cos(alpha);
  double s = (p.ssz1 + p.ssz2 * f_y / f_z0 + (p.ssz3 + p.ssz4 * dfz) * gamma) *
             r0 * p.ls;
  double m_z = -t * (f_y - s_vyk) + m_zr + s * f_x;

  return TireForce{.longitudinal = f_x * newton,
                   .lateral = f_y * newton,
                   .aligning = m_z * newton_meter};
}

auto compute_tire_force(const Tire& tire, const TireSlip& slip, Force load,
                        TireSide side) -> TireForce {
  struct Mount final {
    auto operator()(const CommonRoadTire& model) const -> TireForce {
      return compute_tire_force(model, slip, load);
    }
    auto operator()(const MagicFormulaTire& model) const -> TireForce {
      if (side == TireSide::LEFT) {
        return compute_tire_force(model, slip, load);
      }
      TireForce mirrored =
          compute_tire_force(model,
                             {.longitudinal = slip.longitudinal,
                              .lateral = -slip.lateral,
                              .camber = -slip.camber},
                             load);
      return {.longitudinal = mirrored.longitudinal,
              .lateral = -mirrored.lateral,
              .aligning = -mirrored.aligning};
    }
    const TireSlip& slip;
    Force load;
    TireSide side;
  };
  return std::visit(Mount{.slip = slip, .load = load, .side = side}, tire);
}

// The Magic Formula's cornering stiffness at the nominal load Fz0' is
// p_ky1 Fz0' sin(2 atan(1 / p_ky2)) lambda_Ky; per unit load and friction,
// with its sign turned as CommonRoad turns -p_ky1.
auto compute_linear_tire(const Tire& tire) -> LinearTire {
  struct Linearize final {
    auto operator()(const CommonRoadTire& p) const -> LinearTire {
      return {.friction = p.p_dy1, .cornering_stiffness = -p.p_ky1 / p.p_dy1};
    }
    auto operator()(const MagicFormulaTire& p) const -> LinearTire {
      double friction = p.pdy1 * p.lmuy;
      double stiffness =
          -p.pky1 * std::sin(2.0 * std::atan(1.0 / p.pky2)) * p.lky;
      return {.friction = friction,
              .cornering_stiffness = stiffness / friction};
    }
  };
  return std::visit(Linearize{}, tire);
}

}  // namespace simon::model
