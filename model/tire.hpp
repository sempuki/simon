// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include "model/units.hpp"

// A tire's forces by Pacejka's Magic Formula 5.2 (Pacejka, Tire and Vehicle
// Dynamics; see model/REFERENCES.md), in the subset CommonRoad's vehicle
// models use (Althoff and Wuersching, "CommonRoad: Vehicle Models"): pure and
// combined slip, every scaling factor 1, turn slip and load dependence beyond
// the linear left out. Forces and slips are in ISO tire axes, x forward along
// the wheel's heading and y to its left.
//
//   F = D sin(C atan(B x - E (B x - atan(B x)))) + S_V,   x = slip + S_H
//
// The coefficients are named as Pacejka and CommonRoad name them, and are
// plain numbers, since each carries its own unit.
namespace simon::model {

// The defaults are CommonRoad's tire, from the ADAMS handbook, which all its
// vehicles share.
struct MagicFormulaTire final {
  // Longitudinal, pure slip.
  double p_cx1 = 1.6411;
  double p_dx1 = 1.1739;
  double p_dx3 = 0.0;
  double p_ex1 = 0.46403;
  double p_kx1 = 22.303;
  double p_hx1 = 0.0012297;
  double p_vx1 = -8.8098e-06;

  // Longitudinal, combined slip.
  double r_bx1 = 13.276;
  double r_bx2 = -13.778;
  double r_cx1 = 1.2568;
  double r_ex1 = 0.65225;
  double r_hx1 = 0.0050722;

  // Lateral, pure slip.
  double p_cy1 = 1.3507;
  double p_dy1 = 1.0489;
  double p_dy3 = -2.8821;
  double p_ey1 = -0.0074722;
  double p_ky1 = -21.92;
  double p_hy1 = 0.0026747;
  double p_hy3 = 0.031415;
  double p_vy1 = 0.037318;
  double p_vy3 = -0.32931;

  // Lateral, combined slip.
  double r_by1 = 7.1433;
  double r_by2 = 9.1916;
  double r_by3 = -0.027856;
  double r_cy1 = 1.0719;
  double r_ey1 = -0.27572;
  double r_hy1 = 5.7448e-06;
  double r_vy1 = -0.027825;
  double r_vy3 = -0.27568;
  double r_vy4 = 12.12;
  double r_vy5 = 1.9;
  double r_vy6 = -10.704;
};

// How a tire slips: kappa, the longitudinal slip (R omega - u) / u, positive
// driving; alpha, the slip angle, the wheel's ground velocity's angle from its
// heading, positive to the left; and gamma, the camber.
struct TireSlip final {
  double longitudinal = 0.0;
  Angle lateral = 0.0 * radian;
  Angle camber = 0.0 * radian;
};

struct TireForce final {
  Force longitudinal = 0.0 * newton;
  Force lateral = 0.0 * newton;
};

// The tire's forces at `slip` under vertical `load`, none if the tire is off
// the ground.
auto compute_tire_force(const MagicFormulaTire& tire, const TireSlip& slip,
                        Force load) -> TireForce;

}  // namespace simon::model
