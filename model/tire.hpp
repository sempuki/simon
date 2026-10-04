// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#pragma once

#include <cstdint>
#include <variant>

#include "model/units.hpp"

// A tire's forces by Pacejka's Magic Formula 5.2, as PAC2002 (Pacejka, Tyre
// and Vehicle Dynamics, 2002, chapter 4; Kuiper and van Oosten; see
// model/REFERENCES.md): steady state, in pure and
// combined slip. Forces and slips are in ISO tire axes, x forward along the
// wheel's heading, y to its left and z up.
//
//   F = D sin(C atan(B x - E (B x - atan(B x)))) + S_V,   x = slip + S_H
//
// MagicFormulaTire has the whole of 5.2's steady state as TNO's tire
// property files (.tir) give it, read by format/tire_file. CommonRoadTire has
// the subset CommonRoad's vehicle models use (Althoff and Wuersching,
// "CommonRoad: Vehicle Models"): every scaling factor 1, the load dependence
// beyond the linear and the aligning moment left out, and the lateral shifts
// mirrored by the camber's sign. The coefficients are named as Pacejka and
// the property files name them, and are plain numbers, since each carries
// its own unit.
namespace simon::model {

// CommonRoad's subset. The defaults are CommonRoad's tire, from the ADAMS
// handbook, which all its vehicles share.
struct CommonRoadTire final {
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

// A tire's forces, and its aligning moment about z.
struct TireForce final {
  Force longitudinal = 0.0 * newton;
  Force lateral = 0.0 * newton;
  Torque aligning = 0.0 * newton_meter;
};

// The Magic Formula 5.2's steady-state coefficients, named as a tire property
// file names them. Scaling factors default to 1 and the rest to 0.
struct MagicFormulaTire final {
  double unloaded_radius = 0.0;  // R0, m.
  double fnomin = 0.0;           // Fz0, the nominal load, N.

  // Scaling factors.
  double lfzo = 1.0;
  double lcx = 1.0;
  double lmux = 1.0;
  double lex = 1.0;
  double lkx = 1.0;
  double lhx = 1.0;
  double lvx = 1.0;
  double lgax = 1.0;
  double lcy = 1.0;
  double lmuy = 1.0;
  double ley = 1.0;
  double lky = 1.0;
  double lhy = 1.0;
  double lvy = 1.0;
  double lgay = 1.0;
  double ltr = 1.0;
  double lres = 1.0;
  double lgaz = 1.0;
  double lxal = 1.0;
  double lyka = 1.0;
  double lvyka = 1.0;
  double ls = 1.0;

  // Longitudinal, pure and combined slip.
  double pcx1 = 0.0;
  double pdx1 = 0.0;
  double pdx2 = 0.0;
  double pdx3 = 0.0;
  double pex1 = 0.0;
  double pex2 = 0.0;
  double pex3 = 0.0;
  double pex4 = 0.0;
  double pkx1 = 0.0;
  double pkx2 = 0.0;
  double pkx3 = 0.0;
  double phx1 = 0.0;
  double phx2 = 0.0;
  double pvx1 = 0.0;
  double pvx2 = 0.0;
  double rbx1 = 0.0;
  double rbx2 = 0.0;
  double rcx1 = 0.0;
  double rex1 = 0.0;
  double rex2 = 0.0;
  double rhx1 = 0.0;

  // Lateral, pure and combined slip.
  double pcy1 = 0.0;
  double pdy1 = 0.0;
  double pdy2 = 0.0;
  double pdy3 = 0.0;
  double pey1 = 0.0;
  double pey2 = 0.0;
  double pey3 = 0.0;
  double pey4 = 0.0;
  double pky1 = 0.0;
  double pky2 = 0.0;
  double pky3 = 0.0;
  double phy1 = 0.0;
  double phy2 = 0.0;
  double phy3 = 0.0;
  double pvy1 = 0.0;
  double pvy2 = 0.0;
  double pvy3 = 0.0;
  double pvy4 = 0.0;
  double rby1 = 0.0;
  double rby2 = 0.0;
  double rby3 = 0.0;
  double rcy1 = 0.0;
  double rey1 = 0.0;
  double rey2 = 0.0;
  double rhy1 = 0.0;
  double rhy2 = 0.0;
  double rvy1 = 0.0;
  double rvy2 = 0.0;
  double rvy3 = 0.0;
  double rvy4 = 0.0;
  double rvy5 = 0.0;
  double rvy6 = 0.0;

  // Aligning moment, pure and combined slip.
  double qbz1 = 0.0;
  double qbz2 = 0.0;
  double qbz3 = 0.0;
  double qbz4 = 0.0;
  double qbz5 = 0.0;
  double qbz9 = 0.0;
  double qbz10 = 0.0;
  double qcz1 = 0.0;
  double qdz1 = 0.0;
  double qdz2 = 0.0;
  double qdz3 = 0.0;
  double qdz4 = 0.0;
  double qdz6 = 0.0;
  double qdz7 = 0.0;
  double qdz8 = 0.0;
  double qdz9 = 0.0;
  double qez1 = 0.0;
  double qez2 = 0.0;
  double qez3 = 0.0;
  double qez4 = 0.0;
  double qez5 = 0.0;
  double qhz1 = 0.0;
  double qhz2 = 0.0;
  double qhz3 = 0.0;
  double qhz4 = 0.0;
  double ssz1 = 0.0;
  double ssz2 = 0.0;
  double ssz3 = 0.0;
  double ssz4 = 0.0;
};

// Either tire, as a vehicle carries it.
using Tire = std::variant<CommonRoadTire, MagicFormulaTire>;

// The tire's forces at `slip` under vertical `load`, none if the tire is off
// the ground. CommonRoad's tire has no aligning moment.
auto compute_tire_force(const CommonRoadTire& tire, const TireSlip& slip,
                        Force load) -> TireForce;
auto compute_tire_force(const MagicFormulaTire& tire, const TireSlip& slip,
                        Force load) -> TireForce;
// The side of the vehicle a tire is mounted on.
enum class TireSide : std::uint8_t { LEFT, RIGHT };

// The forces of `tire` mounted on `side`. A Magic Formula tire's property
// file describes it on the left, as TYRESIDE = 'LEFT' says, so on the right
// it is mirrored about its wheel plane: its forces at (kappa, -alpha,
// -gamma), F_y and M_z turned, so that a pair's asymmetries cancel as the
// tires' do when mounted mirrored. CommonRoad's tire mirrors itself through
// the camber's sign and is the same on either side.
auto compute_tire_force(const Tire& tire, const TireSlip& slip, Force load,
                        TireSide side = TireSide::LEFT) -> TireForce;

// A tire linear in its slip angle, as the dynamic single-track model has it:
// its lateral force -C_S mu F_z alpha, with mu its friction and C_S its
// cornering stiffness per unit load and friction.
struct LinearTire final {
  double friction = 0.0;             // mu.
  double cornering_stiffness = 0.0;  // C_S, per radian.
};

// The linear tire matching `tire` at small slip: CommonRoad's -p_ky1 / p_dy1
// and p_dy1, or the Magic Formula's at its nominal load.
auto compute_linear_tire(const Tire& tire) -> LinearTire;

}  // namespace simon::model
