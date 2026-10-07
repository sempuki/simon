// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

#include "application/automotive/testing.hpp"
#include "base/testing.hpp"
#include "format/tire_file.hpp"
#include "model/tire.hpp"

// simon's Magic Formula 5.2 tire against Project Chrono's Pac02, on the
// Sedan's tire, from the table reference/chrono_reference.cpp recorded.
// Chrono departs from Pacejka in five places, so the checks are where the
// two coincide, and the departures are measured:
//
// - Each B divides by C D + 0.1 rather than C D.
// - B x is clamped to within pi/2 - 0.01, which flattens the curve beyond.
// - Camber raises the lateral friction by 1 + p_dy3 gamma^2, where Pacejka
//   lowers it by 1 - p_dy3 gamma^2.
// - In combined slip the trail's equivalent slip angle takes kappa's sign,
//   where Pacejka gives it the slip angle's.
// - By default it combines slip by a friction ellipsis, not Pacejka's
//   weighting functions.
namespace simon::automotive {

namespace {

using namespace testing;
using model::MagicFormulaTire;

auto load_sedan_tire() -> MagicFormulaTire {
  auto tire =
      format::load_tire_file(std::string{TIRES} + "Sedan_Pac02Tire.tir");
  REQUIRE(tire.has_value());
  return *tire;
}

// Whether Chrono's clamp on B x leaves the row alone, longitudinal and
// lateral.
auto unclamped(const MagicFormulaTire& p, const Row& row) -> bool {
  double load = number(row, "load");
  double kappa = number(row, "kappa");
  double alpha = number(row, "alpha");
  double f_z0 = p.fnomin * p.lfzo;
  double dfz = (load - f_z0) / f_z0;
  double d_x = (p.pdx1 + p.pdx2 * dfz) * load;
  double b_x = load * (p.pkx1 + p.pkx2 * dfz) * std::exp(p.pkx3 * dfz) /
               (p.pcx1 * d_x + 0.1);
  double k_y =
      p.pky1 * f_z0 * std::sin(2.0 * std::atan(load / (p.pky2 * f_z0)));
  double d_y = (p.pdy1 + p.pdy2 * dfz) * load;
  double b_y = k_y / (p.pcy1 * d_y + 0.1);
  double limit = std::numbers::pi / 2.0 - 0.01;
  return std::abs(b_x * (kappa + p.phx1 + p.phx2 * dfz)) < limit &&
         std::abs(b_y * (alpha + p.phy1 + p.phy2 * dfz)) < limit;
}

}  // namespace

TEST_CASE("MagicFormulaTireAgainstChrono") {
  MagicFormulaTire tire = load_sedan_tire();
  std::vector<Row> rows = load_rows("chrono_tires.csv");
  REQUIRE(rows.size() == 4 * 25 * 21 * 2);

  auto force_of = [&](const Row& row) {
    return model::compute_tire_force(tire,
                                     {.longitudinal = number(row, "kappa"),
                                      .lateral = number(row, "alpha") * radian,
                                      .camber = number(row, "gamma") * radian},
                                     number(row, "load") * newton);
  };
  auto relative = [](double ours, double theirs, double scale) {
    return std::abs(ours - theirs) / std::max(1.0, scale);
  };

  SECTION("ShouldReadTheSedanTire") {
    CHECK(tire.fnomin == 4850.0);
    CHECK(tire.unloaded_radius == 0.344);
    CHECK(tire.lfzo == 0.81);
    CHECK(tire.pky1 == -21.92);
    CHECK(tire.qbz1 == 10.904);
  }

  SECTION("ShouldMatchForcesWhereChronoKeepsTheFormula") {
    // Without camber and inside the clamp, the forces differ only by
    // Chrono's 0.1 in each B, relative to the load.
    double longitudinal = 0.0;
    double lateral = 0.0;
    double aligning = 0.0;
    int compared = 0;
    for (const Row& row : rows) {
      if (number(row, "gamma") != 0.0 || !unclamped(tire, row)) {
        continue;
      }
      model::TireForce force = force_of(row);
      double load = number(row, "load");
      longitudinal = std::max(
          longitudinal, relative(force.longitudinal.numerical_value_in(newton),
                                 number(row, "fx"), load));
      lateral =
          std::max(lateral, relative(force.lateral.numerical_value_in(newton),
                                     number(row, "fy"), load));
      // The trail's equivalent slip angle agrees where kappa and the slip
      // angle share a sign. Chrono's moment is in its own frame, turned.
      double kappa = number(row, "kappa");
      double alpha = number(row, "alpha");
      if (kappa * alpha > 0.0) {
        aligning = std::max(
            aligning, relative(force.aligning.numerical_value_in(newton_meter),
                               number(row, "mz"), load * tire.unloaded_radius));
      }
      ++compared;
    }
    CAPTURE(compared, longitudinal, lateral, aligning);
    CHECK(compared > 400);
    CHECK(longitudinal < 1e-4);
    CHECK(lateral < 1e-4);
    CHECK(aligning < 1e-4);
  }

  SECTION("ShouldDepartWhereChronoDeparts") {
    // Beyond the clamp, with camber, and by the friction ellipsis, Chrono's
    // forces differ from the formula: by up to 7.7% of the load beyond the
    // clamp, 0.48% at a camber of 0.03 rad, and 60% where the ellipsis
    // combines large slips both ways.
    double clamped = 0.0;
    double cambered = 0.0;
    double ellipsis = 0.0;
    for (const Row& row : rows) {
      model::TireForce force = force_of(row);
      double load = number(row, "load");
      double fx = force.longitudinal.numerical_value_in(newton);
      double fy = force.lateral.numerical_value_in(newton);
      double apart = std::max(relative(fx, number(row, "fx"), load),
                              relative(fy, number(row, "fy"), load));
      if (!unclamped(tire, row)) {
        clamped = std::max(clamped, apart);
      } else if (number(row, "gamma") != 0.0) {
        cambered = std::max(cambered, apart);
      } else {
        ellipsis =
            std::max({ellipsis, relative(fx, number(row, "ellipsis_fx"), load),
                      relative(fy, number(row, "ellipsis_fy"), load)});
      }
    }
    CAPTURE(clamped, cambered, ellipsis);
    CHECK(clamped > 0.05);
    CHECK(cambered > 0.003);
    CHECK(ellipsis > 0.5);
  }
}

}  // namespace simon::automotive
