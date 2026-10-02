// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "application/flight/testing.hpp"
#include "base/testing.hpp"
#include "model/aerodynamics.hpp"
#include "model/aircraft_data.hpp"
#include "model/units.hpp"

// The 737's aerodynamics, converted from JSBSim by tools/jsbsim/convert.py,
// against JSBSim's own at states that reference/jsbsim_737_aero.py recorded.
// Both evaluate the same build-up from the same inputs, so they should agree
// to rounding.
namespace simon::model {

namespace {

using namespace flight::testing;

constexpr char REFERENCE[] = "application/flight/reference/jsbsim_737_aero.csv";

// The inputs `row` records, by their JSBSim names.
auto read_inputs(const Row& row) -> AeroInputs {
  AeroInputs inputs;
  for (std::size_t i = 0; i < AERO_VARIABLE_COUNT; ++i) {
    auto variable = static_cast<AeroVariable>(i);
    for (const auto& [name, value] : row) {
      if (aero_variable_named(name) == variable) {
        inputs[variable] = value;
      }
    }
  }
  return inputs;
}

}  // namespace

TEST_CASE("Aerodynamics737") {
  auto aircraft = load_aircraft(AIRCRAFT);
  REQUIRE(aircraft);
  std::vector<Row> rows = load_rows(REFERENCE);
  REQUIRE(rows.size() > 200);

  // They differ by rounding in JSBSim's sums and in the unit conversions:
  // under 10^-13 of the largest load.
  constexpr double TOLERANCE = 1e-12;

  SECTION("ShouldMatchJsbsimGivenWindAxisForces") {
    double worst = 0.0;
    for (const Row& row : rows) {
      AeroSums sums = aircraft->aero(read_inputs(row));
      double scale = std::max(std::abs(row.at("lift")), 1.0);
      worst = worse(worst, sums[0], row.at("drag"), scale);
      worst = worse(worst, sums[1], row.at("side"), scale);
      worst = worse(worst, sums[2], row.at("lift"), scale);
    }
    CAPTURE(worst);
    CHECK(worst < TOLERANCE);
  }

  SECTION("ShouldMatchJsbsimGivenBodyLoadsAboutCenterOfMass") {
    double worst_force = 0.0;
    double worst_moment = 0.0;
    for (const Row& row : rows) {
      AeroSums sums = aircraft->aero(read_inputs(row));

      // The reference point from the center of mass, from the structural
      // frame (x aft, z up) to body axes (x forward, z down).
      QuantityVector apart =
          aircraft->aero_reference.numerical_value_in(meter) -
          QuantityVector{row.at("cg_x"), row.at("cg_y"), row.at("cg_z")};
      Displacement reference =
          QuantityVector{-apart.eigen().x(), apart.eigen().y(),
                         -apart.eigen().z()} *
          meter;
      AeroLoads loads = aero_loads(sums, row.at("alpha") * radian,
                                   row.at("beta") * radian, reference);

      QuantityVector force = loads.force.numerical_value_in(newton);
      QuantityVector moment = loads.moment.numerical_value_in(newton_meter);
      QuantityVector expected_force{row.at("force_x"), row.at("force_y"),
                                    row.at("force_z")};
      QuantityVector expected_moment{row.at("moment_x"), row.at("moment_y"),
                                     row.at("moment_z")};
      double force_scale = std::max(magnitude(expected_force), 1.0);
      double moment_scale =
          std::max(magnitude(expected_moment), force_scale * 1.0);
      worst_force = std::max(worst_force,
                             magnitude(force - expected_force) / force_scale);
      worst_moment = std::max(
          worst_moment, magnitude(moment - expected_moment) / moment_scale);
    }
    CAPTURE(worst_force, worst_moment);
    CHECK(worst_force < TOLERANCE);
    CHECK(worst_moment < TOLERANCE);
  }
}

}  // namespace simon::model
