// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "application/flight/testing.hpp"
#include "base/testing.hpp"
#include "model/aerodynamics.hpp"
#include "model/aircraft_data.hpp"
#include "model/flight_control.hpp"
#include "model/units.hpp"

// The 737's and the F-16's aerodynamics, converted from JSBSim by
// tools/jsbsim/convert.py, against JSBSim's own at states that
// reference/jsbsim_737_aero.py and jsbsim_f16_aero.py recorded.
// Both evaluate the same build-up from the same inputs, so they should agree
// to rounding.
namespace simon::model {

namespace {

using namespace flight::testing;

// The inputs `row` records, by simon's names: the variables, and the
// aircraft's flight control signals.
auto read_inputs(const Row& row, const AircraftData& aircraft) -> AeroInputs {
  AeroInputs inputs;
  FlightSignals signals;
  for (const auto& [name, value] : row) {
    if (std::optional<AeroVariable> variable = find_aero_variable(name)) {
      inputs[*variable] = value;
    } else if (std::optional<std::size_t> signal =
                   find_signal(aircraft.flight_controls, name)) {
      signals.values[*signal] = value;
    }
  }
  inputs.read(aircraft.aero.signals, signals.values);
  return inputs;
}

// Checks the aircraft at `path` against JSBSim's recording at `reference`.
auto check_against_jsbsim(std::string_view path, std::string_view reference)
    -> void {
  auto aircraft = load_aircraft(std::string{path});
  if (!aircraft) {
    FAIL(aircraft.error().message());
  }
  std::vector<Row> rows = load_rows(reference);
  REQUIRE(rows.size() > 200);

  // They differ by rounding in JSBSim's sums and in the unit conversions:
  // under 10^-13 of the largest load.
  constexpr double TOLERANCE = 1e-12;

  SECTION("ShouldMatchJsbsimGivenWindAxisForces") {
    double worst = 0.0;
    for (const Row& row : rows) {
      AeroSums sums = aircraft->aero(read_inputs(row, *aircraft));
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
      AeroSums sums = aircraft->aero(read_inputs(row, *aircraft));

      // The reference point from the center of mass, from the structural
      // frame (x aft, z up) to body axes (x forward, z down).
      QuantityVector apart =
          aircraft->aero_reference.numerical_value_in(meter) -
          QuantityVector{row.at("cg_x"), row.at("cg_y"), row.at("cg_z")};
      Displacement reference =
          QuantityVector{-apart.eigen().x(), apart.eigen().y(),
                         -apart.eigen().z()} *
          meter;
      AeroLoads loads = compute_aero_loads(sums, row.at("alpha") * radian,
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

}  // namespace

TEST_CASE("Aerodynamics737") {
  check_against_jsbsim(BOEING_737,
                       "application/flight/reference/jsbsim_737_aero.csv");
}

TEST_CASE("AerodynamicsF16") {
  check_against_jsbsim(F16, "application/flight/reference/jsbsim_f16_aero.csv");
}

}  // namespace simon::model
