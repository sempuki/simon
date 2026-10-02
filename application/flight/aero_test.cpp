// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

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

constexpr char AIRCRAFT[] = "application/flight/aircraft/737.aircraft";
constexpr char REFERENCE[] = "application/flight/reference/jsbsim_737_aero.csv";

// The reference as columns by name.
using Row = std::map<std::string, double, std::less<>>;

auto load_reference() -> std::vector<Row> {
  std::ifstream file{REFERENCE};
  REQUIRE(file);
  std::string line;
  std::getline(file, line);
  std::vector<std::string> names;
  for (std::size_t at = 0; at <= line.size();) {
    std::size_t comma = std::min(line.find(',', at), line.size());
    names.emplace_back(line.substr(at, comma - at));
    at = comma + 1;
  }

  std::vector<Row> rows;
  while (std::getline(file, line)) {
    Row row;
    const char* next = line.data();
    const char* end = line.data() + line.size();
    for (const std::string& name : names) {
      double value = 0.0;
      auto [stop, error] = std::from_chars(next, end, value);
      REQUIRE(error == std::errc{});
      row[name] = value;
      next = stop + 1;
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

auto inputs_of(const Row& row) -> AeroInputs {
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

// How far `actual` is from `expected`, relative to `scale`.
auto relative(double actual, double expected, double scale) -> double {
  return std::abs(actual - expected) / scale;
}

}  // namespace

TEST_CASE("Aerodynamics737") {
  auto aircraft = load_aircraft(AIRCRAFT);
  REQUIRE(aircraft);
  std::vector<Row> rows = load_reference();
  REQUIRE(rows.size() > 200);

  // They differ by rounding in JSBSim's sums and in the unit conversions:
  // under 10^-13 of the largest load.
  constexpr double TOLERANCE = 1e-12;

  SECTION("ShouldMatchJsbsimGivenWindAxisForces") {
    double worst = 0.0;
    for (const Row& row : rows) {
      AeroSums sums = aircraft->aero(inputs_of(row));
      double scale = std::max(std::abs(row.at("lift")), 1.0);
      worst = std::max({worst, relative(sums[0], row.at("drag"), scale),
                        relative(sums[1], row.at("side"), scale),
                        relative(sums[2], row.at("lift"), scale)});
    }
    CAPTURE(worst);
    CHECK(worst < TOLERANCE);
  }

  SECTION("ShouldMatchJsbsimGivenBodyLoadsAboutCenterOfMass") {
    double worst_force = 0.0;
    double worst_moment = 0.0;
    for (const Row& row : rows) {
      AeroSums sums = aircraft->aero(inputs_of(row));

      // The reference point from the center of mass, from the structural
      // frame (x aft, z up) to body axes (x forward, z down).
      Vector3d apart = aircraft->aero_reference.numerical_value_in(meter) -
                       Vector3d{row.at("cg_x"), row.at("cg_y"), row.at("cg_z")};
      Displacement reference =
          Vector3d{-apart.eigen().x(), apart.eigen().y(), -apart.eigen().z()} *
          meter;
      AeroLoads loads = aero_loads(sums, row.at("alpha") * radian,
                                   row.at("beta") * radian, reference);

      Vector3d force = loads.force.numerical_value_in(newton);
      Vector3d moment = loads.moment.numerical_value_in(newton_meter);
      Vector3d expected_force{row.at("force_x"), row.at("force_y"),
                              row.at("force_z")};
      Vector3d expected_moment{row.at("moment_x"), row.at("moment_y"),
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
