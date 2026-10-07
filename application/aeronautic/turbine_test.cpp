// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "application/aeronautic/testing.hpp"
#include "base/testing.hpp"
#include "core/units.hpp"
#include "format/aircraft_file.hpp"
#include "model/aircraft_data.hpp"
#include "model/turbine.hpp"

// The 737's turbines and the F-16's, converted from JSBSim, against
// JSBSim's own, frame by frame through throttle steps, ramps and reheat that
// reference/jsbsim_737_turbine.py and jsbsim_f16_turbine.py recorded: given the
// same air and throttle, both should turn their spools, make thrust and burn
// fuel alike.
namespace simon::model {

namespace {

using namespace aeronautic::testing;

constexpr std::string_view REFERENCE =
    "application/aeronautic/reference/jsbsim_737_turbine.csv";
constexpr double DT = 1.0 / 60.0;  // The reference's frame, seconds.

auto read_air(const Row& row) -> EngineAir {
  return EngineAir{
      .mach = row.at("mach"),
      .density_altitude = row.at("density_altitude") * meter,
      .density_ratio = row.at("density_ratio"),
      .temperature = units::delta<kelvin>(row.at("temperature")),
  };
}

auto column(std::string_view name, int engine) -> std::string {
  return std::string{name} + "_" + std::to_string(engine);
}

// Checks each engine of the aircraft at `path` against JSBSim's recording
// at `reference`, made every `dt` seconds.
auto check_against_jsbsim(std::string_view path, std::string_view reference,
                          double dt) -> void {
  auto aircraft = format::load_aircraft(std::string{path});
  REQUIRE(aircraft);
  std::vector<Row> rows = load_rows(reference);
  REQUIRE(rows.size() > 1000);

  for (std::size_t engine = 0; engine < aircraft->engines.size(); ++engine) {
    const TurbineData& turbine = aircraft->engines[engine];
    int column_engine = static_cast<int>(engine);
    // From where JSBSim's first frame left the engine.
    TurbineState state{
        .n1 = rows[0].at(column("n1", column_engine)),
        .n2 = rows[0].at(column("n2", column_engine)),
        .fuel_flow = rows[0].at(column("fuel_flow", column_engine)),
        .reheat = rows[0].at(column("throttle", column_engine)) > 1.0,
    };
    double worst_speed = 0.0;   // Percent.
    double worst_thrust = 0.0;  // Relative to military thrust.
    double worst_flow = 0.0;    // kg/s.
    for (std::size_t i = 1; i < rows.size(); ++i) {
      const Row& row = rows[i];
      state =
          run_turbine(turbine, state, row.at(column("throttle", column_engine)),
                      read_air(row), dt * second);
      worst_speed =
          std::max({worst_speed,
                    std::abs(state.n1 - row.at(column("n1", column_engine))),
                    std::abs(state.n2 - row.at(column("n2", column_engine)))});
      worst_thrust = std::max(
          worst_thrust, std::abs(state.thrust.numerical_value_in(newton) -
                                 row.at(column("thrust", column_engine))) /
                            turbine.military_thrust.numerical_value_in(newton));
      worst_flow = std::max(
          worst_flow, std::abs(state.fuel_flow -
                               row.at(column("fuel_flow", column_engine))));
    }
    CAPTURE(engine, worst_speed, worst_thrust, worst_flow);
    CHECK(worst_speed < 1e-9);
    CHECK(worst_thrust < 1e-9);
    CHECK(worst_flow < 1e-9);
  }
}

}  // namespace

TEST_CASE("Turbine737") {
  SECTION("ShouldMatchJsbsimGivenRecordedAirAndThrottle") {
    check_against_jsbsim(BOEING_737, REFERENCE, DT);
  }

  SECTION("ShouldMatchJsbsimGivenSettledEngine") {
    auto aircraft = format::load_aircraft(std::string{BOEING_737});
    REQUIRE(aircraft);
    // By the end, the second engine has held 0.9 for 13 s.
    std::vector<Row> rows = load_rows(REFERENCE);
    const Row& last = rows.back();
    TurbineState steady =
        compute_steady_turbine(aircraft->engines[1], 0.9, read_air(last));
    CHECK(std::abs(steady.n2 - last.at("n2_1")) < 1e-9);
    CHECK(std::abs(steady.thrust.numerical_value_in(newton) -
                   last.at("thrust_1")) /
              last.at("thrust_1") <
          1e-9);
  }
}

TEST_CASE("TurbineF16") {
  SECTION("ShouldMatchJsbsimGivenRecordedAirAndThrottleThroughReheat") {
    check_against_jsbsim(
        F16, "application/aeronautic/reference/jsbsim_f16_turbine.csv",
        1.0 / 120.0);
  }
}

}  // namespace simon::model
