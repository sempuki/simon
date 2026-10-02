// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "application/flight/testing.hpp"
#include "base/testing.hpp"
#include "model/aircraft_data.hpp"
#include "model/turbine.hpp"
#include "model/units.hpp"

// The 737's turbines, converted from JSBSim, against JSBSim's own, frame by
// frame through throttle steps and ramps that reference/jsbsim_737_turbine.py
// recorded: given the same air and throttle, both should turn their spools,
// make thrust and burn fuel alike.
namespace simon::model {

namespace {

using namespace flight::testing;

constexpr char REFERENCE[] =
    "application/flight/reference/jsbsim_737_turbine.csv";
constexpr double DT = 1.0 / 60.0;  // The reference's frame, seconds.

auto read_air(const Row& row) -> EngineAir {
  return EngineAir{
      .mach = row.at("mach"),
      .density_altitude = row.at("density_altitude") * meter,
      .density_ratio = row.at("density_ratio"),
      .temperature = units::delta<kelvin>(row.at("temperature")),
  };
}

auto column(const char* name, int engine) -> std::string {
  return std::string{name} + "_" + std::to_string(engine);
}

}  // namespace

TEST_CASE("Turbine737") {
  auto aircraft = load_aircraft(BOEING_737);
  REQUIRE(aircraft);
  REQUIRE(aircraft->engines.size() == 2);
  std::vector<Row> rows = load_rows(REFERENCE);
  REQUIRE(rows.size() > 1000);

  SECTION("ShouldMatchJsbsimGivenRecordedAirAndThrottle") {
    for (int engine = 0; engine < 2; ++engine) {
      const TurbineData& turbine = aircraft->engines[engine];
      // From where JSBSim's first frame left the engine.
      TurbineState state{
          .n1 = rows[0].at(column("n1", engine)),
          .n2 = rows[0].at(column("n2", engine)),
          .fuel_flow = rows[0].at(column("fuel_flow", engine)),
      };
      double worst_speed = 0.0;   // Percent.
      double worst_thrust = 0.0;  // Relative to military thrust.
      double worst_flow = 0.0;    // kg/s.
      for (std::size_t i = 1; i < rows.size(); ++i) {
        const Row& row = rows[i];
        state = run_turbine(turbine, state, row.at(column("throttle", engine)),
                            read_air(row), DT * second);
        worst_speed = std::max(
            {worst_speed, std::abs(state.n1 - row.at(column("n1", engine))),
             std::abs(state.n2 - row.at(column("n2", engine)))});
        worst_thrust =
            std::max(worst_thrust,
                     std::abs(state.thrust.numerical_value_in(newton) -
                              row.at(column("thrust", engine))) /
                         turbine.military_thrust.numerical_value_in(newton));
        worst_flow = std::max(
            worst_flow,
            std::abs(state.fuel_flow - row.at(column("fuel_flow", engine))));
      }
      CAPTURE(engine, worst_speed, worst_thrust, worst_flow);
      CHECK(worst_speed < 1e-9);
      CHECK(worst_thrust < 1e-9);
      CHECK(worst_flow < 1e-9);
    }
  }

  SECTION("ShouldMatchJsbsimGivenSettledEngine") {
    // By the end, the second engine has held 0.9 for 13 s.
    const Row& last = rows.back();
    TurbineState steady =
        steady_turbine(aircraft->engines[1], 0.9, read_air(last));
    CHECK(std::abs(steady.n2 - last.at("n2_1")) < 1e-9);
    CHECK(std::abs(steady.thrust.numerical_value_in(newton) -
                   last.at("thrust_1")) /
              last.at("thrust_1") <
          1e-9);
  }
}

}  // namespace simon::model
