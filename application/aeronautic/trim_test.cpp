// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "application/aeronautic/testing.hpp"
#include "base/testing.hpp"
#include "core/units.hpp"
#include "format/aircraft_file.hpp"
#include "model/aircraft/definition.hpp"
#include "model/aircraft/rigid_aircraft.hpp"
#include "model/aircraft/trim.hpp"
#include "model/earth/atmosphere.hpp"

// simon's trim of the 737 and the F-16 against JSBSim's, at the condition
// JSBSim trimmed each for its check cases: 6 km, 200 m/s, 30 degrees north,
// heading northeast, round the turning Earth, straight in space as JSBSim's
// trim is. check_case_test flies simon's level trims.
namespace simon::model {

namespace {

using namespace aeronautic::testing;

struct Compared final {
  aircraft::Trim simon;
  // JSBSim's trim.
  double alpha = 0.0;
  double bank = 0.0;
  double throttle = 0.0;
  double pitch_trim = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
};

auto compare(std::string_view path, std::string_view initial) -> Compared {
  auto aircraft = format::load_aircraft(std::string{path});
  REQUIRE(aircraft);
  std::vector<Row> rows = load_rows(initial);
  REQUIRE(rows.size() == 1);
  const Row& trimmed = rows[0];

  aircraft::Earth earth = aircraft::Earth::round(earth::wgs84::Geodetic{});
  earth::StandardAirTable air;
  RigidBody body = read_body(trimmed);
  aircraft::AirState state = earth.air_state(body, 0.0 * second);
  aircraft::FlightCondition condition{
      .position = state.position,
      .speed = state.speed,
      .heading = state.heading,
      .flight_path_angle = state.flight_path_angle,
      .level = false,  // Straight in space, as JSBSim's trim is.
  };
  for (std::size_t i = 0; i < aircraft->tanks.size(); ++i) {
    condition.tanks.contents[i] =
        trimmed.at("fuel_" + std::to_string(i)) * kilogram;
  }
  auto result = trim(*aircraft, condition, earth, air);
  if (!result) {
    FAIL(result.error().message());
  }

  Vector3 uvw =
      earth.air_velocity(body).numerical_value_in(meter_per_second).eigen();
  Matrix3 attitude = earth.convert_body_to_north_east_down(body, 0.0 * second);
  return Compared{
      .simon = *result,
      .alpha = std::atan2(uvw.z(), uvw.x()),
      .bank = std::atan2(attitude(2, 1), attitude(2, 2)),
      .throttle = trimmed.at("throttle_0"),
      .pitch_trim = trimmed.at("pitch_trim"),
      .aileron = trimmed.at("aileron"),
      .rudder = trimmed.at("rudder"),
  };
}

}  // namespace

TEST_CASE("Trim") {
  for (auto [path, initial] :
       {std::pair{BOEING_737,
                  std::string_view{"application/aeronautic/reference/"
                                   "jsbsim_737_check_initial.csv"}},
        std::pair{F16, std::string_view{"application/aeronautic/reference/"
                                        "jsbsim_f16_check_initial.csv"}}}) {
    Compared c = compare(path, initial);
    CAPTURE(path);
    // simon balances to rounding; JSBSim's trim stops at a tolerance, which
    // leaves its controls a few parts in 10^5 from simon's.
    CHECK(c.simon.residual < 1e-12);
    CHECK(std::abs(radians(c.simon.alpha) - c.alpha) < 1e-6);
    CHECK(std::abs(radians(c.simon.bank) - c.bank) < 2e-5);
    CHECK(std::abs(c.simon.throttle - c.throttle) < 1e-5);
    CHECK(std::abs(c.simon.pitch_trim - c.pitch_trim) < 1e-5);
    CHECK(std::abs(c.simon.aileron - c.aileron) < 1e-4);
    CHECK(std::abs(c.simon.rudder - c.rudder) < 1e-4);
  }
}

}  // namespace simon::model
