// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
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
#include "model/earth.hpp"
#include "model/rigid_aircraft.hpp"
#include "model/rigid_body.hpp"

// simon's equations of motion against JSBSim's, at states that
// reference/jsbsim_737_rigid_body.py recorded from a maneuvering 737: given
// the same state, forces, moments and mass properties, both should find the
// same rates of change, round a rotating WGS84 Earth.
namespace simon::model {

namespace {

using namespace aeronautic::testing;

constexpr std::string_view REFERENCE =
    "application/aeronautic/reference/jsbsim_737_rigid_body.csv";

}  // namespace

TEST_CASE("RigidBody737") {
  std::vector<Row> rows = load_rows(REFERENCE);
  REQUIRE(rows.size() > 400);

  SECTION("ShouldMatchJsbsimRatesGivenRecordedStates") {
    double worst_acceleration = 0.0;
    double worst_angular = 0.0;
    for (const Row& row : rows) {
      RigidBody body = read_body(row);
      // JSBSim reports the tensor's xz element as it is, but its xy and yz
      // elements negated (FGMassBalance's GetIxz, GetIxy and GetIyz).
      Matrix3 inertia;
      inertia << row.at("ixx"), -row.at("ixy"), row.at("ixz"),  //
          -row.at("ixy"), row.at("iyy"), -row.at("iyz"),        //
          row.at("ixz"), -row.at("iyz"), row.at("izz");
      MassProperties mass =
          compute_mass_properties(row.at("mass") * kilogram, inertia);

      // Gravitation, found where the body is on the turning Earth.
      Matrix3 to_fixed =
          wgs84::convert_inertial_to_fixed(row.at("earth_angle") * radian);
      Position fixed =
          QuantityVector{to_fixed *
                         body.position.numerical_value_in(meter).eigen()} *
          meter;
      QuantityVector gravity{to_fixed.transpose() *
                             wgs84::compute_gravitation(fixed)
                                 .numerical_value_in(meter_per_second_squared)
                                 .eigen()};

      RigidBodyRate rate = compute_rigid_body_rate(
          body, read_vector(row, "force_x", "force_y", "force_z") * newton,
          read_vector(row, "moment_x", "moment_y", "moment_z") * newton_meter,
          mass, gravity * meter_per_second_squared);

      QuantityVector acceleration =
          rate.acceleration.numerical_value_in(meter_per_second_squared);
      QuantityVector expected = read_vector(row, "ax", "ay", "az");
      worst_acceleration = std::max(worst_acceleration,
                                    magnitude(acceleration - expected) / 9.8);

      QuantityVector angular = rate.angular_acceleration.numerical_value_in(
          radian_per_second_squared);
      QuantityVector expected_angular =
          read_vector(row, "pdot", "qdot", "rdot");
      worst_angular = std::max(worst_angular,
                               magnitude(angular - expected_angular) /
                                   std::max(magnitude(expected_angular), 1e-3));
    }
    CAPTURE(worst_acceleration, worst_angular);
    CHECK(worst_acceleration < 1e-12);
    CHECK(worst_angular < 1e-12);
  }

  auto aircraft = format::load_aircraft(std::string{BOEING_737});
  REQUIRE(aircraft);
  Earth earth = Earth::round(wgs84::Geodetic{});
  StandardAirTable air;

  SECTION("ShouldMatchJsbsimMassBalanceGivenRecordedFuel") {
    double worst_inertia = 0.0;
    double worst_center = 0.0;
    double worst_mass = 0.0;
    for (const Row& row : rows) {
      std::array<Mass, 3> fuel{row.at("fuel_0") * kilogram,
                               row.at("fuel_1") * kilogram,
                               row.at("fuel_2") * kilogram};
      MassBalance balance = compute_mass_balance(*aircraft, fuel);
      worst_mass = worse(worst_mass,
                         balance.properties.mass.numerical_value_in(kilogram),
                         row.at("mass"), row.at("mass"));
      QuantityVector center = balance.center_of_mass.numerical_value_in(meter);
      worst_center = std::max(
          worst_center,
          magnitude(center - read_vector(row, "cg_x", "cg_y", "cg_z")));
      const Matrix3& j = balance.properties.inertia;
      double scale = row.at("izz");
      worst_inertia = worse(worst_inertia, j(0, 0), row.at("ixx"), scale);
      worst_inertia = worse(worst_inertia, j(1, 1), row.at("iyy"), scale);
      worst_inertia = worse(worst_inertia, j(2, 2), row.at("izz"), scale);
      worst_inertia = worse(worst_inertia, j(0, 1), -row.at("ixy"), scale);
      worst_inertia = worse(worst_inertia, j(0, 2), row.at("ixz"), scale);
      worst_inertia = worse(worst_inertia, j(1, 2), -row.at("iyz"), scale);
    }
    // JSBSim turns pounds into slugs by a rounded 1/32.174049, which is
    // 1.4e-8 off the definition simon uses.
    CAPTURE(worst_mass, worst_center, worst_inertia);
    CHECK(worst_mass < 2e-8);
    CHECK(worst_center < 1e-9);  // Meters.
    CHECK(worst_inertia < 2e-8);
  }

  SECTION("ShouldMatchJsbsimAirDataGivenRecordedStates") {
    double worst_angle = 0.0;     // Radians.
    double worst_speed = 0.0;     // Relative.
    double worst_rate = 0.0;      // Radians per second.
    double worst_altitude = 0.0;  // Meters.
    double worst_density = 0.0;   // Relative: dynamic pressure and Mach.
    double worst_ground = 0.0;    // Height over span.
    for (const Row& row : rows) {
      RigidBody body = read_body(row);
      Time time = row.at("time") * second;
      std::array<Mass, 3> fuel{row.at("fuel_0") * kilogram,
                               row.at("fuel_1") * kilogram,
                               row.at("fuel_2") * kilogram};
      MassBalance balance = compute_mass_balance(*aircraft, fuel);
      Displacement reference =
          compute_body_offset(aircraft->aero_reference, balance.center_of_mass);
      AeroInputs inputs = compute_aero_inputs(body, FlightSignals{}, *aircraft,
                                              reference, earth, air, time);

      using enum AeroVariable;
      worst_angle = worse(worst_angle, inputs[ALPHA], row.at("alpha"), 1.0);
      worst_angle = worse(worst_angle, inputs[BETA], row.at("beta"), 1.0);
      double speed = magnitude(
          earth.air_velocity(body).numerical_value_in(meter_per_second));
      worst_speed =
          worse(worst_speed, speed, row.at("airspeed"), row.at("airspeed"));
      worst_rate = worse(worst_rate, inputs[ROLL_RATE], row.at("p_air"), 1.0);
      worst_rate = worse(worst_rate, inputs[PITCH_RATE], row.at("q_air"), 1.0);
      worst_rate = worse(worst_rate, inputs[YAW_RATE], row.at("r_air"), 1.0);
      worst_altitude = worse(
          worst_altitude, earth.altitude(body, time).numerical_value_in(meter),
          row.at("altitude"), 1.0);
      worst_density =
          worse(worst_density, inputs[DYNAMIC_PRESSURE],
                row.at("dynamic_pressure"), row.at("dynamic_pressure"));
      worst_density =
          worse(worst_density, inputs[MACH], row.at("mach"), row.at("mach"));
      worst_ground =
          worse(worst_ground, inputs[HEIGHT_OVER_SPAN],
                row.at("height_over_span"), row.at("height_over_span"));
    }
    CAPTURE(worst_angle, worst_speed, worst_rate, worst_altitude, worst_density,
            worst_ground);
    CHECK(worst_angle < 1e-12);
    CHECK(worst_speed < 1e-12);
    CHECK(worst_rate < 1e-12);
    // JSBSim's geodetic altitude is an approximation, 2.5 cm off at 60
    // degrees north and 6 km up; simon's is exact (see earth_test).
    CHECK(worst_altitude < 0.03);
    // simon's atmosphere keeps the 1976 standard's constants, and JSBSim
    // rounds some in English units.
    CHECK(worst_density < 2e-5);
    CHECK(worst_ground < 1e-8);
  }
}

}  // namespace simon::model
