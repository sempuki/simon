// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "model/aircraft_data.hpp"
#include "model/earth.hpp"
#include "model/rigid_aircraft.hpp"
#include "model/rigid_body.hpp"
#include "model/units.hpp"

// simon's equations of motion against JSBSim's, at states that
// reference/jsbsim_737_rigid_body.py recorded from a maneuvering 737: given
// the same state, forces, moments and mass properties, both should find the
// same rates of change, round a rotating WGS84 Earth.
namespace simon::model {

namespace {

constexpr char REFERENCE[] =
    "application/flight/reference/jsbsim_737_rigid_body.csv";
constexpr char AIRCRAFT[] = "application/flight/aircraft/737.aircraft";

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

auto vector_of(const Row& row, const char* x, const char* y, const char* z)
    -> Vector3d {
  return Vector3d{row.at(x), row.at(y), row.at(z)};
}

auto body_of(const Row& row) -> RigidBody {
  return RigidBody{
      .position = vector_of(row, "x", "y", "z") * meter,
      .velocity = vector_of(row, "vx", "vy", "vz") * meter_per_second,
      .attitude =
          Quaternion{row.at("qw"), row.at("qx"), row.at("qy"), row.at("qz")},
      .rate = vector_of(row, "p", "q", "r") * radian_per_second,
  };
}

// The largest of `worst` and how far `actual` is from `expected`, relative
// to `scale`.
auto worse(double worst, double actual, double expected, double scale)
    -> double {
  return std::max(worst, std::abs(actual - expected) / scale);
}

}  // namespace

TEST_CASE("RigidBody737") {
  std::vector<Row> rows = load_reference();
  REQUIRE(rows.size() > 400);

  SECTION("ShouldMatchJsbsimRatesGivenRecordedStates") {
    double worst_acceleration = 0.0;
    double worst_angular = 0.0;
    for (const Row& row : rows) {
      RigidBody body = body_of(row);
      // JSBSim reports the tensor's xz element as it is, but its xy and yz
      // elements negated (FGMassBalance's GetIxz, GetIxy and GetIyz).
      Eigen::Matrix3d inertia;
      inertia << row.at("ixx"), -row.at("ixy"), row.at("ixz"),  //
          -row.at("ixy"), row.at("iyy"), -row.at("iyz"),        //
          row.at("ixz"), -row.at("iyz"), row.at("izz");
      MassProperties mass =
          MassProperties::of(row.at("mass") * kilogram, inertia);

      // Gravitation, found where the body is on the turning Earth.
      Eigen::Matrix3d to_fixed =
          wgs84::inertial_to_fixed(row.at("earth_angle") * radian);
      Position fixed =
          Vector3d{to_fixed * body.position.numerical_value_in(meter).eigen()} *
          meter;
      Vector3d gravity{to_fixed.transpose() *
                       wgs84::gravitation(fixed)
                           .numerical_value_in(meter_per_second_squared)
                           .eigen()};

      RigidBodyRate rate = rigid_body_rate(
          body, vector_of(row, "force_x", "force_y", "force_z") * newton,
          vector_of(row, "moment_x", "moment_y", "moment_z") * newton_meter,
          mass, gravity * meter_per_second_squared);

      Vector3d acceleration =
          rate.acceleration.numerical_value_in(meter_per_second_squared);
      Vector3d expected = vector_of(row, "ax", "ay", "az");
      worst_acceleration = std::max(worst_acceleration,
                                    magnitude(acceleration - expected) / 9.8);

      Vector3d angular = rate.angular_acceleration.numerical_value_in(
          radian_per_second_squared);
      Vector3d expected_angular = vector_of(row, "pdot", "qdot", "rdot");
      worst_angular = std::max(worst_angular,
                               magnitude(angular - expected_angular) /
                                   std::max(magnitude(expected_angular), 1e-3));
    }
    CAPTURE(worst_acceleration, worst_angular);
    CHECK(worst_acceleration < 1e-12);
    CHECK(worst_angular < 1e-12);
  }

  auto aircraft = load_aircraft(AIRCRAFT);
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
      MassBalance balance = mass_balance_of(*aircraft, fuel);
      worst_mass = worse(worst_mass,
                         balance.properties.mass.numerical_value_in(kilogram),
                         row.at("mass"), row.at("mass"));
      Vector3d center = balance.center_of_mass.numerical_value_in(meter);
      worst_center =
          std::max(worst_center,
                   magnitude(center - vector_of(row, "cg_x", "cg_y", "cg_z")));
      const Eigen::Matrix3d& j = balance.properties.inertia;
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
      RigidBody body = body_of(row);
      Time time = row.at("time") * second;
      std::array<Mass, 3> fuel{row.at("fuel_0") * kilogram,
                               row.at("fuel_1") * kilogram,
                               row.at("fuel_2") * kilogram};
      MassBalance balance = mass_balance_of(*aircraft, fuel);
      Displacement reference =
          body_offset(aircraft->aero_reference, balance.center_of_mass);
      AeroInputs inputs = aero_inputs_of(body, ControlSurfaces{}, *aircraft,
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
