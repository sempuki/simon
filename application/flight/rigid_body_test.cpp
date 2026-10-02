// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "base/testing.hpp"
#include "model/earth.hpp"
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

}  // namespace

TEST_CASE("RigidBody737") {
  std::vector<Row> rows = load_reference();
  REQUIRE(rows.size() > 400);

  SECTION("ShouldMatchJsbsimRatesGivenRecordedStates") {
    double worst_acceleration = 0.0;
    double worst_angular = 0.0;
    for (const Row& row : rows) {
      RigidBody body{
          .position = vector_of(row, "x", "y", "z") * meter,
          .velocity = vector_of(row, "vx", "vy", "vz") * meter_per_second,
          .attitude = Quaternion{row.at("qw"), row.at("qx"), row.at("qy"),
                                 row.at("qz")},
          .rate = vector_of(row, "p", "q", "r") * radian_per_second,
      };
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
}

}  // namespace simon::model
