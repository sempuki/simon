// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/aircraft/rigid_aircraft.hpp"

#include <chrono>

#include "base/testing.hpp"

namespace simon::model {

namespace {

using namespace std::chrono_literals;

// A body climbing, banked and turning at 200 m/s through the air, 6 km up,
// under an arbitrary force.
struct Moving final {
  Earth earth;
  RigidBody body;
  RigidBodyRate rate;
  Acceleration specific_force = meters_per_second_squared(0.0, 0.0, 0.0);
  Acceleration gravity = meters_per_second_squared(0.0, 0.0, 0.0);
};

auto moving(const Earth& earth, const Wind& wind) -> Moving {
  Time time = 0.0 * second;
  RigidBody body = earth.body_at(
      meters(1000.0, -2000.0, 6000.0), 0.4 * radian, 0.1 * radian, 0.7 * radian,
      meters_per_second(195.0, 8.0, 12.0),
      QuantityVector{0.05, 0.12, -0.08} * radian_per_second, time);
  // Moving with the air.
  body.velocity +=
      QuantityVector{
          earth.place(body, time).north_east_down *
          wind.north_east_down.numerical_value_in(meter_per_second).eigen()} *
      meter_per_second;
  MassProperties mass = compute_mass_properties(
      60000.0 * kilogram,
      Matrix3{{1.0e6, 0.0, 2.0e4}, {0.0, 3.0e6, 0.0}, {2.0e4, 0.0, 4.0e6}});
  ForceVector force = QuantityVector{30000.0, -5000.0, -550000.0} * newton;
  Acceleration gravity = earth.gravity(body, time);
  return Moving{
      .earth = earth,
      .body = body,
      .rate = compute_rigid_body_rate(
          body, force, QuantityVector{1.0e4, -2.0e4, 5.0e3} * newton_meter,
          mass, gravity),
      .specific_force = force / mass.mass,
      .gravity = gravity,
  };
}

// The rate of the body's air velocity, by central differences along its
// motion.
auto differenced(const Moving& m, const Wind& wind) -> Vector3 {
  constexpr Duration H = 100us;
  auto at = [&](Duration dt) {
    RigidBody moved = advance(m.body, m.rate, dt);
    Time time = seconds(dt);
    return compute_air_velocity(moved, m.earth, m.earth.place(moved, time),
                                wind)
        .numerical_value_in(meter_per_second)
        .eigen();
  };
  return (at(H) - at(-H)) / (2.0 * seconds(H).numerical_value_in(second));
}

auto computed(const Moving& m, const Wind& wind) -> Vector3 {
  return compute_air_acceleration(m.body, m.earth,
                                  m.earth.place(m.body, 0.0 * second), wind,
                                  m.specific_force, m.gravity)
      .numerical_value_in(meter_per_second_squared)
      .eigen();
}

}  // namespace

TEST_CASE("RigidAircraft") {
  Wind wind{.north_east_down = meters_per_second(8.0, -12.0, 2.0)};

  SECTION("ShouldGiveAirAccelerationGivenWindOverFlatEarth") {
    Moving m = moving(Earth::flat(), wind);
    CHECK((computed(m, wind) - differenced(m, wind)).norm() < 1e-6);
  }

  SECTION("ShouldGiveAirAccelerationGivenWindRoundEarth") {
    // Within the local frame's turning as the body moves over the Earth,
    // which it leaves out: 200 m/s over 6,400 km in a 15 m/s wind.
    Moving m = moving(Earth::round(wgs84::Geodetic{}), wind);
    CHECK((computed(m, wind) - differenced(m, wind)).norm() < 1e-3);
  }

  SECTION("ShouldGiveAirAccelerationGivenStillAir") {
    Moving m = moving(Earth::round(wgs84::Geodetic{}), Wind{});
    CHECK((computed(m, Wind{}) - differenced(m, Wind{})).norm() < 1e-6);
  }

  SECTION("ShouldDifferFromGroundRateByTurningWindGivenWind") {
    // JSBSim takes the rate of angle of attack from the rate of the velocity
    // over the ground, which leaves out w x R^T u.
    Moving m = moving(Earth::flat(), wind);
    Vector3 ground = computed(m, Wind{});
    Vector3 u =
        m.body.attitude.conjugate() *
        (m.earth.place(m.body, 0.0 * second).north_east_down *
         wind.north_east_down.numerical_value_in(meter_per_second).eigen());
    Vector3 turning =
        m.body.rate.numerical_value_in(radian_per_second).eigen().cross(u);
    CHECK(turning.norm() > 0.1);
    CHECK((differenced(m, wind) - (ground + turning)).norm() < 1e-6);
  }
}

}  // namespace simon::model
