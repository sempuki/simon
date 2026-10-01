// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/guidance.hpp"

#include "base/testing.hpp"

namespace simon::model {

namespace {
auto value_of(const Acceleration& acceleration) -> Vector3d {
  return acceleration.numerical_value_in(meter_per_second_squared);
}
}  // namespace

TEST_CASE("ProportionalNavigation") {
  Kinematics self;
  Kinematics target;

  SECTION("ShouldCommandNothingGivenCollisionCourse") {
    // Target straight ahead, closing head-on: the line of sight does not turn.
    self.velocity = meters_per_second(100.0, 0.0, 0.0);
    target.position = meters(1000.0, 0.0, 0.0);
    target.velocity = meters_per_second(-50.0, 0.0, 0.0);

    CHECK(value_of(proportional_navigation(self, target, 4.0))
              .is_approximately(Vector3d{0.0, 0.0, 0.0}));
  }

  SECTION("ShouldTurnTowardTargetGivenCrossingTarget") {
    // Target ahead, moving +y: the command turns self toward +y.
    self.velocity = meters_per_second(100.0, 0.0, 0.0);
    target.position = meters(1000.0, 0.0, 0.0);
    target.velocity = meters_per_second(0.0, 50.0, 0.0);

    Vector3d command = value_of(proportional_navigation(self, target, 3.0));

    // omega = (r x v)/|r|^2 = 0.05 rad/s about z; Vc = 100 m/s;
    // a = 3 * 100 * 0.05 = 15 m/s^2 along +y.
    CHECK(command.is_approximately(Vector3d{0.0, 15.0, 0.0}, 1e-9));
  }

  SECTION("ShouldCommandNothingGivenTargetAtSelf") {
    CHECK(value_of(proportional_navigation(self, target, 4.0)) ==
          Vector3d{0.0, 0.0, 0.0});
  }
}

TEST_CASE("Limit") {
  SECTION("ShouldScaleToLimitGivenLargerCommand") {
    Acceleration limited = limit(meters_per_second_squared(30.0, 40.0, 0.0),
                                 10.0 * meter_per_second_squared);
    CHECK(value_of(limited).is_approximately(Vector3d{6.0, 8.0, 0.0}));
  }

  SECTION("ShouldPassThroughGivenSmallerCommand") {
    Acceleration small = meters_per_second_squared(3.0, 4.0, 0.0);
    CHECK(limit(small, 10.0 * meter_per_second_squared) == small);
  }

  SECTION("ShouldPassThroughGivenCommandExactlyAtLimit") {
    Acceleration exact = meters_per_second_squared(6.0, 8.0, 0.0);
    CHECK(limit(exact, 10.0 * meter_per_second_squared) == exact);
  }

  SECTION("ShouldPassThroughGivenZeroCommandAndZeroLimit") {
    Acceleration zero = meters_per_second_squared(0.0, 0.0, 0.0);
    CHECK(limit(zero, 0.0 * meter_per_second_squared) == zero);
  }
}

TEST_CASE("SteerToward") {
  SECTION("ShouldAccelerateTowardGoalGivenRest") {
    Kinematics self;
    Acceleration command =
        steer_toward(self, meters(100.0, 0.0, 0.0), 20.0 * meter_per_second,
                     0.5 * per_second);
    CHECK(value_of(command).is_approximately(Vector3d{10.0, 0.0, 0.0}));
  }
}

TEST_CASE("HoldSpeed") {
  SECTION("ShouldSpeedUpAlongVelocityGivenSlowerThanTarget") {
    Kinematics self;
    self.velocity = meters_per_second(0.0, 50.0, 0.0);
    Acceleration command =
        hold_speed(self, 100.0 * meter_per_second, 2.0 * per_second);
    CHECK(value_of(command).is_approximately(Vector3d{0.0, 100.0, 0.0}));
  }
}

}  // namespace simon::model
