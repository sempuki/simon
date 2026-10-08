// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/guidance.hpp"

#include "base/testing.hpp"

namespace simon::model {

namespace {
auto convert_to_si(const Acceleration& acceleration) -> QuantityVector {
  return acceleration.numerical_value_in(meter_per_second_squared);
}
}  // namespace

TEST_CASE("ProportionalNavigation") {
  Kinematics self;
  Kinematics target;

  SECTION("ShouldCommandNothingGivenCollisionCourse") {
    // Preconditions.
    // Target straight ahead, closing head-on: the line of sight does not turn.
    self.velocity = meters_per_second(100.0, 0.0, 0.0);
    target.position = meters(1000.0, 0.0, 0.0);
    target.velocity = meters_per_second(-50.0, 0.0, 0.0);

    // Under Test.
    QuantityVector command =
        convert_to_si(compute_proportional_navigation(self, target, 4.0));

    // Postconditions.
    CHECK(command.is_approximately(QuantityVector{0.0, 0.0, 0.0}));
  }

  SECTION("ShouldTurnTowardTargetGivenCrossingTarget") {
    // Preconditions.
    // Target ahead, moving +y: the command turns self toward +y.
    self.velocity = meters_per_second(100.0, 0.0, 0.0);
    target.position = meters(1000.0, 0.0, 0.0);
    target.velocity = meters_per_second(0.0, 50.0, 0.0);

    // Under Test.
    QuantityVector command =
        convert_to_si(compute_proportional_navigation(self, target, 3.0));

    // Postconditions.
    // omega = (r x v)/|r|^2 = 0.05 rad/s about z; Vc = 100 m/s;
    // a = 3 * 100 * 0.05 = 15 m/s^2 along +y.
    CHECK(command.is_approximately(QuantityVector{0.0, 15.0, 0.0}, 1e-9));
  }

  SECTION("ShouldCommandNothingGivenTargetAtSelf") {
    // Postconditions.
    CHECK(convert_to_si(compute_proportional_navigation(self, target, 4.0)) ==
          QuantityVector{0.0, 0.0, 0.0});
  }
}

TEST_CASE("Limit") {
  SECTION("ShouldScaleToLimitGivenLargerCommand") {
    // Under Test.
    Acceleration limited = limit(meters_per_second_squared(30.0, 40.0, 0.0),
                                 10.0 * meter_per_second_squared);

    // Postconditions.
    CHECK(
        convert_to_si(limited).is_approximately(QuantityVector{6.0, 8.0, 0.0}));
  }

  SECTION("ShouldPassThroughGivenSmallerCommand") {
    // Preconditions.
    Acceleration small = meters_per_second_squared(3.0, 4.0, 0.0);

    // Under Test.
    Acceleration limited = limit(small, 10.0 * meter_per_second_squared);

    // Postconditions.
    CHECK(limited == small);
  }

  SECTION("ShouldPassThroughGivenCommandExactlyAtLimit") {
    // Preconditions.
    Acceleration exact = meters_per_second_squared(6.0, 8.0, 0.0);

    // Under Test.
    Acceleration limited = limit(exact, 10.0 * meter_per_second_squared);

    // Postconditions.
    CHECK(limited == exact);
  }

  SECTION("ShouldPassThroughGivenZeroCommandAndZeroLimit") {
    // Preconditions.
    Acceleration zero = meters_per_second_squared(0.0, 0.0, 0.0);

    // Under Test.
    Acceleration limited = limit(zero, 0.0 * meter_per_second_squared);

    // Postconditions.
    CHECK(limited == zero);
  }
}

TEST_CASE("SteerToward") {
  SECTION("ShouldAccelerateTowardGoalGivenRest") {
    // Preconditions.
    Kinematics self;

    // Under Test.
    Acceleration command =
        steer_toward(self, meters(100.0, 0.0, 0.0), 20.0 * meter_per_second,
                     0.5 * per_second);

    // Postconditions.
    CHECK(convert_to_si(command).is_approximately(
        QuantityVector{10.0, 0.0, 0.0}));
  }
}

TEST_CASE("HoldSpeed") {
  SECTION("ShouldSpeedUpAlongVelocityGivenSlowerThanTarget") {
    // Preconditions.
    Kinematics self;
    self.velocity = meters_per_second(0.0, 50.0, 0.0);

    // Under Test.
    Acceleration command =
        hold_speed(self, 100.0 * meter_per_second, 2.0 * per_second);

    // Postconditions.
    CHECK(convert_to_si(command).is_approximately(
        QuantityVector{0.0, 100.0, 0.0}));
  }
}

}  // namespace simon::model
