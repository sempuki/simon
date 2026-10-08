// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/kinematics.hpp"
#include "core/argument.hpp"

#include "base/testing.hpp"

namespace simon::model {

namespace {
auto near(const Displacement& a, const Displacement& b) -> bool {
  return a.numerical_value_in(meter).is_approximately(
      b.numerical_value_in(meter));
}
auto near(const Velocity& a, const Velocity& b) -> bool {
  return a.numerical_value_in(meter_per_second)
      .is_approximately(b.numerical_value_in(meter_per_second));
}
}  // namespace

TEST_CASE("IntegrateMidpoint") {
  Kinematics kinematics;

  SECTION("ShouldMoveByVelocityTimesStepGivenZeroAcceleration") {
    // Preconditions.
    kinematics.position = meters(1.0, 2.0, 3.0);
    kinematics.velocity = meters_per_second(4.0, -2.0, 0.0);

    // Under Test.
    integrate_midpoint(meters_per_second_squared(0.0, 0.0, 0.0), 0.5 * second,
                       InOut(kinematics));

    // Postconditions.
    CHECK(near(kinematics.position, meters(3.0, 1.0, 3.0)));
    CHECK(near(kinematics.velocity, meters_per_second(4.0, -2.0, 0.0)));
  }

  SECTION("ShouldMatchKinematicEquationsGivenConstantAcceleration") {
    // Preconditions.
    kinematics.velocity = meters_per_second(1.0, 0.0, 0.0);

    // Under Test.
    integrate_midpoint(meters_per_second_squared(0.0, 2.0, 0.0), 0.5 * second,
                       InOut(kinematics));

    // Postconditions.
    // p = v*t + a*t^2/2, v' = v + a*t
    CHECK(near(kinematics.position, meters(0.5, 0.25, 0.0)));
    CHECK(near(kinematics.velocity, meters_per_second(1.0, 1.0, 0.0)));
  }
}

TEST_CASE("Distance") {
  SECTION("ShouldBeEuclideanGivenTwoPositions") {
    // Preconditions.
    Kinematics a, b;
    b.position = meters(3.0, 4.0, 0.0);

    // Under Test.
    Length d = distance(a, b);

    // Postconditions.
    CHECK(d == 5.0 * meter);
  }

  SECTION("ShouldBeWithinGivenReachAtLeastTheDistance") {
    // Preconditions.
    Kinematics a, b;
    b.position = meters(3.0, 4.0, 0.0);

    // Postconditions.
    CHECK(within_distance(a, b, 5.0 * meter));
    CHECK(within_distance(a, b, 6.0 * meter));
    CHECK_FALSE(within_distance(a, b, 4.9 * meter));
  }
}

}  // namespace simon::model
