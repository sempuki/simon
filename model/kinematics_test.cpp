// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/kinematics.hpp"

#include "base/testing.hpp"

namespace simon::model {

namespace {
bool near(const Displacement& a, const Displacement& b) {
  return a.numerical_value_in(metre).is_approximately(b.numerical_value_in(metre));
}
bool near(const Velocity& a, const Velocity& b) {
  return a.numerical_value_in(metre_per_second)
      .is_approximately(b.numerical_value_in(metre_per_second));
}
}  // namespace

TEST_CASE("IntegrateMidpoint") {
  Kinematics kinematics;

  SECTION("ShouldMoveByVelocityTimesStepGivenZeroAcceleration") {
    kinematics.position = metres(1.0, 2.0, 3.0);
    kinematics.velocity = metres_per_second(4.0, -2.0, 0.0);

    integrate_midpoint(kinematics, metres_per_second_squared(0.0, 0.0, 0.0), 0.5 * second);

    CHECK(near(kinematics.position, metres(3.0, 1.0, 3.0)));
    CHECK(near(kinematics.velocity, metres_per_second(4.0, -2.0, 0.0)));
  }

  SECTION("ShouldMatchKinematicEquationsGivenConstantAcceleration") {
    kinematics.velocity = metres_per_second(1.0, 0.0, 0.0);

    integrate_midpoint(kinematics, metres_per_second_squared(0.0, 2.0, 0.0), 0.5 * second);

    // p = v*t + a*t^2/2, v' = v + a*t
    CHECK(near(kinematics.position, metres(0.5, 0.25, 0.0)));
    CHECK(near(kinematics.velocity, metres_per_second(1.0, 1.0, 0.0)));
  }
}

TEST_CASE("Distance") {
  SECTION("ShouldBeEuclideanGivenTwoPositions") {
    Kinematics a, b;
    b.position = metres(3.0, 4.0, 0.0);
    CHECK(distance(a, b) == 5.0 * metre);
  }
}

}  // namespace simon::model
