// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "model/control.hpp"
#include "core/argument.hpp"

#include <cmath>
#include <limits>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"

namespace simon::model {

using Catch::Matchers::WithinAbs;

TEST_CASE("Lag") {
  SECTION("ShouldMatchExponentialGivenOneTimeConstant") {
    // Under Test.
    double value = lag(0.0, 1.0, 2.0 * second, 2.0 * second);

    // Postconditions.
    CHECK_THAT(value, WithinAbs(1.0 - std::exp(-1.0), 1e-12));
  }

  SECTION("ShouldMatchOneLongStepGivenManyShortOnes") {
    // Preconditions.
    double short_steps = 0.0;

    // Under Test.
    for (int i = 0; i < 100; ++i) {
      short_steps = lag(short_steps, 1.0, 0.5 * second, 0.01 * second);
    }
    double long_step = lag(0.0, 1.0, 0.5 * second, 1.0 * second);

    // Postconditions.
    CHECK_THAT(short_steps, WithinAbs(long_step, 1e-12));
  }

  SECTION("ShouldNotOvershootGivenStepMuchLongerThanTimeConstant") {
    // Under Test.
    double value = lag(0.0, 1.0, 0.01 * second, 100.0 * second);

    // Postconditions.
    CHECK(value <= 1.0);
    CHECK_THAT(value, WithinAbs(1.0, 1e-12));
  }

  SECTION("ShouldFollowInputGivenZeroTimeConstant") {
    // Postconditions.
    CHECK(lag(0.0, 3.0, 0.0 * second, 0.1 * second) == 3.0);
  }

  SECTION("ShouldLagQuantities") {
    // Under Test.
    Speed speed = lag(0.0 * meter_per_second, 10.0 * meter_per_second,
                      1.0 * second, 1.0 * second);

    // Postconditions.
    CHECK_THAT(speed.numerical_value_in(meter_per_second),
               WithinAbs(10.0 * (1.0 - std::exp(-1.0)), 1e-12));
  }
}

TEST_CASE("Approach") {
  SECTION("ShouldMoveByMostGivenFarTarget") {
    // Postconditions.
    CHECK(approach(0.0, 10.0, 2.0) == 2.0);
    CHECK(approach(0.0, -10.0, 2.0) == -2.0);
  }

  SECTION("ShouldReachTargetGivenNearTarget") {
    // Postconditions.
    CHECK(approach(0.0, 1.0, 2.0) == 1.0);
  }
}

TEST_CASE("PiControl") {
  // A throttle, a plain number, for a speed error.
  PiGains<Speed> gains{.proportional = 0.5 * second / meter,
                       .integral = 0.1 / meter,
                       .low = 0.0,
                       .high = 1.0};
  auto speed = [](double value) { return value * meter_per_second; };

  SECTION("ShouldAddProportionalAndIntegralGivenSmallError") {
    // Preconditions.
    double integral = 0.2;

    // Under Test.
    double output =
        pi_control(speed(0.4), gains, 1.0 * second, InOut(integral));

    // Postconditions.
    CHECK_THAT(integral, WithinAbs(0.24, 1e-12));
    CHECK_THAT(output, WithinAbs(0.44, 1e-12));
  }

  SECTION("ShouldHoldIntegralGivenOutputAtLimit") {
    // Preconditions.
    double integral = 0.9;

    // Under Test.
    double output =
        pi_control(speed(2.0), gains, 1.0 * second, InOut(integral));

    // Postconditions.
    CHECK(output == 1.0);
    CHECK(integral == 0.9);
  }

  SECTION("ShouldUnwindGivenErrorAwayFromLimit") {
    // Preconditions.
    double integral = 0.9;

    // Under Test.
    pi_control(speed(-1.0), gains, 1.0 * second, InOut(integral));

    // Postconditions.
    CHECK_THAT(integral, WithinAbs(0.8, 1e-12));
  }

  SECTION("ShouldCarryUnitsGivenQuantityOutput") {
    // Preconditions.
    // An acceleration for a speed error.
    PiGains<Speed, AccelerationMagnitude> accelerate{
        .proportional = 2.0 * per_second,
        .integral = 0.5 * per_second / second,
        .low = -10.0 * meter_per_second_squared,
        .high = 10.0 * meter_per_second_squared};
    AccelerationMagnitude integral = 0.0 * meter_per_second_squared;

    // Under Test.
    AccelerationMagnitude output =
        pi_control(speed(1.0), accelerate, 2.0 * second, InOut(integral));

    // Postconditions.
    CHECK(integral == 1.0 * meter_per_second_squared);
    CHECK(output == 3.0 * meter_per_second_squared);
  }
}

TEST_CASE("Table1WithQuantities") {
  // Preconditions.
  // Drag coefficient, a plain number, by speed.
  Table1<Speed> table{{100.0 * meter_per_second, 200.0 * meter_per_second},
                      {0.02, 0.04}};

  // Postconditions.
  CHECK_THAT(table(150.0 * meter_per_second), WithinAbs(0.03, 1e-12));
}

TEST_CASE("Table1") {
  Table1 table{{0.0, 1.0, 3.0}, {0.0, 10.0, 30.0}};

  SECTION("ShouldInterpolateGivenPointBetweenBreakpoints") {
    // Postconditions.
    CHECK_THAT(table(0.5), WithinAbs(5.0, 1e-12));
    CHECK_THAT(table(2.0), WithinAbs(20.0, 1e-12));
  }

  SECTION("ShouldReturnValueGivenBreakpoint") {
    // Postconditions.
    CHECK(table(1.0) == 10.0);
    CHECK(table(3.0) == 30.0);
  }

  SECTION("ShouldClampGivenPointOutsideBreakpoints") {
    // Postconditions.
    CHECK(table(-1.0) == 0.0);
    CHECK(table(9.0) == 30.0);
  }

  SECTION("ShouldBeConstantGivenOneBreakpoint") {
    // Preconditions.
    Table1 constant{{2.0}, {7.0}};

    // Postconditions.
    CHECK(constant(-5.0) == 7.0);
    CHECK(constant(5.0) == 7.0);
  }

  SECTION("ShouldStayInBoundsGivenNaN") {
    // Postconditions.
    CHECK(table(std::numeric_limits<double>::quiet_NaN()) == 0.0);
  }
}

TEST_CASE("Table2") {
  // f(row, column) = row + 10 * column, which bilinear interpolation repeats
  // exactly.
  Table2 table{{0.0, 1.0}, {0.0, 1.0, 2.0}, {0.0, 10.0, 20.0, 1.0, 11.0, 21.0}};

  SECTION("ShouldInterpolateGivenPointInsideTable") {
    // Postconditions.
    CHECK_THAT(table(0.5, 1.5), WithinAbs(15.5, 1e-12));
  }

  SECTION("ShouldClampGivenPointOutsideTable") {
    // Postconditions.
    CHECK_THAT(table(2.0, 5.0), WithinAbs(21.0, 1e-12));
    CHECK_THAT(table(-1.0, 0.5), WithinAbs(5.0, 1e-12));
  }

  SECTION("ShouldStayInBoundsGivenNaN") {
    // Preconditions.
    double nan = std::numeric_limits<double>::quiet_NaN();

    // Postconditions.
    CHECK_THAT(table(nan, 0.5), WithinAbs(5.0, 1e-12));
    CHECK_THAT(table(0.5, nan), WithinAbs(0.5, 1e-12));
  }
}

}  // namespace simon::model
