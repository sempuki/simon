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
    double value = lag(0.0, 1.0, 2.0 * second, 2.0 * second);
    CHECK_THAT(value, WithinAbs(1.0 - std::exp(-1.0), 1e-12));
  }

  SECTION("ShouldMatchOneLongStepGivenManyShortOnes") {
    double short_steps = 0.0;
    for (int i = 0; i < 100; ++i) {
      short_steps = lag(short_steps, 1.0, 0.5 * second, 0.01 * second);
    }
    double long_step = lag(0.0, 1.0, 0.5 * second, 1.0 * second);
    CHECK_THAT(short_steps, WithinAbs(long_step, 1e-12));
  }

  SECTION("ShouldNotOvershootGivenStepMuchLongerThanTimeConstant") {
    double value = lag(0.0, 1.0, 0.01 * second, 100.0 * second);
    CHECK(value <= 1.0);
    CHECK_THAT(value, WithinAbs(1.0, 1e-12));
  }

  SECTION("ShouldFollowInputGivenZeroTimeConstant") {
    CHECK(lag(0.0, 3.0, 0.0 * second, 0.1 * second) == 3.0);
  }

  SECTION("ShouldLagQuantities") {
    Speed speed = lag(0.0 * meter_per_second, 10.0 * meter_per_second,
                      1.0 * second, 1.0 * second);
    CHECK_THAT(speed.numerical_value_in(meter_per_second),
               WithinAbs(10.0 * (1.0 - std::exp(-1.0)), 1e-12));
  }
}

TEST_CASE("Approach") {
  SECTION("ShouldMoveByMostGivenFarTarget") {
    CHECK(approach(0.0, 10.0, 2.0) == 2.0);
    CHECK(approach(0.0, -10.0, 2.0) == -2.0);
  }

  SECTION("ShouldReachTargetGivenNearTarget") {
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
    double integral = 0.2;
    double output =
        pi_control(speed(0.4), gains, 1.0 * second, InOut(integral));
    CHECK_THAT(integral, WithinAbs(0.24, 1e-12));
    CHECK_THAT(output, WithinAbs(0.44, 1e-12));
  }

  SECTION("ShouldHoldIntegralGivenOutputAtLimit") {
    double integral = 0.9;
    double output =
        pi_control(speed(2.0), gains, 1.0 * second, InOut(integral));
    CHECK(output == 1.0);
    CHECK(integral == 0.9);
  }

  SECTION("ShouldUnwindGivenErrorAwayFromLimit") {
    double integral = 0.9;
    pi_control(speed(-1.0), gains, 1.0 * second, InOut(integral));
    CHECK_THAT(integral, WithinAbs(0.8, 1e-12));
  }

  SECTION("ShouldCarryUnitsGivenQuantityOutput") {
    // An acceleration for a speed error.
    PiGains<Speed, AccelerationMagnitude> accelerate{
        .proportional = 2.0 * per_second,
        .integral = 0.5 * per_second / second,
        .low = -10.0 * meter_per_second_squared,
        .high = 10.0 * meter_per_second_squared};
    AccelerationMagnitude integral = 0.0 * meter_per_second_squared;

    AccelerationMagnitude output =
        pi_control(speed(1.0), accelerate, 2.0 * second, InOut(integral));

    CHECK(integral == 1.0 * meter_per_second_squared);
    CHECK(output == 3.0 * meter_per_second_squared);
  }
}

TEST_CASE("Table1WithQuantities") {
  // Drag coefficient, a plain number, by speed.
  Table1<Speed> table{{100.0 * meter_per_second, 200.0 * meter_per_second},
                      {0.02, 0.04}};
  CHECK_THAT(table(150.0 * meter_per_second), WithinAbs(0.03, 1e-12));
}

TEST_CASE("Table1") {
  Table1 table{{0.0, 1.0, 3.0}, {0.0, 10.0, 30.0}};

  SECTION("ShouldInterpolateGivenPointBetweenBreakpoints") {
    CHECK_THAT(table(0.5), WithinAbs(5.0, 1e-12));
    CHECK_THAT(table(2.0), WithinAbs(20.0, 1e-12));
  }

  SECTION("ShouldReturnValueGivenBreakpoint") {
    CHECK(table(1.0) == 10.0);
    CHECK(table(3.0) == 30.0);
  }

  SECTION("ShouldClampGivenPointOutsideBreakpoints") {
    CHECK(table(-1.0) == 0.0);
    CHECK(table(9.0) == 30.0);
  }

  SECTION("ShouldBeConstantGivenOneBreakpoint") {
    Table1 constant{{2.0}, {7.0}};
    CHECK(constant(-5.0) == 7.0);
    CHECK(constant(5.0) == 7.0);
  }

  SECTION("ShouldStayInBoundsGivenNaN") {
    CHECK(table(std::numeric_limits<double>::quiet_NaN()) == 0.0);
  }
}

TEST_CASE("Table2") {
  // f(row, column) = row + 10 * column, which bilinear interpolation repeats
  // exactly.
  Table2 table{{0.0, 1.0}, {0.0, 1.0, 2.0}, {0.0, 10.0, 20.0, 1.0, 11.0, 21.0}};

  SECTION("ShouldInterpolateGivenPointInsideTable") {
    CHECK_THAT(table(0.5, 1.5), WithinAbs(15.5, 1e-12));
  }

  SECTION("ShouldClampGivenPointOutsideTable") {
    CHECK_THAT(table(2.0, 5.0), WithinAbs(21.0, 1e-12));
    CHECK_THAT(table(-1.0, 0.5), WithinAbs(5.0, 1e-12));
  }

  SECTION("ShouldStayInBoundsGivenNaN") {
    double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THAT(table(nan, 0.5), WithinAbs(5.0, 1e-12));
    CHECK_THAT(table(0.5, nan), WithinAbs(0.5, 1e-12));
  }
}

}  // namespace simon::model
