// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "scenario/transition.hpp"

#include <cmath>
#include <numbers>

#include "base/testing.hpp"
#include "scenario/openscenario.hpp"

namespace simon::scenario {

TEST_CASE("Transition") {
  SECTION("ShouldFollowEachShapeGivenAQuarterOfTheWay") {
    // Preconditions.
    Transition transition{.shape = DynamicsShape::LINEAR,
                          .start = 2.0,
                          .target = 6.0,
                          .end = 4.0,
                          .parameter = 1.0};

    // Under Test.
    double linear = transition.evaluate(DynamicsShape::LINEAR);
    double cubic = transition.evaluate(DynamicsShape::CUBIC);
    double sinusoidal = transition.evaluate(DynamicsShape::SINUSOIDAL);

    // Postconditions.
    CHECK(linear == 3.0);
    CHECK(cubic == 2.625);  // 2 + 4 (3 x^2 - 2 x^3) at x = 1/4.
    CHECK(std::abs(sinusoidal -
                   (2.0 + 2.0 * (1.0 - std::cos(0.25 * std::numbers::pi)))) <
          1e-15);
  }

  SECTION("ShouldJumpToTargetGivenStepOrNoLength") {
    // Preconditions.
    Transition step{
        .shape = DynamicsShape::STEP, .start = 2.0, .target = 6.0, .end = 4.0};
    Transition empty{
        .shape = DynamicsShape::LINEAR, .start = 2.0, .target = 6.0};

    // Under Test.
    double stepped = step.evaluate();
    double emptied = empty.evaluate();

    // Postconditions.
    CHECK(stepped == 6.0);
    CHECK(emptied == 6.0);
  }

  SECTION("ShouldStopAtItsEndGivenAdvancePastIt") {
    // Preconditions.
    Transition transition{.shape = DynamicsShape::LINEAR,
                          .start = 2.0,
                          .target = 6.0,
                          .end = 4.0};

    // Under Test.
    transition.advance(10.0);

    // Postconditions.
    CHECK(transition.parameter == 4.0);
    CHECK(transition.done());
    CHECK(transition.evaluate() == 6.0);
  }

  SECTION("ShouldPeakAtTheLimitGivenStretched") {
    // Preconditions.
    Transition transition{
        .shape = DynamicsShape::CUBIC, .start = 0.0, .target = 3.0, .end = 1.0};

    // Under Test.
    transition.stretch_to(1.5);

    // Postconditions.
    // A cubic's rate peaks at 1.5 |b| / c, halfway.
    CHECK(transition.end == 3.0);
    transition.parameter = 1.5;
    CHECK(transition.compute_slope() == 1.5);
  }
}

}  // namespace simon::scenario
