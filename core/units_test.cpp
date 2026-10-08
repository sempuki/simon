// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "core/units.hpp"

#include <chrono>

#include "base/testing.hpp"

namespace simon {

template <typename FirstType, typename SecondType>
concept Addable = requires(FirstType a, SecondType b) { a + b; };

TEST_CASE("Units") {
  SECTION("ShouldCarryUnitsThroughArithmeticGivenVectors") {
    // Preconditions.
    Velocity velocity = meters_per_second(1.0, 2.0, 3.0);
    Time dt = 2.0 * second;

    // Under Test.
    Displacement moved = velocity * dt;

    // Postconditions.
    CHECK(moved.numerical_value_in(meter).is_approximately(
        QuantityVector{2.0, 4.0, 6.0}));
  }

  SECTION("ShouldConvertChronoDurationGivenNanoseconds") {
    // Postconditions.
    CHECK(seconds(std::chrono::milliseconds{250}) == 0.25 * second);
  }

  SECTION("ShouldComputeNormDotAndCrossGivenQuantities") {
    // Preconditions.
    Displacement a = meters(3.0, 4.0, 0.0);
    Displacement b = meters(0.0, 0.0, 2.0);

    // Postconditions.
    CHECK(norm(a) == 5.0 * meter);
    CHECK(dot(a, b) == 0.0 * units::square(meter));
    CHECK(cross(a, b)
              .numerical_value_in(units::square(meter))
              .is_approximately(QuantityVector{8.0, -6.0, 0.0}));
  }

  SECTION("ShouldCarryUnitsGivenAnglesAndForces") {
    // Preconditions.
    AngularRate turn = 0.1 * radian_per_second;
    Density density = 1.2 * kilogram_per_cubic_meter;

    // Under Test.
    Angle turned = turn * (2.0 * second);
    Force force = 10.0 * kilogram * (2.0 * meter_per_second_squared);
    AccelerationMagnitude acceleration = force / (4.0 * kilogram);
    Force lift = 0.5 * density * (10.0 * meter_per_second) *
                 (10.0 * meter_per_second) * (2.0 * square_meter);

    // Postconditions.
    CHECK(radians(turned) == 0.2);
    CHECK(acceleration == 5.0 * meter_per_second_squared);
    CHECK(lift.numerical_value_in(newton) == 120.0);
  }

  SECTION("ShouldNotAddGivenDifferentUnits") {
    // Postconditions.
    static_assert(!Addable<Displacement, Velocity>);
    static_assert(Addable<Displacement, Displacement>);
  }
}

}  // namespace simon
