// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "model/units.hpp"

#include <chrono>

#include "base/testing.hpp"

namespace simon::model {

template <typename A, typename B>
concept Addable = requires(A a, B b) { a + b; };

TEST_CASE("Units") {
  SECTION("ShouldCarryUnitsThroughArithmeticGivenVectors") {
    Velocity velocity = metres_per_second(1.0, 2.0, 3.0);
    Time dt = 2.0 * second;

    Displacement moved = velocity * dt;

    CHECK(moved.numerical_value_in(metre).is_approximately(Vector3{2.0, 4.0, 6.0}));
  }

  SECTION("ShouldConvertChronoDurationGivenNanoseconds") {
    CHECK(seconds(std::chrono::milliseconds{250}) == 0.25 * second);
  }

  SECTION("ShouldComputeNormDotAndCrossGivenQuantities") {
    Displacement a = metres(3.0, 4.0, 0.0);
    Displacement b = metres(0.0, 0.0, 2.0);

    CHECK(norm(a) == 5.0 * metre);
    CHECK(dot(a, b) == 0.0 * units::square(metre));
    CHECK(cross(a, b).numerical_value_in(units::square(metre))
              .is_approximately(Vector3{8.0, -6.0, 0.0}));
  }

  SECTION("ShouldNotAddGivenDifferentUnits") {
    static_assert(!Addable<Displacement, Velocity>);
    static_assert(Addable<Displacement, Displacement>);
  }
}

}  // namespace simon::model
