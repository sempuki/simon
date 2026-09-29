// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "component/movement.hpp"

#include "base/testing.hpp"

namespace simon {

TEST_CASE("Movement") {
  SECTION("ShouldBeZeroGivenDefaultConstruction") {
    component::Movement movement;
    CHECK(movement.position.isZero());
    CHECK(movement.velocity.isZero());
  }
}

}  // namespace simon
