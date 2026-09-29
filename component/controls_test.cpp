// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "component/controls.hpp"

#include "base/testing.hpp"

namespace simon {

TEST_CASE("Controls") {
  SECTION("ShouldBeZeroGivenDefaultConstruction") {
    component::Controls controls;
    CHECK(controls.acceleration.isZero());
  }
}

}  // namespace simon
