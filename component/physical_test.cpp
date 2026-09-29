// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "component/physical.hpp"

#include "base/testing.hpp"

namespace simon {

TEST_CASE("Physical") {
  SECTION("ShouldBeZeroGivenDefaultConstruction") {
    component::Physical physical;
    CHECK(physical.radius == 0.0);
  }
}

}  // namespace simon
