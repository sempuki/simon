// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "component/environment.hpp"

#include "base/testing.hpp"

namespace simon {

TEST_CASE("Environment") {
  SECTION("ShouldBeZeroGivenDefaultConstruction") {
    component::Environment environment;
    CHECK(environment.wind.isZero());
  }
}

}  // namespace simon
