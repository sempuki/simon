// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/entity.hpp"

#include <type_traits>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("Component") {
  SECTION("ShouldHaveStronglyTypedName") {
    REQUIRE(!std::is_same_v<EntityName, Name>);
  }

  Entity a, b;
  struct T final : public Component<T> {
  } c;

  SECTION("ShouldHaveDifferentNamesForDifferentObjects") {
    CHECK(a.entity_name() != b.entity_name());
  }

  SECTION("ShouldAttachAndFindComponents") {
    a.attach(&c);
    CHECK(a.component<T>() == &c);
  }
}

}  // namespace simon::framework
