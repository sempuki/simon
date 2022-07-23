// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/component.hpp"

#include <type_traits>

#include "framework/testing.hpp"

namespace simon::framework {

TEST_CASE("Component") {
  SECTION("ShouldHaveStronglyTypedName") {
    REQUIRE(!std::is_same_v<ComponentName, Name>);
  }

  struct T final : public Component<T> {
  } c;

  SECTION("ShouldHaveSameNameForObjectAsType") {
    CHECK(c.component_name() == T::name());
  }

  SECTION("ShouldHaveSameNameForBaseObjectAsType") {
    CHECK(static_cast<ComponentBase*>(&c)->component_name() == T::name());
  }
}

}  // namespace simon::framework
