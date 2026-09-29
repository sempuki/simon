// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/entity.hpp"

#include <stdexcept>
#include <type_traits>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("Entity") {
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

  SECTION("ShouldReturnNullGivenNoComponentOfType") {
    CHECK(a.component<T>() == nullptr);
  }

  SECTION("ShouldFindEachComponentGivenSeveralTypes") {
    struct U final : public Component<U> {
    } d;
    a.attach(&c);
    a.attach(&d);
    CHECK(a.component<T>() == &c);
    CHECK(a.component<U>() == &d);
  }

  SECTION("ShouldThrowAndKeepFirstGivenSecondComponentOfSameType") {
    T other;
    a.attach(&c);
    CHECK_THROWS_AS(a.attach(&other), std::logic_error);
    CHECK(a.component<T>() == &c);
  }
}

}  // namespace simon::framework
