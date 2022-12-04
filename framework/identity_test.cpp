// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/identity.hpp"

#include <sstream>
#include <type_traits>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("Name") {
  SECTION("ShouldNotBeDefaultable") {
    REQUIRE(!std::is_default_constructible_v<Name>);
  }

  SECTION("ShouldBeCopyable") {
    CHECK(std::is_copy_constructible_v<Name>);
    CHECK(std::is_copy_assignable_v<Name>);
  }

  SECTION("ShouldOutputToStdOstream") {
    std::stringstream s;
    s << Identity{}.name();
    CHECK(s);
  }

  SECTION("ShouldParseAsSizeT") {
    std::size_t n;
    std::stringstream s;
    s << Identity{}.name();
    s >> n;
    CHECK(s);
  }
}

TEST_CASE("Identity") {
  SECTION("ShouldBeDefaultable") {
    REQUIRE(std::is_default_constructible_v<Identity>);
  }

  SECTION("ShouldBeMoveOnly") {
    CHECK(std::is_move_constructible_v<Identity>);
    CHECK(std::is_move_assignable_v<Identity>);
    CHECK(!std::is_copy_constructible_v<Identity>);
    CHECK(!std::is_copy_assignable_v<Identity>);
  }

  SECTION("ShouldHaveDifferentIdentitiesForDifferentObjects") {
    REQUIRE(Identity{} != Identity{});
  }

  SECTION("ShouldHaveSameIdentityForSameObject") {
    Identity identity;
    REQUIRE(identity == identity);
  }

  SECTION("ShouldNotCompareLessForSameObject") {
    Identity identity;
    REQUIRE(!(identity < identity));
  }

  SECTION("ShouldHaveDifferentNamesForDifferentIdentities") {
    REQUIRE(Identity{}.name() != Identity{}.name());
  }

  SECTION("ShouldHaveSameNamesForSameIdentities") {
    Identity identity;
    REQUIRE(identity.name() == identity.name());
  }

  SECTION("ShouldNotCompareLessNamesForSameIdentities") {
    Identity identity;
    REQUIRE(!(identity.name() < identity.name()));
  }
}

}  // namespace simon::framework
