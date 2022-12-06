// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "base/contract.hpp"

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("Contracts") {
  SECTION("ShouldThrowOnPreconditionFailure") {
    REQUIRE_THROWS_AS(EXPECT(false), contract::PreconditionError);
  }
  SECTION("ShouldThrowOnPostconditionFailure") {
    REQUIRE_THROWS_AS(ENSURE(false), contract::PostconditionError);
  }
  SECTION("ShouldThrowOnInvariantFailure") {
    REQUIRE_THROWS_AS(ASSERT(false), contract::InvariantError);
  }
  SECTION("ShouldHaveSourceLocationOnPreconditionFailure") {
    try {
      EXPECT(false);
    } catch (const contract::PreconditionError& e) {
      CHECK(static_cast<std::string>(e.origin.file_name()).size());
    }
  }
  SECTION("ShouldHaveSourceLocationOnPostconditionFailure") {
    try {
      EXPECT(false);
    } catch (const contract::PreconditionError& e) {
      CHECK(static_cast<std::string>(e.origin.file_name()).size());
    }
  }
  SECTION("ShouldHaveSourceLocationOnInvariantFailure") {
    try {
      EXPECT(false);
    } catch (const contract::PreconditionError& e) {
      CHECK(static_cast<std::string>(e.origin.file_name()).size());
    }
  }
  SECTION("ShouldNotThrowOnPreconditionSuccess") {
    REQUIRE_NOTHROW(EXPECT(true));
  }
  SECTION("ShouldNotThrowOnPostconditionSuccess") {
    REQUIRE_NOTHROW(ENSURE(true));
  }
  SECTION("ShouldNotThrowOnInvariantSuccess") {
    REQUIRE_NOTHROW(ASSERT(true));
  }
}

}  // namespace simon::framework
