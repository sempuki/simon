// Copyright 2022 -- CONTRIBUTORS. See LICENSE.

#include "framework/contract.hpp"

#include "catch2/catch.hpp"

namespace simon::framework {

TEST_CASE("Contracts") {
  SECTION("ShouldThrowOnPreconditionFailure") {
    REQUIRE_THROWS_AS(EXPECTS(false), contract::PreconditionError);
  }
  SECTION("ShouldThrowOnPostconditionFailure") {
    REQUIRE_THROWS_AS(ENSURES(false), contract::PostconditionError);
  }
  SECTION("ShouldThrowOnInvariantFailure") {
    REQUIRE_THROWS_AS(ASSERT(false), contract::InvariantError);
  }
  SECTION("ShouldHaveSourceLocationOnPreconditionFailure") {
    try {
      EXPECTS(false);
    } catch (const contract::PreconditionError& e) {
      CHECK(static_cast<std::string>(e.origin.file_name()).size());
    }
  }
  SECTION("ShouldHaveSourceLocationOnPostconditionFailure") {
    try {
      EXPECTS(false);
    } catch (const contract::PreconditionError& e) {
      CHECK(static_cast<std::string>(e.origin.file_name()).size());
    }
  }
  SECTION("ShouldHaveSourceLocationOnInvariantFailure") {
    try {
      EXPECTS(false);
    } catch (const contract::PreconditionError& e) {
      CHECK(static_cast<std::string>(e.origin.file_name()).size());
    }
  }
  SECTION("ShouldNotThrowOnPreconditionSuccess") {
    REQUIRE_NOTHROW(EXPECTS(true));
  }
  SECTION("ShouldNotThrowOnPostconditionSuccess") {
    REQUIRE_NOTHROW(ENSURES(true));
  }
  SECTION("ShouldNotThrowOnInvariantSuccess") {
    REQUIRE_NOTHROW(ASSERT(true));
  }
}

}  // namespace simon::framework
