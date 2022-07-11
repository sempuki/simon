// Copyright 2022 -- CONTRIBUTORS.

#include "framework/contract.hpp"

#include <catch2/catch.hpp>

TEST_CASE("Contracts") {
  SECTION("ShouldThrowOnPreconditionFailure") {
    REQUIRE_THROWS_AS(EXPECTS(false), framework::contract::PreconditionError);
  }
  SECTION("ShouldThrowOnPostconditionFailure") {
    REQUIRE_THROWS_AS(ENSURES(false), framework::contract::PostconditionError);
  }
  SECTION("ShouldThrowOnInvariantFailure") {
    REQUIRE_THROWS_AS(ASSERT(false), framework::contract::InvariantError);
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
