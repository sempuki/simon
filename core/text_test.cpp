// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "core/text.hpp"

#include "base/testing.hpp"

namespace simon {

TEST_CASE("Text") {
  SECTION("ShouldParseNumberGivenBlanksAndPlus") {
    // Postconditions.
    CHECK(parse_number("1.5") == 1.5);
    CHECK(parse_number(" \t-2e3 ") == -2000.0);
    CHECK(parse_number("+7") == 7.0);
    CHECK(parse_number(".5") == 0.5);
  }

  SECTION("ShouldRefuseNumberGivenOtherText") {
    // Postconditions.
    CHECK_FALSE(parse_number(""));
    CHECK_FALSE(parse_number("wide"));
    CHECK_FALSE(parse_number("1.5 m"));
    CHECK_FALSE(parse_number("inf"));
    CHECK_FALSE(parse_number("nan"));
    CHECK_FALSE(parse_number("++1"));
  }

  SECTION("ShouldParseIntegerGivenBlanksAndPlus") {
    // Postconditions.
    CHECK(parse_integer("42") == 42);
    CHECK(parse_integer(" -7\n") == -7);
    CHECK(parse_integer("+3") == 3);
  }

  SECTION("ShouldRefuseIntegerGivenFractionOrOtherText") {
    // Postconditions.
    CHECK_FALSE(parse_integer("1.5"));
    CHECK_FALSE(parse_integer("4o"));
    CHECK_FALSE(parse_integer(""));
  }

  SECTION("ShouldTrim") {
    // Postconditions.
    CHECK(trim("  a b \n") == "a b");
    CHECK(trim("   ").empty());
  }
}

}  // namespace simon
