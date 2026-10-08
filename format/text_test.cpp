// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/text.hpp"

#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"

namespace simon::format {
namespace {

using Catch::Matchers::ContainsSubstring;

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

  SECTION("ShouldTrim") {
    // Postconditions.
    CHECK(trim("  a b \n") == "a b");
    CHECK(trim("   ").empty());
  }

  SECTION("ShouldFindLineGivenOffset") {
    // Preconditions.
    std::string_view text = "one\ntwo\nthree";

    // Postconditions.
    CHECK(find_line(text, 0) == 1);
    CHECK(find_line(text, 3) == 1);
    CHECK(find_line(text, 4) == 2);
    CHECK(find_line(text, 9) == 3);
    CHECK(find_line(text, -1) == 1);
    CHECK(find_line(text, 100) == 3);
  }

  SECTION("ShouldSayLineGivenFailure") {
    // Under Test.
    Failure failure = fail_at(7, "needs id");

    // Postconditions.
    CHECK(failure.error().kind() == lib::watch(FormatError::MALFORMED));
    CHECK_THAT(std::string{failure.error().message()},
               ContainsSubstring("line 7: needs id"));
  }

  SECTION("ShouldNameWhatIsRefused") {
    // Under Test.
    Failure failure = refuse("the CG solver");

    // Postconditions.
    CHECK(failure.error().kind() == lib::watch(FormatError::UNSUPPORTED));
    CHECK(failure.error().message() == "the CG solver");
  }

  SECTION("ShouldRefuseToReadMissingFile") {
    // Under Test.
    auto text = read_text_file("no/such/file.txt");

    // Postconditions.
    REQUIRE_FALSE(text);
    CHECK(text.error().kind() == lib::watch(FormatError::UNREADABLE));
  }
}

}  // namespace
}  // namespace simon::format
