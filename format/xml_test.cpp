// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "format/xml.hpp"

#include <string>

#include "base/testing.hpp"
#include "catch2/matchers/catch_matchers_string.hpp"

namespace simon::format {
namespace {

using Catch::Matchers::ContainsSubstring;

constexpr std::string_view DOCUMENT = R"(<?xml version="1.0"?>
<root>
  <item id="3" width="wide"/>
  <item id="+4"/>
</root>
)";

TEST_CASE("XmlDocument") {
  XmlDocument document;
  REQUIRE(document.load(std::string{DOCUMENT}));
  auto root = document.find_root("root");
  REQUIRE(root);
  pugi::xml_node first = root->child("item");
  pugi::xml_node second = first.next_sibling("item");

  SECTION("ShouldFindLineOfNode") {
    // Postconditions.
    CHECK(document.find_line(*root) == 2);
    CHECK(document.find_line(first) == 3);
    CHECK(document.find_line(second) == 4);
  }

  SECTION("ShouldHoldItsOwnNodes") {
    // Preconditions.
    XmlDocument other;
    REQUIRE(other.load("<root><item/></root>"));

    // Postconditions.
    CHECK(document.holds(first));
    CHECK_FALSE(document.holds(other.node().child("root").child("item")));
    CHECK_FALSE(document.holds(pugi::xml_node{}));
  }

  SECTION("ShouldReadNumber") {
    // Postconditions.
    CHECK(document.read_number(first, "id") == 3.0);
    CHECK(document.read_number(second, "id") == 4.0);
  }

  SECTION("ShouldSayWhereGivenMissingOrBadNumber") {
    // Under Test.
    auto missing = document.read_number(second, "width");

    // Postconditions.
    REQUIRE_FALSE(missing);
    CHECK_THAT(std::string{missing.error().message()},
               ContainsSubstring("line 4: <item> needs width"));

    // Under Test.
    auto bad = document.read_number(first, "width");

    // Postconditions.
    REQUIRE_FALSE(bad);
    CHECK_THAT(std::string{bad.error().message()},
               ContainsSubstring("line 3: <item> width `wide` is not a "
                                 "finite number"));
  }

  SECTION("ShouldSayWhereGivenFailure") {
    // Under Test.
    Failure failure = document.fail(second, "needs a name");

    // Postconditions.
    CHECK(failure.error().kind() == lib::watch(FormatError::MALFORMED));
    CHECK(failure.error().message() == "line 4: <item> needs a name");
  }

  SECTION("ShouldSayWhereGivenRefusal") {
    // Postconditions.
    CHECK(document.refuse(first).error().message() == "line 3: <item>");
    CHECK(document.refuse(first, "width").error().message() ==
          "line 3: <item> width");
    CHECK(document.refuse(first).error().kind() ==
          lib::watch(FormatError::UNSUPPORTED));
  }

  SECTION("ShouldSayOnlyWhyGivenNoNode") {
    // Postconditions.
    CHECK(document.fail(pugi::xml_node{}, "no root").error().message() ==
          "no root");
    CHECK(document.refuse(pugi::xml_node{}, "closed").error().message() ==
          "closed");
  }

  SECTION("ShouldRefuseGivenMissingRoot") {
    // Under Test.
    auto other = document.find_root("other");

    // Postconditions.
    REQUIRE_FALSE(other);
    CHECK_THAT(std::string{other.error().message()},
               ContainsSubstring("no other element"));
  }

  SECTION("ShouldSayWhereGivenMalformedXml") {
    // Preconditions.
    XmlDocument broken;

    // Under Test.
    auto loaded = broken.load("<root>\n  <item>\n</root>");

    // Postconditions.
    REQUIRE_FALSE(loaded);
    CHECK(loaded.error().kind() == lib::watch(FormatError::MALFORMED));
    CHECK_THAT(std::string{loaded.error().message()},
               ContainsSubstring("line 3"));
  }
}

}  // namespace
}  // namespace simon::format
