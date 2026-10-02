// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "framework/name.hpp"

#include <format>
#include <string>
#include <string_view>
#include <type_traits>

#include "base/testing.hpp"

namespace simon::framework {

TEST_CASE("Name") {
  SECTION("ShouldBeEqualGivenSameKindAndInstance") {
    CHECK(Name{Kind::ENTITY, 2} == Name{Kind::ENTITY, 2});
    CHECK(Name{Kind::ENTITY, 2} != Name{Kind::SYSTEM, 2});
    CHECK(Name{Kind::ENTITY, 2} != Name{Kind::ENTITY, 3});
  }

  SECTION("ShouldBeNoneGivenDefault") {
    CHECK(Name{}.kind == static_cast<std::uint32_t>(Kind::NONE));
  }

  SECTION("ShouldRoundTripComponentGivenEntityComponentName") {
    Name name = entity_component_name(3, 2);
    CHECK(is_entity_component(name));
    CHECK(component_of(name) == 3u);
    CHECK(name.instance == 2u);
    CHECK_FALSE(is_entity_component(Name{Kind::ENTITY, 2}));
  }
}

TEST_CASE("Identity") {
  SECTION("ShouldFormatRestPathGivenEachKind") {
    CHECK(format_identity(1, Name{Kind::WORLD, 1}) == "/world/1");
    CHECK(format_identity(1, Name{Kind::ARCHETYPE, 0}) ==
          "/world/1/archetype/0");
    CHECK(format_identity(1, Name{Kind::COMPONENT, 3}) ==
          "/world/1/component/3");
    CHECK(format_identity(1, Name{Kind::SYSTEM, 4}) == "/world/1/system/4");
    CHECK(format_identity(1, Name{Kind::ENTITY, 2}) == "/world/1/entity/2");
    CHECK(format_identity(1, entity_component_name(3, 2)) ==
          "/world/1/entity/2/component/3");
  }

  SECTION("ShouldRoundTripGivenFormattedIdentity") {
    for (Name name : {Name{Kind::WORLD, 7}, Name{Kind::ARCHETYPE, 1},
                      Name{Kind::COMPONENT, 3}, Name{Kind::SYSTEM, 4},
                      Name{Kind::ENTITY, 2}, entity_component_name(5, 9)}) {
      auto parsed = parse_identity(format_identity(7, name));
      REQUIRE(parsed);
      CHECK(parsed->world == 7u);
      CHECK(parsed->name == name);
    }
  }

  SECTION("ShouldRejectGivenMalformedIdentity") {
    for (std::string_view text :
         {"", "world/1", "/world", "/world/x", "/world/1/entity",
          "/world/1/entity/2/component", "/world/1/robot/2",
          "/world/1/entity/-2", "/world/1/entity/2/extra/3",
          "/world/1/entity/2x"}) {
      CHECK_FALSE(parse_identity(text));
    }
  }

  SECTION("ShouldRejectGivenComponentNumberBeyondKindRange") {
    // ENTITY_COMPONENT + component must fit in 32 bits.
    CHECK_FALSE(
        parse_identity(Identity{"/world/1/entity/0/component/4294967295"}));
  }

  SECTION("ShouldRejectGivenTrailingSlashOrLeadingZero") {
    CHECK_FALSE(parse_identity(Identity{"/world/1/"}));
    CHECK_FALSE(parse_identity(Identity{"/world/007"}));
    CHECK_FALSE(parse_identity(Identity{"/world/1/entity/02"}));
    CHECK(parse_identity(Identity{"/world/0/entity/0"}));
  }
}

TEST_CASE("TaggedString") {
  SECTION("ShouldConvertImplicitlyGivenStringLikeValues") {
    Alias from_literal = "ego";
    Alias from_view = std::string_view{"ego"};
    Alias from_string = std::string{"ego"};
    CHECK(from_literal == from_view);
    CHECK(from_view == from_string);
    CHECK(from_literal == "ego");
    CHECK(from_literal.view() == "ego");
  }

  SECTION("ShouldNotConvertGivenOtherTag") {
    static_assert(!std::is_convertible_v<Identity, Alias>);
    static_assert(!std::is_convertible_v<Alias, Identity>);
    static_assert(!std::is_convertible_v<Alias, std::string_view>);
  }

  SECTION("ShouldFormatAsTextGivenStdFormat") {
    CHECK(std::format("[{}]", Alias{"Luke Skywalker"}) == "[Luke Skywalker]");
  }
}

}  // namespace simon::framework
